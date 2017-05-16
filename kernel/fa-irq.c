/*
 * Copyright CERN 2012
 * Author: Federico Vaga <federico.vaga@gmail.com>
 *
 * IRQ-related code
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/bitops.h>
#include <linux/spinlock.h>
#include <linux/io.h>

#include "fmc-adc-100m14b4cha.h"
#include "fa-spec.h"

/*
 * fat_get_irq_status
 * @fa: adc descriptor
 * @irq_status: destination of irq status
 *
 * Get irq and clear the register. To clear an interrupt we have to write 1
 * on the handled interrupt. We handle all interrupt so we clear all interrupts
 */
static void fa_get_irq_status(struct fa_dev *fa, int irq_core_base,
			      uint32_t *irq_status)
{
	/* Get current interrupts status */
	*irq_status = fa_readl(fa, irq_core_base, &zfad_regs[ZFA_IRQ_ADC_SRC]);
	dev_dbg(fa->msgdev,
		"IRQ 0x%x fired an interrupt. IRQ status register: 0x%x\n",
		irq_core_base, *irq_status);

	if (*irq_status)
		/* Clear current interrupts status */
		fa_writel(fa, irq_core_base, &zfad_regs[ZFA_IRQ_ADC_SRC],
				*irq_status);
}

/*
 * fa_irq_handler
 * @irq:
 * @ptr: pointer to fmc_device
 *
 * The ADC svec firmware fires interrupt from a single wishbone core
 * and throught the VIC ACQ_END and TRIG events.  Note about "TRIG"
 * event: the main reason to listen this interrupt was to read the
 * intermediate time stamps in case of multishots.
 * With the new firmware (>=3.0) the stamps come with the data,
 * therefore the driver doesn't have to listen "TRIG" event. This
 * enhancement remove completely the risk of loosing interrupt in case
 * of small number of samples and makes the retry loop in the hanlder
 * obsolete.
 */
irqreturn_t fa_irq_handler(int irq_core_base, void *dev_id)
{
	struct fmc_device *fmc = dev_id;
	struct fa_dev *fa = fmc_get_drvdata(fmc);
	struct zio_cset *cset = fa->zdev->cset;
	uint32_t status;
	unsigned long flags;
	struct zfad_block *zfad_block;

	/* irq to handle */
	fa_get_irq_status(fa, irq_core_base, &status);
	if (!status)
		return IRQ_NONE; /* No interrupt fired by this mezzanine */

	dev_dbg(fa->msgdev, "Handle ADC interrupts fmc slot: %d\n",
		fmc->slot_id);

	if (status & FA_IRQ_ADC_ACQ_END) {
		/*
		 * Acquiring samples is a critical section
		 * protected against any concurrent abbort trigger.
		 * This is done by raising the flag CSET_BUSY at ACQ_END
		 * and lowered it at the end of DMA_END.
		 */
		spin_lock_irqsave(&cset->lock, flags);
		zfad_block = cset->interleave->priv_d;
		/* Check first if any concurrent trigger stop */
		/* has deleted zio blocks. In such a case */
		/* the flag is not raised and nothing is done */
		if (zfad_block != NULL && (cset->ti->flags & ZIO_TI_ARMED))
			cset->flags |= ZIO_CSET_HW_BUSY;
		spin_unlock_irqrestore(&cset->lock, flags);
		if (cset->flags & ZIO_CSET_HW_BUSY) {
			/* Job deferred to the workqueue: */
			/* Start DMA and ack irq on the carrier */
			queue_work(fa_workqueue, &fa->irq_work);
			/* register the core firing the IRQ in order to */
			/* check right IRQ seq.: ACQ_END followed by DMA_END */
			fa->last_irq_core_src = irq_core_base;
		} else /* current Acquiistion has been stopped */
			fmc_irq_ack(fmc);
	} else { /* unexpected interrupt we have to ack anyway */
		dev_err(fa->msgdev,
			"%s unexpected interrupt 0x%x\n",
			__func__, status);
		fmc_irq_ack(fmc);
	}

	return IRQ_HANDLED;

}

int fa_setup_irqs(struct fa_dev *fa)
{
	struct fmc_device *fmc = fa->fmc;
	int err;

	/* Request IRQ */
	dev_dbg(fa->msgdev, "%s request irq fmc slot: %d\n",
		__func__, fa->fmc->slot_id);
	/* VIC svec setup */
	fa_writel(fa, fa->fa_irq_vic_base,
			&zfad_regs[ZFA_IRQ_VIC_CTRL],
			0x3);
	fa_writel(fa, fa->fa_irq_vic_base,
			&zfad_regs[ZFA_IRQ_VIC_ENABLE_MASK],
			0x3);

	/* trick : vic needs the base address of teh core firing the irq
	 * It cannot provided throught irq_request() call therefore the trick
	 * is to set it by means of the field irq provided by the fmc device
	 */
	fmc->irq = fa->fa_irq_adc_base;
	err = fmc_irq_request(fmc, fa_irq_handler,
			      "fmc-adc-100m14b",
			      0 /*VIC is used */);
	if (err) {
		dev_err(fa->msgdev, "can't request irq %i (error %i)\n",
			fa->fmc->irq, err);
		return err;
	}
	/* workqueue is required to execute DMA transaction */
	INIT_WORK(&fa->irq_work, fa_irq_work);

	/* set IRQ sources to listen */
	fa->irq_src = FA_IRQ_SRC_ACQ;

	if (fa->carrier_op->setup_irqs)
		err = fa->carrier_op->setup_irqs(fa);

	return err;
}

int fa_free_irqs(struct fa_dev *fa)
{
	struct fmc_device *fmc = fa->fmc;

	/*
	 * When we unload the driver the FPGA is still running so it may
	 * rises interrupts. Disable IRQs in order to prevent spurious
	 * interrupt when the driver is not there to handle them.
	 */
	fa_disable_irqs(fa);

	/* Release carrier IRQs (if any) */
	if (fa->carrier_op->free_irqs)
		fa->carrier_op->free_irqs(fa);

	/* Release ADC IRQs */
	fmc->irq = fa->fa_irq_adc_base;
	fmc_irq_free(fmc);

	return 0;
}

int fa_enable_irqs(struct fa_dev *fa)
{
	dev_dbg(fa->msgdev, "%s Enable interrupts fmc slot:%d\n",
		__func__, fa->fmc->slot_id);

	fa_writel(fa, fa->fa_irq_adc_base,
			&zfad_regs[ZFA_IRQ_ADC_ENABLE_MASK],
			FA_IRQ_ADC_ACQ_END);

	if (fa->carrier_op->enable_irqs)
		fa->carrier_op->enable_irqs(fa);
	return 0;
}

int fa_disable_irqs(struct fa_dev *fa)
{
	dev_dbg(fa->msgdev, "%s Disable interrupts fmc slot:%d\n",
		__func__, fa->fmc->slot_id);

	fa_writel(fa, fa->fa_irq_adc_base,
			&zfad_regs[ZFA_IRQ_ADC_DISABLE_MASK],
			FA_IRQ_ADC_ACQ_END);

	if (fa->carrier_op->disable_irqs)
		fa->carrier_op->disable_irqs(fa);
	return 0;
}

int fa_ack_irq(struct fa_dev *fa, int irq_id)
{
	return 0;
}
