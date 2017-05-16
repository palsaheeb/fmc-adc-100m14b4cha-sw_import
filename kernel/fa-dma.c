/*
 * Copyright CERN 2017
 * Author: Federico Vaga <federico.vaga@vaga.pv.it>
 */

#include "fmc-adc-100m14b4cha.h"


/**
 * zfad_disable_trigger
 * @ti: ZIO trigger instance
 *
 * It disables the trigger.
 */
static void zfad_trigger_disable(struct zio_ti *ti)
{
	struct zio_cset *cset = ti->cset;
	struct fa_dev *fa = cset->zdev->priv_d;

	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFAT_CFG_HW_EN], 0);
	fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFAT_CFG_SW_EN], 0);
}


/**
 * zfad_mem_offset
 * @cset: ZIO channel set
 * It gets the memory offset of the first sample. Useful in single shot mode
 */
static uint32_t zfad_mem_offset(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	struct zio_control *ctrl = cset->chan[FA100M14B4C_NCHAN].current_ctrl;
	uint32_t pre_samp, trg_pos, dev_mem_off;

	/* get pre-samples from the current control (interleave chan) */
	pre_samp = ctrl->attr_trigger.std_val[ZIO_ATTR_TRIG_PRE_SAMP];
	/* Get trigger position in DDR */
	trg_pos = fa_readl(fa, fa->fa_adc_csr_base,
			   &zfad_regs[ZFAT_POS]);
	/*
	 * compute mem offset (in bytes): pre-samp is converted to
	 * bytes
	 */
	dev_mem_off = trg_pos - (pre_samp * cset->ssize * FA100M14B4C_NCHAN);
	dev_dbg(fa->msgdev,
		"Trigger @ 0x%08x, pre_samp %i, offset 0x%08x\n",
		trg_pos, pre_samp, dev_mem_off);

	return dev_mem_off;
}


/**
 * zfad_wait_idle
 * @cset: ZIO channel set
 * @try: how many times poll for IDLE
 * @udelay: us between two consecutive delay
 *
 * @return: it returns 0 on success. If the IDLE status never comes, then it returns
 * the status value
 */
static int zfad_wait_idle(struct zio_cset *cset, unsigned int try,
			  unsigned int udelay)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	uint32_t val = 0;

	while (try-- && val != FA100M14B4C_STATE_IDLE) {
		udelay(udelay);
		val = fa_readl(fa, fa->fa_adc_csr_base,
			       &zfad_regs[ZFA_STA_FSM]);
	}

	return val != FA100M14B4C_STATE_IDLE ? val : 0;
}

/**
 * It maps the ZIO blocks with an sg table, then it starts the DMA transfer
 * from the ADC to the host memory.
 *
 * @param cset
 */
static int zfad_dma_start(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	struct zfad_block *zfad_block = cset->interleave->priv_d;
	int err;

	/*
	 * All programmed triggers fire, so the acquisition is ended.
	 * If the state machine is _idle_ we can start the DMA transfer.
	 * If the state machine it is not idle, try again 5 times
	 */
	err = zfad_wait_idle(cset, 5, 0);
	if (unlikely(err)) {
		dev_warn(fa->msgdev,
			 "Can't start DMA on the last acquisition, "
			 "State Machine is not IDLE (status:%d)\n", err);
		return -EBUSY;
	}

	/*
	 * Disable all triggers to prevent fires between
	 * different DMA transfers required for multi-shots
	 */
	zfad_trigger_disable(cset->ti);

	/* Fix dev_mem_addr in single-shot mode */
	if (fa->n_shots == 1)
		zfad_block[0].dev_mem_off = zfad_mem_offset(cset);

	dev_dbg(fa->msgdev, "Start DMA transfer\n");
	err = fa->carrier_op->dma_start(cset);
	if (err)
		return err;

	return 0;
}


/**
 * zfad_dma_update_block_tstamp
 * @cset: ZIO channel set
 * @block: ZIO block
 *
 * It updates a give ZIO block
 * - extract timestamp from data, and copy to control
 * - remove timestamp from data
 */
static void zfad_update_block_tstamp(struct zio_cset *cset,
				     struct zio_block *block)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	struct zio_control *ctrl= zio_get_ctrl(block);
	uint32_t *trig_timetag;

	trig_timetag = (uint32_t *)(block->data + block->datalen
				    - FA_TRIG_TIMETAG_BYTES);
	/* Timetag marker (metadata) used for debugging */
	dev_dbg(fa->msgdev, "trig_timetag metadata of "
		" (expected value: 0x6fc8ad2d): 0x%x\n",
		*trig_timetag);
	ctrl->tstamp.secs = *(++trig_timetag);
	ctrl->tstamp.ticks = *(++trig_timetag);
	ctrl->tstamp.bins = *(++trig_timetag);

	/* resize the datalen, by removing the trigger tstamp */
	block->datalen = block->datalen - FA_TRIG_TIMETAG_BYTES;

	/* Sync the channel current control with the last ctrl block*/
	memcpy(&cset->interleave->current_ctrl->tstamp, &ctrl->tstamp,
	       sizeof(struct zio_timestamp));
}


/**
 * zfad_dma_update_block_tstamp_start
 * @cset: ZIO channel set
 * @block: ZIO block
 *
 * It copies the given timestamp into the acquisition start time stamp
 * attributes
 */
static void zfad_update_block_tstamp_start(struct zio_cset *cset,
					   struct zio_block *block,
					   struct zio_timestamp *ts)
{
	struct zio_control *ctrl = zio_get_ctrl(block);

	/* Acquisition start Timetag */
	ctrl->attr_channel.ext_val[FA100M14B4C_DATTR_ACQ_START_S] = ts->secs;
	ctrl->attr_channel.ext_val[FA100M14B4C_DATTR_ACQ_START_C] = ts->ticks;
	ctrl->attr_channel.ext_val[FA100M14B4C_DATTR_ACQ_START_F] = ts->bins;
}


/**
 * zfad_tstamp_start_get
 * @cset: ZIO channel set
 * @ts: where to write the timestamp
 *
 * It copies the acquisition timestamp from the hardware to the given pointer
 */
static void zfad_tstamp_start_get(struct zio_cset *cset, struct zio_timestamp *ts)
{
	struct fa_dev *fa = cset->zdev->priv_d;

	ts->secs = fa_readl(fa, fa->fa_utc_base,
			    &zfad_regs[ZFA_UTC_ACQ_START_SECONDS]);
	ts->ticks = fa_readl(fa, fa->fa_utc_base,
			     &zfad_regs[ZFA_UTC_ACQ_START_COARSE]);
	ts->bins = fa_readl(fa, fa->fa_utc_base,
			    &zfad_regs[ZFA_UTC_ACQ_START_FINE]);
}


/**
 * zfad_enable_trigger
 * @ti: ZIO trigger instance
 *
 * It enables the trigger.
 * Hardware trigger depends on the enable status
 * of the trigger. Software trigger depends on the previous
 * status taken form zio attributes (index 5 of extended one)
 * If the user is using a software trigger, enable the software
 * trigger.
 */
static void zfad_trigger_enable(struct zio_ti *ti)
{
	struct zio_cset *cset = ti->cset;
	struct fa_dev *fa = cset->zdev->priv_d;

	/*
	 * Hardware trigger depends on the enable status
	 * of the trigger. Software trigger depends on the previous
	 * status taken form zio attributes (index 5 of extended one)
	 * If the user is using a software trigger, enable the software
	 * trigger.
	 */
	if (cset->trig == &zfat_type) {
		fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFAT_CFG_HW_EN],
				    (ti->flags & ZIO_STATUS ? 0 : 1));
		fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFAT_CFG_SW_EN],
				    ti->zattr_set.ext_zattr[6].value);
	} else {
		dev_dbg(fa->msgdev, "Software acquisition over\n");
		fa_writel(fa, fa->fa_adc_csr_base, &zfad_regs[ZFAT_CFG_SW_EN],
			  1);
	}
}


/**
 * It completes a DMA transfer.
 * It tells to the ZIO framework that all blocks are done. Then, it re-enable
 * the trigger for the next acquisition. If the device is configured for
 * continuous acquisition, the function automatically start the next
 * acquisition
 *
 * @param cset
 */
void zfad_dma_done(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	struct zfad_block *zfad_block = cset->interleave->priv_d;
	struct zio_timestamp ts;
	int i;

	fa->carrier_op->dma_done(cset);

	zfad_tstamp_start_get(cset, &ts);
	for (i = 0; i < fa->n_shots; ++i) {
		struct zio_control *ctrl = zio_get_ctrl(zfad_block[i].block);

		zfad_update_block_tstamp(cset, zfad_block[i].block);
		zfad_update_block_tstamp_start(cset, zfad_block[i].block, &ts);

		/* update seq num */
		ctrl->seq_num = i;
		cset->interleave->current_ctrl->seq_num = i;
	}

	/*
	 * All DMA transfers done! Inform the trigger about this, so
	 * it can store blocks into the buffer
	 */
	dev_dbg(fa->msgdev, "%i blocks transfered\n", fa->n_shots);
	zio_trigger_data_done(cset);

	/* we can safely re-enable triggers */
	zfad_trigger_enable(cset->ti);
}


/**
 * It handles the error condition of a DMA transfer.
 * The function turn off the state machine by sending the STOP command
 *
 * @param cset
 */
void zfad_dma_error(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;

	fa->carrier_op->dma_error(cset);

	zfad_fsm_command(fa, FA100M14B4C_CMD_STOP);
	fa->n_dma_err++;

	if (fa->n_fires == 0)
		dev_err(fa->msgdev,
			"DMA error occurs but no block was acquired\n");
}


/*
 * job executed within a work thread
 * Depending of the carrier the job slightly differs:
 * SVEC: dma_start() blocks till the the DMA ends
 *      (fully managed by the vmebus driver)
 * Therefore the DMA outcome can be processed immediately
 * SPEC: dma_start() launch the job an returns immediately.
 * An interrupt DMA_DONE or ERROR is expecting to signal the end
 *       of the DMA transaction
 * (See fa-spec-irq.c::fa-spec_irq_handler)
 */

void fa_irq_work(struct work_struct *work)
{
	struct fa_dev *fa = container_of(work, struct fa_dev, irq_work);
	struct zio_cset *cset = fa->zdev->cset;
	int res;

	/*
	 * This check is not crucial because the HW implements
	 * a solid state machine and acq-end can happens only after
	 * the execution of the n requested shots.
	 */
	fa->n_fires = fa->n_shots - fa_readl(fa, fa->fa_adc_csr_base,
					     &zfad_regs[ZFAT_SHOTS_REM]);

	if (fa->n_fires != fa->n_shots) {
		dev_err(fa->msgdev,
			"Expected %i trigger fires, but %i occurs\n",
			fa->n_shots, fa->n_fires);
	}

	res = zfad_dma_start(cset);
	if (!res) {
		/*
		 * No error.
		 * If there is an IRQ DMA src to notify the ends of the DMA,
		 * leave the workqueue.
		 * dma_done will be proceed on DMA_END reception.
		 * Otherwhise call dma_done in sequence
		 */
		if (fa->irq_src & FA_IRQ_SRC_DMA)
			/*
			 * waiting for END_OF_DMA IRQ
			 * with the CSET_BUSY flag Raised
			 * The flag will be lowered by the irq_handler
			 * handling END_DMA
			 */
			goto end;

		zfad_dma_done(cset);
	}
	/*
	 * Lower CSET_HW_BUSY
	 */
	spin_lock(&cset->lock);
	cset->flags &= ~ZIO_CSET_HW_BUSY;
	spin_unlock(&cset->lock);

end:
	if (res) {
		/* Stop acquisition on error */
		zfad_dma_error(cset);
	} else if (fa->enable_auto_start) {
		/* Automatic start next acquisition */
		dev_dbg(fa->msgdev, "Automatic start\n");
		zfad_fsm_command(fa, FA100M14B4C_CMD_START);
	}

	/* ack the irq */
	fmc_irq_ack(fa->fmc);
}
