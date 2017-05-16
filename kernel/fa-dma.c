/*
 * Copyright CERN 2017
 * Author: Federico Vaga <federico.vaga@vaga.pv.it>
 */

#ifdef CONFIG_FMC_ADC_SVEC
#include "vmebus.h"
#endif

#include "fmc-adc-100m14b4cha.h"
#include "fa-svec.h"

dma_cap_mask_t dma_mask;


/* Endianess */
#ifndef LITTLE_ENDIAN
#define LITTLE_ENDIAN 0
#endif

#ifndef BIG_ENDIAN
#define BIG_ENDIAN 1
#endif

static int __get_endian(void)
{
	int i = 1;
	char *p = (char *)&i;

	if (p[0] == 1)
		return LITTLE_ENDIAN;
	else
		return BIG_ENDIAN;
}


/**
 * Fix endianess from big to host endianess (32bit)
 */
static void __endianness(unsigned int byte_length, void *buffer)
{
	int i, size;
	uint32_t *ptr;

	/* CPU may be little endian, VME is big endian */
	if (__get_endian() == LITTLE_ENDIAN) {
		ptr = buffer;
		/* swap samples and trig timetag all seen as 32bits words */
		size = byte_length/4;
		for (i = 0; i < size; ++i, ++ptr)
			*ptr = __be32_to_cpu(*ptr);
	}

}


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

static unsigned int zfad_block_n_pages(struct zio_block *block)
{
	unsigned int nr_pages;
	long kaddr = (long)block->data;

	nr_pages = ((kaddr & ~PAGE_MASK) + block->datalen + ~PAGE_MASK);
	nr_pages >>= PAGE_SHIFT;

	return nr_pages;
}


#ifdef CONFIG_FMC_ADC_SVEC

#define VME_NO_ADDR_INCREMENT 1

/* FIXME: move to include again */
#ifndef lower_32_bits
#define lower_32_bits(n) ((u32)(n))
#endif /* lower_32_bits */

static void build_dma_desc(struct vme_dma *desc, unsigned long vme_addr,
			void *addr_dest, ssize_t len)
{
	struct vme_dma_attr *vme;
	struct vme_dma_attr *pci;

	memset(desc, 0, sizeof(struct vme_dma));

	vme = &desc->src;
	pci = &desc->dst;

	desc->dir	= VME_DMA_FROM_DEVICE;
	desc->length    = len;
	desc->novmeinc  = VME_NO_ADDR_INCREMENT;

	desc->ctrl.pci_block_size   = VME_DMA_BSIZE_4096;
	desc->ctrl.pci_backoff_time = VME_DMA_BACKOFF_0;
	desc->ctrl.vme_block_size   = VME_DMA_BSIZE_4096;
	desc->ctrl.vme_backoff_time = VME_DMA_BACKOFF_0;

	vme->data_width = VME_D32;
	vme->am         = VME_A24_USER_DATA_SCT;
	/*vme->am         = VME_A24_USER_MBLT;*/
	vme->addru	= upper_32_bits(vme_addr);
	vme->addrl	= lower_32_bits(vme_addr);

	pci->addru = upper_32_bits((unsigned long)addr_dest);
	pci->addrl = lower_32_bits((unsigned long)addr_dest);
}
#endif


static int zfad_dma_block_to_pages(struct page **pages, unsigned int nr_pages,
				   struct zio_block *block)
{
	int i;
	void *data = (void *) block->data;

	if (is_vmalloc_addr(data)) {
		for (i = 0; i < nr_pages; ++i)
			pages[i] = vmalloc_to_page(data + PAGE_SIZE * i);
	} else {
		for (i = 0; i < nr_pages; ++i)
			pages[i] = virt_to_page(data + PAGE_SIZE * i);
	}

	return 0;
}


static void zfad_dma_context_exit(struct zio_cset *cset,
				  struct zfad_block *zfad_block)
{
	struct fa_dev *fa = cset->zdev->priv_d;

	if (!strcmp(fa->fmc->carrier_name, "SVEC")) {
		__endianness(zfad_block->block->datalen,
			     zfad_block->block->data);

		kfree(zfad_block->dma_ctx);
	}
}


/**
 * It initialize the DMA context for the given block transfer
 */
static int zfad_dma_context_init(struct zio_cset *cset,
				 struct zfad_block *zfad_block)
{
	struct fa_dev *fa = cset->zdev->priv_d;

#ifdef CONFIG_FMC_ADC_SVEC
	if (!strcmp(fa->fmc->carrier_name, "SVEC")) {
		struct fa_svec_data *svec_data = fa->carrier_data;
		unsigned long vme_addr;
		struct vme_dma *desc;

		desc = kmalloc(sizeof(struct vme_dma), GFP_ATOMIC);
		if (!desc)
			return -ENOMEM;

		if (zfad_block == cset->interleave->priv_d) {
			/*
			 * Only for the first block:
			 * write the data address in the ddr_addr register: this
			 * address has been computed after ACQ_END by looking to the
			 * trigger position see fa-irq.c::irq_acq_end.
			 * Be careful: the SVEC HW version expects an address of 32bits word
			 * therefore mem-offset in byte is translated into 32bit word
			 */
			fa_writel(fa, svec_data->fa_dma_ddr_addr,
				  &fa_svec_regfield[FA_DMA_DDR_ADDR],
				  zfad_block->dev_mem_off/4);
		}

		zfad_block->dma_ctx = desc;
		vme_addr = svec_data->vme_base + svec_data->fa_dma_ddr_data;
		build_dma_desc(desc, vme_addr,
			       zfad_block->block->data,
			       zfad_block->block->datalen);
	}
#endif

	return 0;
}


/**
 * The proper function from the DMA engine does not allow us to set
 * the context, but we need it (e.g. VME bus)
 */
static inline struct dma_async_tx_descriptor *dmaengine_prep_slave_sg_ctx(
		struct dma_chan *chan,
		struct scatterlist *sgl, unsigned int sg_len,
		enum dma_transfer_direction dir,
		unsigned long flags, void *ctx)
{
	if (!(chan->device && chan->device->device_prep_slave_sg))
		return NULL;

	return chan->device->device_prep_slave_sg(chan, sgl, sg_len,
						  DMA_DEV_TO_MEM, 0, ctx);
}


/**
 * zfad_dma_complete
 * @arg: data block instance
 *
 * It handles the data transfer completion of a block
 */
static void zfad_dma_complete(void *arg)
{
	struct zfad_block *zfad_block = arg;
	struct zio_cset *cset = zfad_block->cset;
	struct fa_dev *fa = cset->zdev->priv_d;

	/* Release DMA resources */
	dma_unmap_sg(fa->fmc->hwdev,
		     zfad_block->sgt.sgl,
		     zfad_block->sgt.nents,
		     DMA_DEV_TO_MEM);
	sg_free_table(&zfad_block->sgt);

	/* Clean/fix the context */
	zfad_dma_context_exit(cset, zfad_block);

	/* Complete the full acquisition when the last transfer is over */
	--fa->transfers_left;
	if (!fa->transfers_left) {
		dma_release_channel(zfad_block->tx->chan);
		zfad_dma_done(cset);
	}
}


/**
 * zfad_dma_prep_slave_sg
 * @dchan: DMA channel to use
 * @cset: ZIO channel set that owns the data
 * @zfad_block: data block instance to transfer
 *
 * It prepare the scatterlist for the block transfer and it submits it
 * to the dma engine.
 */
static int zfad_dma_prep_slave_sg(struct dma_chan *dchan,
				  struct zio_cset *cset,
				  struct zfad_block *zfad_block)
{

	struct fa_dev *fa = cset->zdev->priv_d;
	struct dma_async_tx_descriptor *tx;
	struct page **pages;
	unsigned int nr_pages, sg_mapped;
	int err;

	/* prepare the context for the block transfer */
	zfad_dma_context_init(cset, zfad_block);

	/* Convert buffer to pages */
	nr_pages = zfad_block_n_pages(zfad_block->block);
	pages = kcalloc(nr_pages, sizeof(struct page *), GFP_KERNEL);
	if (!pages) {
		err = -ENOMEM;
		goto err_alloc_pages;
	}
	err = zfad_dma_block_to_pages(pages, nr_pages, zfad_block->block);
	if (err)
		goto err_to_pages;

	/* With some version we cannot use the version from the Linux kernel */
	fa->sg_alloc_table_from_pages(&zfad_block->sgt, pages, nr_pages,
				      offset_in_page(zfad_block->block->data),
				      zfad_block->block->datalen, GFP_KERNEL);
	if (unlikely(err))
		goto err_sgt;

	sg_mapped = dma_map_sg(fa->fmc->hwdev,
			       zfad_block->sgt.sgl,
			       zfad_block->sgt.nents,
			       DMA_DEV_TO_MEM);
	if (sg_mapped < 0) {
		err = sg_mapped;
		goto err_map;
	}

	/* Prepare the DMA transmisison */
	tx = dmaengine_prep_slave_sg_ctx(dchan, zfad_block->sgt.sgl, sg_mapped,
					 DMA_DEV_TO_MEM, 0, zfad_block->dma_ctx);
	if (!tx) {
		dev_err(&cset->head.dev,
			"Failed to prepare dmaengine transfer descriptor\n");
		return -EBUSY;
	}
	tx->callback = zfad_dma_complete;
	tx->callback_param = (void *)zfad_block;
	zfad_block->tx = tx;

	/* Submit the DMA transmission to the DMA engine */
	zfad_block->cookie = dmaengine_submit(tx);
	if (zfad_block->cookie < 0) {
		err = zfad_block->cookie;
		goto err_submit;
	}

	/* we do not need the pages anymore */
	kfree(pages);

	return 0;

err_submit:
	dma_unmap_sg(fa->fmc->hwdev,
		     zfad_block->sgt.sgl,
		     zfad_block->sgt.nents,
		     DMA_DEV_TO_MEM);
err_map:
	sg_free_table(&zfad_block->sgt);
err_sgt:
err_to_pages:
	kfree(pages);
err_alloc_pages:
	return err;
}

/**
 * It matches a valid DMA channel
 */
static bool fa_dmaengine_filter(struct dma_chan *dchan, void *arg)
{
	struct zio_cset *cset = arg;
	struct fa_dev *fa = cset->zdev->priv_d;
	struct device *device_ref = NULL;

	if (!strcmp(fa->fmc->carrier_name, "SPEC")) {
		/* The DMA channel and the ADC must be on the same carrier */
		device_ref = fa->fmc->hwdev;
	} else if (!strcmp(fa->fmc->carrier_name, "SVEC")) {
		/* The channel must be on the VME bus */
		device_ref = fa->fmc->hwdev->parent;
	} else {
		dev_warn(&cset->head.dev,
			"Carrier not recognized. Accept the first available DMA channel\n");
		return 1;
	}

	return (dchan->device->dev == device_ref);
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
	struct dma_chan *dchan;
	int err, i;

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
	dchan = dma_request_channel(dma_mask, fa_dmaengine_filter, cset);
	if (!dchan) {
		err = -ENODEV;
		goto err;
	}

	/* Prepare and enqueue all the transfers */
	for (i = 0; i < fa->n_shots; ++i) {
		err = zfad_dma_prep_slave_sg(dchan, cset, &zfad_block[i]);
		if (err)
			goto err_prep;
	}

	/* Effectively start the DMA transfer */
	fa->transfers_left = fa->n_shots;
	dma_async_issue_pending(dchan);

	return 0;

err_prep:
	dmaengine_terminate_all(dchan);
	dma_release_channel(dchan);
err:
	zfad_trigger_enable(cset->ti);
	dev_err(fa->msgdev, "Failed to run a DMA transfer\n");
	return err;
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

	/*
	 * Lower CSET_HW_BUSY
	 */
	spin_lock(&cset->lock);
	cset->flags &= ~ZIO_CSET_HW_BUSY;
	spin_unlock(&cset->lock);

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

	if (fa->enable_auto_start) {
		/* Automatic start next acquisition */
		dev_dbg(fa->msgdev, "Automatic start\n");
		zfad_fsm_command(fa, FA100M14B4C_CMD_START);
	}

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

	/*
	 * Lower CSET_HW_BUSY
	 */
	spin_lock(&cset->lock);
	cset->flags &= ~ZIO_CSET_HW_BUSY;
	spin_unlock(&cset->lock);

	zfad_fsm_command(fa, FA100M14B4C_CMD_STOP);
	fa->n_dma_err++;

	/* FIXME stop pending */
	//dmaengine_terminate_all();

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
	if (res) {
		/* Stop acquisition on error */
		zfad_dma_error(cset);
	}

	/* ack the irq */
	fmc_irq_ack(fa->fmc);
}
