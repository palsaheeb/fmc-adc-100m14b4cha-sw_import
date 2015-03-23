/*
 * Copyright CERN 2012
 * Author: Federico Vaga <federico.vaga@gmail.com>
 *
 * handle DMA mapping
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>

#include <linux/fmc.h>

#include <linux/zio.h>
#include <linux/zio-dma.h>
#include <linux/zio-buffer.h>
#include <linux/zio-trigger.h>

#include "fmc-adc-100m14b4cha.h"
#include "fa-spec.h"



static int gncore_dma_fill(struct zio_dma_sg *zsg)
{
	struct gncore_dma_item *item = (struct gncore_dma_item *)zsg->page_desc;
	struct scatterlist *sg = zsg->sg;
	struct zio_channel *chan = zsg->zsgt->chan;
	struct fa_dev *fa = chan->cset->zdev->priv_d;
	struct fa_spec_data *spec_data = fa->carrier_data;
	dma_addr_t tmp;

	/* Prepare DMA item */
	item->start_addr = zsg->dev_mem_off;
	item->dma_addr_l = sg_dma_address(sg) & 0xFFFFFFFF;
	item->dma_addr_h = (uint64_t)sg_dma_address(sg) >> 32;
	item->dma_len = sg_dma_len(sg);

	if (sg_is_last(sg) && zsg->block_idx == zsg->zsgt->n_blocks - 1) {
		item->attribute = 0x0;	/* last item */
		pr_debug("This will be the last dma transfer\n");
	} else {
		/* more transfers */
		/* uint64_t so it works on 32 and 64 bit */
		tmp = zsg->zsgt->dma_page_desc_pool;
		tmp += (zsg->zsgt->page_desc_size * (zsg->page_idx + 1));
		item->next_addr_l = ((uint64_t)tmp) & 0xFFFFFFFF;
		item->next_addr_h = ((uint64_t)tmp) >> 32;
		item->attribute = 0x1;	/* more items */
	}

	/* The first item is written on the device */
	if (zsg->page_idx == 0) {
		fa_writel(fa, spec_data->fa_dma_base,
			  &fa_spec_regs[ZFA_DMA_ADDR], item->start_addr);
		fa_writel(fa, spec_data->fa_dma_base,
			  &fa_spec_regs[ZFA_DMA_ADDR_L], item->dma_addr_l);
		fa_writel(fa, spec_data->fa_dma_base,
			  &fa_spec_regs[ZFA_DMA_ADDR_H], item->dma_addr_h);
		fa_writel(fa, spec_data->fa_dma_base,
			  &fa_spec_regs[ZFA_DMA_LEN], item->dma_len);
		fa_writel(fa, spec_data->fa_dma_base,
			  &fa_spec_regs[ZFA_DMA_NEXT_L], item->next_addr_l);
		fa_writel(fa, spec_data->fa_dma_base,
			  &fa_spec_regs[ZFA_DMA_NEXT_H], item->next_addr_h);
		/* Set that there is a next item */
		fa_writel(fa, spec_data->fa_dma_base,
			  &fa_spec_regs[ZFA_DMA_BR_LAST], item->attribute);
	}

	dev_dbg(zsg->zsgt->hwdev, "DMA item %d (block %d)\n"
		"    addr   0x%x\n"
		"    addr_l 0x%x\n"
		"    addr_h 0x%x\n"
		"    length %d\n"
		"    next_l 0x%x\n"
		"    next_h 0x%x\n"
		"    last   0x%x\n",
		zsg->page_idx, zsg->block_idx,
		item->start_addr, item->dma_addr_l, item->dma_addr_h,
		item->dma_len, item->next_addr_l, item->next_addr_h,
		item->attribute);

	return 0;
}

int fa_spec_dma_start(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	struct fa_spec_data *spec_data = fa->carrier_data;
	struct zio_channel *interleave = cset->interleave;
	struct zfad_block *zfad_block = interleave->priv_d;
	struct zio_block *blocks[fa->n_shots];
	int i, err;

	/*
	 *  FIXME very inefficient because arm trigger already prepare
	 * something like zio_block_sg. In the future ZIO can alloc more
	 * than 1 block at time
	 */
	for (i = 0; i < fa->n_shots; ++i)
		blocks[i] = zfad_block[i].block;

	fa->zdma = zio_dma_alloc_sg(interleave, fa->fmc->hwdev, blocks,
				    fa->n_shots, GFP_ATOMIC);
	if (IS_ERR(fa->zdma))
		return PTR_ERR(fa->zdma);

	/* Fix block memory offset
	 * FIXME when official ZIO has multishot and DMA
	 */
	for (i = 0; i < fa->zdma->n_blocks; ++i)
		fa->zdma->sg_blocks[i].dev_mem_off = zfad_block->dev_mem_off;

	err = zio_dma_map_sg(fa->zdma, sizeof(struct gncore_dma_item),
			     gncore_dma_fill);
	if (err)
		goto out_map_sg;

	/* Start DMA transfer */
	fa_writel(fa, spec_data->fa_dma_base,
			&fa_spec_regs[ZFA_DMA_CTL_START], 1);
	return 0;

out_map_sg:
	zio_dma_free_sg(fa->zdma);
	return err;
}

void fa_spec_dma_done(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;

	zio_dma_unmap_sg(fa->zdma);
	zio_dma_free_sg(fa->zdma);
}

void fa_spec_dma_error(struct zio_cset *cset)
{
	struct fa_dev *fa = cset->zdev->priv_d;
	struct fa_spec_data *spec_data = fa->carrier_data;
	uint32_t val;

	fa_spec_dma_done(cset);
	val = fa_readl(fa, spec_data->fa_dma_base,
			&fa_spec_regs[ZFA_DMA_STA]);
	if (val)
		dev_err(&fa->fmc->dev,
			"DMA error (status 0x%x). All acquisition lost\n", val);
}
