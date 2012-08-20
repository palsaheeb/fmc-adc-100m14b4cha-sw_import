/*
 * Copyright CERN 2012
 * Author: Federico Vaga <federico.vaga@gmail.com>
 *
 * DMA handle
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>

#include <linux/zio.h>
#include <linux/zio-buffer.h>
#include <linux/zio-trigger.h>

#include "spec.h"
#include "fmc-adc.h"

/* Initialize each element of the scatter list */
static void zfad_setup_dma_scatter(struct spec_fa *fa, struct zio_block *block)
{
	struct scatterlist *sg;
	int bytesleft = block->datalen;
	void *bufp = block->data;
	int mapbytes;
	int i;

	dev_dbg(&fa->zdev->head.dev, "Setup dma scatterlist");
	for_each_sg(fa->sgt.sgl, sg, fa->sgt.nents, i) {
		/*
		 * If there are less bytes left than what fits
		 * in the current page (plus page alignment offset)
		 * we just feed in this, else we stuff in as much
		 * as we can.
		 */
		if (bytesleft < (PAGE_SIZE - offset_in_page(bufp)))
			mapbytes = bytesleft;
		else
			mapbytes = PAGE_SIZE - offset_in_page(bufp);
		sg_set_buf(sg, bufp, mapbytes);
		/* Configure next values */
		bufp += mapbytes;
		bytesleft -= mapbytes;
	}

	BUG_ON(bytesleft);
}

/*
 * Map a scatter/gather table for the DMA transfer from the FMC-ADC.
 * The DMA controller can store a single item, but more then one transfer
 * could be necessary
 */
int zfad_map_dma(struct zio_cset *cset)
{
	struct spec_fa *fa = cset->zdev->priv_d;
	struct scatterlist *sg;
	struct zio_block *block = cset->interleave->active_block;
	struct dma_item *items;
	unsigned int i, pages, sglen, dev_data_mem = 0;
	dma_addr_t tmp;
	int err;

	/* Create sglists for the transfers, PAGE_SIZE granularity */
	pages = DIV_ROUND_UP(block->datalen, PAGE_SIZE);
	dev_dbg(&cset->head.dev, "using %d pages for transfer\n", pages);

	/* Create sglists for the transfers */
	err = sg_alloc_table(&fa->sgt, pages, GFP_ATOMIC);
	if (err) {
		dev_err(&cset->head.dev, "cannot allocate sg table\n");
		goto out;
	}

	/* FIXME maybe dma_pool */
	/* Limited to 32-bit (kernel limit) */
	fa->items = dma_alloc_coherent(fa->fmc->hwdev,
				       sizeof(struct dma_item) * fa->sgt.nents,
				       &fa->dma_list_item, GFP_ATOMIC);
	if (!items)
		goto out_mem;

	/* Setup the scatter list for the provided block */
	zfad_setup_dma_scatter(fa, block);
	/* Map DMA buffers */
	sglen = dma_map_sg(fa->fmc->hwdev, fa->sgt.sgl, fa->sgt.nents,
			   DMA_FROM_DEVICE);
	if (!sglen) {
		dev_err(fa->fmc->hwdev, "cannot map dma memory\n");
		goto out_free;
	}
	/* Configure DMA items */
	for_each_sg(fa->sgt.sgl, sg, fa->sgt.nents, i) {
		dev_dbg(&cset->head.dev, "configure DMA item %d(%p)"
			"(addr: 0x%p, len: %d)\n",
			i, sg, sg_dma_address(sg), sg_dma_len(sg));
		/* Prepare DMA item */
		items[i].start_addr = dev_data_mem;
		items[i].dma_addr_l = sg_dma_address(sg) & 0xFFFFFFFF;
		items[i].dma_addr_h = sg_dma_address(sg) >> 32;
		items[i].dma_len = sg_dma_len(sg);
		dev_data_mem += items[i].dma_len;
		if (!sg_is_last(sg)) {/* more transfers */
			/* uint64_t so it works on 32 and 64 bit */
			tmp = fa->dma_list_item;
			tmp += (sizeof(struct dma_item) * ( i+ 1 ));
			items[i].next_addr_l = ((uint64_t)tmp) & 0xFFFFFFFF;
			items[i].next_addr_h = ((uint64_t)tmp) >> 32;
			items[i].attribute = 0x1;	/* more items */
		} else {
			items[i].attribute = 0x0;	/* last item */
		}
		/* The first item is written on the device */
		if (i == 0) {
			zfa_common_conf_set(&cset->head.dev,
					    &zfad_regs[ZFA_DMA_ADDR],
					    items[i].start_addr);
			zfa_common_conf_set(&cset->head.dev,
					    &zfad_regs[ZFA_DMA_ADDR_L],
					    items[i].dma_addr_l);
			zfa_common_conf_set(&cset->head.dev,
					    &zfad_regs[ZFA_DMA_ADDR_H],
					    items[i].dma_addr_h);
			zfa_common_conf_set(&cset->head.dev,
					    &zfad_regs[ZFA_DMA_LEN],
					    items[i].dma_len);
			zfa_common_conf_set(&cset->head.dev,
					    &zfad_regs[ZFA_DMA_NEXT_L],
					    items[i].next_addr_l);
			zfa_common_conf_set(&cset->head.dev,
					    &zfad_regs[ZFA_DMA_NEXT_H],
					    items[i].next_addr_h);
			/* Set that there is a next item */
			zfa_common_conf_set(&cset->head.dev,
					    &zfad_regs[ZFA_DMA_BR_LAST],
					    items[i].attribute);
		}
	}

	return 0;

out_free:
	kfree(fa->items);
out_mem:
	sg_free_table(&fa->sgt);
out:
	return -ENOMEM;
}
/*
 *
 */
void zfad_unmap_dma(struct zio_cset *cset)
{
	struct spec_fa *fa = cset->zdev->priv_d;

	dev_dbg(fa->fmc->hwdev, "unmap DMA\n");
	dma_unmap_sg(fa->fmc->hwdev, fa->sgt.sgl, fa->sgt.nents,
		     DMA_FROM_DEVICE);

	dma_free_coherent(fa->fmc->hwdev,
			  sizeof(struct dma_item) * fa->sgt.nents,
			  fa->items, fa->dma_list_item);
	fa->items = NULL;
	fa->dma_list_item = 0;
	sg_free_table(&fa->sgt);
}
