/*
 * USB HOST AOTG Controller (Not Support Hub!)
 *
 * Based on aotg host controller driver in linux-kernel
 * by houjingkun and dengtaiping.
 *
 * Reference from U-Boot xhci and musb.
 *
 * Copyright (C) 2017 Actions Semiconductor Co.Ltd
 * Author: lvjinang <lvjinang@actions-semi.com>
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <usb.h>
#include <malloc.h>
#include <watchdog.h>
#include <errno.h>

#include "aotg_hcd.h"


#define AOTG_RW_TIMEOUT	5000

struct aotg_trb *io_trb;
dma_addr_t io_trb_dma;


#define CACHELINE_SIZE		CONFIG_SYS_CACHELINE_SIZE

/**
 * flushes the address passed till the length
 *
 * @param addr	pointer to memory region to be flushed
 * @param len	the length of the cache line to be flushed
 * @return none
 */
static void aotg_flush_cache(uintptr_t addr, u32 len)
{
	BUG_ON((void *)addr == NULL || len == 0);

	flush_dcache_range(addr & ~(CACHELINE_SIZE - 1),
				ALIGN(addr + len, CACHELINE_SIZE));
}

/**
 * invalidates the address passed till the length
 *
 * @param addr	pointer to memory region to be invalidates
 * @param len	the length of the cache line to be invalidated
 * @return none
 */
static void aotg_inval_cache(uintptr_t addr, u32 len)
{
	BUG_ON((void *)addr == NULL || len == 0);

	invalidate_dcache_range(addr & ~(CACHELINE_SIZE - 1),
				ALIGN(addr + len, CACHELINE_SIZE));
}

int aotg_configure_hwep(struct usb_device *dev, struct aotg_ctrl *ctrl,
				unsigned long pipe)
{
	int len;
	int type;
	int epnum;

	len = usb_maxpacket(dev, pipe);
	type = usb_pipetype(pipe);
	epnum = usb_pipeendpoint(pipe);

	if (type == PIPE_CONTROL) {
		/* Configure endpoint number */
		usb_writeb(epnum, ctrl->base + HCEP0CTRL);
		/* Set maxpacket */
		usb_writeb(len, ctrl->base + HCIN0MAXPCK);
	} else if (type == PIPE_BULK) {
		if (usb_pipein(pipe)) {
			/* Clear Error IRQs */
			usb_writew(usb_readw(ctrl->base + HCINxERRIRQ0),
						ctrl->base + HCINxERRIRQ0);
			/* Configure start address of FIFO */
			usb_writel(0x80, ctrl->base + HCIN1STARTADDR);
			/* Set maxpacket */
			usb_writew(len, ctrl->base + HCIN1MAXPCKL);
			/* Confifure Bulk & single FIFO */
			usb_writeb(BULK_TYPE | SINGLE_FIFO, ctrl->base + HCIN1CON);
			/* Configure endpoint number */
			usb_writeb(epnum, ctrl->base + HCIN1CTRL);
			/* Set valid bit: only for In-ep */
			usb_setbitsb(0x1 << 7, ctrl->base + HCIN1CON);
		} else {
			usb_writew(usb_readw(ctrl->base + HCOUTxERRIRQ0),
						ctrl->base + HCOUTxERRIRQ0);
			usb_writel(0x80, ctrl->base + HCOUT1STARTADDR);
			usb_writew(len, ctrl->base + HCOUT1MAXPCKL);
			usb_writeb(BULK_TYPE | SINGLE_FIFO, ctrl->base + HCOUT1CON);
			usb_writeb(epnum, ctrl->base + HCOUT1CTRL);
			/* Set valid bit: only for In-ep */
			usb_setbitsb(0x1 << 7, ctrl->base + HCOUT1CON);
		}
	} else {
		printf("%s wrong pipe type: %ld\n", __func__, usb_pipetype(pipe));
		return -1;
	}

	return 0;
}

/**
 * Set Link Address and Start DMA
 */
static void giveback_first_trb(struct aotg_ctrl *ctrl, unsigned long pipe)
{
	if (usb_pipeout(pipe)) {
		usb_writel((u32)io_trb_dma, ctrl->base + HCOUT1DMALINKADDR);
		usb_writel(DMACTRL_DMACS, ctrl->base + HCOUT1DMACTRL);
	} else {
		usb_writel((u32)io_trb_dma, ctrl->base + HCIN1DMALINKADDR);
		usb_writel(DMACTRL_DMACS, ctrl->base + HCIN1DMACTRL);
	}
}

static int wait_for_completion(struct usb_device *dev, struct aotg_ctrl *ctrl,
				unsigned long pipe)
{
	u16 pending;
	unsigned long ts = get_timer(0);
	int dir_out = usb_pipeout(pipe);

	do {
		if (check_error(ctrl, pipe)) {
			dev->status = USB_ST_STALLED;
			return -2;
		}

		/* Check if Endpoint 1 Transfer Complete */
		if (dir_out)
			pending = usb_readw(ctrl->base + HCOUTxBUFEMPTYIRQ0);
		else
			pending = usb_readw(ctrl->base + HCINxDMAIRQ0);

		/* Arbitrary: endpoint 1 */
		if ((pending & 0x02) == 0)
			continue;

		/* Clear IRQ Pending for Endpoint 1 Transfer Complete and stop DMA*/
		if (dir_out) {
			usb_writew(usb_readw(ctrl->base + HCOUTxBUFEMPTYIRQ0),
						ctrl->base + HCOUTxBUFEMPTYIRQ0);
			usb_writel(DMACTRL_DMACC, ctrl->base + HCOUT1DMACTRL);
		} else {
			usb_writew(usb_readw(ctrl->base + HCINxDMAIRQ0),
						ctrl->base + HCINxDMAIRQ0);
			usb_writel(DMACTRL_DMACC, ctrl->base + HCIN1DMACTRL);
		}

		return 0;

	} while (get_timer(ts) < AOTG_RW_TIMEOUT);

	/* Timeout */
	dev->status = USB_ST_CRC_ERR;
	return -1;
}

static int finish_trb(struct usb_device *dev, struct aotg_ctrl *ctrl,
				int length)
{
	if (io_trb->token & (TRB_IOS | TRB_IOZ)) {
		dev->act_len = io_trb->len - io_trb->remain;
	} else if (io_trb->token & TRB_IOC) {
		dev->act_len = io_trb->len;
	} else {
		dev->act_len = 0;	/* Arbitrary */
		dev->status = -1;
		return -1;
	}

	/* Transfer is complete */
	dev->status = 0;

	/* Babble */
	if (dev->act_len > length) {
		/* FIXME: URB_SHORT_NOT_OK */
		dev->act_len = 0;
	}

	return 0;
}

/**
 * Queues up the BULK Request to AOTG Ring
 *
 * @param dev	pointer to the USB device structure
 * @param pipe	contains the DIR_IN or OUT, devnum
 * @param buffer	buffer to be read/written based on the request
 * @param length	length of the buffer
 * @return returns 0 if successful else -1 on failure
 */
int aotg_bulk_tx(struct usb_device *dev, unsigned long pipe,
				void *buffer, int length)
{
	struct aotg_ctrl *ctrl = aotg_get_ctrl(dev);
	int dir_out = usb_pipeout(pipe);
	u32 token;
	int ret;

	debug("dev=%p, pipe=%lx, buffer=%p, length=%d\n",
		dev, pipe, buffer, length);

	/* Flush the buffer before use */
	aotg_flush_cache((uintptr_t)buffer, length);

	/*
	 * FIXME: Default one trb (URB_ZERO_PACKET)
	 */

	if (dir_out)
		token = TRB_OF | TRB_ITE | TRB_LT;
	else
		token = TRB_CSP | TRB_OF | TRB_ICE | TRB_LT;

	/* Queue TRB */
	io_trb->buf = (u32)(long)buffer;
	io_trb->len = (u32)length;
	io_trb->remain = 0;
	io_trb->token = token;

	/* Flush TRB buffer */
	aotg_flush_cache((uintptr_t)io_trb, sizeof(io_trb));

	giveback_first_trb(ctrl, pipe);

	ret = wait_for_completion(dev, ctrl, pipe);
	if (ret) {
		dev->act_len = 0;	/* Arbitrary */
		return -1;
	}

	ret = finish_trb(dev, ctrl, length);

	aotg_inval_cache((uintptr_t)buffer, length);

	return ret;
}
