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

#ifndef AOTG_HCD_H_
#define AOTG_HCD_H_

#include "aotg.h"


#ifdef CONFIG_AOTG_NEW_MODE
extern struct aotg_trb *io_trb;
extern dma_addr_t io_trb_dma;
#endif

#define aotg_pipeendpoint(pipe)	(usb_pipeendpoint(pipe) | \
					(usb_pipeout(pipe) << 4))


/* Unused: Endpoint of AOTG */
struct aotg_endp {
	u32 errirq;	/* Error IRQ Register */
	u32 error;	/* Error Types and Control Register */
	u32 maxpacket;	/* Maximum packet Register */
	u32 cfg;	/* Configuration Register */
	u32 ctrl;	/* Control Register */
	u32 cs;	/* Control and Status Register */
	u32 count;	/* Byte Counter Register*/
	u32 start;	/* Start Address Register */
	u32 data;	/* FIFO Buffer Register */
};

struct aotg_ctrl *aotg_get_ctrl(struct usb_device *udev);
int check_error(struct aotg_ctrl *ctrl, unsigned long pipe);
int aotg_configure_hwep(struct usb_device *dev, struct aotg_ctrl *ctrl,
				unsigned long pipe);
int aotg_bulk_tx(struct usb_device *dev, unsigned long pipe,
				void *buffer, int length);

#endif /* AOTG_HCD_H_ */
