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

/**
 * For simplicity, we don't support 2 AOTG Host Controllers
 * at the same time!If you choose one, you have to give
 * up the other.C'est la vie!
 */

#include <common.h>
#include <dm.h>
#include <usb.h>
#include <malloc.h>
#include <watchdog.h>
#include <errno.h>
#include <fdtdec.h>
#include <libfdt.h>
#include <asm/gpio.h>
#include <asm/cache.h>
#include <asm/unaligned.h>
#include <asm/byteorder.h>
#include <asm/dma-mapping.h>
#include <linux/compat.h>

#include "aotg_hcd.h"


/* Declare global data pointer */
DECLARE_GLOBAL_DATA_PTR;


/* class requests from the USB 2.0 hub spec, table 11-15 */
/* GetBusState and SetHubDescriptor are optional, omitted */
#define ClearHubFeature		(0x2000 | USB_REQ_CLEAR_FEATURE)
#define ClearPortFeature	(0x2300 | USB_REQ_CLEAR_FEATURE)
#define GetHubDescriptor	(0xa000 | USB_REQ_GET_DESCRIPTOR)
#define GetHubStatus		(0xa000 | USB_REQ_GET_STATUS)
#define GetPortStatus		(0xa300 | USB_REQ_GET_STATUS)
#define SetHubFeature		(0x2000 | USB_REQ_SET_FEATURE)
#define SetPortFeature		(0x2300 | USB_REQ_SET_FEATURE)


/* ep0 timeout(unit: us) */
#ifndef CONFIG_AOTG_EP0_TIMEOUT
#define CONFIG_AOTG_EP0_TIMEOUT	1000000	/* Is 1000ms enough? */
#endif

/* normal ep timeout(unit: us) */
#ifndef CONFIG_AOTG_TX_TIMEOUT
#define CONFIG_AOTG_TX_TIMEOUT	1000000	/* Is 1000ms enough? */
#endif


/**
 * We multiplex the "hwep" for all usb endpoints
 * except control endpoints.
 *
 * That means ep0-in, ep0-out, ep1-in and ep1-out
 * are enough for AOTG Host Controller.
 *
 * It is a quite simple scheme and it works:
 * There is only one endpoint could be working at the
 * same time in U-Boot.
 */
/* struct aotg_endp hwep; */


/**
 * hub desc: borrow from Linux kernel aotg hcd driver
 *               bDescLength: 8-byte, different from Linux kernel.
 *               wHubCharacteristics: 0x01(Individual port power switching).
 *
 * device desc: idVendor: 0x0000; idProduct: 0x000; iManufacturer: 1;
 *                   iProduct: 2; iSerialNumber: 0(no Serial Number).
 */
static struct descriptor {
	struct usb_hub_descriptor hub;
	struct usb_device_descriptor device;
	struct usb_config_descriptor config;
	struct usb_interface_descriptor interface;
	struct usb_endpoint_descriptor endpoint;
} __packed descriptor = {
	{
		0x8,		/* bDescLength */
		0x29,		/* bDescriptorType: hub descriptor */
		1,		/* bNrPorts -- runtime modified */
		cpu_to_le16(0x1), /* wHubCharacteristics */
		0,		/* bPwrOn2PwrGood */
		0,		/* bHubCntrCurrent */
		{},		/* Device removable */
		{}		/* at most 7 ports! XXX */
	},
	{
		0x12,		/* bLength */
		1,		/* bDescriptorType: UDESC_DEVICE */
		cpu_to_le16(0x0200), /* bcdUSB: v2.0 */
		9,		/* bDeviceClass: UDCLASS_HUB */
		0,		/* bDeviceSubClass: UDSUBCLASS_HUB */
		1,		/* bDeviceProtocol: UDPROTO_HSHUBSTT */
		64,		/* bMaxPacketSize: 64 bytes */
		0x0000,		/* idVendor */
		0x0000,		/* idProduct */
		cpu_to_le16(0x0100), /* bcdDevice */
		1,		/* iManufacturer */
		2,		/* iProduct */
		0,		/* iSerialNumber */
		1		/* bNumConfigurations: 1 */
	},
	{
		0x9,
		2,		/* bDescriptorType: UDESC_CONFIG */
		cpu_to_le16(0x19),
		1,		/* bNumInterface */
		1,		/* bConfigurationValue */
		0,		/* iConfiguration */
		0x40,		/* bmAttributes: UC_SELF_POWER */
		0		/* bMaxPower */
	},
	{
		0x9,		/* bLength */
		4,		/* bDescriptorType: UDESC_INTERFACE */
		0,		/* bInterfaceNumber */
		0,		/* bAlternateSetting */
		1,		/* bNumEndpoints */
		9,		/* bInterfaceClass: UICLASS_HUB */
		0,		/* bInterfaceSubClass: UISUBCLASS_HUB */
		0,		/* bInterfaceProtocol: UIPROTO_HSHUBSTT */
		0		/* iInterface */
	},
	{
		0x7,		/* bLength */
		5,		/* bDescriptorType: UDESC_ENDPOINT */
		0x81,		/* bEndpointAddress: IN endpoint 1 */
		3,		/* bmAttributes: UE_INTERRUPT */
		8,		/* wMaxPacketSize */
		255		/* bInterval */
	},
};

struct aotg_ctrl *aotg_get_ctrl(struct usb_device *udev)
{
	struct udevice *dev;

	/* Find the USB controller */
	for (dev = udev->dev;
		device_get_uclass_id(dev) != UCLASS_USB;
		dev = dev->parent)
		;
	return dev_get_priv(dev);
}

/* Reset the specific endpoint of AOTG */
static int aotg_reset_endpoint(struct aotg_ctrl *ctrl, uint32_t pipe)
{
	/* Select endpoint 1 */
	if (usb_pipeout(pipe))
		usb_writeb(ENDPRST_IO | 0x1, ctrl->base + ENDPRST);
	else
		usb_writeb(0x1, ctrl->base + ENDPRST);

	/* Toggle Reset */
	usb_setbitsb(ENDPRST_TOGRST, ctrl->base + ENDPRST);
	udelay(1);

	/* FIFO Reset */
	usb_setbitsb(ENDPRST_FIFORST, ctrl->base + ENDPRST);
	udelay(1);

	return 0;
}

/* Reset all endpoints */
static int aotg_endpoints_init(struct aotg_ctrl *ctrl)
{
	/* Select all HCIN endpoints */
	usb_clearbitsb(ENDPRST_EPNUM_MASK, ctrl->base + ENDPRST);

	/* Toggle Reset */
	usb_setbitsb(ENDPRST_TOGRST, ctrl->base + ENDPRST);
	udelay(1);

	/* FIFO Reset */
	usb_setbitsb(ENDPRST_FIFORST, ctrl->base + ENDPRST);
	udelay(1);

	/* Select all HCOUT endpoints */
	usb_clearbitsb(ENDPRST_EPNUM_MASK, ctrl->base + ENDPRST);
	usb_setbitsb(ENDPRST_IO, ctrl->base + ENDPRST);

	/* Toggle Reset */
	usb_setbitsb(ENDPRST_TOGRST, ctrl->base + ENDPRST);
	udelay(1);

	/* FIFO Reset */
	usb_setbitsb(ENDPRST_FIFORST, ctrl->base + ENDPRST);
	udelay(1);


	/*
	 * Initialize the address and endpoint, they may been changed
	 * at run time. Don't worry, it'll been taken care before transfer.
	 */

	/* Set default peripheral's device address: 0x0 */
	usb_writeb(0x0, ctrl->base + FNADDR);
	ctrl->devnum = 0x0;

	/*
	 * AOTG ep0 can communicate with any control endpoint in the
	 * peripheral device.
	 * Set default peripheral's endpoint address: 0x0
	 */
	usb_writeb(0x0, ctrl->base + HCEP0CTRL);

	return 0;
}

/**
 * Controller: dev->parent id: UCLASS_ROOT
 * Root hub: dev->parent id: UCLASS_USB
 * Normal device: dev->parent id: UCLASS_USB_HUB
 *
 * WARNNING: The fact is kind of awkward: During scanning
 * device, udev->dev = parent: root_hub->dev is controller,
 * and normal_device->dev is root hub (no hub support).
 *
 * How brutal life is!
 */
static inline bool is_root_hub(struct usb_device *udev)
{
#if 1
	/* The easiest way: only root_hub's portnr is zero! */
	if (udev->portnr == 0)
		return true;

	return false;
#else
	enum uclass_id id;
	struct udevice *dev;

	dev = udev->dev;
	id = device_get_uclass_id(dev);

	printf("id: %d\n", id);

	if ((id == UCLASS_USB_HUB || id == UCLASS_USB) &&
		device_get_uclass_id(dev->parent) != UCLASS_USB_HUB)
		return true;

	return false;
#endif
}

static inline bool no_error(struct aotg_ctrl *ctrl)
{
#ifdef CONFIG_AOTG_NEW_MODE
	if (((usb_readw(ctrl->base + HCINxERRIRQ0) & EP_ERROR_MASK) == 0) &&
		((usb_readb(ctrl->base + HCOUTxERRIRQ0) & EP_ERROR_MASK) == 0))
		return true;
#else
	if (((usb_readb(ctrl->base + HCIN07ERRIRQ) & EP_ERROR_MASK) == 0) &&
		((usb_readb(ctrl->base + HCOUT07ERRIRQ) & EP_ERROR_MASK) == 0))
		return true;
#endif

	return false;
}

int check_error(struct aotg_ctrl *ctrl, unsigned long pipe)
{
	uint8_t type;
	int is_in = usb_pipein(pipe);

	if (no_error(ctrl))
		return 0;

	/* There should be only one irq pending in the meanwhile */
#ifndef CONFIG_AOTG_NEW_MODE
	if (is_in) {
		usb_writeb(usb_readb(ctrl->base + HCIN07ERRIRQ),
					ctrl->base + HCIN07ERRIRQ);
		type = usb_readb(ctrl->base + HCIN1ERR) & ERR_TYPE_MASK;
	} else {
		usb_writeb(usb_readb(ctrl->base + HCOUT07ERRIRQ),
					ctrl->base + HCOUT07ERRIRQ);
		type = usb_readb(ctrl->base + HCOUT1ERR) & ERR_TYPE_MASK;
	}
#else
	if (is_in) {
		usb_writew(usb_readw(ctrl->base + HCINxERRIRQ0),
					ctrl->base + HCINxERRIRQ0);
		type = usb_readb(ctrl->base + HCIN1ERR) & ERR_TYPE_MASK;
	} else {
		usb_writew(usb_readw(ctrl->base + HCOUTxERRIRQ0),
					ctrl->base + HCOUTxERRIRQ0);
		type = usb_readb(ctrl->base + HCOUT1ERR) & ERR_TYPE_MASK;
	}
#endif

	/* FIXME: Ignore control endpoint error for now */
	if (usb_pipecontrol(pipe))
		return 0;

	switch (type) {
	/* Timeout, CRC and PID Error... */
	default:
		/* Retry */
		if (is_in)
			usb_setbitsb((0x1 << 5), ctrl->base + HCIN1ERR);
		else
			usb_setbitsb((0x1 << 5), ctrl->base + HCOUT1ERR);
		return 0;

	/* NOTE: There may be ep0 error during bulk transfer */
	case NO_ERR:
		return 0;

	/* STALL */
	case ERR_STALL:
		aotg_reset_endpoint(ctrl, pipe);
		return -1;
	}

	return 0;
}

static inline void ep0_toggle(struct aotg_ctrl *ctrl, uint8_t toggle)
{
	if (toggle == TOGGLE_BIT_SET)
		usb_setbitsb(0x1 << EP0_TOGGLE_SET, ctrl->base + HCEP0CS);
	else
		usb_setbitsb(0x1 << EP0_TOGGLE_CLEAR, ctrl->base + HCEP0CS);
}

static void ep0_write_fifo(struct aotg_ctrl *ctrl, u8 len, void *data)
{
	u8 i;
	u8 *buf = (u8 *)data;

	for (i = 0; i < len; i++)
		writeb(*(buf + i), ctrl->base + EP0INDATA_W0 + i);
}

static void ep0_read_fifo(struct aotg_ctrl *ctrl, u8 len, void *data)
{
	u8 i;
	u8 *buf = (u8 *)data;

	for (i = 0; i < len; i++)
		*(buf + i) = readb(ctrl->base + EP0OUTDATA_W0 + i);
}

/*
 * waits until ep0 is ready.
 *
 * @return returns 0 when ep0 is ready, -1 for timeout, -2 for stall.
 */
static int wait_until_ep0_ready(struct usb_device *dev,
				struct aotg_ctrl *ctrl, bool is_in)
{
	int timeout = CONFIG_AOTG_EP0_TIMEOUT;
	uint8_t busy_bit;
	unsigned long pipe;

	if (is_in) {
		busy_bit = EP0CS_HCINBSY;
		/* we do not care about the endpoint */
		pipe = usb_rcvctrlpipe(dev, 0);
	} else {
		busy_bit = EP0CS_HCOUTBSY;
		pipe = usb_sndctrlpipe(dev, 0);
	}

	do {
		if (check_error(ctrl, pipe)) {
			dev->status = USB_ST_STALLED;
			return -2;
		}

		/* Check if ep0 busy */
		if (!(usb_readb(ctrl->base + HCEP0CS) & busy_bit))
			return 0;
		else
			udelay(1);

	} while (--timeout > 0);

	if (timeout <= 0) {
		dev->status = USB_ST_CRC_ERR;
		return -1;
	}

	return 0;
}

/*
 * This function performs the setup phase of the control transfer
 */
static int ctrlreq_setup_phase(struct usb_device *dev,
				struct aotg_ctrl *ctrl, struct devrequest *setup)
{
	int result;

	/* Initialize SETUP token: it will clear ep0 busy and toggle too */
	usb_setbitsb(EP0CS_HCSET, ctrl->base + EP0CS);

	/* write the control request to ep0 fifo */
	ep0_write_fifo(ctrl, sizeof(struct devrequest), (void *)setup);

	/* enable transfer of setup packet */
	usb_writeb(sizeof(struct devrequest), ctrl->base + HCOUT0BC);

	/* wait until the setup packet is transmitted */
	result = wait_until_ep0_ready(dev, ctrl, false);

	dev->act_len = 0;

	return result;
}

/*
 * This function handles the control transfer in data phase
 */
static int ctrlreq_in_data_phase(struct usb_device *dev,
				struct aotg_ctrl *ctrl, uint32_t len, void *buffer)
{
	int result;
	uint8_t actual;
	uint32_t rxlen = 0;
	uint32_t nextlen = 0;
	uint8_t  maxpktsize = (1 << dev->maxpacketsize) * 8;
	uint8_t *rxbuff = (uint8_t *)buffer;

	while (rxlen < len) {
		/* Determine the next read length */
		nextlen = ((len - rxlen) > maxpktsize) ? maxpktsize : (len - rxlen);

		/* send IN token */
		usb_writeb(0x0, ctrl->base + HCIN0BC);

		result = wait_until_ep0_ready(dev, ctrl, true);
		if (result < 0)
			return result;

		/* Actual number of bytes received by usb */
		actual = usb_readb(ctrl->base + HCIN0BC);

		ep0_read_fifo(ctrl, actual, &rxbuff[rxlen]);

		/* Short Packet? */
		if (actual != nextlen) {
			dev->act_len += actual;
			break;
		}

		rxlen += actual;
		dev->act_len = rxlen;
	}

	return 0;
}

/*
 * This function handles the control transfer out data phase
 */
static int ctrlreq_out_data_phase(struct usb_device *dev,
				struct aotg_ctrl *ctrl, uint32_t len, void *buffer)
{
	int result;
	uint32_t txlen = 0;
	uint32_t nextlen = 0;
	uint8_t  maxpktsize = (1 << dev->maxpacketsize) * 8;
	uint8_t *txbuff = (uint8_t *)buffer;

	while (txlen < len) {
		/* Initialize OUT token */
		usb_clearbitsb(EP0CS_HCSET, ctrl->base + HCEP0CS);

		/* Determine the next read length */
		nextlen = ((len - txlen) > maxpktsize) ? maxpktsize : (len - txlen);

		/* Load the data to send in FIFO */
		ep0_write_fifo(ctrl, nextlen, &txbuff[txlen]);

		/* enable transfer */
		usb_writeb(nextlen, ctrl->base + HCOUT0BC);

		result = wait_until_ep0_ready(dev, ctrl, false);
		if (result < 0)
			return result;

		txlen += nextlen;
		dev->act_len = txlen;
	}

	return 0;
}

/*
 * This function handles the control transfer out status phase
 */
static int ctrlreq_out_status_phase(struct usb_device *dev,
				struct aotg_ctrl *ctrl)
{
	int result;

	/* Initialize OUT token */
	usb_clearbitsb(EP0CS_HCSET, ctrl->base + HCEP0CS);

	/* status always DATA1, so set toggle bit */
	ep0_toggle(ctrl, TOGGLE_BIT_SET);

	/* enable transfer: send status data */
	usb_writeb(0, ctrl->base + HCOUT0BC);

	/* wait until the setup packet is transmitted */
	result = wait_until_ep0_ready(dev, ctrl, false);

	return result;
}

/*
 * This function handles the control transfer in status phase
 */
static int ctrlreq_in_status_phase(struct usb_device *dev,
				struct aotg_ctrl *ctrl)
{
	int result;

	/* status always DATA1, so set toggle bit */
	ep0_toggle(ctrl, TOGGLE_BIT_SET);

	/* send IN token */
	usb_writeb(0x0, ctrl->base + HCIN0BC);

	/* wait until the setup packet is transmitted */
	result = wait_until_ep0_ready(dev, ctrl, true);

	return result;
}

/**
 * Queues up the Control Transfer Request
 *
 * @param dev	pointer to the USB device structure
 * @param pipe	contains the DIR_IN or OUT, devnum
 * @param buffer	buffer to be read/written based on the request
 * @param length	length of the buffer
 * @param setup	request type
 * @return returns 0 if successful else error code on failure
 */
static int aotg_ctrl_tx(struct usb_device *dev, unsigned long pipe,
				void *buffer, int length, struct devrequest *setup)
{
	struct aotg_ctrl *ctrl = aotg_get_ctrl(dev);

	debug("req=%u (%#x), type=%u (%#x), value=%u (%#x), index=%u\n",
		setup->request, setup->request,
		setup->requesttype, setup->requesttype,
		le16_to_cpu(setup->value), le16_to_cpu(setup->value),
		le16_to_cpu(setup->index));

	/*
	 * For endpoints using data toggle, regardless of whether an endpoint
	 * has the Halt feature set, a ClearFeature(ENDPOINT_HALT) request
	 * always results in the data toggle being reinitialized to DATA0.
	 */
	if (setup->request == USB_REQ_CLEAR_FEATURE &&
		setup->requesttype == USB_RECIP_ENDPOINT &&
		setup->value == USB_FEAT_HALT)
		ep0_toggle(ctrl, TOGGLE_BIT_CLEAR);

	/* Control transfer setup phase */
	if (ctrlreq_setup_phase(dev, ctrl, setup) < 0)
		return 0;

	switch (setup->request) {
	case USB_REQ_GET_DESCRIPTOR:
	case USB_REQ_GET_CONFIGURATION:
	case USB_REQ_GET_INTERFACE:
	case USB_REQ_GET_STATUS:
	case US_BBB_GET_MAX_LUN:
		/* control transfer in-data-phase */
		if (ctrlreq_in_data_phase(dev, ctrl, length, buffer) < 0)
			return 0;
		/* control transfer out-status-phase */
		if (ctrlreq_out_status_phase(dev, ctrl) < 0)
			return 0;
		break;

	case USB_REQ_SET_ADDRESS:
	case USB_REQ_SET_CONFIGURATION:
	case USB_REQ_SET_FEATURE:
	case USB_REQ_SET_INTERFACE:
	case USB_REQ_CLEAR_FEATURE:
	case US_BBB_RESET:
		/* control transfer in status phase */
		if (ctrlreq_in_status_phase(dev, ctrl) < 0)
			return 0;
		break;

	case USB_REQ_SET_DESCRIPTOR:
		/* control transfer out data phase */
		if (ctrlreq_out_data_phase(dev, ctrl, length, buffer) < 0)
			return 0;
		/* control transfer in status phase */
		if (ctrlreq_in_status_phase(dev, ctrl) < 0)
			return 0;
		break;

	default:
		/* unhandled control transfer */
		return -1;
	}

	dev->status = 0;
	/* Short Packet may happen */
	/* dev->act_len = length; */

	/* Set device address to USB_FADDR register */
	if (setup->request == USB_REQ_SET_ADDRESS) {
		usb_writeb(dev->devnum, ctrl->base + FNADDR);
		ctrl->devnum = dev->devnum;
	}

	return length;
}


#ifndef CONFIG_AOTG_NEW_MODE

/**
 * NOTICE: It is kind of weird that we have to write FIFO with 4-byte,
 * otherwise the FIFO data will be wrong!
 */
static void write_fifo(struct aotg_ctrl *ctrl, u32 len, void *data)
{
	u32 i;
	u32 length;
	u8 left;
	u32 word;
	u32 *buf = (u32 *)data;

	length = len >> 2;
	left = len & 0x03;

	/* 4-byte alignment */
	if (((long)buf & 0x03) == 0)
		for (i = 0; i < length; i++, buf++)
			usb_writel(*buf, ctrl->base + FIFO1DATA);
	/* 2-byte alignment */
	else if (((long)buf & 0x01) == 0)
		for (i = 0; i < length; i++, buf++) {
			word = *((u16 *)buf) | (*((u16 *)buf + 1) << 16);
			usb_writel(word, ctrl->base + FIFO1DATA);
		}
	else
		for (i = 0; i < length; i++, buf++) {
			u8 *tmp8 = (u8 *)buf;

			word = *tmp8 |
					(*(tmp8 + 1) << 8) |
					(*(tmp8 + 2) << 16) |
					(*(tmp8 + 3) << 24);
			usb_writeb(word, ctrl->base + FIFO1DATA);
		}

	if (left)
		usb_writel(*buf, ctrl->base + FIFO1DATA);
}

/**
 * NOTICE: It is kind of weird that we have to read FIFO with 4-byte,
 * otherwise the FIFO data will be wrong!
 */
static void read_fifo(struct aotg_ctrl *ctrl, u32 len, void *data)
{
	u32 i;
	u32 length;
	u8 left;
	u32 word;
	u32 *buf = (u32 *)data;

	length = len >> 2;
	left = len & 0x03;

	/* 4-byte alignment */
	if (((long)buf & 0x03) == 0)
		for (i = 0; i < length; i++, buf++)
			*buf = usb_readl(ctrl->base + FIFO1DATA);
	/* 2-byte alignment */
	else if (((long)buf & 0x01) == 0)
		for (i = 0; i < length; i++, buf++) {
			word  = usb_readl(ctrl->base + FIFO1DATA);
			*((u16 *)buf) = (u16)(word & 0xffff);
			*((u16 *)buf + 1) = (u16)((word >> 16) & 0xffff);
		}
	else
		for (i = 0; i < len; i++, buf++) {
			word  = usb_readl(ctrl->base + FIFO1DATA);
			*((u8 *)buf) = (u8)(word & 0xff);
			*((u8 *)buf + 1) = (u8)((word >> 8) & 0xff);
			*((u8 *)buf + 2) = (u8)((word >> 16) & 0xff);
			*((u8 *)buf + 3) = (u8)((word >> 24) & 0xff);
		}

	if (left) {
		word = usb_readl(ctrl->base + FIFO1DATA);
		if (left == 1) {
			*((u8 *)buf) = (u8)(word & 0xff);
		} else if (left == 2) {
			*((u8 *)buf) = (u8)(word & 0xff);
			*((u8 *)buf + 1) = (u8)((word >> 8) & 0xff);
		} else {
			*((u8 *)buf) = (u8)(word & 0xff);
			*((u8 *)buf + 1) = (u8)((word >> 8) & 0xff);
			*((u8 *)buf + 2) = (u8)((word >> 16) & 0xff);
		}
	}
}

/*
 * waits until normal ep is ready.
 *
 * @return returns 0 when ep is ready, -1 for timeout, -2 for stall.
 */
static int wait_until_ep_ready(struct usb_device *dev,
				struct aotg_ctrl *ctrl, unsigned long pipe)
{
	int timeout = CONFIG_AOTG_TX_TIMEOUT;
	u32 offset;

	if (usb_pipein(pipe))
		offset = HCIN1CS;
	else
		offset = HCOUT1CS;

	do {
		if (check_error(ctrl, pipe)) {
			dev->status = USB_ST_STALLED;
			return -2;
		}

		/* Check if the ep is busy */
		if (!(usb_readb(ctrl->base + offset) & EPCS_BUSY))
			return 0;
		else
			udelay(1);

	} while (--timeout > 0);

	if (timeout <= 0) {
		dev->status = USB_ST_CRC_ERR;
		return -1;
	}

	return 0;
}

/**
 * Queues up the BULK Request
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
	int maxpacket = usb_maxpacket(dev, pipe);
	u16 packets;
	u32 txlen = 0;
	u32 nextlen = 0;

	debug("dev=%p, pipe=%lx, buffer=%p, length=%d\n",
		dev, pipe, buffer, length);

	if (dir_out) {
		while (txlen < length) {
			nextlen = ((length - txlen) < maxpacket) ?
						(length - txlen) : maxpacket;

			/* Write the data to the FIFO */
			write_fifo(ctrl, nextlen, (void *)(((u8 *)buffer) + txlen));

			/* Set Byte Counter */
			usb_writew(nextlen, ctrl->base + HCOUT1BCL);
			/* Set Busy */
			usb_setbitsb(EPCS_BUSY, ctrl->base + HCOUT1CS);
			/* Or just reset HCOUT1CS ? */
			/* usb_writeb(0x0, ctrl->base + HCOUT1CS); */

			/* Wait until the Busy bit is cleared */
			if (wait_until_ep_ready(dev, ctrl, pipe)) {
				dev->act_len = txlen;
				return -1;
			}
			txlen += nextlen;
		}
	} else {
		packets = DIV_ROUND_UP(length, maxpacket);

		/* Set the Number of IN Packet */
		usb_writew(packets, ctrl->base + HCIN1_COUNTL);

		/* Set busy: Do not set if busy already (Could it happen?) */
		if (!(usb_readb(ctrl->base + HCIN1CS) & EPCS_BUSY))
			usb_setbitsb(EPCS_BUSY, ctrl->base + HCIN1CS);

		/* Short Packet Control */
		usb_setbitsb((0x1 << 1), ctrl->base + HCINXNAKCTRL);
		/* Start IN Token */
		usb_setbitsb((0x1 << 1), ctrl->base + HCINXSTART);

		while (txlen < length) {
			nextlen = ((length - txlen) < maxpacket) ?
						(length - txlen) : maxpacket;

			/* Wait until the Busy bit is cleared */
			if (wait_until_ep_ready(dev, ctrl, pipe)) {
				dev->act_len = txlen;
				return -1;
			}

			if (usb_readw(ctrl->base + HCIN1BCL) < nextlen)
				printf("%s Short Packet, act_size: %d, nextlen: %d!\n",
						__func__, usb_readw(ctrl->base + HCIN1BCL), nextlen);

			/* Read the data from the FIFO */
			read_fifo(ctrl, nextlen, (void *)(((u8 *)buffer) + txlen));

			/* Set Busy */
			usb_setbitsb(EPCS_BUSY, ctrl->base + HCIN1CS);
			/* Or just reset HCIN1CS ? */
			/* usb_writeb(0x0, ctrl->base + HCIN1CS); */

			txlen += nextlen;
		}

		/* Stop IN Token */
		usb_clearbitsb((0x1 << 1), ctrl->base + HCINXSTART);
	}

	/* Bulk transfer is complete */
	dev->status = 0;
	dev->act_len = length;

	return 0;
}

/**
 * Reset FIFO for Endpoint 1.
 */
static int aotg_reset_ep_fifo(struct aotg_ctrl *ctrl, uint32_t pipe)
{
	/* Select endpoint 1 */
	if (usb_pipeout(pipe))
		usb_writeb(ENDPRST_IO | 0x1, ctrl->base + ENDPRST);
	else
		usb_writeb(0x1, ctrl->base + ENDPRST);

	/* FIFO Reset */
	usb_setbitsb(ENDPRST_FIFORST, ctrl->base + ENDPRST);
	udelay(1);

	return 0;
}

/**
 * NOTE: We use the same FIFO for HCIN1 and HCOUT1 both.
 * It seems good and we don't have to reset FIFO every time
 * before transfer starts.
 */
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
		usb_writeb(len, ctrl->base + HCOUT0MAXPCK);
	} else if (type == PIPE_BULK) {
		if (usb_pipein(pipe)) {
			/* Clear Error IRQs */
			usb_writeb(usb_readb(ctrl->base + HCIN07ERRIRQ),
						ctrl->base + HCIN07ERRIRQ);
			/* Configure start address of FIFO */
			usb_writel(0x80, ctrl->base + HCIN1STARTADDR);
			/* Set maxpacket */
			usb_writew(len, ctrl->base + HCIN1MAXPCKL);
			/* Confifure Bulk & single FIFO */
			usb_writeb(BULK_TYPE | SINGLE_FIFO, ctrl->base + HCIN1CON);
			/* Configure endpoint number */
			usb_writeb(epnum, ctrl->base + HCIN1CTRL);
			/* FIFO Control: No auto-fifo, default ep1 */
			usb_writeb(((0x0 << 5) | (0x0 << 4) | 0x1), ctrl->base + FIFOCTRL);
			/* Set valid bit: only for In-ep */
			usb_setbitsb((0x1 << 7), ctrl->base + HCIN1CON);
		} else {
			usb_writeb(usb_readb(ctrl->base + HCOUT07ERRIRQ),
						ctrl->base + HCOUT07ERRIRQ);
			usb_writel(0x80, ctrl->base + HCOUT1STARTADDR);
			usb_writew(len, ctrl->base + HCOUT1MAXPCKL);
			usb_writeb(BULK_TYPE | SINGLE_FIFO, ctrl->base + HCOUT1CON);
			usb_writeb(epnum, ctrl->base + HCOUT1CTRL);
			usb_writeb(((0x0 << 5) | (0x1 << 4) | 0x1), ctrl->base + FIFOCTRL);
		}
	} else {
		printf("%s wrong pipe type: %ld\n", __func__, usb_pipetype(pipe));
		return -1;
	}

	return aotg_reset_ep_fifo(ctrl, pipe);
}
#endif

/**
 * Port Reset
 * Bit5: Force port reset;
 * Bit6: reset signal period: 55ms
 */
static inline void port_reset(struct aotg_ctrl *ctrl)
{
	usb_writeb((0x1 << 5) | (0x1 << 6), ctrl->base + HCPORTCTRL);
}

/**
 * Submits the Requests to the AOTG Host Controller
 *
 * @param udev pointer to the USB device structure
 * @param pipe contains the DIR_IN or OUT , devnum
 * @param buffer buffer to be read/written based on the request
 * @return returns 0 if successful else -1 on failure
 */
static int aotg_submit_root(struct usb_device *udev, unsigned long pipe,
				void *buffer, struct devrequest *req)
{
	uint8_t tmpbuf[4];
	u16 typeReq;
	void *srcptr = NULL;
	int len, srclen;
	struct aotg_ctrl *ctrl = aotg_get_ctrl(udev);

	srclen = 0;

	debug("req=%u (%#x), type=%u (%#x), value=%u, index=%u\n",
		req->request, req->request,
		req->requesttype, req->requesttype,
		le16_to_cpu(req->value), le16_to_cpu(req->index));

	/* root hub only has one port, so we ignore port number(req->index) */

	typeReq = req->request | req->requesttype << 8;

	switch (typeReq) {
	/* DEVICE REQUESTS */

	/* GetStatus: remote wakeup & self powered */
	case DeviceRequest | USB_REQ_GET_STATUS:
		debug("GetStatus\n");
		/* No USB_STATUS_REMOTEWAKEUP */
		tmpbuf[0] = 1;	/* USB_STATUS_SELFPOWERED */
		tmpbuf[1] = 0;
		srcptr = tmpbuf;
		srclen = 2;
		break;
	/* *ClearFeature: remote wakeup & test mode */
	case DeviceOutRequest | USB_REQ_CLEAR_FEATURE:
		debug("ClearFeature 0x%x\n", le16_to_cpu(req->value));
		/* Do nothing */
		break;
	/* *SetFeature: remote wakeup & test mode */
	case DeviceOutRequest | USB_REQ_SET_FEATURE:
		debug("SetFeature 0x%x\n", le16_to_cpu(req->value));
		/* Do nothing */
		break;
	/* *GetConfiguration */
	case DeviceRequest | USB_REQ_GET_CONFIGURATION:
		debug("GetConfiguration\n");
		tmpbuf[0] = 1;	/* current configuration value: default 1 */
		srcptr = tmpbuf;
		len = 1;
		break;
	/* *SetConfiguration */
	case DeviceOutRequest | USB_REQ_SET_CONFIGURATION:
		debug("SetConfiguration\n");
		/* Do nothing */
		break;

	/* GetDescriptor */
	case DeviceRequest | USB_REQ_GET_DESCRIPTOR:
		switch (le16_to_cpu(req->value) >> 8) {
		/* GetDeviceDescriptor */
		case USB_DT_DEVICE:
			debug("USB_DT_DEVICE request\n");
			srcptr = &descriptor.device;
			srclen = descriptor.device.bLength;
			break;
		/* GetConfigDescriptor */
		case USB_DT_CONFIG:
			debug("USB_DT_CONFIG config\n");
			srcptr = &descriptor.config;
			srclen = descriptor.config.bLength +
					descriptor.interface.bLength +
					descriptor.endpoint.bLength;
			break;
		/* GetStringDescriptor */
		case USB_DT_STRING:
			debug("USB_DT_STRING config\n");
			switch (le16_to_cpu(req->value) & 0xff) {
			case 0: /* Language */
				/* Array of LANGID codes (0x0409 is MSFT-speak for "en-us") */
				srcptr = "\4\3\11\4";
				srclen = 4;
				break;
			case 1: /* Manufacturer: U-Boot */
				srcptr = "\16\3U\0-\0B\0o\0o\0t\0";
				srclen = 14;
				break;
			case 2: /* Product: AOTG Host Controller */
				srcptr = "\52\3A\0O\0T\0G\0 "
					 "\0H\0o\0s\0t\0 "
					 "\0C\0o\0n\0t\0r\0o\0l\0l\0e\0r\0";
				srclen = 42;
				break;
			default:
				printf("unknown value DT_STRING %x\n",
					le16_to_cpu(req->value));
				goto unknown;
			}
			break;
		default:
			printf("unknown value %x\n", le16_to_cpu(req->value));
			goto unknown;
		}
		break;
	/* *GetInterface */
	case DeviceRequest | USB_REQ_GET_INTERFACE:
		debug("GetInterface\n");
		tmpbuf[0] = 0;
		srcptr = tmpbuf;
		len = 1;
		break;
	/* *SetInterface */
	case DeviceOutRequest | USB_REQ_SET_INTERFACE:
		debug("SetInterface\n");
		/* Do nothing */
		break;
	/* SetAddress */
	case USB_REQ_SET_ADDRESS | (USB_RECIP_DEVICE << 8):
		/* Ignore Set Address command for root hub */
		debug("SetAddress %d\n", req->value);
		break;

	/* ENDPOINT REQUESTS */

	/* *GetStatus */
	case EndpointRequest | USB_REQ_GET_STATUS:
		/* USB_ENDPOINT_HALT flag */
		debug("Endpoint GetStatus\n");
		tmpbuf[0] = 0;
		tmpbuf[1] = 0;
		srcptr = tmpbuf;
		len = 2;
		break;
	/* *ClearFeature */
	case EndpointOutRequest | USB_REQ_CLEAR_FEATURE:
		debug("Endpoint ClearFeature\n");
		/* Do nothing */
		break;
	/* *SetFeature */
	case EndpointOutRequest | USB_REQ_SET_FEATURE:
		debug("Endpoint SetFeature\n");
		/* Do nothing */
		break;

	/* CLASS(HUB) REQUESTS  */

	case ClearHubFeature:
		/* local power & over current */
		debug("ClearHubFeature\n");
		break;

	case SetHubFeature:
		/* local power & over current */
		debug("SetHubFeature\n");
		break;
	/* GetHubStatus */
	case USB_REQ_GET_STATUS | ((USB_DIR_IN | USB_RT_HUB) << 8):
		/* No power source, over-current reported per port */
		memset(tmpbuf, 0, 4);
		srcptr = tmpbuf;
		srclen = 4;
		break;
	/* GetHubDescriptor */
	case USB_REQ_GET_DESCRIPTOR | ((USB_DIR_IN | USB_RT_HUB) << 8):
		debug("USB_DT_HUB config\n");
		srcptr = &descriptor.hub;
		srclen = descriptor.hub.bLength;
		break;
	/* GetPortStatus: port status and port change */
	case USB_REQ_GET_STATUS | ((USB_RT_PORT | USB_DIR_IN) << 8):
		debug("GetPortStatus\n");
		/* OTG State Enter A_Host: device attached */
		if (((ctrl->portsc & USB_PORT_STAT_CONNECTION) == 0) &&
			(usb_readb(ctrl->base + OTGSTATE) == AOTG_STATE_A_HOST)) {
			ctrl->portsc |= USB_PORT_STAT_CONNECTION;
			ctrl->portsc |= USB_PORT_STAT_C_CONNECTION << 16;
		}
		/* Port Reset done */
		if (usb_readb(ctrl->base + USBIRQ) & USBIEN_URES) {
			/* Clear reset irq pending */
			usb_setbitsb(USBIEN_URES, ctrl->base + USBIRQ);

			aotg_endpoints_init(ctrl);

			/* Port Status: Port is enabled */
			ctrl->portsc |= USB_PORT_STAT_ENABLE;
			/* Port Status: Reset signaling not asserted */
			ctrl->portsc &= ~USB_PORT_STAT_RESET;
			/* Port Change: reset complete */
			ctrl->portsc |= USB_PORT_STAT_C_RESET << 16;
			if (usb_readb(ctrl->base + USBCS) & USBCS_HFMODE) {
				ctrl->portsc |= USB_PORT_STAT_HIGH_SPEED;
				usb_writeb(USBIRQ_HS, ctrl->base + USBIRQ);
				debug("USB device is high-speed\n");
			} else if (usb_readb(ctrl->base + USBCS) & USBCS_LSMODE) {
				ctrl->portsc |= USB_PORT_STAT_LOW_SPEED;
				debug("USB device is low-speed\n");
			} else {
				debug("USB device is full-speed\n");
			}
		}
		srcptr = &ctrl->portsc;
		srclen = 4;
		break;
	/* SetPortFeature */
	case USB_REQ_SET_FEATURE | ((USB_DIR_OUT | USB_RT_PORT) << 8):
		debug("SetPortFeature 0x%x\n", le16_to_cpu(req->value));
		switch (le16_to_cpu(req->value)) {
		case USB_PORT_FEAT_ENABLE:
			ctrl->portsc |= USB_PORT_STAT_ENABLE;
			break;
		case USB_PORT_FEAT_POWER:
			ctrl->portsc |= USB_PORT_STAT_POWER;
			break;
		case USB_PORT_FEAT_RESET:
			port_reset(ctrl);

			/* Port Status: Port is disabled, default High-speed */
			ctrl->portsc &= ~(USB_PORT_STAT_ENABLE
				| USB_PORT_STAT_LOW_SPEED
				| USB_PORT_STAT_HIGH_SPEED);
			/* Port Status: Reset signaling asserted */
			ctrl->portsc |= USB_PORT_STAT_RESET;

			/* Reset irq has been enabled already */
			break;
		default:
			printf("unknown feature %x\n", le16_to_cpu(req->value));
			goto unknown;
		}
		break;
	/* ClearPortFeature */
	case USB_REQ_CLEAR_FEATURE | ((USB_DIR_OUT | USB_RT_PORT) << 8):
		debug("ClearPortFeature 0x%x\n", le16_to_cpu(req->value));
		switch (le16_to_cpu(req->value)) {
		case USB_PORT_FEAT_ENABLE:
			ctrl->portsc &= ~USB_PORT_STAT_ENABLE;
			break;
		case USB_PORT_FEAT_POWER:
			ctrl->portsc &= ~USB_PORT_STAT_POWER;
			break;
		case USB_PORT_FEAT_C_RESET:
		case USB_PORT_FEAT_C_CONNECTION:
		case USB_PORT_FEAT_C_OVER_CURRENT:
		case USB_PORT_FEAT_C_ENABLE:
			ctrl->portsc &= ~(0x1 << le16_to_cpu(req->value));
			break;
		default:
			printf("unknown feature %x\n", le16_to_cpu(req->value));
			goto unknown;
		}
		break;
	default:
		puts("Unknown request\n");
		goto unknown;
	}

	debug("scrlen = %d\n req->length = %d\n",
		srclen, le16_to_cpu(req->length));

	len = min(srclen, (int)le16_to_cpu(req->length));

	if (srcptr != NULL && len > 0)
		memcpy(buffer, srcptr, len);
	else
		debug("Len is 0\n");

	udev->act_len = len;
	udev->status = 0;

	return 0;

unknown:
	debug("requesttype=%x, request=%x, value=%x, index=%x, length=%x\n",
		req->requesttype, req->request, le16_to_cpu(req->value),
		le16_to_cpu(req->index), le16_to_cpu(req->length));

	udev->act_len = 0;
	udev->status = USB_ST_STALLED;

	return -1;
}

/**
 * Configure the related endpoint for AOTG Host Controller
 *
 * @param udev pointer to the USB device structure
 * @param pipe contains the DIR_IN or OUT , devnum
 * @return returns 0 if successful else -1 on failure
 */
static int aotg_configure_endpoint(struct usb_device *dev, unsigned long pipe)
{
	struct aotg_ctrl *ctrl = aotg_get_ctrl(dev);

	/* Support multi-device */
	if (ctrl->devnum && (dev->devnum != ctrl->devnum)) {
		printf("ctrl->devnum: %d, dev->devnum: %d\n",
			ctrl->devnum, dev->devnum);
		usb_writeb(dev->devnum, ctrl->base + FNADDR);
		ctrl->devnum = dev->devnum;
	}

	return aotg_configure_hwep(dev, ctrl, pipe);
}

static int aotg_submit_control_msg(struct udevice *dev,
				struct usb_device *udev, unsigned long pipe,
				void *buffer, int length, struct devrequest *setup)
{
	debug("%s: dev='%s', udev=%p, udev->dev='%s', portnr=%d\n", __func__,
		dev->name, udev, udev->dev->name, udev->portnr);

	if (usb_pipetype(pipe) != PIPE_CONTROL) {
		debug("non-control pipe (type=%lu)\n", usb_pipetype(pipe));
		return -1;
	}

	if (is_root_hub(udev)) {
		debug("%s: dev '%s' is root hub\n", __func__, udev->dev->name);
		return aotg_submit_root(udev, pipe, buffer, setup);
	}

	if (aotg_configure_endpoint(udev, pipe)) {
		debug("%s configure endpoint failed\n", __func__);
		return -1;
	}

	return aotg_ctrl_tx(udev, pipe, buffer, length, setup);
}

static int aotg_submit_bulk_msg(struct udevice *dev, struct usb_device *udev,
				unsigned long pipe, void *buffer, int length)
{
	debug("%s: dev='%s', udev=%p\n", __func__, dev->name, udev);

	if (usb_pipetype(pipe) != PIPE_BULK) {
		printf("non-bulk pipe (type=%lu)\n", usb_pipetype(pipe));
		return -EINVAL;
	}

	if (aotg_configure_endpoint(udev, pipe)) {
		debug("%s configure endpoint failed\n", __func__);
		return -1;
	}

	return aotg_bulk_tx(udev, pipe, buffer, length);
}

static int aotg_submit_int_msg(struct udevice *dev, struct usb_device *udev,
				unsigned long pipe, void *buffer, int length, int interval)
{
	/*
	 * TODO: Not addressing any interrupt type transfer requests
	 * Add support for it later.
	 */
	return -EINVAL;
}

struct dm_usb_ops aotg_hcd_usb_ops = {
	.control = aotg_submit_control_msg,
	.bulk = aotg_submit_bulk_msg,
	.interrupt = aotg_submit_int_msg,
};

static int aotg_hcd_probe(struct udevice *dev)
{
	struct aotg_platdata *plat = dev_get_platdata(dev);
	struct aotg_ctrl *ctrl = dev_get_priv(dev);
	struct usb_bus_priv *priv = dev_get_uclass_priv(dev);
	int ret;

	printf("%s, id: %d\n", __func__, plat->id);

	/* Support get device desc before set address */
	priv->desc_before_addr = true;

	ctrl->id = plat->id;
	ctrl->base = (void __iomem *)plat->base;
	ctrl->usbecs = (void __iomem *)plat->usbecs;

#ifdef CONFIG_AOTG_NEW_MODE
	io_trb = dma_alloc_coherent(sizeof(struct aotg_trb),
				(unsigned long *)&io_trb_dma);
	if (!io_trb) {
		printf("failed to allocate io_trb\n");
		return -ENOMEM;
	}
	printf("io_trb: %p, io_trb_dma: 0x%x\n", io_trb, io_trb_dma);
#endif

	/* setup the Vbus gpio here */
	if (dm_gpio_is_valid(&plat->vbus_gpio)) {
		printf("%s enable the vbus-gpio\n", __func__);
		dm_gpio_set_value(&plat->vbus_gpio, 1);
	}

	/* board_usb_init */
	ret = aotg_host_init(ctrl);
	if (ret) {
		printf("Failed to initialize board for USB\n");
		return ret;
	}

	/* Initialize Port Status and Change */
	ctrl->portsc = 0;
	ctrl->dev = dev;

	return 0;
}

static int aotg_hcd_remove(struct udevice *dev)
{
	struct aotg_platdata *plat = dev_get_platdata(dev);
	struct aotg_ctrl *ctrl = dev_get_priv(dev);

	printf("%s, id: %d\n", __func__, plat->id);

	ctrl->dev = NULL;

	/* board_usb_cleanup */
	aotg_host_exit(ctrl);

	if (dm_gpio_is_valid(&plat->vbus_gpio)) {
		printf("%s disable the vbus-gpio\n", __func__);
		dm_gpio_set_value(&plat->vbus_gpio, 0);
	}

#ifdef CONFIG_AOTG_NEW_MODE
	if (!io_trb)
		dma_free_coherent(io_trb);
#endif

	return 0;
}


static int aotg_usb_ofdata_to_platdata(struct udevice *dev)
{
	struct aotg_platdata *plat = dev_get_platdata(dev);
	const void *blob = gd->fdt_blob;
	unsigned int tmp;

	/*
	 * Get the base address for aotg controller from the device node
	 */
	plat->base = fdtdec_get_addr(blob, dev->of_offset, "reg");
	if (plat->base == FDT_ADDR_T_NONE) {
		debug("Can't get the aotg register base address\n");
		return -ENXIO;
	}

	plat->id = fdtdec_get_int(blob, dev->of_offset, "id", 0);

	/* Compatible with 64-bit */
	tmp = (unsigned int)fdtdec_get_int(blob, dev->of_offset, "usbecs", 0);
	plat->usbecs = (unsigned long)tmp;

	printf("aotg id: %d, usbecs: 0x%lx\n", plat->id, plat->usbecs);

	/* Vbus gpio */
	gpio_request_by_name(dev, "vbus-gpio", 0, &plat->vbus_gpio, GPIOD_IS_OUT);

	return 0;
}

U_BOOT_DRIVER(aotg_hcd) = {
	.name	= "aotg-hcd",
	.id	= UCLASS_USB,
	.of_match = aotg_usb_ids,
	.ofdata_to_platdata = aotg_usb_ofdata_to_platdata,
	.probe = aotg_hcd_probe,
	.remove = aotg_hcd_remove,
	.ops	= &aotg_hcd_usb_ops,
	.platdata_auto_alloc_size = sizeof(struct aotg_platdata),
	.priv_auto_alloc_size = sizeof(struct aotg_ctrl),
	.flags	= DM_FLAG_ALLOC_PRIV_DMA,
};
