/*
 * USB AOTG driver (Actions OTG controller)
 *
 * Copyright (c) 2017 Actions Semiconductor Co.Ltd.
 * lvjinang <lvjinang@actions-semi.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef AOTG_H_
#define AOTG_H_

#include <aotg-uboot.h>
#include <asm/types.h>
#include <asm/cache.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <linux/list.h>
#include <linux/compat.h>

#ifdef CONFIG_AOTG_NEW_MODE
#include "aotg_regs_new.h"
#else
#include "aotg_regs.h"
#endif


/* vbus/id related info */
struct aotg_platdata {
	fdt_addr_t base;
	int id;
	unsigned long usbecs;
	struct gpio_desc vbus_gpio;
};


/* AOTG TRB Flags */
/* Interrupt Enable on transfer complete: only for HCOUT / IN */
#define TRB_ITE	(1 << 11)
/* Chain Bit */
#define TRB_CHN	(1 << 10)
/* Continue on short packet */
#define TRB_CSP	(1 << 9)
/* Cycle over flag */
#define TRB_COF	(1 << 8)
/* Interrupt Enable on transfer complete: only for HCIN / OUT */
#define TRB_ICE	(1 << 7)
/* Interrupt Enable on zero packet: only for HCOUT / OUT */
#define TRB_IZE	(1 << 6)
/* Interrupt Enable on short packet: only for HCIN / OUT */
#define TRB_ISE (1 << 5)
/* Last TRB flag */
#define TRB_LT	(1 << 4)

/* Interrupt Request on transfer complete */
#define TRB_IOC	(1 << 3)
/* Interrupt Request on zero packet */
#define TRB_IOZ	(1 << 2)
/* Interrupt Request on short packet */
#define TRB_IOS	(1 << 1)

/* Owner flag: 0: software, 1: hardware */
#define TRB_OF	(1 << 0)


struct aotg_trb {
	u32 buf;
	u32 len;
	u32 remain;
	u32 token;
};


/* FIFO Size: 64B per unit */
#define FIFO_SIZE	(512 * 10 + 64 * 2)
/* 64-byte per-unit seems fine */
#define FIFO_UNIT	64
#define FIFO_NUMS	(FIFO_SIZE / FIFO_UNIT)
/* 0 and 1 are used for ctrl-ep */
#define FIFO_START	2

/* Mark used FIFO */
extern char aotg_used_fifo[FIFO_NUMS];
int aotg_get_fifo_addr(struct aotg_ctrl *ctrl, unsigned long pipe,
				int len);


extern const struct udevice_id aotg_usb_ids[];

int aotg_host_init(struct aotg_ctrl *ctrl);
void aotg_host_exit(struct aotg_ctrl *ctrl);


#ifdef CONFIG_AOTG_NEW_MODE

#if defined(CONFIG_AOTG_700)
/* power gate */
#define SPS_PG_BASE	0xe01b0100
#define SPS_PG_CTL	(SPS_PG_BASE + 0x0000)
#define SPS_PG_ACK	(SPS_PG_BASE + 0x0018)

#define PWR_USB2H0	(0x1 << 11)
#define PWR_USB2H1	(0x1 << 2)
#define ACK_USB2H0	(0x1 << 11)
#define ACK_USB2H1	(0x1 << 2)

/* clock */
#define CMU_DEVCLKEN0	0xe01680a0
#define USBH1_CCE	(0x1 << 27)
#define USBH0_CCE	(0x1 << 26)

#define USBPLL	0xe01680b0
#define USBH1_PLLEN	(0x1 << 13)
#define USBH0_PLLEN	(0x1 << 12)
#define USBH1_PHY	(0x1 << 11)
#define USBH0_PHY	(0x1 << 10)

/* reset */
#define USBRST	0xe01680a8
#define USBRST_USB2H0	(0x1 << 26)
#define USBRST_USB2H1	(0x1 << 27)
#endif

#else	/* AOTG Legacy Mode */
/* clock */
#define USBCLK	0xb0150080
#define USB2_1PLLEN	(0x1 << 13)
#define USB2_0PLLEN	(0x1 << 12)
#define USB2_1PHY	(0x1 << 11)
#define USB2_0PHY	(0x1 << 10)
#define USB2_1CCE	(0x1 << 9)
#define USB2_0CCE	(0x1 << 8)

/* reset */
#define USBRST	0xb01500ac
#define USBRST_USB2H0	(0x1 << 3)
#define USBRST_USB2H1	(0x1 << 4)
#endif


/* aotg controller timeout(unit: us) */
#ifndef CONFIG_AOTG_RESET_TIMEOUT
#define CONFIG_AOTG_RESET_TIMEOUT	500000
#endif


/**
 * Endpoint Transfer Error Types
 *
 * NOTE: spec for Endpoint Error Register:
 * bit[0:1]: Error count
 * bit[2:5]: Error type
 * bit 6: Resend the transaction otherwise "Endpoint Halt"
 * bit 7: Control if we care about "Underrun", default 0 (Ignore it!)
 */
/* No Error */
#define NO_ERR	0x0
/* CRC Error */
#define ERR_CRC	0x1
/* Date Toggle Mismatch */
#define ERR_TOGGLE	0x2
/* Endpoint STALL */
#define ERR_STALL	0x3
/* No Endpoint Handshake/Timeout */
#define ERR_TIMEOUT	0x4
/* PID Error */
#define ERR_PID	0x5
/* Data Overrun/Babble */
#define ERR_OVERRUN	0x6
/* Data Underrun */
#define ERR_UNDERRUN	0x7
/* Split Transaction Error: only for new mode */
#define ERR_SPLIT	0x8

#define ERR_TYPE_MASK	0x3c


/* Endpoint Error Mask */
#ifdef CONFIG_AOTG_NEW_MODE
/* Support 15 in-ep and 15 out-ep besides control ep */
#define EP_ERROR_MASK	0xffff
#else
/* Support 5 in-ep and 5 out-ep besides control ep */
#define EP_ERROR_MASK	0x3f
#endif


/* Toggle for control endpoint */
#define TOGGLE_BIT_CLEAR	0x00
#define TOGGLE_BIT_SET	0x01
#define EP0_TOGGLE_SET	6
#define EP0_TOGGLE_CLEAR	5


/* FIFO Types */
#define SINGLE_FIFO	0x0
#define DOUBLE_FIFO	0x1
#define TRIPLE_FIFO	0x2
#define QURD_FIFO	0x3


/* Endpoint Types */
#define BULK_TYPE	(0x2 << 2)
#define INTR_TYPE	(0x3 << 2)
#define ISOC_TYPE	(0x1 << 2)


/* Max output endpoints including ep0-out */
#define MAX_OUT_ENDPOINTS	6
/* Max input endpoints including ep0-in */
#define MAX_IN_ENDPOINTS	6
#define MAX_ENDPOINTS	(MAX_OUT_ENDPOINTS + MAX_IN_ENDPOINTS)


static inline void usb_writeb(u8 val, volatile void __iomem *reg)
{
	writeb(val, reg);
}

static inline void usb_writew(u16 val, volatile void __iomem *reg)
{
	writew(val, reg);
}

static inline void usb_writel(u32 val, volatile void __iomem *reg)
{
	writel(val, reg);
}

static inline u8 usb_readb(volatile void __iomem *reg)
{
	return readb(reg);
}

static inline u16 usb_readw(volatile void __iomem *reg)
{
	return readw(reg);
}

static inline u32 usb_readl(volatile void __iomem *reg)
{
	return readl(reg);
}

static inline void usb_setbitsb(u8 mask, volatile void __iomem *mem)
{
	usb_writeb(usb_readb(mem) | mask, mem);
}

static inline void usb_setbitsw(u16 mask, volatile void __iomem *mem)
{
	usb_writew(usb_readw(mem) | mask, mem);
}

static inline void usb_setbitsl(ulong mask, volatile void __iomem *mem)
{
	usb_writel(usb_readl(mem) | mask, mem);
}

static inline void usb_clearbitsb(u8 mask, volatile void __iomem *mem)
{
	usb_writeb(usb_readb(mem) & ~mask, mem);
}

static inline void usb_clearbitsw(u16 mask, volatile void __iomem *mem)
{
	usb_writew(usb_readw(mem) & ~mask, mem);
}

static inline void usb_clearbitsl(ulong mask, volatile void __iomem *mem)
{
	usb_writel(usb_readl(mem) & ~mask, mem);
}

#endif	/* AOTG_H_ */
