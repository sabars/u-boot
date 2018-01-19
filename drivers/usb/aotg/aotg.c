/*
 * USB AOTG driver (Actions OTG controller)
 *
 * Copyright (c) 2017 Actions Semiconductor Co.Ltd.
 * lvjinang <lvjinang@actions-semi.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

/**
 * Depends on CONFIG_DM_USB
 */

/**
 * There are 2 modes for AOTG Host Controller:
 * "legacy mode" and "new mode".
 * Legacy mode: using CPU and FIFO for endpoints.
 * New mode: using CPU for control transfer and
 * DMA chain for others.
 */

#include <common.h>
#include <dm.h>
#include <usb.h>
#include <fdtdec.h>
#include <libfdt.h>
#include <malloc.h>
#include <usb.h>
#include <watchdog.h>
#include <asm/gpio.h>
#include <errno.h>
#include <linux/compat.h>

#include "aotg.h"


/* Declare global data pointer */
DECLARE_GLOBAL_DATA_PTR;


static int aotg_dev_reset(struct aotg_ctrl *ctrl)
{
	int timeout = CONFIG_AOTG_RESET_TIMEOUT;

	if (ctrl->id) {
		usb_clearbitsl(USBRST_USB2H1, (volatile void __iomem *)USBRST);
		udelay(1);
		usb_setbitsl(USBRST_USB2H1, (volatile void __iomem *)USBRST);
	} else {
		usb_clearbitsl(USBRST_USB2H0, (volatile void __iomem *)USBRST);
		udelay(1);
		usb_setbitsl(USBRST_USB2H0, (volatile void __iomem *)USBRST);
	}

	do {
		if ((readb(ctrl->base + USBERESET) & USBERES_USBRESET) == 0)
			break;
		else
			udelay(1);
	} while (--timeout > 0);

	if (!(readb(ctrl->base + USBERESET) & USBERES_USBRESET)) {
		printf("usb reset OK\n");
	} else {
		printf("usb reset failed: 0x%x\n", readb(ctrl->base + USBERESET));
		return -EBUSY;
	}

	return 0;
}

static void aotg_set_phy(void __iomem *base, u8 reg, u8 value)
{
	u8 addrlow,	addrhigh;
	int	time = 1;

	addrlow	= reg & 0x0f;
	addrhigh = (reg >> 4) & 0x0f;

	/* write vstatus: */
	writeb(value, base + VDSTATE);
	mb();

	/* write vcontrol: */
	writeb(addrlow | 0x10, base + VDCTRL);
	udelay(time);	/* the vload period should > 33.3ns */
	writeb(addrlow & 0xf, base + VDCTRL);

	udelay(time);
	mb();

	writeb(addrhigh | 0x10, base + VDCTRL);
	udelay(time);
	writeb(addrhigh & 0x0f, base + VDCTRL);
	udelay(time);
	writeb(addrhigh | 0x10, base + VDCTRL);
	udelay(time);
}

#ifndef CONFIG_AOTG_NEW_MODE
static void aotg_configure_phy_legacy(struct aotg_ctrl *ctrl)
{
	int time = 600;

	/* write vstatus: */
	writeb(0x0a, ctrl->base + VDSTATE);
	mb();
	udelay(time);
	/* write vcontrol: */
	writeb(0x17, ctrl->base + VDCTRL);
	udelay(time); /* the vload period should > 33.3ns */
	writeb(0x07, ctrl->base + VDCTRL);
	udelay(time);
	writeb(0x17, ctrl->base + VDCTRL);
	udelay(time);
	mb();
	writeb(0x1e, ctrl->base + VDCTRL);
	udelay(time);
	writeb(0x0e, ctrl->base + VDCTRL);
	udelay(time);
	writeb(0x1e, ctrl->base + VDCTRL);
	udelay(time);

	if (ctrl->id) {
		aotg_set_phy(ctrl->base, 0xe2, 0x34);
		aotg_set_phy(ctrl->base, 0xe7, 0x0b);
		aotg_set_phy(ctrl->base, 0xe7, 0x0f);
		udelay(1);
		aotg_set_phy(ctrl->base, 0xe2, 0x76);	/* voltage of threshold */
		udelay(1);
		aotg_set_phy(ctrl->base, 0xe4, 0x04);
		aotg_set_phy(ctrl->base, 0x84, 0x1a);
	} else {
		aotg_set_phy(ctrl->base, 0xe2, 0x34);
		aotg_set_phy(ctrl->base, 0xe7, 0x0b);
		aotg_set_phy(ctrl->base, 0xe7, 0x0f);
		udelay(1);
		aotg_set_phy(ctrl->base, 0xe2, 0x86);
		aotg_set_phy(ctrl->base, 0xe4, 0x0b);
		aotg_set_phy(ctrl->base, 0x84, 0x1a);
	}
}
#else
static void aotg_configure_phy(struct aotg_ctrl *ctrl)
{
#if defined(CONFIG_AOTG_700)
	aotg_set_phy(ctrl->base, 0xf4, 0xbb);
	aotg_set_phy(ctrl->base, 0xe1, 0xcf);
	aotg_set_phy(ctrl->base, 0xf4, 0x9b);
	aotg_set_phy(ctrl->base, 0xe6, 0xcc);
	aotg_set_phy(ctrl->base, 0xf4, 0xbb);
	aotg_set_phy(ctrl->base, 0xe2, 0x2);
	aotg_set_phy(ctrl->base, 0xe2, 0x16);
	aotg_set_phy(ctrl->base, 0xf4, 0x9b);
	aotg_set_phy(ctrl->base, 0xe7, 0xa1);
	aotg_set_phy(ctrl->base, 0xf4, 0xbb);
	aotg_set_phy(ctrl->base, 0xe0, 0x31);
	aotg_set_phy(ctrl->base, 0xe0, 0x35);
	aotg_set_phy(ctrl->base, 0xf4, 0x9b);
	aotg_set_phy(ctrl->base, 0xe4, 0xa6);
	aotg_set_phy(ctrl->base, 0xf0, 0xfc);
#elif defined(CONFIG_AOTG_900)
	aotg_set_phy(ctrl->base, 0xe7, 0x1b);
	aotg_set_phy(ctrl->base, 0xe7, 0x1f);
	udelay(10);
	aotg_set_phy(ctrl->base, 0xe0, 0xa3);
	aotg_set_phy(ctrl->base, 0x87, 0x0f);
	aotg_set_phy(ctrl->base, 0xe3, 0x0b);
#endif
}
#endif

static void aotg_phy_init(struct aotg_ctrl *ctrl)
{
#ifndef CONFIG_AOTG_NEW_MODE
	aotg_configure_phy_legacy(ctrl);
#else
	aotg_configure_phy(ctrl);
#endif
}

/**
 * NOTICE: For legacy mode, aotg1 depends on aotg0 phy clock, we
 * should enable phy0 clock always, and disable phy0 when aotg0
 * and aotg1 don't work neither!
 *
 * FIXME: For new mode, we only support 700 for now!
 */
static void aotg_clk_enable(struct aotg_ctrl *ctrl, int enable)
{
#ifdef CONFIG_AOTG_NEW_MODE
	if (ctrl->id) {
		if (enable) {
			usb_setbitsl(USBH1_PHY, (volatile void __iomem *)USBPLL);
			usb_setbitsl(USBH1_PLLEN, (volatile void __iomem *)USBPLL);
		} else {
			usb_clearbitsl(USBH1_PLLEN, (volatile void __iomem *)USBPLL);
			usb_clearbitsl(USBH1_PHY, (volatile void __iomem *)USBPLL);
		}
	} else {
		if (enable) {
			usb_setbitsl(USBH0_PHY, (volatile void __iomem *)USBPLL);
			usb_setbitsl(USBH0_PLLEN, (volatile void __iomem *)USBPLL);
		} else {
			usb_clearbitsl(USBH0_PLLEN, (volatile void __iomem *)USBPLL);
			usb_clearbitsl(USBH0_PHY, (volatile void __iomem *)USBPLL);
		}
	}
#else
	if (ctrl->id) {
		if (enable) {
			usb_setbitsl(USB2_1PHY, (volatile void __iomem *)USBCLK);
			usb_setbitsl(USB2_0PHY, (volatile void __iomem *)USBCLK);
			usb_setbitsl(USB2_1PLLEN, (volatile void __iomem *)USBCLK);
			usb_setbitsl(USB2_1CCE, (volatile void __iomem *)USBCLK);
		} else {
			usb_clearbitsl(USB2_1CCE, (volatile void __iomem *)USBCLK);
			usb_clearbitsl(USB2_1PLLEN, (volatile void __iomem *)USBCLK);
			usb_clearbitsl(USB2_0PHY, (volatile void __iomem *)USBCLK);
			usb_clearbitsl(USB2_1PHY, (volatile void __iomem *)USBCLK);
		}
	} else {
		if (enable) {
			usb_setbitsl(USB2_0PHY, (volatile void __iomem *)USBCLK);
			usb_setbitsl(USB2_0PLLEN, (volatile void __iomem *)USBCLK);
			usb_setbitsl(USB2_0CCE, (volatile void __iomem *)USBCLK);
		} else {
			usb_clearbitsl(USB2_0CCE, (volatile void __iomem *)USBCLK);
			usb_clearbitsl(USB2_0PLLEN, (volatile void __iomem *)USBCLK);
			usb_clearbitsl(USB2_0PHY, (volatile void __iomem *)USBCLK);
		}
	}
#endif
}

/**
 * Power Enable/Disable
 */
static int aotg_power_enable(struct aotg_ctrl *ctrl, int enable)
{
#ifdef CONFIG_AOTG_NEW_MODE
	u32 reg;
	int i = 0;

	if (enable) {
		if (ctrl->id)
			writel(readl(SPS_PG_CTL) | PWR_USB2H1, SPS_PG_CTL);
		else
			writel(readl(SPS_PG_CTL) | PWR_USB2H0, SPS_PG_CTL);
	} else {
		if (ctrl->id)
			writel(readl(SPS_PG_CTL) & ~PWR_USB2H1, SPS_PG_CTL);
		else
			writel(readl(SPS_PG_CTL) & ~PWR_USB2H0, SPS_PG_CTL);
	}

	while (1) {
		if (ctrl->id)
			reg = readl(SPS_PG_ACK) & ACK_USB2H1;
		else
			reg = readl(SPS_PG_ACK) & ACK_USB2H0;

		/*
		 * "enable && power on" or "disable && power off"
		 */
		if ((reg && enable) || (!reg && !enable))
			break;

		/* Default 1ms */
		if (i++ >= 1000) {
			printf("%s Power %s Timeout!\n", __func__,
					enable ? "Enable" : "Disable");
			return -1;
		}
		udelay(1);
	}
#endif

	return 0;
}

/**
 * Don't use interrupt during tranx and OTG state switch, use
 * polling instead.
 * Reset & High-speed mode irqs are neccessary for PORTSC.
 */
static void aotg_configure_irqs(struct aotg_ctrl *ctrl)
{
	writeb(0x0, ctrl->base + USBEIEN);
	writeb(0x0, ctrl->base + OTGIEN);

#ifdef CONFIG_AOTG_NEW_MODE
	writew(0x0, ctrl->base + HCOUTxIEN0);
	writew(0x0, ctrl->base + HCINxIEN0);

	writew(0xffff, ctrl->base + HCINxERRIRQ0);
	writew(0xffff, ctrl->base + HCOUTxERRIRQ0);
	writew(0xffff, ctrl->base + HCINxERRIEN0);
	writew(0xffff, ctrl->base + HCOUTxERRIEN0);
#else
	/* Disable all endpoints' irqs */
	writeb(0x0, ctrl->base + HCOUT07IEN);
	writeb(0x0, ctrl->base + HCIN07IEN);

	/* Enable ep error irq */
	writeb(readb(ctrl->base + HCOUT07ERRIRQ), ctrl->base + HCOUT07ERRIRQ);
	writeb(readb(ctrl->base + HCIN07ERRIRQ), ctrl->base + HCIN07ERRIRQ);
	writeb(0x3f, ctrl->base + HCOUT07ERRIEN);
	writeb(0x3f, ctrl->base + HCIN07ERRIEN);
#endif

	/* Enable reset & high-speed irq */
	writeb(readb(USBIRQ), ctrl->base + USBIRQ);
	writeb(USBIEN_URES | USBIEN_HS, ctrl->base + USBIEN);
}

/**
 * Configure External Control and Status Resgister && Timer Resgisters
 *
 * For USBECS, we care about Soft idpin and Soft Vbus that control the
 * OTG FSM.
 * Bit name:         idpin Enable    idpin    Vbus Enable   Vbus
 * Legacy Mode:         bit 26        bit 27       bit 24       bit 25
 * New Mode (500):    bit 26        bit 27       bit 24       bit 25
 * New Mode (700):    bit 26        bit 27       bit 24       bit 25
 * New Mode (900):    bit 26        bit 27       bit 24       bit 25
 */
static void aotg_configure_ecs(struct aotg_ctrl *ctrl)
{
#ifdef CONFIG_AOTG_NEW_MODE
	/* Set DMA Chain Mode */
	writel(0x1, ctrl->base + HCDMABCKDOOR);

#if defined(CONFIG_AOTG_700)
	/* Set idpin: 0; Set Soft Vbus: high */
	writel(0x37000000 | (0x3 << 4), ctrl->usbecs);
#elif defined(CONFIG_AOTG_500)
	writel(0x37000000 | (0x10 << 13) | (0xb << 4), ctrl->usbecs);
#elif defined(CONFIG_AOTG_900)
	writel(0x17000000 | (0x3 << 4), ctrl->usbecs);
#endif
	udelay(100);

	/***** TA_BCON_COUNT *****/
	writeb(0x0, ctrl->base + TA_BCON_COUNT);	/* 110ms */

	/*set TA_SUSPEND_BDIS timeout never generate */
	usb_writeb(0xff, ctrl->base + TAAIDLBDIS);
	/*set TA_AIDL_BDIS timeout never generate */
	usb_writeb(0xff, ctrl->base + TAWAITBCON);
	/*set TA_WAIT_BCON timeout never generate */
	usb_writeb(0x28, ctrl->base + TBVBUSDISPLS);
	usb_setbitsb(1 << 7, ctrl->base + TAWAITBCON);
	usb_writew(0x1000, ctrl->base + VBUSDBCTIMERL);
#else	/* AOTG Legacy Mode */
	writel(0x0, ctrl->usbecs);
	udelay(1);
	/* set soft vbus and id, vbus threshold 3.43~3.57 */
	writel(0x07000000 | (0x3 << 4), ctrl->usbecs);

	writeb(0xff, ctrl->base + TAAIDLBDIS);
	/* set TA_AIDL_BDIS timeout never generate */
	writeb(0xff, ctrl->base + TAWAITBCON);
	/* set TA_WAIT_BCON timeout never generate */
	writeb(0x28, ctrl->base + TBVBUSDISCHPLS);
	usb_setbitsb(1 << 7, ctrl->base + TAWAITBCON);
	writew(0x1000, ctrl->base + VBUSDBCTIMERL);
#endif
}

/**
 * Wait for OTG FSM entering a_host
 */
static int aotg_wait_for_host(struct aotg_ctrl *ctrl)
{
	unsigned int times;

	/* wait for a_host... */
	usb_setbitsb(OTGCTRL_BUSREQ, ctrl->base + OTGCTRL);

	/* Default 500ms */
	for (times = 100; times != 0; times--) {
		if (readb(ctrl->base + OTGSTATE) == AOTG_STATE_A_HOST)
			break;
		mdelay(5);
	}
	if (times == 0) {
		printf("No Device Plug-in!\n");
		return -1;
	}

	return 0;
}

/*
 * After entering a_host, bus reset signal will be sent.
 */
static int aotg_reset_done(struct aotg_ctrl *ctrl)
{
	unsigned int times;

	/* Default 500ms */
	for (times = 100; times != 0; times--) {
		if (usb_readb(ctrl->base + USBIRQ) & USBIEN_URES) {
			printf("Bus Reset done\n");
			break;
		}
		mdelay(5);
	}
	if (times == 0) {
		printf("Bus Reset Timeout\n");
		/* return -1; */
	}

	/* Clear reset irq pending */
	usb_setbitsb(USBIEN_URES, ctrl->base + USBIRQ);

	return 0;
}

/**
 * Unused
 *
 * Get the start address of consecutive FIFO
 */
int aotg_get_fifo_addr(struct aotg_ctrl *ctrl, unsigned long pipe, int len)
{
	int i;
	int num;
	int units;
	int start;

	start = FIFO_START;
	units = DIV_ROUND_UP(len, FIFO_UNIT);

	for (i = FIFO_START, num = 0; i < FIFO_NUMS && num < units; i++) {
		if (aotg_used_fifo[i] != 0) {
			num = 0;
			start = i + 1;
			continue;
		}
		num++;
	}
	if (num < units)
		return -1;

	return start;
}

#ifdef CONFIG_USB_AOTG_UDC
int aotg_gadget_init(struct aotg_ctrl *ctrl)
{
	return 0;
}

void aotg_gadget_exit(struct aotg_ctrl *ctrl)
{
	return;
}

void aotg_uboot_handle_interrups(struct aotg_ctrl *ctrl)
{
	return;
}
#endif

int aotg_host_init(struct aotg_ctrl *ctrl)
{
	int ret;

	aotg_power_enable(ctrl, 1);
	aotg_clk_enable(ctrl, 1);
	aotg_dev_reset(ctrl);

	aotg_phy_init(ctrl);

	aotg_configure_ecs(ctrl);

	/* FIXME: spec says bit6 is not used by host mode */
	writeb(0x40, ctrl->base + USBCS);

	/* Enable high-speed default: no need to disable HS mode */
	/* usb_setbitsb(1 << 7, ctrl->base + BKDOOR); */

	ret = aotg_wait_for_host(ctrl);
	if (ret)
		goto error;

	aotg_configure_irqs(ctrl);

	ret = aotg_reset_done(ctrl);
	if (ret)
		goto error;

	/* Legacy Mode: Byte Counter doesn't reset: weird! */
	debug("HCIN1BCL: %d\n", usb_readw(ctrl->base + HCIN1BCL));

	return 0;

error:
	/*
	 * Set soft idpin: OTG state may be a_wait_bcon or a_host
	 * now, so there are two cases:
	 * 1. a_wait_bcon -> a_wait_vfall -> a_idle -> b_idle.
	 * 2. a_host will enter a_wait_bcon, then do as above.
	 */
	usb_setbitsl(0x1 << 27, ctrl->usbecs);

	aotg_clk_enable(ctrl, 0);
	aotg_power_enable(ctrl, 0);

	return ret;
}

void aotg_host_exit(struct aotg_ctrl *ctrl)
{
	/* OTG state is a_host */

	/* End the session: -> a_suspend */
	usb_clearbitsb(OTGCTRL_BUSREQ, ctrl->base + OTGCTRL);

	/* Set soft idpin: -> a_wait_vfall -> a_idle -> b_idle */
	usb_setbitsl(0x1 << 27, ctrl->usbecs);

	aotg_clk_enable(ctrl, 0);
	aotg_power_enable(ctrl, 0);
}

/* dev_get_driver_data() */
const struct udevice_id aotg_usb_ids[] = {
#ifdef CONFIG_AOTG_NEW_MODE
	{ .compatible = "actions,owl-usb2.0-0", },
	{ .compatible = "actions,owl-usb2.0-1", },
#else
	{ .compatible = "actions,ats3605-usb2.0-0", },
	{ .compatible = "actions,ats3605-usb2.0-1", },
#endif
	{ }
};
