/* include/aotg-uboot.h
 *
 * Copyright (c) 2017 Actions Semiconductor Co.Ltd.
 *
 * Actions OTG USB uboot init
 *
 * SPDX-License-Identifier:	GPL-2.0+0
 */

#ifndef __AOTG_UBOOT_H_
#define __AOTG_UBOOT_H_

#include <dm.h>

struct aotg_ctrl {
#ifdef CONFIG_DM_USB
	struct udevice *dev;
#endif
	int id;

	void __iomem *base;
	void __iomem *usbecs;

	u32 portsc;		/* Port Status & Port Change */
	u8 devnum;		/* Device Address */
};

#ifdef CONFIG_USB_AOTG_UDC
int aotg_gadget_init(struct aotg_ctrl *ctrl);
void aotg_gadget_exit(struct aotg_ctrl *ctrl);

void aotg_uboot_handle_interrups(struct aotg_ctrl *ctrl);
#endif

#endif /* __AOTG_UBOOT_H_ */
