/*
 * Copyright (C) 2015 Actions Semi Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_ATS3605_NAND_ZXF86_H__
#define __CONFIG_ATS3605_NAND_ZXF86_H__

#include "ats3605_common.h"

#define CONFIG_IDENT_STRING	 "ATS3605 NAND GJLX"

#define CONFIG_EXTRA_ENV_SETTINGS	CONFIG_EXTRA_ENV_SETTINGS_COMMON \
					"devif=nand\0"			 \
					"bootdisk=0\0"

#define CONFIG_BOOTCOMMAND		"run nandboot;"


#define CONFIG_BOOTDELAY		0	/* autoboot after 1 seconds */
#define CONFIG_PWM_OWL

#define CONFIG_OWL_NAND


#define CONFIG_CMD_MMC
#define CONFIG_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_OWL_MMC
#define SLOT0			0

/* USB Configs */
#define CONFIG_CMD_USB
#define CONFIG_USB_STORAGE

/* AOTG USB Host Controller */
#define CONFIG_USB_AOTG_HCD

#endif /* __CONFIG_ATS3605_EVB_H__ */
