/*
 * Copyright (C) 2015 Actions Semi Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_ATS3605_NAND_ZXF86_H__
#define __CONFIG_ATS3605_NAND_ZXF86_H__

#include "ats3605_common.h"

#define CONFIG_IDENT_STRING	 "ATS3605 NAND ZXF86"

#define CONFIG_EXTRA_ENV_SETTINGS	CONFIG_EXTRA_ENV_SETTINGS_COMMON \
					"devif=nand\0"			 \
					"bootdisk=0\0"

#define CONFIG_BOOTCOMMAND		"run nandboot;"


#define CONFIG_BOOTDELAY		1	/* autoboot after 1 seconds */
#define CONFIG_PWM_OWL

/* video support */
#define CONFIG_VIDEO
#define CONFIG_VIDEO_OWL
#define CONFIG_VIDEO_OWL_DE_ATS3605
#define CONFIG_VIDEO_OWL_LCD

#define CONFIG_CFB_CONSOLE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_VIDEO_LOGO

/* splash image */
#define CONFIG_SPLASH_SCREEN
#define CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_VIDEO_BMP_GZIP
#define CONFIG_SYS_VIDEO_LOGO_MAX_SIZE		(10 * 1024 * 1024)
#define CONFIG_SYS_VIDEO_LOGO_NAME              "boot_logo.bmp.gz"
#define CONFIG_SYS_BATTERY_LOW_NAME             "battery_low.bmp.gz"
#define CONFIG_SYS_CHARGER_LOGO_NAME            "charger_logo.bmp.gz"
#define CONFIG_SYS_RECOVERY_LOGO_NAME           "recovery_logo.bmp.gz"
#define CONFIG_SYS_CHARGER_FRAME_NAME           "charger_frame.bmp.gz"



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

#define CONFIG_CHECK_UPDATE

#define  CONFIG_OTA_CHECK_VER

#endif /* __CONFIG_ATS3605_EVB_H__ */
