/*
 * Copyright (C) 2015 Actions Semi Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_ATS3605_EVB_H__
#define __CONFIG_ATS3605_EVB_H__

#include "ats3605_common.h"

#define CONFIG_IDENT_STRING	 "ATS3605 EVB"

#define CONFIG_EXTRA_ENV_SETTINGS	CONFIG_EXTRA_ENV_SETTINGS_COMMON \
					"devif=nor\0"			 \
					"bootdisk=2\0"

#define CONFIG_BOOTCOMMAND		"run norboot;"


#define CONFIG_BOOTDELAY		0	/* autoboot after 1 seconds */
/*#define CONFIG_PWM_OWL */

#define CONFIG_OWL_NOR
#define CONFIG_CMD_MTDPARTS
#define CONFIG_MTD_DEVICES


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

/*CHARGER*/
#define CONFIG_POWER_CHARGER
#define CONFIG_ATC260X_CHARGER
#define CONFIG_ATC2603C_CHARGER
/*BATTERY*/
#define CONFIG_POWER_BATTERY
#define CONFIG_POWER_BATTERY_ATC260X
#define CONFIG_POWER_BATTERY_ATC2603C

/*check battery, if low, power off*/
#define CONFIG_CHECK_POWER
/*hardkey*/
#define CONFIG_HARD_POWERKEY


/*update check*/
#define CONFIG_CHECK_UPDATE


#endif /* __CONFIG_ATS3605_EVB_H__ */
