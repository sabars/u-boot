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
/*#define CONFIG_CMD_MTDPARTS*/
#define CONFIG_MTD_DEVICES

#undef CONFIG_CMD_BOOTZ
#undef CONFIG_SUPPORT_RAW_INITRD
#undef CONFIG_FAT_WRITE
#undef CONFIG_DOS_PARTITION
#undef CONFIG_BOOTLOADER
#undef CONFIG_SYS_LONGHELP


#endif /* __CONFIG_ATS3605_EVB_H__ */
