/*
 * Copyright (C) 2015 Actions Semi Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_ATS3605_UPGRADE_H__
#define __CONFIG_ATS3605_UPGRADE_H__
#include "ats3605_common.h"

#define CONFIG_IDENT_STRING	 "ATS3605 upgrade"

#define CONFIG_EXTRA_ENV_SETTINGS	CONFIG_EXTRA_ENV_SETTINGS_COMMON

#define CONFIG_BOOTCOMMAND		"run ramboot;"
#define CONFIG_BOOTDELAY		0	/* autoboot after 1 seconds */
#define CONFIG_PWM_OWL

#endif /* __CONFIG_ATS3605_UPGRADE_H__ */
