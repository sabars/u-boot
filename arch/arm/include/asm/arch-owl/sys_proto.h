/*
 * Copyright (c) 2015 Actions Semi Co., Ltd.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __ASM_ARCH_SYS_PROTO_H__
#define __ASM_ARCH_SYS_PROTO_H__

#ifdef CONFIG_BOOTDEV_AUTO
#define BOOTDEV_NAND 0x0
#define BOOTDEV_EMMC 0x22
#define BOOTDEV_SD 0x20
#define BOOTDEV_NOR 0x40
void owl_bootdev_init(void);
int owl_get_bootdev(void);
int owl_bootdev_env(void);
#endif

void kernel_cmdline_add(char * cmd);

#ifdef CONFIG_ATS3605_BOOTPARA
void ats3605_bootpara_init(void);
void ats3605_get_serial(char *buf,  int len);
#endif

#ifdef CONFIG_HARD_POWERKEY
int hardkey_init(void);
/*return -1 is not config, 1 is on, 0 is off*/
int hardkey_check_on(void);
#endif

#ifdef CONFIG_SMC
unsigned int __read_data(int Instruction);
#endif

#ifdef CONFIG_ANDROID_RECOVERY
int check_recovery_mode(void);
void setup_recovery_env(void);
#endif

#ifdef CONFIG_CHECK_KEY
int owl_check_key(void);
#endif

#ifdef  CONFIG_OWL_MMC
int owl_mmc_check_init(int dev_index);
#endif

u64 __invoke_fn_smc(u64 function_id, u64 arg0, u64 arg1,
					 u64 arg2);

#endif	/* __ASM_ARCH_SYS_PROTO_H__ */
