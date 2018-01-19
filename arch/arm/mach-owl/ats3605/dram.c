/*
 * Copyright (c) 2015 Actions Semi Co., Ltd.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>

DECLARE_GLOBAL_DATA_PTR;

#define DMM_DCU_FEA	(0xB0230004)

/* get ddr capacity in MB */
unsigned int owl_get_ddrcap(void)
{
	static const unsigned int ddr_cap[] =
    	{32, 64, 128, 256, 512, 1024, 2048, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	unsigned int i;
	i = (readl(DMM_DCU_FEA) >> 4) & 0xf;
	if (i >= sizeof(ddr_cap)) {
		printf("invalid memory size, DDR_FEA: 0x%x\n",  readl(DMM_DCU_FEA));
		i = 1;
	}
	printf("DDR cap %d MB\n", ddr_cap[i]);
	return ddr_cap[i];
}


int dram_init(void)
{
#ifdef CONFIG_SYS_SDRAM_SIZE
	gd->ram_size = CONFIG_SYS_SDRAM_SIZE;
#else
	unsigned long ddrcap;

	ddrcap = owl_get_ddrcap();


	gd->ram_size = ddrcap * 1024 * 1024;
#endif

	return 0;
}

void dram_init_banksize(void)
{
#if defined(CONFIG_NR_DRAM_BANKS) && defined(CONFIG_SYS_SDRAM_BASE)
	if (gd->ram_size >= 0xe0000000ul) {
		gd->bd->bi_dram[0].start = CONFIG_SYS_SDRAM_BASE;
		gd->bd->bi_dram[0].size = 0xe0000000ul;
#if (CONFIG_NR_DRAM_BANKS == 2)
		gd->bd->bi_dram[1].start = 0x1e0000000ul;
		gd->bd->bi_dram[1].size = 0x20000000ul;
#endif
	} else {
		gd->bd->bi_dram[0].start = CONFIG_SYS_SDRAM_BASE;
		gd->bd->bi_dram[0].size = gd->ram_size;
#if (CONFIG_NR_DRAM_BANKS == 2)
		gd->bd->bi_dram[1].start = 0x0;
		gd->bd->bi_dram[1].size = 0x0;
#endif
	}
#endif
}
