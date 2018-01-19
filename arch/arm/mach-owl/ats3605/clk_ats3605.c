/*
 * Copyright (c) 2015 Actions Semi Co., Ltd.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <fdtdec.h>
#include <asm/io.h>
#include <asm/arch/regs.h>
#include <asm/arch/clk.h>

DECLARE_GLOBAL_DATA_PTR;

int owl_clk_init(void)
{
#if 0
	unsigned int core_freq;
	unsigned int dev_freq;
	unsigned int display_freq;

	int node;

	node = fdt_node_offset_by_compatible(gd->fdt_blob, 0,
					     "actions,owl-clk");
	if (node < 0) {
		printf("cannot find 'actions,owl-clk' config\n");
		return -1;
	}

	core_freq = fdtdec_get_int(gd->fdt_blob, node, "core_pll", 600);
	dev_freq = fdtdec_get_int(gd->fdt_blob, node, "dev_pll", 654);
	display_freq = fdtdec_get_int(gd->fdt_blob, node, "display_pll", 510);
	printf("clk: core_pll %uMHz, dev_pll %uMHz, display_pll %uMHz\n",
	       core_freq, dev_freq, display_freq);

	/* dev_clk = hosc */
	clrsetbits_le32(CMU_DEVPLL, 0x1000, 0x0);

	/* core_clk = hosc */
	clrsetbits_le32(CMU_BUSCLK, 0x3, 0x1);

	/*
	    1. CorePLL EN, HOSC EN, CORECLK=50*12MHz=600MHz
	*/
	writel( ((0x1<<8) | (0x1<<9) | core_freq/12), CMU_COREPLL);

	/* dev pll  */
	writel(0x100 | (dev_freq / 6), CMU_DEVPLL);

	/* display pll  */
	writel(0x100 | (display_freq / 6), CMU_DISPLAYPLL);

	udelay(200);

	/* dev_clk = dev_pll */
	clrsetbits_le32(CMU_DEVPLL, 0x1000, 0x1000);

	/* core_clk = core_pll */
	clrsetbits_le32(CMU_BUSCLK, 0x3, 0x2);

	udelay(50);
#endif
	return 0;
}

