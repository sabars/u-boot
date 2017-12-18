/*
 * dss/dsic.h
 *
 * DSI Controller driver.
 *
 * Copyright (C) 2015 Actions Corporation
 * Author: lipeng<lipeng@actions-semi.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#define DEBUGX
#define DSS_SUBSYS_NAME "DSIC"

#define pr_fmt(fmt) "owl_dsi: " fmt

#include <common.h>
#include <libfdt.h>
#include <fdtdec.h>
#include <asm/io.h>
#include <asm/arch/regs.h>
#include <asm/arch/clk.h>
#include <asm/arch/reset.h>
#include <asm/arch/periph.h>
#include <asm/arch/dma.h>
#include <dss.h>

#include "dsic.h"

/*
 * mipi dsi low-power rx status
 * */
enum dsi_transfer_state {
	RX_TRIGGER		= 15,
	RX_ACK_AND_ERRO_PACK	= 16,
	RX_DAT_DONE		= 17,

};

#define LONG_CMD_MODE		0x39
#define DATE_TYPE_NO_PAR	0x05
#define	DATE_TYPE_ONE_PAR	0x15

#define DSIC_ID_S500		0
#define DSIC_ID_S700		1
#define DSIC_ID_S900		2

#define p_round(x) ((int)((x) + 500) / 1000)
/************************************************/
#define HSS 4	/*H sync start			*/
#define HSE 4	/*H sync end			*/
#define HAF 6	/*long packet header and footer	*/
/************************************************/

struct dsic_diffs {
	int id;
};
struct owl_dsi_config {
	uint32_t	lcd_mode;	/* 0: command mode, 1: video mode */
	uint32_t	lane_count;	/* 1: 1, 2: 1~2, 3: 1~3, 4: 1~4 */
	uint32_t	lane_polarity;	/* 0: normal,1: reversed */
	uint32_t	lane_swap;	/* sequence of data lines */
	uint32_t	burst_bllp;	/* 0 ~ 300 */
	uint32_t	video_mode;	/* 0:sync mode,1:de mode,2:burst mode*/
	uint32_t	pclk_rate;

	uint32_t	dsi_phy_t0;
	uint32_t	dsi_phy_t1;
	uint32_t	dsi_phy_t2;
};
struct dsic_init_cmd {
	uint32_t cmd_nums;
	uint32_t *mipi_cmd;
};
struct dsic_data {
	fdt_addr_t			base;	/* register address */

	uint32_t			hsclk_pll;/* dsi pll clk */

	struct dsic_init_cmd		*cmd;
	struct owl_dma_dev		*dma_channel;
	u32				dma_irq;/* DSI Controller DMA IRQ */

	struct owl_display_ctrl		*ctrl;

	struct owl_dsi_config		configs;

	struct dsic_diffs		diffs;
};


#define dsic_writel(dsic, index, val) writel((val), dsic->base + (index))
#define dsic_readl(dsic, index) readl(dsic->base + (index))

#define calc_align(n, align) ((n + align - 1) & (~(align - 1)))

static int dsic_parse_config(void *blob, int node, struct dsic_data *dsic)
{
	int entry;

	debug("%s:\n", __func__);

	/*
	 * parse hw configs ...
	 */
	entry = fdtdec_lookup_phandle(blob, node, "panel_configs");
	if (entry < 0) {
		error("no entry for 'panel_configs'\n");
		return -1;
	}
	debug("entry = 0x%d\n", entry);

	dsic->configs.lcd_mode = fdtdec_get_int(blob,
						entry, "lcd_mode", 0);

	dsic->configs.lane_count = fdtdec_get_int(blob,
						entry, "lane_count", 0);

	dsic->configs.lane_polarity = fdtdec_get_int(blob,
						entry, "lane_polarity", 0);

	dsic->configs.lane_swap = fdtdec_get_int(blob,
						entry, "lane_swap", 0);

	dsic->configs.video_mode = fdtdec_get_int(blob,
						entry, "video_mode", 0);

	dsic->configs.burst_bllp = fdtdec_get_int(blob,
						entry, "burst_bllp", 0);

	dsic->configs.pclk_rate = fdtdec_get_int(blob,
						entry, "pclk_rate", 0);


	if (0 == dsic->configs.pclk_rate) {
		error("pclk_rate is 0!!\n");
		return -1;
	}
	if (0 == dsic->configs.lane_count) {
		error("lane_count is 0!!\n");
		return -1;
	}

	debug("lcd_mode %d\n", dsic->configs.lcd_mode);
	debug("lane_count %d, lane_polarity %d\n",
	      dsic->configs.lane_count, dsic->configs.lane_polarity);
	debug("lane_swap %x\n", dsic->configs.lane_swap);
	debug("video_mode %d\n", dsic->configs.video_mode);
	debug("burst_bllp %d\n", dsic->configs.burst_bllp);
	debug("pclk_rate %d\n", dsic->configs.pclk_rate);

	return 0;
}
static void send_normal_long_packet(struct dsic_data *dsic, uint8_t data_type,
					int data_len, uint8_t *send_data,
					int trans_mode)
{
	uint32_t reg_val, tmp, wait_times;
	int i, ret, len;

	debug("%s, data_type 0x%x, n_data  %d\n",
			__func__, data_type, data_len);

	for (i = 0; i < data_len; i++)
		debug(" 0x%x\n", send_data[i]);

	/* change to command mode */
	tmp = dsic_readl(dsic, DSI_CTRL);
	tmp &= 0xffffefff;
	dsic_writel(dsic, DSI_CTRL, tmp);

	flush_dcache_all();
	dsic->dma_irq = DMA_DRQ_DSI_TX;
	owl_dma_stop(dsic->dma_channel);

	/* Word alignment is required */
	len = calc_align(data_len, 4);
	debug("alignment len %d\n", len);

	ret = owl_dma_config(dsic->dma_channel,
			dsic->dma_irq,
			send_data,
			dsic->base + DSI_FIFO_ODAT,
			len, 0);
	if (ret != 0)
		printf("owl dma config error %d\n", ret);

	dsic_writel(dsic, DSI_PACK_HEADER, len);
	tmp = data_type << 8;
	tmp |= trans_mode << 14;
	tmp |= 0x01 << 18; /* long packet */
	dsic_writel(dsic, DSI_PACK_CFG, tmp);
	mdelay(10);

	/* start dma transmission*/
	owl_dma_start(dsic->dma_channel);

	/* start dsi packet transmission */
	tmp = dsic_readl(dsic, DSI_PACK_CFG);
	tmp |= 0x1;
	dsic_writel(dsic, DSI_PACK_CFG, tmp);

	/* waiting for transmitting Complete */
	wait_times = 10;
	do {
		reg_val = dsic_readl(dsic, DSI_TR_STA);
		if (((reg_val >> 19) & 0x1) == 1)
			break;
		mdelay(1);
	} while (--wait_times);
	if (wait_times <= 0)
		printf("%s, long cmd transmission timout!\n", __func__);

	if (owl_dma_wait_finished(dsic->dma_channel, 1000))
		printf("%s, wait dma finished timeout!\n", __func__);

	/* Clear TCIP */
	dsic_writel(dsic, DSI_TR_STA, 0x80000);
	dsic_writel(dsic, DSI_TR_STA, 0xfff);
}

static void dsic_send_short_packet(struct dsic_data *dsic, uint8_t data_type,
					uint32_t *sp_data, int trans_mode)
{
	uint32_t reg_val, wait_times;

	/*
	 * DSI_CTRL register bit 6 must 0, no-continue clk,
	 * between mode transmit
	 * TODO
	 * */
	debug("%s, data_type 0x%x, sp_data 0x%x\n",
		__func__, data_type, *sp_data);

	reg_val = dsic_readl(dsic, DSI_CTRL);
	reg_val &= 0xffffefff;
	dsic_writel(dsic, DSI_CTRL, reg_val);

	dsic_writel(dsic, DSI_PACK_HEADER, *sp_data);

	reg_val = (data_type << 8) | (trans_mode << 14);
	dsic_writel(dsic, DSI_PACK_CFG, reg_val);
	mdelay(1);

	/* start dsi packet transmission */
	reg_val = dsic_readl(dsic, DSI_PACK_CFG);
	reg_val |= 1;
	dsic_writel(dsic, DSI_PACK_CFG, reg_val);

	/* waiting for transmitting Complete */
	wait_times = 10;
	do {
		reg_val = dsic_readl(dsic, DSI_TR_STA);
		if (((reg_val >> 19) & 0x1) == 1)
			break;
		mdelay(1);
	} while (--wait_times);
	if (wait_times <= 0)
		printf("%s, short cmd transmission timout!\n", __func__);

	dsic_writel(dsic, DSI_TR_STA, 0x80000);
}

static uint32_t dsic_get_hsclk(struct dsic_data *dsic, uint16_t vtotal)
{
	uint16_t pixel2pro = 1;
	uint32_t h_bit = 0;		/*horizontal bit*/
	uint32_t h_byte = 0;		/*horizontal byte*/
	uint32_t per_lane_bps = 0;
	uint32_t htotal;
	uint32_t dsi_hsclk = 0;

	uint32_t bpp = owl_panel_get_bpp(dsic->ctrl->panel);
	struct owl_videomode *mode = &dsic->ctrl->panel->current_mode;

	switch (bpp) {
	case 16:
		pixel2pro = 2;
	break;
	case 18:
		pixel2pro = 3;
	break;
	case 24:
		pixel2pro = 3;
	break;
	}
	debug("pixel2pro = %d\n", pixel2pro);
	switch (dsic->configs.video_mode) {
	case 0:/*sync mode*/
		h_byte = HSS + (mode->hsw * pixel2pro + HAF) + HSE
		+ (mode->hbp * pixel2pro + HAF) + (mode->xres * pixel2pro + HAF)
		+ (mode->hfp * pixel2pro + HAF);
	break;
	case 1:/*de mode*/
		h_byte = HSS
			+ ((mode->hsw + mode->hbp)*pixel2pro + HAF)
			+ ((mode->xres * pixel2pro) + HAF)
			+ (mode->hfp * pixel2pro + HAF);
	break;
	case 2: /* burst mode */
		/* in burst mode bllp just blanking */
		h_byte = HSS + ((mode->hsw + mode->hbp)*pixel2pro + HAF)
		+ ((mode->xres * pixel2pro) + HAF)
		+ ((mode->hfp - dsic->configs.burst_bllp) * pixel2pro + HAF);
		debug("h_byte %d\n", h_byte);
	break;
	}
	h_bit = h_byte * 8;
	htotal = h_bit / dsic->configs.lane_count;
	per_lane_bps = htotal * vtotal * mode->refresh;
	dsi_hsclk = (per_lane_bps >> 1) / 1000000;/*Double edge transport*/

	if ((dsi_hsclk > 840) || (dsi_hsclk < 120)) {
		error("get_hsclk failed !\n");
		return -1;
	}

	dsi_hsclk /= 6;
	dsi_hsclk *= 6;/* multiple 6 */

	debug("%s dsi_hsclk = %d\n", __func__, dsi_hsclk);

	return dsi_hsclk;
}

static int dsic_clk_get_divider(unsigned int parent_rate,
					unsigned int target_rate)
{
	unsigned int divider, temp;
	if ((parent_rate < 0) || (target_rate < 0)) {
		error("divider is error!\n");
		return -1;
	}
	/*
	 * the speed of display device fetch data must be faster than DE
	 * sending, so the dsi phy actual clock can be higher than target_rate.
	 * */
	temp = ((parent_rate * 1000) / target_rate);
	divider = p_round(temp);
	temp = parent_rate / divider;
	if (temp < target_rate)
		divider -= 1;

	debug("%s: parent_rate %d, target_rate %d, divider %d\n",
	      __func__, parent_rate, target_rate, divider);
	if (divider <= 0) {
		error("divider is error!\n");
		return 0;
	}
	return divider - 1;
}
static int dsic_set_dsi_clk(struct dsic_data *dsic)
{
	struct owl_videomode *mode = &dsic->ctrl->panel->current_mode;
	uint32_t reg_val = 0;

	uint32_t cmu_dsiclk;
	uint32_t vtotal;
	uint16_t divider_reg = 0;
	int tmp;

	debug("%s\n", __func__);
	vtotal = mode->yres + mode->vsw + mode->vfp + mode->vbp;

	dsic->hsclk_pll = dsic_get_hsclk(dsic, vtotal);

	divider_reg = dsic_clk_get_divider(dsic->hsclk_pll,
					   dsic->configs.pclk_rate);
	if (dsic->diffs.id == DSIC_ID_S900) {
		debug("%s, s900\n", __func__);
		cmu_dsiclk = (0x3 << 8) | (divider_reg << 16) |
				(dsic->hsclk_pll / 6);

		debug("cmu_dsiclk  %x\n", cmu_dsiclk);
	} else if (dsic->diffs.id == DSIC_ID_S500 ||
		   dsic->diffs.id == DSIC_ID_S700) {
		debug("%s, s500 or s700\n", __func__);
		cmu_dsiclk = (1 << 16) | (dsic->hsclk_pll / 6) << 8 |
				divider_reg;

		debug("cmu_dsiclk  %x\n", cmu_dsiclk);
	}
	writel(cmu_dsiclk, CMU_DSICLK);
	debug("%s end.\n", __func__);
	return 0;
}

static void dsic_power_on(struct dsic_data *dsic)
{
	int temp;
	int ret = 0;

	debug("%s\n", __func__);

	/* assert reset */
	owl_reset_assert(RESET_DSI);

	/* enable sps ldo for s900 TODO*/
	if (dsic->diffs.id == DSIC_ID_S900) {
		temp = readl(SPS_LDO_CTL);
		temp |= (1 << 9);
		writel(temp, SPS_LDO_CTL);
	}

	/* enable dsiclk from devpll */
	owl_clk_enable(CLOCK_DSI);

	/* S700's DSI need CSI & MIPI 24M(bit 13) */
	if (dsic->diffs.id == DSIC_ID_S700)
		owl_clk_enable_by_perip_id(PERIPH_ID_CSI);

	dsic_set_dsi_clk(dsic);
	mdelay(10);

	/* deassert reset*/
	owl_reset_deassert(RESET_DSI);

	debug("%s end.\n", __func__);
}

static void dsic_deinit(struct dsic_data *dsic)
{
	int temp;

	/* disable sps ldo for s900 TODO */
	if (dsic->diffs.id == DSIC_ID_S900) {
		temp = dsic_readl(dsic, SPS_LDO_CTL);
		temp &= ~(1 << 9);
		dsic_writel(dsic, SPS_LDO_CTL, temp);
	}
	/* reset */

	/* clk disable */
}

static void dsic_ctl_config(struct dsic_data *dsic)
{
	int val = 0;

	debug("%s\n", __func__);
	/*
	val = dsic_readl(dsic, DSI_CTRL);
	val |= (1 << 31);
	dsic_writel(dsic, DSI_CTRL, val);
	*/

	/*color from*/
	val = dsic_readl(dsic, DSI_CTRL);
	val = REG_SET_VAL(val, 0, 7, 7);/* 0 DE, 1 default color */
	dsic_writel(dsic, DSI_CTRL, val);

	val = 0;
	val = dsic_readl(dsic, DSI_CTRL);/* 0: command mode,1: video mode*/
	val = REG_SET_VAL(val, dsic->configs.lcd_mode, 12, 12);
	dsic_writel(dsic, DSI_CTRL, val);
	/*0: 1 data lane,1: 2 data lanes 2: 3 data lanes 3: 4 data lanes*/
	val = 0;
	val = dsic_readl(dsic, DSI_CTRL);
	val = REG_SET_VAL(val, (dsic->configs.lane_count - 1), 9, 8);
	dsic_writel(dsic, DSI_CTRL, val);


	/* others,to do*/
	val = 0;
	val = dsic_readl(dsic, DSI_CTRL);
	val = REG_SET_VAL(val, 0, 4, 4);/* EOTP enable*/
	dsic_writel(dsic, DSI_CTRL, val);
}

static void dsic_phy_t0_calculate(struct dsic_data *dsic, uint32_t tphy_clk,
		uint32_t ui, uint32_t hs_prepare)
{
	/*about phy_t0*/
	uint8_t t_clk_prepare[6] = {1, 2, 4, 8, 12, 16};
	uint32_t N_clk_trail_cal, N_clk_trail;
	uint32_t N_clk_post_cal, N_clk_post;
	uint32_t N_clk_pre_cal, N_clk_pre;
	uint32_t N_clk_zero_cal, N_clk_zero;
	uint32_t N_clk_prepare_cal, N_clk_prepare;

	debug("%s\n", __func__);
	/*calculate about phy_t0*/
	N_clk_prepare_cal = (38000 + 95000) * 10 / 2 / tphy_clk;
	N_clk_prepare = get_dsi_phy_tx(N_clk_prepare_cal,
				N_clk_prepare_cal_arra, 5,
					N_clk_prepare_arra, &clk_prepar);
	debug("clk_prepare = %d\n", N_clk_prepare);

	N_clk_zero_cal = (300 * 1000 - t_clk_prepare[hs_prepare] * tphy_clk
					+ 20 * ui) * 10 / tphy_clk;
	N_clk_zero = get_dsi_phy_tx(N_clk_zero_cal, N_clk_zero_cal_arra, 4,
					N_clk_zero_arra, &clk_zero);
	debug("clk_zero = %d\n", N_clk_zero);

	N_clk_pre_cal = dsic_ceil((2 * 8 * ui), tphy_clk);
	N_clk_pre = get_dsi_phy_tx(N_clk_pre_cal, N_clk_pre_cal_arra, 2,
					N_clk_pre_arra, &clk_pre);
	debug("clk_pre = %d\n", N_clk_pre);

	N_clk_post_cal = dsic_ceil(dsic_ceil(((60 * 1000 + 52 * ui) + 20 * ui),
					tphy_clk), 4) - 2;
	N_clk_post = get_dsi_phy_tx(N_clk_post_cal, N_clk_post_cal_arra, 1,
					N_clk_post_arra, &clk_post);
	debug("clk_post = %d\n", N_clk_post);

	N_clk_trail_cal = 90 * 1000 * 10 / tphy_clk;
	N_clk_trail = get_dsi_phy_tx(N_clk_trail_cal + 1, N_clk_trail_cal_arra,
					6, N_clk_trail_arra, &clk_trail);
	debug("clk_trail = %d\n", N_clk_trail);

	dsic->configs.dsi_phy_t0 = (N_clk_trail << 11) | (N_clk_post << 8) |
			(N_clk_pre << 6) | (N_clk_zero << 3) | N_clk_prepare;

	debug("calculate phy_t0 end!\n");
}
static uint32_t dsic_phy_t1_calculate(struct dsic_data *dsic,
			uint32_t tphy_clk, uint32_t ui)
{
	/*about phy_t1*/
	uint8_t t_hs_prepare[6]  = {1, 2, 4, 8, 12, 16};
	uint32_t N_hs_exit_cal, N_hs_exit;
	uint32_t N_hs_trail;
	uint32_t N_hs_zero_cal, N_hs_zero;
	uint32_t N_hs_prepare_cal, N_hs_prepare;
	uint32_t hs_prepare_time_min, hs_prepare_time_max;

	debug("%s\n", __func__);
	/*in pico second, 0.000 000 000 001s*/
	hs_prepare_time_min = (40 * 1000 + 4 * ui);
	hs_prepare_time_max = (85 * 1000 + 6 * ui);
	N_hs_prepare_cal = (hs_prepare_time_min + hs_prepare_time_max) * 10 / 2
					/ tphy_clk;
	N_hs_prepare = get_dsi_phy_tx(N_hs_prepare_cal, N_hs_prepare_cal_arra,
					4, N_hs_prepare_arra, &hs_prepare);
	debug("hs_prepare = %d\n", N_hs_prepare);
	N_hs_zero_cal = ((145 * 1000 + 10 * ui) - t_hs_prepare[N_hs_prepare]
					* tphy_clk + 10 * ui) * 10 / tphy_clk;
	N_hs_zero = get_dsi_phy_tx(N_hs_zero_cal, N_hs_zero_cal_arra, 6,
					N_hs_zero_arra, &hs_zero);
	debug("hs_zero = %d\n", N_hs_zero);

	/*
	 * Find Ths_trail time setting deviation from the theoretical value is
	 * very large set. Other LP HS turn turn set the time parameters with
	 * the theoretical value of HS with LP, there are differences.
	 * */
	N_hs_trail = 3;
	debug("N_hs_trail = %d\n", N_hs_trail);

	N_hs_exit_cal = 180000 * 10 / tphy_clk;
	N_hs_exit = get_dsi_phy_tx(N_hs_exit_cal, N_hs_exit_cal_arra, 6,
					N_hs_exit_arra, &hs_exit);
	debug("N_hs_exit = %d\n", N_hs_exit);

	dsic->configs.dsi_phy_t1 = (N_hs_exit << 11) | (N_hs_trail << 8) |
				(N_hs_zero << 3) | N_hs_prepare;
	debug("calculate phy_t1 end!\n");
	return N_hs_prepare;
}
static void dsic_phy_t2_calculate(struct dsic_data *dsic, uint32_t tphy_clk)
{
	/*about phy_t2*/
	uint32_t N_wakeup, N_pre_scalar;

	debug("%s\n", __func__);
	/*calculate about phy_t2*/
	N_pre_scalar = dsic_ceil((TLPX * 1000), tphy_clk) - 1;
	N_wakeup = dsic_ceil((T_WAKEUP / TLPX - 1), 256);

	debug("pre_scalar %d, wakeup %d\n", N_pre_scalar, N_wakeup);

	dsic->configs.dsi_phy_t2 = (N_wakeup << 8) | N_pre_scalar;
	debug("calculate phy_t2 end!\n");
}

/*
 * pll clk at 30MHz to 600MHz, this calculation is correct !!!
 * */
static int dsic_phy_tx_calculate(struct dsic_data *dsic)
{
	uint32_t fphy_clk, tphy_clk, ui;
	uint32_t hs_prepare;
	debug("%s, hsclk_pll %d\n", __func__, dsic->hsclk_pll);

	if (dsic->hsclk_pll < 30 || dsic->hsclk_pll > 600)
		return -1;

	/* (MHz) */
	fphy_clk = dsic_ceil(dsic->hsclk_pll, 4);
	/* in pico second, 0.000 000 000 001s*/
	tphy_clk = (1000000 / fphy_clk);
	ui = tphy_clk / 8;

	/* calculate about phy_t2 */
	dsic_phy_t2_calculate(dsic, tphy_clk);

	/* calculate about phy_t1 */
	hs_prepare = dsic_phy_t1_calculate(dsic, tphy_clk, ui);

	/* calculate about phy_t0 */
	dsic_phy_t0_calculate(dsic, tphy_clk, ui, hs_prepare);

	debug("dsic_phy_t0: %x\n", dsic->configs.dsi_phy_t0);
	debug("dsic_phy_t1: %x\n", dsic->configs.dsi_phy_t1);
	debug("dsic_phy_t2: %x\n", dsic->configs.dsi_phy_t2);

	dsic_writel(dsic, DSI_PHY_T0, dsic->configs.dsi_phy_t0);
	dsic_writel(dsic, DSI_PHY_T1, dsic->configs.dsi_phy_t1);
	dsic_writel(dsic, DSI_PHY_T2, dsic->configs.dsi_phy_t2);

	return 0;
}
/*
 * Set the dsi controller`s sequence of data lines to match
 * the peripheral lcd interface.
 *
 * e.g. 0x3210 means that controller`s sequence of data lines is
 * 	data3
 * 	data2
 * 	data1
 * 	data0.
 * so we need to config the lane_swap parameters in dts.
 * */
static void dsic_set_data_lane_sequence(struct dsic_data *dsic)
{
	unsigned int lane_sequence, tmp;
	int i, val;

	lane_sequence = dsic->configs.lane_swap;
	debug("%s, the real lane sequence is 0x%x.\n", __func__, lane_sequence);

	val = dsic_readl(dsic, DSI_PIN_MAP);
	for (i = 0; i < 4; i++)
		val = REG_SET_VAL(val, ((lane_sequence >> i * 4) & 0xf),
					2 + (i * 3), 0 + (i * 3));
	dsic_writel(dsic, DSI_PIN_MAP, val);
}

static void dsic_phy_config(struct dsic_data *dsic)
{
	int tmp = 0;
	int ret;

	debug("%s\n", __func__);
	tmp = dsic_readl(dsic, DSI_CTRL);
	if (dsic->diffs.id == DSIC_ID_S900) {
		debug("%s, s900\n", __func__);
		tmp |= (3 << 8);
		tmp |= (1 << 31);
	} else if (dsic->diffs.id == DSIC_ID_S500 ||
		   dsic->diffs.id == DSIC_ID_S700) {
		debug("%s, s500\n", __func__);
		tmp |= (3 << 8);
		tmp |= ((dsic->configs.lcd_mode) << 12);
	}
	dsic_writel(dsic, DSI_CTRL, tmp);
	if (dsic->diffs.id == DSIC_ID_S900) {
		debug("%s, s900\n", __func__);
		ret = dsic_phy_tx_calculate(dsic);
		if (ret < 0)
			error("%s, s900 phy_tx set error!!!\n", __func__);
		/*
		dsic_writel(dsic, DSI_PHY_T0, 0x1ba3);
		dsic_writel(dsic, DSI_PHY_T1, 0x1b1b);
		dsic_writel(dsic, DSI_PHY_T2, 0x2f06);
		*/
	} else if (dsic->diffs.id == DSIC_ID_S500 ||
		   dsic->diffs.id == DSIC_ID_S700) {
		debug("%s, s500\n", __func__);
		ret = dsic_phy_tx_calculate(dsic);
		if (ret < 0)
			error("%s, s500 or s700 phy_tx set error!!!\n",
			      __func__);
		/*
		dsic_writel(dsic, DSI_PHY_T0, 0xa5a);
		dsic_writel(dsic, DSI_PHY_T1, 0x1b12);
		dsic_writel(dsic, DSI_PHY_T2, 0x2f05);
		*/
	}
	tmp = 0;
	if (dsic->diffs.id == DSIC_ID_S900) {
		debug("%s, s900\n", __func__);
		dsic_writel(dsic, DSI_PHY_CTRL, 0x7c600000);
		if (dsic->configs.lane_polarity) {
			tmp = dsic_readl(dsic, DSI_PHY_CTRL);
			tmp |= 0xb4083;/* swap N P,clk and data TODO*/
			dsic_writel(dsic, DSI_PHY_CTRL, tmp);
		} else {
		 tmp = dsic_readl(dsic, DSI_PHY_CTRL);
		 tmp |= 0x800fb;/* normal TODO*/
		 dsic_writel(dsic, DSI_PHY_CTRL, tmp);
		}
	} else if (dsic->diffs.id == DSIC_ID_S500 ||
		   dsic->diffs.id == DSIC_ID_S700) {
		debug("%s, s500\n", __func__);
		tmp = dsic_readl(dsic, DSI_PHY_CTRL);
		tmp |= (0x01 << 2);
		tmp |= (0xfc << 8);
		tmp |= (1 << 24);
		dsic_writel(dsic, DSI_PHY_CTRL, tmp);
	}
	mdelay(10);
	/* set data lanes sequence */
	dsic_set_data_lane_sequence(dsic);

	/* cal */
	debug("cal start\n");
	if (dsic->diffs.id == DSIC_ID_S900) {
		debug("%s, s900\n", __func__);
		tmp = dsic_readl(dsic, DSI_CTRL);
		tmp |= (1 << 30);
		dsic_writel(dsic, DSI_CTRL, tmp);
		/* wait for cal done */
		do {
			tmp = dsic_readl(dsic, DSI_CTRL);
		} while ((tmp & (1 << 30)));
		debug("cal done\n");
		debug("phy set end~~~~\n");
	} else if (dsic->diffs.id == DSIC_ID_S500 ||
		   dsic->diffs.id == DSIC_ID_S700) {
		debug("%s, s500\n", __func__);
		tmp = dsic_readl(dsic, DSI_PHY_CTRL);
		tmp |= ((1 << 25) | (1 << 28));
		dsic_writel(dsic, DSI_PHY_CTRL, tmp);
		/*wait for phy clk cal done*/
		do {
			tmp = dsic_readl(dsic, DSI_PHY_CTRL);
		} while ((!(tmp & (1 << 31))));
		debug("cal done\n");
		/*wait for cal done*/
		do {
			tmp = dsic_readl(dsic, DSI_PHY_CTRL);
		} while ((tmp & (1 << 25)));
	}
	debug("wait line stop\n");
	/* data0 line ,clk line in stop state */
	do {
		tmp = dsic_readl(dsic, DSI_LANE_STA);
	} while (!(tmp & 0x1020));

	/* continue clock EN */
	tmp = dsic_readl(dsic, DSI_CTRL);
	tmp |= 0x40;
	dsic_writel(dsic, DSI_CTRL, tmp);
	mdelay(1);

	debug("%s end.\n", __func__);
}


static void dsic_set_size(struct dsic_data *dsic)
{
	int tmp = 0;
	struct owl_videomode *mode = &dsic->ctrl->panel->current_mode;

	tmp = dsic_readl(dsic, DSI_SIZE);
	tmp = REG_SET_VAL(tmp, mode->yres, 27, 16);
	dsic_writel(dsic, DSI_SIZE, tmp);
}


static void dsic_set_default_color(struct dsic_data *dsic, uint32_t color)
{
		debug("%s\n", __func__);
		dsic_writel(dsic, DSI_COLOR, color);
}

static void dsic_set_timings(struct dsic_data *dsic)
{
	uint32_t bpp = owl_panel_get_bpp(dsic->ctrl->panel);
	struct owl_videomode *mode = &dsic->ctrl->panel->current_mode;
	uint32_t tmp = 0, preline = 0;
	uint32_t pixel2pro = 0;
	uint32_t hsw_reg = 0, hbp_reg = 0, hfp_reg = 0, bllp_reg = 0;
	uint32_t rgbht0_reg = 0, rgbht1_reg = 0, rgbvt0_reg = 0, rgbvt1_reg = 0;

	uint32_t vtotal = mode->vbp + mode->vfp + mode->vsw + mode->yres;

	debug("%s\n", __func__);
	switch (bpp) {
	case 16:
		pixel2pro = 2;
	break;
	case 18:
		pixel2pro = 3;
	break;
	case 24:
		pixel2pro = 3;
	break;
	}
	/**/
	switch (dsic->configs.video_mode) {
	case 0:/*sync mode*/
		hsw_reg  = mode->hsw * pixel2pro;
		hbp_reg  = mode->hbp * pixel2pro;
		hfp_reg  = mode->hfp * pixel2pro;
		bllp_reg = 0;
	break;
	case 1:/*de mode*/
		hsw_reg  = 0;
		hbp_reg  = (mode->hsw + mode->hbp) * pixel2pro;
		hfp_reg  = mode->hfp * pixel2pro;
		bllp_reg = 0;
	break;
	case 2:/*burst mode*/
		hsw_reg  = 0;
		hbp_reg  = (mode->hsw + mode->hbp) * pixel2pro;
		hfp_reg  = (mode->hfp - dsic->configs.burst_bllp) * pixel2pro;
		bllp_reg = dsic->configs.burst_bllp * pixel2pro;
	break;
	}

	rgbht0_reg = ((hsw_reg << 20) | (hfp_reg << 10) | (hbp_reg))
			& 0x3fffffff;
	rgbht1_reg = (bllp_reg) & 0xfff;
	/*preline | vsw | vtotal*/
	preline = owl_panel_get_preline_num(dsic->ctrl->panel);
	preline -= 1;
	preline = (preline < 0 ? 0 : preline);
	preline = (preline > 0xf ? 0xf : preline);

	rgbvt0_reg = (1 << 24) | (preline << 20) | (mode->vsw << 13) | vtotal;
	rgbvt1_reg = mode->vbp;
	debug("ht0 = %x\n", rgbht0_reg);
	debug("ht1 = %x\n", rgbht1_reg);
	debug("vt0 = %x\n", rgbvt0_reg);
	debug("vt1 = %x\n", rgbvt1_reg);

	dsic_writel(dsic, DSI_RGBHT0, rgbht0_reg);
	dsic_writel(dsic, DSI_RGBHT1, rgbht1_reg);
	dsic_writel(dsic, DSI_RGBVT0, rgbvt0_reg);
	dsic_writel(dsic, DSI_RGBVT1, rgbvt1_reg);

	/* others,to do*/
	dsic_writel(dsic, DSI_PACK_CFG, 0x0);
	dsic_writel(dsic, DSI_PACK_HEADER, (mode->xres * pixel2pro));

	tmp = dsic_readl(dsic, DSI_VIDEO_CFG);
	/*vidoe mode select*/
	tmp = REG_SET_VAL(tmp, dsic->configs.video_mode, 2, 1);
	dsic_writel(dsic, DSI_VIDEO_CFG, tmp);

	tmp = 0;
	tmp = dsic_readl(dsic, DSI_VIDEO_CFG);
	tmp = REG_SET_VAL(tmp, 1, 3, 3);
	dsic_writel(dsic, DSI_VIDEO_CFG, tmp);

	tmp = 0;
	tmp = dsic_readl(dsic, DSI_VIDEO_CFG);
	tmp = REG_SET_VAL(tmp, 0x3, 10, 8);/*RGB color format*/
	dsic_writel(dsic, DSI_VIDEO_CFG, tmp);

	debug("DSI_VIDEO_CFG = %x\n", tmp);
}


static void dsic_display_init(struct dsic_data *dsic)
{
	debug("%s\n", __func__);

	dsic_set_size(dsic);
	dsic_set_default_color(dsic, 0xff0000);
	dsic_set_timings(dsic);
}

static void dsic_single_enable(struct dsic_data *dsic, bool enable)
{
	int tmp;

	tmp = dsic_readl(dsic, DSI_VIDEO_CFG);

	if (enable)
		tmp |= 0x01;
	else
		tmp &= (~0x01);

	dsic_writel(dsic, DSI_VIDEO_CFG, tmp);
}
static void dsic_init(struct dsic_data *dsic)
{
	debug("%s\n", __func__);

	dsic_ctl_config(dsic);

	dsic_display_init(dsic);

	dsic_single_enable(dsic, true);
}

static int bta_test_init(struct dsic_data *dsic)
{
	debug("%s, start\n", __func__);
	int wait_times;
	uint32_t reg_val;

	/* set to command mode and no continue clock */
	reg_val = dsic_readl(dsic, DSI_CTRL);
	reg_val &= 0xffffefbf;
	dsic_writel(dsic, DSI_CTRL, reg_val);

	/* soft reset RX FIFO */
	reg_val = dsic_readl(dsic, DSI_CTRL);
	reg_val |= 1 << 3;
	dsic_writel(dsic, DSI_CTRL, reg_val);

	/* wait Input FIFO Empty Flag */
	debug("%s, wait input fifo empty\n", __func__);
	wait_times = 10;
	do {
		reg_val = dsic_readl(dsic, DSI_TR_STA);
		if ((reg_val & (1 << 8)) != 0)
			break;
		mdelay(1);
	} while (--wait_times);
	if (wait_times <= 0) {
		printf("%s, wait input fifo empty timeout\n", __func__);
		return -1;
	}

	/* end RX FIFO Reset */
	reg_val = dsic_readl(dsic, DSI_CTRL);
	reg_val &= ~(1 << 3);
	dsic_writel(dsic, DSI_CTRL, reg_val);

	/* clear Error */
	reg_val = dsic_readl(dsic, DSI_TR_STA);
	reg_val |= 0xff;
	dsic_writel(dsic, DSI_TR_STA, reg_val);

	/* set BTA timeout */
	dsic_writel(dsic, DSI_TIMEOUT, 0x1000);

	debug("%s, end\n", __func__);

	return 0;
}

static int bta_test_waiting_turn_over(struct dsic_data *dsic)
{
	int wait_times;
	uint32_t reg_val, tmp;

	debug("%s, start\n", __func__);

	/* force Data lane0 into BTA mode */
	tmp = dsic_readl(dsic, DSI_LANE_CTRL);
	tmp |= 0x1;
	dsic_writel(dsic, DSI_LANE_CTRL, tmp);

	/* judge if BTA turn over */
	wait_times = 10;
	do {
		reg_val = dsic_readl(dsic, DSI_TR_STA);
		if ((reg_val & 0x00040000) != 0)
			break;
		mdelay(1);
	} while (--wait_times);
	if (wait_times <= 0) {
		printf("%s, timeout\n", __func__);
		return -1;
	}

	/* clear BTA interrupt pending */
	dsic_writel(dsic, DSI_TR_STA, 0x00040000);

	debug("%s, end\n", __func__);
	return 0;
}

static enum dsi_transfer_state bta_test_received_mode(struct dsic_data *dsic)
{
	unsigned char rx_datatype, rx_dataid, rx_word_cnt, rx_trigger;
	int i;
	uint32_t tmp;

	/* bit17 Rx_Data_Done */
	if (dsic_readl(dsic, DSI_TR_STA) & 0x00020000) {

		/* clear Rx_Data_Done interrupt pending */
		tmp = dsic_readl(dsic, DSI_TR_STA);
		tmp |= 1 << RX_DAT_DONE;
		dsic_writel(dsic, DSI_TR_STA, tmp);

		debug("DSI_IPACK 0x%x\n", dsic_readl(dsic, DSI_IPACK));
		/*
		 * get Input Packet Type, bit 24
		 * 0:short packet; 1: long packet
		 */
		rx_datatype = (dsic_readl(dsic, DSI_IPACK) >> 24) & 0x1;
		rx_dataid = (dsic_readl(dsic, DSI_IPACK) >> 16) & 0xff;
		rx_word_cnt = dsic_readl(dsic, DSI_IPACK) & 0xffff;
		debug("%s, rx_datatype %d, rx_dataid %d, rx_word_cnt %d\n",
				__func__, rx_datatype, rx_dataid, rx_word_cnt);

		for (i = 0; i <= rx_word_cnt; i++) {
			printf("DSI_FIFO_IDAT 0x%x\n", dsic_readl(dsic, DSI_FIFO_IDAT));
		}

		return RX_DAT_DONE;
	}

	/* bit16 Rx_ACK_Packet */
	if (dsic_readl(dsic, DSI_TR_STA) & 0x00010000) {

		/* clear Rx_ACK_Packet interrupt pending */
		tmp = dsic_readl(dsic, DSI_TR_STA);
		tmp |= 1 << RX_ACK_AND_ERRO_PACK;
		dsic_writel(dsic, DSI_TR_STA, tmp);
		return RX_ACK_AND_ERRO_PACK;
	}

	/* bit15 Rx_Trigger */
	if (dsic_readl(dsic, DSI_TR_STA) & 0x00008000) {

		/* clear Rx_Trigger interrupt pending */
		tmp = dsic_readl(dsic, DSI_TR_STA);
		tmp |= 1 << RX_TRIGGER;
		dsic_writel(dsic, DSI_TR_STA, tmp);

		rx_trigger = dsic_readl(dsic, DSI_RX_TRIGGER);
		printf("rx_trigger 0x%x\n", rx_trigger);
		return RX_TRIGGER;
	}

	return -1;
}

/*
 * Not: this function is used to test whether the communication is normal
 * between dsi controller and panel.
 * */
static int owl_dsi_bta_test(struct dsic_data *dsic)
{
	int ret;
	enum dsi_transfer_state dsi_bta_resault;
	uint32_t get_pixel_format = 0x000c;

	/*
	 * before dsi BTA, need send mipi read command
	 * in HP mode
	 * */
	dsic_send_short_packet(dsic, 0x06, &get_pixel_format, 0);

	ret = bta_test_init(dsic);
	if (ret < 0) {
		printf("%s, bta test init failed\n", __func__);
		return -1;
	}

	ret = bta_test_waiting_turn_over(dsic);
	if (ret < 0) {
		printf("%s, bta test waiting turn over failed\n", __func__);
		return -1;
	}

	dsi_bta_resault = bta_test_received_mode(dsic);
	switch (dsi_bta_resault) {
	case RX_TRIGGER:
		debug("%s, rx trigger\n", __func__);
		break;
	case RX_ACK_AND_ERRO_PACK:
		debug("%s error, dsi ctrl need to re-enable!\n", __func__);
		break;
	case RX_DAT_DONE:
		debug("%s, rx data done\n", __func__);
		break;

	default:
		return -1;
	}

	return 0;
}

int owl_dsic_enable(struct owl_display_ctrl *ctrl)
{
	debug("%s\n", __func__);

	struct dsic_data *dsic = owl_ctrl_get_drvdata(ctrl);
	dsic_init(dsic);

	return 0;
}

void owl_dsic_disable(struct owl_display_ctrl *ctrl)
{
	struct dsic_data *dsic = owl_ctrl_get_drvdata(ctrl);

	debug("%s\n", __func__);

	dsic_single_enable(dsic, false);
}
int owl_dsic_power_on(struct owl_display_ctrl *ctrl)
{
	struct dsic_data *dsic = owl_ctrl_get_drvdata(ctrl);

	debug("%s, ... ...\n", __func__);
	dsic_power_on(dsic);

	/*
	 * config dsi phy, and ready for sending initial command
	 * */
	dsic_phy_config(dsic);

	return 0;
}
int owl_dsic_power_off(struct owl_display_ctrl *ctrl)
{
	struct dsic_data *dsic = owl_ctrl_get_drvdata(ctrl);

	return 0;
}
int owl_dsic_aux_read(struct owl_display_ctrl *ctrl, char *buf, int count)
{
	struct dsic_data *dsic = owl_ctrl_get_drvdata(ctrl);

	return 0;
}

/*
 * command buffer format:
 * 	buffer 4 ~ (MIPI_MAX_PARS - 1) ---> parameters
 * 	buffer 3---> address
 * 	buffer 2---> number of parameters
 * 	buffer 1---> dcs data type
 * 	buffer 0---> cmd delay
 *
 */
int owl_dsic_aux_write(struct owl_display_ctrl *ctrl,
			const char *buf, int count)
{
	struct dsic_data *dsic = owl_ctrl_get_drvdata(ctrl);
	int trans_mode = 1;
	uint8_t data_type, cmd_delay;
	uint8_t *buffer = buf;

	uint8_t *data, n_data;

	cmd_delay = buffer[0];
	data_type = buffer[1];
	n_data = buffer[2] + 1;
	data = &buffer[3];

	switch (data_type) {
	case MIPI_DSI_GENERIC_LONG_WRITE:
	case MIPI_DSI_DCS_LONG_WRITE:
		send_normal_long_packet(dsic, data_type,
					n_data, data, trans_mode);
		break;

	case MIPI_DSI_GENERIC_SHORT_WRITE_0_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_1_PARAM:
	case MIPI_DSI_GENERIC_SHORT_WRITE_2_PARAM:
	case MIPI_DSI_DCS_SHORT_WRITE:
	case MIPI_DSI_DCS_SHORT_WRITE_PARAM:
		dsic_send_short_packet(dsic, data_type,
					(uint32_t*)data, trans_mode);
		break;
	default:
		return -1;
	}

	mdelay(cmd_delay);
	return 0;
}
struct owl_display_ctrl_ops owl_dsi_ctrl_ops = {
	.enable = owl_dsic_enable,
	.disable = owl_dsic_disable,

	.power_on = owl_dsic_power_on,
	.power_off = owl_dsic_power_off,

	.aux_read = owl_dsic_aux_read,
	.aux_write = owl_dsic_aux_write,
};

static struct owl_display_ctrl owl_dsi_ctrl = {
	.name = "dsi_ctrl",
	.type = OWL_DISPLAY_TYPE_DSI,
	.ops = &owl_dsi_ctrl_ops,
};

static struct dsic_data	owl_dsic_data;

int owl_dsic_init(const void *blob)
{
	int node;

	int ret = 0;
	debug("%s\n", __func__);
	struct dsic_data *dsic = &owl_dsic_data;

	/* DTS match */
	node = fdt_node_offset_by_compatible(blob, 0, "actions,s900-dsi");
	if (node > 0) {
		dsic->diffs.id = DSIC_ID_S900;
	} else {
		node = fdt_node_offset_by_compatible(blob, 0,
							"actions,s700-dsi");
		if (node > 0) {
			dsic->diffs.id = DSIC_ID_S700;
		} else {
			node = fdt_node_offset_by_compatible(blob, 0,
							"actions,s500-dsi");
			if (node > 0) {
				dsic->diffs.id = DSIC_ID_S500;
			} else {
				debug("no match in DTS\n");
				return 0;
			}
		}
	}
	debug("%s, ic_type = %d\n", __func__, dsic->diffs.id);

	dsic->dma_channel = owl_dma_request();
	if (!dsic->dma_channel) {
		error("%s, owl_dma_request failed!\n", __func__);
		return -1;
	}

	dsic->base = fdtdec_get_addr(blob, node, "reg");
	if (dsic->base == FDT_ADDR_T_NONE) {
		error("Cannot find dsic reg address\n");
		return -1;
	}
	debug("%s: base is 0x%llx\n", __func__, dsic->base);

	ret = dsic_parse_config(blob, node, dsic);
	if (ret < 0)
		goto err_parse_config;

	dsic->ctrl = &owl_dsi_ctrl;
	owl_ctrl_set_drvdata(&owl_dsi_ctrl, dsic);

	ret = owl_ctrl_register(&owl_dsi_ctrl);
	if (ret < 0)
		goto err_ctrl_register;

	return 0;

err_ctrl_register:
err_parse_config:
	return ret;
}
