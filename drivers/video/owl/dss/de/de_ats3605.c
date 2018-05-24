/*
 * Copyright (c) 2015 Actions Semi Co., Ltd.
 *
 * Lipeng<lipeng@actions-semi.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#define DEBUGX
#define pr_fmt(fmt) "de_ats3605: " fmt

#include <common.h>
#include <asm/io.h>
#include <asm/arch/regs.h>
#include <asm/arch/clk.h>
#include <asm/arch/reset.h>
#include <asm/arch/powergate.h>

#include <dss.h>

#include "de_ats3605.h"

static struct owl_de_device	owl_de_ats3605;

#define de_readl(idx)		readl(owl_de_ats3605.base + (idx))
#define de_writel(val, idx)	writel(val, owl_de_ats3605.base + (idx))

#define SHARESRAM_BASE		0xB0200004	
#define SHARESRAM_CTL		(SHARESRAM_BASE + 0x0000)

/*===================================================================
 *			ATS3605 DE path
 *==================================================================*/

static int de_ats3605_path_enable(struct owl_de_path *path, bool enable)
{
	uint32_t val;

	debug("%s: path %d enable %d\n", __func__, path->id, enable);

	val = de_readl(DE_PATH_EN(path->id));
	val |= enable;
	de_writel(val, DE_PATH_EN(path->id));

	return 0;
}

static void __path_display_type_set(struct owl_de_path *path,
				    enum owl_display_type type)
{
	uint32_t val;

	debug("%s: path %d type %d\n", __func__, path->id, type);

	if (path->id == 1) {
		/* LCD */
		switch (type) {
		case OWL_DISPLAY_TYPE_LCD:
			break;
		case OWL_DISPLAY_TYPE_DSI:
			break;
		default:
			BUG();
			break;
		}
	} else {
		/* digit */
		switch (type) {
		case OWL_DISPLAY_TYPE_HDMI:
			break;
		case OWL_DISPLAY_TYPE_CVBS:
			break;
		default:
			BUG();
			break;
		}
	}

}

static void __path_size_set(struct owl_de_path *path,
			    uint32_t width, uint32_t height)
{
	uint32_t val;

	debug("%s: path %d %dx%d\n", __func__, path->id, width, height);

	BUG_ON((width > DE_PATH_SIZE_WIDTH) || (height > DE_PATH_SIZE_HEIGHT));

	val = REG_VAL(height - 1, DE_PATH_SIZE_HEIGHT_END_BIT,
			  DE_PATH_SIZE_HEIGHT_BEGIN_BIT)
		| REG_VAL(width - 1, DE_PATH_SIZE_WIDTH_END_BIT,
			  DE_PATH_SIZE_WIDTH_BEGIN_BIT);

	de_writel(val, DE_PATH_SIZE(path->id));
}

static void __path_dither_enable(struct owl_de_path *path, bool enable)
{
	uint32_t val;

	debug("%s: path %d, enable %d\n", __func__, path->id, enable);

	/* only valid for LCD, TODO */
	if (path->id != 1)
		return;

	val = de_readl(DE_PATH_CTL(path->id));

	val = REG_SET_VAL(val, enable, DE_PATH_DITHER_ENABLE_BIT,
				DE_PATH_DITHER_ENABLE_BIT);

	de_writel(val, DE_PATH_CTL(path->id));
}

static void __path_dither_set(struct owl_de_path *path,
			      enum owl_dither_mode mode)
{
	uint32_t val;

	debug("%s: path %d, mode %d\n", __func__, path->id, mode);

	if (path->id != 1)	/* TODO */
		return;

	if (mode == DITHER_DISABLE) {
		__path_dither_enable(path, false);
		return;
	}

	val = de_readl(DE_PATH_CTL(path->id));

	switch (mode) {
	case DITHER_24_TO_18:
		val = REG_SET_VAL(val, 0x7, 14, 12);/* rgb666 */
		break;

	case DITHER_24_TO_16:
		val = REG_SET_VAL(val, 0x2, 14, 12);/* rgb565 */
		break;

	default:
		return;
	}

	de_writel(val, DE_PATH_CTL(path->id));

	__path_dither_enable(path, true);

}
static void __path_output_format_set(struct owl_de_path *path, uint32_t is_yuv)
{
/* path1 can output yuv formate  FIXME*/
}

static void __path_vmode_set(struct owl_de_path *path, uint32_t vmode)
{
	uint32_t val;
	debug("%s: path %d vmode %d\n", __func__, path->id, vmode);

	val = de_readl(DE_PATH_CTL(path->id));
	if (vmode == DSS_VMODE_INTERLACED)
		val = REG_SET_VAL(val, 1, DE_PATH_CTL_INTERLACE_BIT,
				  DE_PATH_CTL_INTERLACE_BIT);
	else
		val = REG_SET_VAL(val, 0, DE_PATH_CTL_INTERLACE_BIT,
				  DE_PATH_CTL_INTERLACE_BIT);
	de_writel(val, DE_PATH_CTL(path->id));
}

static void __path_default_color_set(struct owl_de_path *path, uint32_t color)
{
	debug("%s: path %d color %x\n", __func__, path->id, color);

	de_writel(color, DE_PATH_BK(path->id));
}

static void de_ats3605_path_apply_info(struct owl_de_path *path)
{
	struct owl_de_path_info *info = &path->info;

	debug("%s: path%d\n", __func__, path->id);

	__path_size_set(path, info->width, info->height);

	__path_dither_set(path, info->dither_mode);

	__path_vmode_set(path, info->vmode);

	/* for test */
	__path_default_color_set(path, 0x0);
}

static void de_ats3605_path_set_go(struct owl_de_path *path)
{
	uint32_t val;

	debug("%s: path%d\n", __func__, path->id);
	val = de_readl(DE_PATH_FCR(path->id));
	val |= 0x1;
	de_writel(val, DE_PATH_FCR(path->id));
}

static void __path_gamma_set(struct owl_de_path *path, uint32_t *gamma_val)
{
	bool is_busy;
	uint32_t idx, val;

	for (idx = 0; idx < (256 * 3 / 4); idx++) {
		/* write index */
		val = REG_SET_VAL(val, idx, DE_PATH_GAMMA_IDX_INDEX_END_BIT,
				DE_PATH_GAMMA_IDX_INDEX_BEGIN_BIT);
		de_writel(val, DE_PATH_GAMMA_IDX(path->id));

		/* write ram */
		de_writel(gamma_val[idx], DE_PATH_GAMMA_RAM(path->id));

	}

	/* write finish, clear write bit and index */
	val = de_readl(DE_PATH_GAMMA_IDX(path->id));
	val = REG_SET_VAL(val, 0, DE_PATH_GAMMA_IDX_INDEX_END_BIT,
					DE_PATH_GAMMA_IDX_INDEX_BEGIN_BIT);
	de_writel(val, DE_PATH_GAMMA_IDX(path->id));
}

static void de_ats3605_path_set_gamma_table(struct owl_de_path *path)
{
	struct owl_de_path_info *info = &path->info;
	uint8_t gamma_data[256 * 3];
	int i = 0;

	debug("%s, path%d, gamma_r %d, gamma_g %d, gamma_b %d\n",
		__func__, path->id, info->gamma_r_val, info->gamma_g_val, info->gamma_b_val);

	if ((info->gamma_r_val < 0) ||
		(info->gamma_g_val < 0) || (info->gamma_b_val < 0))
		error("%s, unavailable gamma val!", __func__);

	/* only valid for LCD, TODO */
	if (path->id != 1)
		return;

	/* R */
	for (i = 0; i < 256; i++) {
		gamma_data[i] = i * info->gamma_r_val / 100;
	}

	/* G */
	for (i = 0; i < 256; i++) {
		gamma_data[i + 256] = i * info->gamma_g_val / 100;
	}

	/* B */
	for (i = 0; i < 256; i++) {
		gamma_data[i + 256 * 2] = i * info->gamma_b_val / 100;
	}

	__path_gamma_set(path, &gamma_data[0]);

	debug("%s, ok!\n", __func__);
}

static void de_ats3605_path_get_gamma_table(struct owl_de_path *path)
{
	struct owl_de_path_info *info = &path->info;

	debug("%s, path%d\n", __func__, path->id);
}

static int de_ats3605_path_gamma_enable(struct owl_de_path *path, bool enable)
{
	uint32_t val;

	debug("%s,path %d enable %d\n", __func__, path->id, enable);

	val = de_readl(DE_PATH_GAMMA_ENABLE(path->id));

	val = REG_SET_VAL(val, enable, DE_PATH_GAMMA_ENABLE_BIT,
			DE_PATH_GAMMA_ENABLE_BIT);

	de_writel(val, DE_PATH_GAMMA_ENABLE(path->id));

}

static struct owl_de_path_ops de_ats3605_path_ops = {
	.enable = de_ats3605_path_enable,
	.apply_info = de_ats3605_path_apply_info,
	.set_go = de_ats3605_path_set_go,

	.set_gamma_table = de_ats3605_path_set_gamma_table,
	.get_gamma_table = de_ats3605_path_get_gamma_table,
	.gamma_enable = de_ats3605_path_gamma_enable,
};

static struct owl_de_path de_ats3605_paths[] = {
	{
		.id			= 0,
		.name			= "digit",
		.supported_displays	= OWL_DISPLAY_TYPE_HDMI,
		.ops			= &de_ats3605_path_ops,
	},
	{
		.id			= 1,
		.name			= "lcd",
		.supported_displays	= OWL_DISPLAY_TYPE_LCD
					| OWL_DISPLAY_TYPE_DUMMY,
		.ops			= &de_ats3605_path_ops,
	},
};


/*===================================================================
 *			ATS3605 DE video layer
 *==================================================================*/
static int de_ats3605_video_enable(struct owl_de_video *video, bool enable)
{
	uint32_t val;

	debug("%s: %d\n", __func__, enable);

	if (video->path == NULL) {
		error("set a path before enable/disable\n");
		return -1;
	}

	if (video->path->info.type == OWL_DISPLAY_TYPE_DUMMY)
		return 0;

	val = de_readl(DE_PATH_CTL(video->path->id));
	val = REG_SET_VAL(val, enable, DE_PATH_VIDEO_ENABLE_BIT,
				  DE_PATH_VIDEO_ENABLE_BIT);
	de_writel(val, DE_PATH_CTL(video->path->id));

	return 0;
}

static int __de_color_mode_to_hw_mode(enum owl_color_mode color_mode)
{
	int hw_format = 0;
	/*
	 * Pay attention to s700:
	 * 		NOTE: R G B/Y U V is inversed between
	 * 		de color mode and datasheet!!
	 * to ats3605 ???
	 */
	switch (color_mode) {
	case OWL_DSS_COLOR_BGR16:
		hw_format = 0;
		break;

	case OWL_DSS_COLOR_RGBA16:
		hw_format = 4;
		break;

	case OWL_DSS_COLOR_BGRA32:
		hw_format = 1;
		break;

	case OWL_DSS_COLOR_RGBA32:
		hw_format = 5;
		break;

	default:
		BUG();
		break;
	}

	return hw_format;
}

static void __video_format_set(struct owl_de_video *video,
			       enum owl_color_mode color_mode)
{
	uint32_t val;
	int hw_format = 0;
	
	hw_format = __de_color_mode_to_hw_mode(color_mode);

	debug("%s: color_mode = 0x%x, hw_format = 0x%x\n",
	      __func__, color_mode, hw_format);

	val = de_readl(DE_VIDEO_CFG(video->id));
	val |= 0x1 << 26;/* video critial ctl signal */
	val |= hw_format;
	de_writel(val, DE_VIDEO_CFG(video->id));
}

static void __video_rotate_set(struct owl_de_video *video, int rotation)
{
	uint32_t val;
	/* do not support xflip yflip */
}

static void __video_isize_set(struct owl_de_video *video,
			      uint32_t width, uint32_t height)
{
	uint32_t val;

	debug("%s: video %d, %dx%d\n", __func__, video->id, width, height);

	de_writel(((height -1) << 16) | (width  - 1),
			DE_VIDEO_ISIZE(video->id));
}

static void __video_osize_set(struct owl_de_video *video,
			      uint32_t width, uint32_t height)
{
	uint32_t val;

	debug("%s, video %d, %dx%d\n", __func__, video->id, width, height);
	de_writel(((height -1) << 16) | (width  - 1),
			DE_VIDEO_OSIZE(video->id));
	BUG_ON((width > DE_PATH_SIZE_WIDTH) || (height > DE_PATH_SIZE_HEIGHT));
}

static void __video_position_set(struct owl_de_path *path,
				 struct owl_de_video *video,
				 uint32_t x_pos, uint32_t y_pos)
{
	uint32_t val;

	debug("%s: video %d, (%d, %d)\n", __func__, video->id, x_pos, y_pos);
	val = de_readl(DE_PATH_A_COOR(path->id));
	val |= y_pos << 16;
	val |= x_pos;
	de_writel(val, DE_PATH_A_COOR(path->id));

	BUG_ON((x_pos > DE_PATH_SIZE_WIDTH) || (y_pos > DE_PATH_SIZE_HEIGHT));
}

static void __video_set_scal_coef(struct owl_de_video *video, uint8_t scale_mode)
{
	struct owl_de_path *path = video->path;

	switch (scale_mode) {
	case DE_SCLCOEF_ZOOMIN:
		de_writel(0X00004000, DE_VIDEO_SCOEF(video->id, 0));
		de_writel(0xFF073EFC, DE_VIDEO_SCOEF(video->id, 1));
		de_writel(0xFE1038FA, DE_VIDEO_SCOEF(video->id, 2));
		de_writel(0xFC1B30F9, DE_VIDEO_SCOEF(video->id, 3));
		de_writel(0xFA2626FA, DE_VIDEO_SCOEF(video->id, 4));
		de_writel(0xF9301BFC, DE_VIDEO_SCOEF(video->id, 5));
		de_writel(0xFA3810FE, DE_VIDEO_SCOEF(video->id, 6));
		de_writel(0xFC3E07FF, DE_VIDEO_SCOEF(video->id, 7));
		break;

	case DE_SCLCOEF_HALF_ZOOMOUT:
		de_writel(0x00004000, DE_VIDEO_SCOEF(video->id, 0));
		de_writel(0x00083800, DE_VIDEO_SCOEF(video->id, 1));
		de_writel(0x00103000, DE_VIDEO_SCOEF(video->id, 2));
		de_writel(0x00182800, DE_VIDEO_SCOEF(video->id, 3));
		de_writel(0x00202000, DE_VIDEO_SCOEF(video->id, 4));
		de_writel(0x00281800, DE_VIDEO_SCOEF(video->id, 5));
		de_writel(0x00301000, DE_VIDEO_SCOEF(video->id, 6));
		de_writel(0x00380800, DE_VIDEO_SCOEF(video->id, 7));
		break;

	case DE_SCLCOEF_SMALLER_ZOOMOUT:
		de_writel(0X00102010, DE_VIDEO_SCOEF(video->id, 0));
		de_writel(0X02121E0E, DE_VIDEO_SCOEF(video->id, 1));
		de_writel(0X04141C0C, DE_VIDEO_SCOEF(video->id, 2));
		de_writel(0X06161A0A, DE_VIDEO_SCOEF(video->id, 3));
		de_writel(0X08181808, DE_VIDEO_SCOEF(video->id, 4));
		de_writel(0X0A1A1606, DE_VIDEO_SCOEF(video->id, 5));
		de_writel(0X0C1C1404, DE_VIDEO_SCOEF(video->id, 6));
		de_writel(0X0E1E1202, DE_VIDEO_SCOEF(video->id, 7));
		break;

	default:
		BUG();
	}
}

static void __video_scaling_set(struct owl_de_video *video,
				uint32_t width, uint32_t height,
				uint32_t out_width, uint32_t out_height)
{
	uint8_t scale_mode;
	uint16_t w_factor, h_factor;
	uint16_t factor;
	uint32_t val = 0;

	debug("%s: video %d, %dx%d->%dx%d\n", __func__, video->id,
	      width, height, out_width, out_height);
	
	w_factor = (width * 8192 +  out_width - 1) / out_width;
	h_factor = (height * 8192 + out_height - 1) / out_height;
	
	val = h_factor << 16 | w_factor;
	de_writel(val, DE_VIDEO_SR(video->id));

	factor = (width * height * 10) / (out_width * out_height);
	if (factor <= 10)
		scale_mode = DE_SCLCOEF_ZOOMIN;
	else if (factor <= 40)
		scale_mode = DE_SCLCOEF_HALF_ZOOMOUT;
	else if (factor > 40)
		scale_mode = DE_SCLCOEF_SMALLER_ZOOMOUT;

	__video_set_scal_coef(video, scale_mode);
}

static void __video_alpha_set(struct owl_de_video *video,
			      enum owl_blending_type blending, uint8_t alpha)
{
	uint32_t val;
	/* do not support */
}

static void de_ats3605_video_apply_info(struct owl_de_video *video)
{
	uint16_t outw, outh;
	struct owl_de_video_info *info = &video->info;

	debug("%s: video %d, dirty %d\n",
	      __func__, video->id, video->info.dirty);

	outw = (info->out_width == 0 ? info->width : info->out_width);
	outh = (info->out_height == 0 ? info->height : info->out_height);

	/* fb addr, only fb0 is used, TODO */
	de_writel(info->addr[0], DE_VIDEO_FB(video->id, 0));

	/* stride val requested double word(8 bytes) in length for IC ats3605 DE */
	de_writel(info->pitch[0] / 8, DE_VIDEO_STR(video->id));

	__video_format_set(video, info->color_mode);
	__video_rotate_set(video, info->rotate);

	/* TODO */
	__video_isize_set(video, info->width, info->height);
	__video_osize_set(video, outw, outh);

	__video_scaling_set(video, info->width, info->height, outw, outh);
	__video_position_set(video->path, video, info->pos_x, info->pos_y);

	__video_alpha_set(video, info->blending, info->alpha);
}

static struct owl_de_video_ops de_ats3605_video_ops = {
	.enable = de_ats3605_video_enable,
	.apply_info = de_ats3605_video_apply_info,
};


#define OWL_ats3605_SUPPORTED_COLORS (OWL_DSS_COLOR_BGR16 \
	| OWL_DSS_COLOR_BGRA32 | OWL_DSS_COLOR_RGBA32 \
	| OWL_DSS_COLOR_YU12 | OWL_DSS_COLOR_NV21 \
	| OWL_DSS_COLOR_RGBA16)

/* 2 video layers in boot */
static struct owl_de_video de_ats3605_videos[] = {
	{
		.id			= 0,
		.name			= "video0",
		.supported_colors	= OWL_ats3605_SUPPORTED_COLORS,
		.ops			= &de_ats3605_video_ops,
	},
	{
		.id			= 1,
		.name			= "video1",
		.supported_colors	= OWL_ats3605_SUPPORTED_COLORS,
		.ops			= &de_ats3605_video_ops,
	},
};


/*===================================================================
 *			ATS3605 DE video layer
 *==================================================================*/

static int de_ats3605_device_power_on(struct owl_de_device *de)
{
	uint32_t tmp, i;
	uint32_t display_clk, de_div;
	debug("%s\n", __func__);

	/* assert reset */
	owl_reset_assert(RESET_DE);

	/* power on */


	/*
	 * DECLK:
	 * source is DEV_CLK,
	 * divider is ?, 330MHz ???
	 */
	display_clk = (readl(CMU_DEVPLL) & 0x7f) * 6;
	de_div = (display_clk + 125) / 330 - 1;

	tmp = readl(CMU_DECLK);
	tmp |= (de_div + 2) << 8 
		| de_div  << 4
		| de_div
		| 1 << 12;
	writel(tmp, CMU_DECLK);

	/* enable declk from devpll */
	owl_clk_enable(CLOCK_DE);
	mdelay(1);

	/* de-assert reset */
	owl_reset_deassert(RESET_DE);
	mdelay(1);

	debug("%s: end\n", __func__);
	return 0;
}


static int de_ats3605_device_init(struct owl_de_device *de)
{
	/*
	 * some special init, must be same as kernel
	 */
	writel(0x1, SHARESRAM_CTL);

	return 0;
}

static void de_ats3605_device_dump_regs(struct owl_de_device *de)
{
	int i, j;

#define DUMPREG(r) printf("%08x ~~ %08x ~~ %s\n", r, de_readl(r), #r)
	DUMPREG(0xB02F0000);
	DUMPREG(DE_IRQENABLE);
	DUMPREG(DE_IRQSTATUS);
	DUMPREG(DE_OUTPUT_CON);
	DUMPREG(DE_OUTPUT_STAT);

	for (i = 0; i < 2; i++) {
		printf("\npath %d ------------------>\n", i);
		DUMPREG(DE_PATH_CTL(i));
		DUMPREG(DE_PATH_FCR(i));
		DUMPREG(DE_PATH_EN(i));
		DUMPREG(DE_PATH_BK(i));
		DUMPREG(DE_PATH_SIZE(i));
		DUMPREG(DE_PATH_A_COOR(i));
		DUMPREG(DE_PATH_GAMMA_IDX(i));
	}

	for (i = 0; i < 2; i++) {
		printf("\nlayer %d ------------------>\n", i);
		DUMPREG(DE_VIDEO_CFG(i));
		DUMPREG(DE_VIDEO_ISIZE(i));
		DUMPREG(DE_VIDEO_OSIZE(i));
		DUMPREG(DE_VIDEO_FB(i, 0));
		DUMPREG(DE_VIDEO_FB(i, 1));
		DUMPREG(DE_VIDEO_FB(i, 2));
		DUMPREG(DE_VIDEO_STR(i));
	}

	for (i = 0; i < 2; i++) {
		printf("\nscaler %d ------------------>\n", i);
		DUMPREG(DE_VIDEO_SR(i));
		DUMPREG(DE_VIDEO_SCOEF(i, 0));
		DUMPREG(DE_VIDEO_SCOEF(i, 1));
		DUMPREG(DE_VIDEO_SCOEF(i, 2));
		DUMPREG(DE_VIDEO_SCOEF(i, 3));
		DUMPREG(DE_VIDEO_SCOEF(i, 4));
		DUMPREG(DE_VIDEO_SCOEF(i, 5));
		DUMPREG(DE_VIDEO_SCOEF(i, 6));
		DUMPREG(DE_VIDEO_SCOEF(i, 7));
	}
#undef DUMPREG
}

static struct owl_de_device_ops de_ats3605_device_ops = {
	.power_on = de_ats3605_device_power_on,
	.init = de_ats3605_device_init,
	.dump_regs = de_ats3605_device_dump_regs,
};

static struct owl_de_device owl_de_ats3605 = {
	.hw_id			= DE_HW_ID_ATS3605,

	.num_paths		= 2,
	.paths			= de_ats3605_paths,

	.num_videos		= 2,
	.videos			= de_ats3605_videos,

	.ops			= &de_ats3605_device_ops,
};


/*============================================================================
 *			register to DE core
 *==========================================================================*/

int owl_de_ats3605_init(const void *blob)
{
	int node;

	node = fdt_node_offset_by_compatible(blob, 0, "actions,ats3605-de");
	if (node < 0) {
		debug("%s: no match in DTS\n", __func__);
		return -1;
	}
	debug("%s\n", __func__);

	owl_de_ats3605.blob = blob;
	owl_de_ats3605.node = node;

	owl_de_register(&owl_de_ats3605);

	return 0;
}
