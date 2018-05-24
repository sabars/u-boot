/*
 * Copyright (c) 2015 Actions Semi Co., Ltd.
 *
 * Lipeng<lipeng@actions-semi.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef __DE_ATS3605_H_
#define __DE_ATS3605_H_

/*================================================================
 *           Definition of registers and bit position
 *==============================================================*/

#define DE_SCLCOEF_ZOOMIN			0
#define DE_SCLCOEF_HALF_ZOOMOUT			1
#define DE_SCLCOEF_SMALLER_ZOOMOUT		2

/*
 * common registers
 */
#define DE_IRQENABLE				0x0000
#define DE_IRQSTATUS				0x0004

/*
 * path registers
 */
#define DE_PATH_BASE				0x0100

#define DE_PATH_CTL(n)				(DE_PATH_BASE + (n) * 0x040 + 0x000)
 #define DE_PATH_VIDEO_ENABLE_BIT		0
 #define DE_PATH_DITHER_ENABLE_BIT		5
 #define DE_PATH_CTL_INTERLACE_BIT		9

#define DE_PATH_FCR(n)				(DE_PATH_BASE + (n) * 0x040 + 0x004)
 #define DE_PATH_FCR_BIT			0

#define DE_PATH_EN(n)				(DE_PATH_BASE + (n) * 0x040 + 0x008)
 #define DE_PATH_ENABLE_BIT			0

#define DE_PATH_BK(n)				(DE_PATH_BASE + (n) * 0x040 + 0x00c)

#define DE_PATH_SIZE(n)				(DE_PATH_BASE + (n) * 0x040 + 0x010)
 #define DE_PATH_SIZE_WIDTH		(1 << 12)
 #define DE_PATH_SIZE_HEIGHT		(1 << 12)
 #define DE_PATH_SIZE_WIDTH_BEGIN_BIT	0
 #define DE_PATH_SIZE_WIDTH_END_BIT	11
 #define DE_PATH_SIZE_HEIGHT_BEGIN_BIT	16
 #define DE_PATH_SIZE_HEIGHT_END_BIT	27

#define DE_PATH_A_COOR(n)			(DE_PATH_BASE + (n) * 0x040 + 0x014)

#define DE_PATH_GAMMA_IDX(n)			(DE_PATH_BASE + (n) * 0x040 + 0x028)
 #define DE_PATH_GAMMA_IDX_BUSY_BIT		(14)
 #define DE_PATH_GAMMA_IDX_OP_SEL_BEGIN_BIT	(12)
 #define DE_PATH_GAMMA_IDX_OP_SEL_END_BIT	(13)
 #define DE_PATH_GAMMA_IDX_INDEX_BEGIN_BIT	(0)
 #define DE_PATH_GAMMA_IDX_INDEX_END_BIT	(7)

#define DE_PATH_GAMMA_ENABLE(n)		DE_PATH_CTL((n))
 #define DE_PATH_GAMMA_ENABLE_BIT	6

#define DE_PATH_GAMMA_RAM(n)			(DE_PATH_BASE + (n) * 0x040 + 0x02c)

/*
 * video register, include 2 video layers
 */
#define DE_VIDEO_BASE			0x0220

#define DE_VIDEO_CFG(x)			(DE_VIDEO_BASE + (x) * 0x080  +  0x0000)

#define DE_VIDEO_ISIZE(x)		(DE_VIDEO_BASE + (x) * 0x080 + 0x0004)
#define DE_VIDEO_OSIZE(x)		(DE_VIDEO_BASE + (x) * 0x080 + 0x0008)
#define DE_VIDEO_SR(x)			(DE_VIDEO_BASE + (x) * 0x080 + 0x000c)
#define DE_VIDEO_SCOEF(x, y)		(DE_VIDEO_BASE + (x) * 0x080 + 0x0010 + (y) * 0x04)

#define DE_VIDEO_FB(x, y)		(DE_VIDEO_BASE + (x) * 0x080 + 0x0030 + (y) * 0x04)
#define DE_VIDEO_FB_RIGHT(x, y)		(DE_VIDEO_BASE + (x) * 0x080 + 0x003c + (y) * 0x04)

#define DE_VIDEO_STR(x)			(DE_VIDEO_BASE + (x) * 0x080 + 0x0048)

#define DE_VIDEO_CRITICAL_CFG(x)	(DE_VIDEO_BASE + (x) * 0x080 + 0x004c)

#define DE_VIDEO_REMAPPING(x)		(DE_VIDEO_BASE + (x) * 0x080 + 0x0050)

#define	DE_OUTPUT_CON			(0x4000)
#define	DE_OUTPUT_STAT			(0x402c)
#define	DE_WB_CON			(0x4070)
#define DE_WB_ADDR			(0x4074)
#define DE_WB_CNT			(0x4078)

/*
 * scaler registers
 */
#endif	/* __DE_ATS3605_H_ */
