/*
 * (C) Copyright 2017 Actions co.ltd
 * Yiguang <liuyiguang@actions-semi.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <errno.h>
#include <malloc.h>
#include <spi.h>
#include <asm/arch/periph.h>
#include <fdtdec.h>
#include <asm/arch/clk.h>
#include <asm/arch/regs.h>
#include <asm/arch/gpio.h>
#include <asm/arch/pinmux.h>
#include <asm/io.h>

DECLARE_GLOBAL_DATA_PTR;

/* SPI controller registers */
#define SPI_CTL				    0x00
#define SPI_CLKDIV			    0x04
#define SPI_STAT			    0x08
#define SPI_RXDAT			    0x0C
#define SPI_TXDAT			    0x10
#define SPI_TCNT			    0x14
#define SPI_SEED			    0x18
#define SPI_TXCR			    0x1C
#define SPI_RXCR			    0x20

/* SPI_CTL */
#define SPI_CTL_SDT_MASK		(0x7 << 29)
#define SPI_CTL_SDT(x)			(((x) & 0x7) << 29)
#define SPI_CTL_BM				(0x1 << 28)
#define SPI_CTL_GM				(0x1 << 27)
#define SPI_CTL_CEB				(0x1 << 26)
#define SPI_CTL_RANEN(x)		(0x1 << 24)
#define SPI_CTL_RDIC_MASK		(0x3 << 22)
#define SPI_CTL_RDIC(x)			(((x) & 0x3) << 22)
#define SPI_CTL_TDIC_MASK		(0x3 << 20)
#define SPI_CTL_TDIC(x)			(((x) & 0x3) << 20)
#define SPI_CTL_TWME			(0x1 << 19)
#define SPI_CTL_EN				(0x1 << 18)
#define SPI_CTL_RWC_MASK		(0x3 << 16)
#define SPI_CTL_RWC(x)			(((x) & 0x3) << 16)
#define SPI_CTL_DTS			    (0x1 << 15)
#define SPI_CTL_SSATEN			(0x1 << 14)
#define SPI_CTL_DM_MASK			(0x3 << 12)
#define SPI_CTL_DM(x)			(((x) & 0x3) << 12)
#define SPI_CTL_LBT			    (0x1 << 11)
#define SPI_CTL_MS			    (0x1 << 10)
#define SPI_CTL_DAWS_MASK		(0x3 << 8)
#define SPI_CTL_DAWS(x)			(((x) & 0x3) << 8)
#define		SPI_CTL_DAWS_8BIT	(SPI_CTL_DAWS(0))
#define		SPI_CTL_DAWS_16BIT	(SPI_CTL_DAWS(1))
#define		SPI_CTL_DAWS_32BIT	(SPI_CTL_DAWS(2))
#define SPI_CTL_CPOS_MASK		(0x3 << 6)
#define SPI_CTL_CPOS(x)			(((x) & 0x3) << 6)
#define		SPI_CTL_CPOS_CPHA	(0x1 << 7)
#define		SPI_CTL_CPOS_CPOL	(0x1 << 6)
#define SPI_CTL_LMFS			(0x1 << 5)
#define SPI_CTL_SSCO			(0x1 << 4)
#define SPI_CTL_TIEN			(0x1 << 3)
#define SPI_CTL_RIEN			(0x1 << 2)
#define SPI_CTL_TDEN			(0x1 << 1)
#define SPI_CTL_RDEN			(0x1 << 0)

/* SPI_CLKDIV */
#define SPI_CLKDIV_CLKDIV_MASK	(0x3FF << 0)
#define SPI_CLKDIV_CLKDIV(x)	(((x) & 0x3FF) << 0)

/*SPI_STAT*/
/*bit 10-31 Reserved*/
#define SPI_STAT_TFEM			(0x1 << 9)
#define SPI_STAT_RFFU			(0x1 << 8)
#define SPI_STAT_TFFU			(0x1 << 7)
#define SPI_STAT_RFEM			(0x1 << 6)
#define SPI_STAT_TFER			(0x1 << 5)
#define SPI_STAT_RFER			(0x1 << 4)
#define SPI_STAT_BEB			(0x1 << 3)
#define SPI_STAT_TCOM			(0x1 << 2)
#define SPI_STAT_TIP			(0x1 << 1)
#define SPI_STAT_PIP			(0x1 << 0)

#define MAX_SPI_POLL_LOOPS      5000
#define MAX_SPI_DMA_LEN			15360

#define BDMA_BASE               0xb0220100
#define BDMA_MODE               0x0
#define BDMA_SRC                0x4
#define BDMA_DST                0x8
#define BDMA_CNT                0xc
#define BDMA_REM                0x10
#define BDMA_CMD                0x14
#define BDMA_CACHE              0x18

#define DMA_CHANNEL             2

#define DMA_MODE_ST(x)			(((x) & 0x1f) << 0)
#define		DMA_MODE_ST_DDR		DMA_MODE_ST(18)
#define DMA_MODE_DT(x)			(((x) & 0x1f) << 16)
#define		DMA_MODE_DT_DDR		DMA_MODE_DT(18)
#define DMA_MODE_SAM(x)			(((x) & 0x1) << 6)
#define		DMA_MODE_SAM_CONST	DMA_MODE_SAM(1)
#define		DMA_MODE_SAM_INC	DMA_MODE_SAM(0)
#define DMA_MODE_DAM(x)			(((x) & 0x1) << 22)
#define		DMA_MODE_DAM_CONST	DMA_MODE_DAM(1)
#define		DMA_MODE_DAM_INC	DMA_MODE_DAM(0)

#define STOP_DMA(dmaNo)            \
{ \
    writel( readl(BDMA_BASE + (0x30 * dmaNo) + BDMA_CMD) & (~0x1), BDMA_BASE + (0x30 * dmaNo) + BDMA_CMD ); \
}

#define RESET_DMA(dmaNo)    STOP_DMA(dmaNo)

#define START_DMA(dmaNo)            \
{ \
    writel( 0x1, BDMA_BASE + (0x30 * dmaNo) + BDMA_CMD ); \
}

#define SET_DMA_COUNT(dmaNo, count)        \
{ \
    writel(count & 0x1FFFFFF, BDMA_BASE + (0x30 * dmaNo) + BDMA_CNT); \
}

#define SET_DMA_SRC_ADDR(dmaNo, addrForDma)        \
{ \
    writel(addrForDma, BDMA_BASE + (0x30 * dmaNo) + BDMA_SRC); \
}

#define SET_DMA_DST_ADDR(dmaNo, addrForDma)        \
{ \
    writel(addrForDma, BDMA_BASE + (0x30 * dmaNo) + BDMA_DST); \
}

#define SET_DMA_MODE(dmaNo, mode)        \
{ \
    writel(mode, BDMA_BASE + (0x30 * dmaNo) + BDMA_MODE); \
}

struct owl_spi {
	u32 ctl;		/* SPI Control Register */
	u32 clkdiv;		/* SPI Clk Divide Register */
	u32 stat;		/* SPI Status Register */
	u32 rxdat;		/* SPI RX Data Register */
	u32 txdat;		/* SPI TX Data Register */
	u32 tcnt;		/* SPI Data transmit counter */
	u32 seed;		/* SPI Ramdomizer seed */
	u32 txcr;		/* SPI TX DMA counter Register */
	u32 rxcr;		/* SPI RX DMA counter Register */
};

struct owl_spi_platdata {
	enum periph_id periph_id;
	s32 frequency;		/* Default clock frequency, -1 for none */
	int	mfp_config;
	struct owl_spi *regs;
};

struct owl_spi_priv {
	enum periph_id periph_id;
	int	speed;
    int bits_per_word;
	struct owl_spi *regs;
	int	mfp_config;
	unsigned int freq;		/* Default frequency */
	unsigned int mode;
	u32 rx_irq;		/* DMA IRQ */
    dma_addr_t rx_addr;
    dma_addr_t tx_addr;
    unsigned int rx_size;
    unsigned int tx_size;
    void *rx_buf;
    void *tx_buf;
	struct owl_dma_dev *rx_channel;
	struct owl_dma_dev *tx_channel;
    void *tmp1_buf;
    dma_addr_t tmp1_addr;
    void *tmp2_buf;
    dma_addr_t tmp2_addr;
};

static int owl_spi_ofdata_to_platdata(struct udevice *bus)
{
	struct owl_spi_platdata *plat = bus->platdata;
	const void *blob = gd->fdt_blob;
	int node = bus->of_offset;

	plat->periph_id = PERIPH_ID_SPI0 + bus->seq;
	plat->regs = (struct owl_spi *)fdtdec_get_addr(blob, node, "reg");
	/* Use 10MHz as a suitable default */
	plat->frequency = fdtdec_get_int(blob, node, "spi-max-frequency",
					10000000);

	/*pinmux config*/
	plat->mfp_config = fdtdec_get_int(blob, node, "mfp", 0);

	return 0;
}

static void owl_spi_set_freq(struct owl_spi_priv *priv)
{
	u32 spi_source_clk_hz;
	u32 clk_div;

	/* 2MHz by default */
	if (priv->speed == 0)
		priv->speed = 30000000;

	spi_source_clk_hz = 120000000;

	/* setup SPI clock register */
	clk_div = (spi_source_clk_hz + (2 * priv->speed) - 1) / (priv->speed) / 2;
	if (clk_div == 0)
		clk_div = 1;

	/* SPICLK = HCLK/(CLKDIV*2) */
	writel(SPI_CLKDIV_CLKDIV(clk_div), &priv->regs->clkdiv);
}

static void owl_spi_init_hw(struct owl_spi_priv *priv)
{
	owl_clk_enable_by_perip_id(priv->periph_id);
	owl_reset_by_perip_id(priv->periph_id);
	pinmux_select(priv->periph_id, priv->mfp_config);

    writel(0x1, &priv->regs->clkdiv);
	writel(0x00c0, &priv->regs->ctl);         /* select SPI 8bit mode 3 */
	writel(0xFFFFFFFF, &priv->regs->stat);    /* clear SPI status register */
	setbits_le32(&priv->regs->ctl, (0x1 << 18));
	setbits_le32(&priv->regs->ctl, (0x1 << 26));/* convert Endian*/
	setbits_le32(&priv->regs->ctl, (0x1 << 29));/* delay 1 cycle time */
}

static int owl_spi_probe(struct udevice *bus)
{
	struct owl_spi_platdata *plat = dev_get_platdata(bus);
	struct owl_spi_priv *priv = dev_get_priv(bus);

    priv->regs = plat->regs;

	priv->freq = plat->frequency;
	priv->periph_id = plat->periph_id;
	priv->mfp_config = plat->mfp_config;

    owl_spi_init_hw(priv);

	if (!priv->bits_per_word)
		priv->bits_per_word = 8;

	priv->rx_channel = owl_dma_request();
	if (!(priv->rx_channel)) {
		printf("!!!err:owl_dma_request\n");
		return -1;
	}
    priv->rx_irq = 7; //DMA_DRQ_SPI0 = 7
	return 0;
}

static int spi_cs_activate(struct owl_spi_priv *priv)
{
    clrbits_le32(&priv->regs->ctl, 0x10);

	return 0;
}

static int spi_cs_deactivate(struct owl_spi_priv *priv)
{
    setbits_le32(&priv->regs->ctl, 0x10);

	return 0;
}

static int owl_spi_claim_bus(struct udevice *dev)
{
    return 0;
}

static int owl_spi_release_bus(struct udevice *dev)
{
	return 0;
}

static inline void spi_clear_stat(struct owl_spi_priv *priv)
{
	writel(SPI_STAT_TFER	/* clear the rx FIFO */
		| SPI_STAT_RFER	/* clear the tx FIFO */
		| SPI_STAT_BEB	/* clear the Bus error bit */
		| SPI_STAT_TCOM	/* clear the transfer complete bit */
		| SPI_STAT_TIP	/* clear the tx IRQ pending bit */
		| SPI_STAT_PIP,	/* clear the rx IRQ pending bit */
		&priv->regs->stat);
}

static void owl_spi_dump_regs(struct owl_spi_priv *priv)
{
	printf("dump phys regs:\n"
		"  ctl:      %.8x  clkdiv: %.8x  stat:    %.8x\n",
		readl(&priv->regs->ctl),
		readl(&priv->regs->clkdiv),
		readl(&priv->regs->stat));
}

static inline int owl_spi_wait_tcom(struct owl_spi_priv *priv)
{
	int i;

	for (i = 0; i < MAX_SPI_POLL_LOOPS; i++) {
		if (readl(&priv->regs->stat) & SPI_STAT_TCOM) {
			writel(readl(&priv->regs->stat) | SPI_STAT_TCOM, &priv->regs->stat);
			return 1;
		}
	}
	
	return -1;
}

static int owl_spi_write_read_8bit(struct owl_spi_priv *priv, unsigned int bitlen,
        const void *dout, void *din)
{
	const u8 *tx_buf = dout;
	u8 *rx_buf = din;
	unsigned int count = bitlen / 8;

    unsigned int val;
    //stat must be clear first
    writel(0x30, &priv->regs->stat);
    clrbits_le32(&priv->regs->ctl, 0x300);
    val = readl(&priv->regs->ctl),
	val &= (~(SPI_CTL_RWC(3) |	SPI_CTL_RDIC(3) | SPI_CTL_TDIC(3) | SPI_CTL_SDT(7) |
			  SPI_CTL_DTS | SPI_CTL_TIEN | SPI_CTL_RIEN | SPI_CTL_TDEN | SPI_CTL_RDEN) \
            | SPI_CTL_DAWS(3));
	val |= SPI_CTL_SDT(1);
	writel(val, &priv->regs->ctl);

    if (count <= 5)
        clrbits_le32(&priv->regs->ctl, (0x1 << 12));/* singal wire read mode*/
    else
        setbits_le32(&priv->regs->ctl, (0x1 << 12));/* dual wire read mode */

	do {
		if (tx_buf)
			writel(*tx_buf++, &priv->regs->txdat);
		else
			writel(0, &priv->regs->txdat);

		if (owl_spi_wait_tcom(priv) < 0) {
			return -ETIMEDOUT;
		}

		if (rx_buf)
			*rx_buf++ = readl(&priv->regs->rxdat) & 0xff;

		count -= 1;
	} while (count);
	return count;
}

static int owl_spi_write_read_16bit(struct owl_spi_priv *priv, unsigned int bitlen,
        const void *dout, void *din)
{
	const u16 *tx_buf = dout;
	u16 *rx_buf = din;
	unsigned int count = bitlen / 8;

    unsigned int val;
    //stat must be clear first
    writel(0x30, &priv->regs->stat);
    clrsetbits_le32(&priv->regs->ctl, 0x300, 0x100);

	do {
		if (tx_buf)
			writel(*tx_buf++, &priv->regs->txdat);
		else
			writel(0, &priv->regs->txdat);

		if (owl_spi_wait_tcom(priv) < 0) {
			return -ETIMEDOUT;
		}

		if (rx_buf)
			*rx_buf++ = readl(&priv->regs->rxdat) & 0xffff;

		count -= 2;
	} while (count);

	return 0;
}

static int owl_spi_write_read_32bit(struct owl_spi_priv *priv, unsigned int bitlen,
        const void *dout, void *din)
{
	const u32 *tx_buf = dout;
	u32 *rx_buf = din;
	unsigned int count = bitlen / 8;

    unsigned int val;
    //stat must be clear first
    writel(0x30, &priv->regs->stat);
    clrsetbits_le32(&priv->regs->ctl, 0x300, 0x200);

	do {
		if (tx_buf)
			writel(*tx_buf++, &priv->regs->txdat);
		else
			writel(0, &priv->regs->txdat);

		if (owl_spi_wait_tcom(priv) < 0) {
			return -ETIMEDOUT;
		}

		if (rx_buf)
			*rx_buf++ = readl(&priv->regs->rxdat);

		count -= 4;
	} while (count);

	return count;
}

static void print_spi_regs(struct owl_spi_priv *priv)
{
    printf("SPI0_CTL(0x%x) = 0x%x \n", &priv->regs->ctl, readl(&priv->regs->ctl));
    printf("SPI0_CLKDIV(0x%x) = 0x%x \n", &priv->regs->clkdiv, readl(&priv->regs->clkdiv));
    printf("SPI0_STAT(0x%x) = 0x%x \n", &priv->regs->stat, readl(&priv->regs->stat));
    //Notice: rxdat and txdat can't be read here while in xfer!
    //printf("SPI0_RXDAT(0x%x) = 0x%x \n", &priv->regs->rxdat, readl(&priv->regs->rxdat));
    //printf("SPI0_TXDAT(0x%x) = 0x%x \n", &priv->regs->txdat, readl(&priv->regs->txdat));
    printf("SPI0_TCNT(0x%x) = 0x%x \n", &priv->regs->tcnt, readl(&priv->regs->tcnt));
    printf("SPI0_SEED(0x%x) = 0x%x \n", &priv->regs->seed, readl(&priv->regs->seed));
    printf("SPI0_TXCR(0x%x) = 0x%x \n", &priv->regs->txcr, readl(&priv->regs->txcr));
    printf("SPI0_RXCR(0x%x) = 0x%x \n", &priv->regs->rxcr, readl(&priv->regs->rxcr));
}

static void print_dma_regs()
{
    printf("BDMA%d_MODE(0x%x) = 0x%x \n", DMA_CHANNEL, BDMA_BASE + (0x30 * DMA_CHANNEL) + BDMA_MODE, \
            readl(BDMA_BASE + (0x30 * DMA_CHANNEL) + BDMA_MODE));
    printf("BDMA%d_SRC(0x%x) = 0x%x \n", DMA_CHANNEL, BDMA_BASE + (0x30 * DMA_CHANNEL) + BDMA_SRC, \
            readl(BDMA_BASE + (0x30 * DMA_CHANNEL) + BDMA_SRC));
    printf("BDMA%d_DST(0x%x) = 0x%x \n", DMA_CHANNEL, BDMA_BASE + (0x30 * DMA_CHANNEL) + BDMA_DST, \
            readl(BDMA_BASE + (0x30 * DMA_CHANNEL) + BDMA_DST));
    printf("BDMA%d_CNT(0x%x) = 0x%x \n", DMA_CHANNEL, BDMA_BASE + (0x30 * DMA_CHANNEL) + BDMA_CNT, \
            readl(BDMA_BASE + (0x30 * DMA_CHANNEL) + BDMA_CNT));
    printf("BDMA%d_REM(0x%x) = 0x%x \n", DMA_CHANNEL, BDMA_BASE + (0x30 * DMA_CHANNEL) + BDMA_REM, \
            readl(BDMA_BASE + (0x30 * DMA_CHANNEL) + BDMA_REM));
    printf("BDMA%d_CMD(0x%x) = 0x%x \n", DMA_CHANNEL, BDMA_BASE + (0x30 * DMA_CHANNEL) + BDMA_CMD, \
            readl(BDMA_BASE + (0x30 * DMA_CHANNEL) + BDMA_CMD));
    printf("BDMA%d_CACHE(0x%x) = 0x%x \n", DMA_CHANNEL, BDMA_BASE + (0x30 * DMA_CHANNEL) + BDMA_CACHE, \
            readl(BDMA_BASE + (0x30 * DMA_CHANNEL) + BDMA_CACHE));

}

static int owl_spi_dma_rx(struct owl_spi_priv *priv, unsigned int len, void *din)
{
    int ret = 0;
    int i = 0;
    int left;
	unsigned int val, mode;

    unsigned int dma_len;

	left = len;
	dma_len = left > MAX_SPI_DMA_LEN ? MAX_SPI_DMA_LEN : left;
    if (din) {
	    while(left > 0) {
	    	dma_len = left > MAX_SPI_DMA_LEN ? MAX_SPI_DMA_LEN : left;
	        mode = DMA_MODE_ST(7) | DMA_MODE_DT_DDR | DMA_MODE_SAM_CONST | DMA_MODE_DAM_INC;

            //stat must be clear first
            writel(0x30, &priv->regs->stat);
            //stop the last dma xfer.
            setbits_le32(&priv->regs->stat, 0x4);
            val = readl(&priv->regs->ctl);
	        val &= (~(SPI_CTL_RWC(3) |	SPI_CTL_RDIC(3) | SPI_CTL_TDIC(3) | SPI_CTL_SDT(7) |
	        		  SPI_CTL_DTS | SPI_CTL_TIEN | SPI_CTL_RIEN | SPI_CTL_TDEN | SPI_CTL_RDEN) \
                    | SPI_CTL_DAWS(3));
	        val |= (SPI_CTL_RWC(2) | SPI_CTL_RDIC(3) | SPI_CTL_TDIC(3) | SPI_CTL_SDT(1) |
	        		SPI_CTL_DTS | SPI_CTL_RDEN | SPI_CTL_DAWS(2));
	        writel(val, &priv->regs->ctl);
	        setbits_le32(&priv->regs->ctl, (0x1 << 12));/* dual wire read mode */
            //change len to word.
            writel(dma_len / 4, &priv->regs->tcnt);
            writel(dma_len / 4, &priv->regs->rxcr);

            flush_dcache_range(din+(len-left), din+(len-left) + dma_len);
            SET_DMA_SRC_ADDR(DMA_CHANNEL, &priv->regs->rxdat);
            SET_DMA_DST_ADDR(DMA_CHANNEL, din+(len -left));
            SET_DMA_MODE(DMA_CHANNEL, mode);
            SET_DMA_COUNT(DMA_CHANNEL, dma_len);

            START_DMA(DMA_CHANNEL);

            //wait util dma xfer finish.
            while(readl( BDMA_BASE + (0x30 * DMA_CHANNEL) + BDMA_CMD ));
	    	left -= dma_len;
            STOP_DMA(DMA_CHANNEL);
            clrbits_le32(&priv->regs->ctl, (0x1 << 12));/* cancel dual wire read mode */
        }
    }

    return 0;
}

static int owl_spi_xfer(struct udevice *dev, unsigned int bitlen,
			   const void *dout, void *din, unsigned long flags)
{
	struct udevice *bus = dev->parent;
	struct owl_spi_priv *priv = dev_get_priv(bus);
	int upto, todo;
	int ret = 0;
	unsigned int val;
	unsigned int bytes = bitlen / 8;

	/* spi core configured to do 8 bit transfers */
	if (bitlen <= 0) {
		debug("Non byte aligned SPI transfer.\n");
		goto out;
	}

	/* Start the transaction, if necessary. */
	if ((flags & SPI_XFER_BEGIN))
		spi_cs_activate(priv);

    //if read len larger than 64 bytes and len align 4, use dma read.
    if ((bytes > 64) && (bytes % 4 == 0))
        ret = owl_spi_dma_rx(priv, bytes, din);
    else
	    ret = owl_spi_write_read_8bit(priv, bitlen, dout, din);

out:	
	/* Stop the transaction, if necessary. */
	if (flags & SPI_XFER_END) {
		spi_cs_deactivate(priv);
	}
	return ret;
}

static int owl_spi_set_speed(struct udevice *bus, uint speed)
{
	struct owl_spi_priv *priv = dev_get_priv(bus);

    priv->speed = speed;
	owl_spi_set_freq(priv);

	return 0;
}

static int owl_spi_set_mode(struct udevice *bus, uint mode)
{
	unsigned int val;

	struct owl_spi_priv *priv = dev_get_priv(bus);
	val = readl(&priv->regs->ctl);
	val &= ~(SPI_CTL_CPOS_MASK | SPI_CTL_LMFS | SPI_CTL_LBT |
			SPI_CTL_DAWS_MASK | SPI_CTL_CEB);

	if (mode & SPI_CPOL)
		val |= SPI_CTL_CPOS_CPOL;

	if (mode & SPI_CPHA)
		val |= SPI_CTL_CPOS_CPHA;

	if (mode & SPI_LSB_FIRST)
		val |= SPI_CTL_LMFS;
	else
		val |= SPI_CTL_CEB;

	if (mode & SPI_LOOP)
		val |= SPI_CTL_LBT;

	writel(val, &priv->regs->ctl);

	return 0;
}

static const struct dm_spi_ops owl_spi_ops = {
	.claim_bus	 = owl_spi_claim_bus,
	.release_bus = owl_spi_release_bus,
	.xfer		 = owl_spi_xfer,
	.set_speed	 = owl_spi_set_speed,
	.set_mode	 = owl_spi_set_mode,
};

static const struct udevice_id owl_spi_ids[] = {
	{ .compatible = "actions,ats3605-spi" },
	{ }
};

U_BOOT_DRIVER(owl_spi) = {
	.name	= "owl_spi",
	.id	= UCLASS_SPI,
	.of_match = owl_spi_ids,
	.probe	= owl_spi_probe,
	.ofdata_to_platdata = owl_spi_ofdata_to_platdata,
	.platdata_auto_alloc_size = sizeof(struct owl_spi_platdata),
	.priv_auto_alloc_size = sizeof(struct owl_spi_priv),
	.ops	= &owl_spi_ops,
};
