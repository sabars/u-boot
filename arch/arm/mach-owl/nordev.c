/*
 * Copyright (c) 2015 Actions Semi Co., Ltd.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <spi_flash.h>
#include <common.h>
#include <malloc.h>
#include <part.h>
#include <asm/arch/sys_proto.h>

//for dts
#include <config.h>
#include <libfdt.h>
#include <fdtdec.h>
//
DECLARE_GLOBAL_DATA_PTR;

#define NOR_MAX_DEV_NUM	3

static int nor_initialized;

struct owl_block_dev {
	uint32_t off;
	int ptn;
	block_dev_desc_t blk_dev;   /* uboot/include/part.h */
};

static struct owl_block_dev nor_dev[NOR_MAX_DEV_NUM];
static struct spi_flash *flash = NULL;
static uint64_t system_offset = 0;
static uint64_t system_size = 0;

int nor_sync(void)
{
	return 0;
}

int get_cfg_by_name(const char *name)
{
	int node = -1;
	int value = -1;
	if(name == NULL){
		printf("%s, name is NULL\n", __func__);
		return -1;
	}
	
	node = fdt_node_offset_by_compatible(gd->fdt_blob, 0, "actions,s700-nor");
	if (node < 0) {
		printf("%s no match in dts\n", __func__);
		return -1;
	}

	value = fdtdec_get_int(gd->fdt_blob, node, name, -1);
	
	return value;
}

static unsigned long owl_nor_block_read(int dev,
			unsigned long start,
			lbaint_t blkcnt,
			void *buffer)
{
    int ret;
	struct owl_block_dev *blkdev = &nor_dev[dev];

	if (!nor_initialized) {
		printf("nor has not been initialized.\n");
		return -1;
	}

	if (dev >= NOR_MAX_DEV_NUM) {
		printf("%s partion %d not initialized.\n", __func__, dev);
		return -1;
	}

	if (blkdev->ptn <= 0) {
		printf("%s partion %d is physical.\n", __func__, dev);
		return -1;
	}

	if (blkcnt == 0)
		return 0;

	if (start >= blkdev->blk_dev.lba ||
	    (start + blkcnt) > blkdev->blk_dev.lba ||
	    (start + blkcnt) <= start ||
	    buffer == NULL) {
		printf("%s: invalid param, dev %d, start %lu, blkcnt: %u, "
		       "buffer: 0x%p, lba is %u\n",
		       __func__, dev, start, (unsigned)blkcnt, buffer,
		       (unsigned)blkdev->blk_dev.lba);
		return -1;
	}

    if (flash != NULL) {
	    ret = spi_flash_read(flash, start * 512 + system_offset, blkcnt * 512, buffer);
	    if(ret != 0){
	    	printf("spi_flash_read failed! %d\n", ret);
	    	return ret;
	    }
    }

	return blkcnt;
}

static unsigned long owl_nor_block_write(int dev,
				      unsigned long start,
				      lbaint_t blkcnt,
				      const void *buffer)
{
    int ret;
	struct owl_block_dev *blkdev = &nor_dev[dev];

	if (!nor_initialized) {
		printf("nor has not been initialized.\n");
		return -1;
	}

	if (dev >= NOR_MAX_DEV_NUM) {
		printf("%s partion %d not initialized.\n", __func__, dev);
		return -1;
	}

	if (blkdev->ptn <= 0) {
		printf("%s partion %d is physical.\n", __func__, dev);
		return -1;
	}

	if (blkcnt == 0)
		return 0;

	if (start >= blkdev->blk_dev.lba ||
	    (start + blkcnt) > blkdev->blk_dev.lba ||
	    (start + blkcnt) <= start ||
	    buffer == NULL) {
		printf("%s: invalid param, dev %d, start %lu, blkcnt: %u, "
		       "buffer: 0x%p, lba is %u\n",
		       __func__, dev, start, (unsigned)blkcnt, buffer,
		       (unsigned)blkdev->blk_dev.lba);
		return -1;
	}

    if (flash != NULL) {
	    ret = spi_flash_write(flash, start * 512 + system_offset, blkcnt * 512, buffer);
	    if(ret != 0){
	    	printf("spi_flash_write failed! %d\n", ret);
	    	return ret;
	    }
    }
	return blkcnt;
}

int RegisterOneDevice(void)
{
	unsigned int offset;
	struct owl_block_dev *blkdev;

	blkdev = &nor_dev[2];
	blkdev->blk_dev.if_type = -1;
	blkdev->blk_dev.dev = 2;
	blkdev->off = 0;

	blkdev->ptn = 3;
	blkdev->blk_dev.lba = system_size / 512;
	blkdev->blk_dev.blksz = 512;
	blkdev->blk_dev.block_read = owl_nor_block_read;
	blkdev->blk_dev.block_write = owl_nor_block_write;
	blkdev->blk_dev.part_type = PART_TYPE_EFI;

	return 0;
}

int parse_partition(uint64_t *offset, uint64_t *size)
{
	fdt_addr_t addr;
	fdt_size_t sz;
    int node;

	node = fdt_node_offset_by_compatible(gd->fdt_blob, 0, "m25p80");
	if (!node)
		return -1;
    addr = fdtdec_get_addr_size(gd->fdt_blob, node, "system-partition", &sz);
    *offset = addr;
    *size = sz;

	return 0;
}

int nor_init(void)
{
	#ifdef CONFIG_BOOTDEV_AUTO
	if (owl_get_bootdev() != BOOTDEV_NOR)
		return 0;
	#endif

	debug("init owl NOR\n");

	/* nor_dev array init */
	memset(&nor_dev, 0, sizeof(nor_dev));

	nor_initialized = 1;

    flash = spi_flash_probe(0, 0, 60000000, (0x2|0x1));
	if (!flash) {
		printf("SPI probe failed.\n");
		return -1;
	}

    if (parse_partition(&system_offset, &system_size) == 0)
        printf("get system_offset = 0x%llx; system_size = 0x%llx.\n", system_offset, system_size);

	if (RegisterOneDevice() < 0)
		goto exit;

	return 0;
exit:
	printf("init nor error.\n");
	return -1;
}

void nor_uninit(void)
{
	if (nor_initialized) {
		printf("freeze owl NOR..\n");
        spi_flash_free(flash);
	}
}

#ifdef CONFIG_PARTITIONS
block_dev_desc_t *owl_nor_get_dev(int dev)
{
	struct owl_block_dev *blkdev;

	if (dev >= NOR_MAX_DEV_NUM) {
		printf("ERROR dev num %d.\n", dev);
		return NULL;
	}

	blkdev = &nor_dev[dev];
	if (blkdev->blk_dev.dev != dev) {
		printf("dev %d not init.\n", dev);
		return NULL;
	}

	return &blkdev->blk_dev;
}
#endif
