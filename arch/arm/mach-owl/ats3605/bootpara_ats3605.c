/*
 * Copyright (c) 2015 Actions Semi Co., Ltd.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <fdtdec.h>
#include <errno.h>
#include <fs.h>
#include <asm/io.h>
#include <asm/arch/regs.h>
#include <asm/arch/clk.h>
#include <asm/arch/sys_proto.h>
#include <power/atc260x/owl_atc260x.h>

DECLARE_GLOBAL_DATA_PTR;

#define OWL_BOOTDEV_NAND       (0x00)
#define OWL_BOOTDEV_SD0        (0x20)
#define OWL_BOOTDEV_SD1        (0x21)
#define OWL_BOOTDEV_SD2        (0x22)
#define OWL_BOOTDEV_SPI        (0x40)



/*bootloader para*/
#define BOOT_PARAM_LOAD_ADDR  (0xb4068000)
#define OWL_BOOTPARAM_MAGIC		0x49464121	/* '!AFI' */
struct owl_boot_param {
	uint32_t magic;
	uint32_t reserved[14];
	uint32_t dvfs;
	uint32_t reserved1[2];
	uint32_t bootdev;
	uint32_t serial_no_lo;
	uint32_t serial_no_hi;
} ;

#ifdef CONFIG_CHECK_UPDATE



const static char *ota_path = "update.zip";
const static char *ota_fin_flag ="ota_fin.txt";
const static char *env_addr[3] = {"kernel_addr_r", "fdt_addr_r",  "ramdisk_addr_r"};
const char *boot_fname[3]= {"uImage",  "kernel.dtb", "ota_initrafms.img"};

#define OTA_MAGIC      (0x4F544150)


struct ota_file
{
	char name[32];
	unsigned char boot;
	unsigned char parti;
	unsigned short res;
	unsigned int size;
	unsigned int offset;
	unsigned int file_checksum;
	char     soft_ver[16];
};


struct ota_pack_header
{
    unsigned int magic;
    char  ota_ver[32];
    unsigned char Reserve1[24];
    unsigned int numPart;
    struct ota_file  file[30];
    unsigned char Reserve2[60];
    unsigned int checksum;
};

struct ota_pack_header  ota_head;


static unsigned int dwchecksum(unsigned int * buf, int start, int length)
{
    unsigned int sum = 0;
    int tmp_start = start / sizeof(unsigned int);
    int tmp_length = length / sizeof(unsigned int);
    int i;
    for(i = tmp_start; i < tmp_length; i++)
        sum = sum + buf[i];
    return sum;
}

static unsigned int get_env_addr(const char * env_addr)
{
	char *penv;
	unsigned int  addr;
	penv = getenv(env_addr);
	if (penv == NULL) {
		printf("ota: env addr  %s is not exist\n", env_addr);
		return 0;
	}
	addr = simple_strtoul (penv, NULL, 16);
	printf("ota: env=%s, addr=0x%x\n", env_addr, addr);
	return addr;
}

static int check_ota_update(void)
{
#if 0
	unsigned int   addr;
	loff_t size , len;
	int ret;
	char *pbuf, seril_no[20];

	fs_set_blk_dev("mmc", "0", FS_TYPE_FAT);
	if (fs_size(ota_fin_flag, &size) < 0)
		return  1;
	addr = get_env_addr(env_addr[0]);
	fs_set_blk_dev("mmc", "0", FS_TYPE_FAT);
	ret =  fs_read(ota_fin_flag , addr,  0 ,  size, &len);
	if ( ret < 0 )
		return 1;
	pbuf = (char *)addr;
	pbuf[size] = 0;
	ats3605_get_serial(seril_no, 19);
	printf("ota check, rlen=0x%llx: %s, %s\n", len, seril_no, pbuf);
	if (strstr(pbuf, seril_no)  !=  NULL )
		return 0;
	return 1;
#endif
	return 1;
}

static unsigned int ota_uppack(const char *image_name, const char *env_addr)
{
	int  i, ret;
	unsigned int ck, addr;

	struct ota_file *pf = NULL;
	loff_t  len;

	printf("ota uppack: image=%s, env_addr=%s\n", image_name, env_addr);
 	for (i = 0; i < ota_head.numPart; i++) {
		if (0 == strcmp(image_name, ota_head.file[i].name)) {
			pf = &ota_head.file[i];
			break;
		}
	}
	if (pf == NULL) {
		printf("ota: image %s is not exist\n", image_name);
		return -1;
	}
	addr  = get_env_addr(env_addr);
	if (addr == 0) {
		printf("ota: env addr  %s is not exist\n", env_addr);
		return -1;
	}

	fs_set_blk_dev("mmc", "0", FS_TYPE_FAT);
	ret =  fs_read(ota_path , addr,  pf->offset*512 ,  pf->size*512, &len);
 	if (ret   <  0) {
		printf("read iamge fail %d \n", ret);
		return -1;
 	}
	printf("file %s  size = 0x%llx\n", image_name,  len);
	ck =  dwchecksum((unsigned int*)addr, 0, pf->size*512);
	if (ck != pf->file_checksum) {
		printf("checksum err: ck=0x%x, sck=0x%x \n", ck,  pf->file_checksum);
		return -1;
	}
	return 0;

}
static void dump_mem(unsigned char * buf, int size )
{
    int i;
    for (i = 0; i < size; i++ )
    {
        printf("%02x ",*(buf + i));
        if (i % 16 == 15)
        {
            printf("\n");
        }
    }
}
static int ota_read_image(void)
{

	loff_t  len;
	int ret, i;
	unsigned int ck;

	fs_set_blk_dev("mmc", "0", FS_TYPE_FAT);
	ret = fs_read(ota_path , ( ulong)  (&ota_head),  0 , 0x800, &len);
 	if (ret   <  0) {
		printf("ota: head read fail, %d\n", ret);
		return -1;
 	}
	if(ota_head.magic != OTA_MAGIC) {
		printf("ota:  magic err \n");
		return -1;
	}
	printf("head len  size = 0x%llx\n",  len);
	ck =  dwchecksum((unsigned int *)(&ota_head), 0, 0x800-4);
	if (ota_head.checksum != ck){
		printf("ota head checksum err: ck=0x%x, sck=0x%x \n", ck, ota_head.checksum);
		dump_mem((unsigned char *)(&ota_head),  0x800);
		//return -1;
	}
	for (i = 0; i < 3; i++) {
		ret = ota_uppack(boot_fname[i] , env_addr[i]);
		if (ret < 0 )
			break;
	}
	return ret;
}

int check_app_ota_flag(void)
{
	int ret;
	unsigned int recovery_mode;
	ret = atc260x_pstore_get(ATC260X_PSTORE_TAG_REBOOT_RECOVERY,
				 &recovery_mode);
	//printf("check_app_ota_flag ret = %d,recovery_mode = %d\n", ret,  recovery_mode);
	if (ret || recovery_mode == 0) 
		return 0;

 	atc260x_pstore_set(ATC260X_PSTORE_TAG_REBOOT_RECOVERY, 0);
	if (recovery_mode == 1) {
		printf("PMU ota flag founded!\n");
		return 1;
	} else if (recovery_mode == 2) {
		atc260x_pstore_set(ATC260X_PSTORE_TAG_REBOOT_RECOVERY, 0);
		printf("bootloader flag founded!\n");
		setenv("owlbootcmdline", SET_ENTER_BOOT_CMDLINE);
		return -1;
	} else {
		printf("update reboot, not check update package\n");
		return -1;
	}

}

int check_ota_package(void)
{
	loff_t size = 0;
	int ota_index;
	ota_index = check_app_ota_flag();
	if (ota_index < 0)
		return -1;
#ifdef SLOT0
	if (!owl_mmc_check_init(SLOT0) )
		return -1;
#endif

	printf("====check_ota_package =%d=====\n", ota_index);
	if (fs_set_blk_dev("mmc", "0", FS_TYPE_FAT))
		return -1;

	if (fs_size(ota_path, &size) < 0)
		return -1;

	if (size < 1024 * 2)
		return -1;
	printf("file  size = 0x%llx\n", size);
	if (check_ota_update() == 0)
		return -1;

	if ( ota_read_image() < 0 ) {
		printf("ota package err\n");
		return -1;
	}
	printf("ota package is ok, run ramboot\n");
	if (ota_index == 1) // OTA update
		kernel_cmdline_add("androidboot.mode=recovery");
	setenv("bootcmd", "run ramboot;");
	return 0;

}

#endif


void ats3605_bootpara_init(void)
{
	char *bootargs_env;
	char new_env[CONFIG_SYS_BARGSIZE];
	char str_dvfs[32];
	struct owl_boot_param *param;
	param = (struct  owl_boot_param *)BOOT_PARAM_LOAD_ADDR;
	if(param->magic != OWL_BOOTPARAM_MAGIC) {
		printf("bootpara magic err:0x%x\n", param->magic);
		return;
	}
	//printf("bootpara dvfs:0x%x\n", param->dvfs);
	snprintf(str_dvfs, 31, "dvfslevel=0x%x", param->dvfs);
	bootargs_env = getenv("bootargs.preadd");
	if (bootargs_env != NULL) {
		snprintf(new_env, CONFIG_SYS_BARGSIZE, "%s %s",
			 bootargs_env, str_dvfs);
	} else {
		strcpy(new_env, str_dvfs);
	}

	// add bootdev

	if (param->bootdev == OWL_BOOTDEV_SPI)  {
		//printf("bootdev=nor\n");
		strcat(new_env, "  bootdev=nor");
	} else if (param->bootdev == OWL_BOOTDEV_NAND) {
		strcat(new_env, "  bootdev=nand");
	} else {
		strcat(new_env, "  bootdev=sd");
	}

	setenv("bootargs.preadd", new_env);

#ifdef CONFIG_CHECK_UPDATE
#ifdef CONFIG_HARD_POWERKEY
	if(hardkey_check_on() )
#endif
		check_ota_package();
#endif

}

void ats3605_get_serial(char *buf,  int len)
{
	struct owl_boot_param *param;
	param = (struct  owl_boot_param *)BOOT_PARAM_LOAD_ADDR;
	snprintf(buf, len, "%.8x%.8x", param->serial_no_hi , param->serial_no_lo);
}


