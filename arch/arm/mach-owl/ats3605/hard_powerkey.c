
#include <common.h>
#include <fdtdec.h>
#include <asm/io.h>
#include <asm/arch/regs.h>
#include <asm/arch/clk.h>
#include <asm-generic/gpio.h>
#include <power/atc260x/owl_atc260x.h>

DECLARE_GLOBAL_DATA_PTR;


struct hardkey_info {
	struct gpio_desc cpu_gpio;
	int pmu_sgpio;
	int spgio_ative;
	int is_config;
};

static struct hardkey_info key_info;


/*return -1 is not config, 1 is on, 0 is off*/
int hardkey_check_on(void)
{
	int ret;
	if (key_info.is_config == 0)
		return -1;
	if (key_info.pmu_sgpio == -1) {
		ret = dm_gpio_get_value(&key_info.cpu_gpio);
		printf("gpiokey =%d\n", ret);
		return ret;
	} else {

	}
	return 1;
}

int hardkey_init(void)
{
	int node;
	int ret;
	u32 val[2];
	key_info.is_config = 0;
	key_info.pmu_sgpio = -1;
	node = fdt_node_offset_by_compatible(gd->fdt_blob, 0, "actions,hard_powerkey");
	if (node < 0) {
		printf("%s no match in dts, node:%d\n", __func__, node);
		return -1;
	}
	ret = gpio_request_by_name_nodev(gd->fdt_blob, node, "cpu_gpios", 0,
				       &key_info.cpu_gpio, GPIOD_IS_IN);
	if (ret != 0) {
		printf("%s cpu_gpios ont config\n", __func__);
		ret = fdtdec_get_int_array(gd->fdt_blob, node, "pmu_sgpio",val, 2);
		if (ret != 0) {
			printf("%s pmu_sgpio ont config\n", __func__);
			return -1;
		}
		if(val[0] > 6 ) {
			printf("pmu_sgpio config fail %d\n", val[0]);
			return -1;
		}
		printf("pmu_sgpio=%d, atcitve=%d\n", val[0], val[1]);
		key_info.pmu_sgpio = val[0];
		key_info.spgio_ative = val[1];
	}
	key_info.is_config = 1;
	return 0;
}
