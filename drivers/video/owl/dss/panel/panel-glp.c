/*
 * OWL GLP(generic LCD/LVDS Panel)
 *
 * Copyright (c) 2015 Actions Semi Co., Ltd.
 *
 * Author: Lipeng<lipeng@actions-semi.com>
 *
 * Change log:
 *	2015/11/30: Created by Lipeng.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#define DEBUGX
#define pr_fmt(fmt) "owl_panel_glp: " fmt

#include <asm/io.h>
#include <malloc.h>
#include <errno.h>

#include <dss.h>

#define MIPI_MAX_PARS 100

struct glp_init_cmd {
	uint8_t		address;
	uint8_t		parameters[MIPI_MAX_PARS];
	uint8_t		n_parameters;
	uint8_t		delay;
};

struct panel_glp_data {
	struct gpio_desc		power_gpio;
	struct gpio_desc		reset_gpio;
	
	/* Specific data can be added here */
	const void 			*blob;
	int 				node;
	struct glp_init_cmd		init_cmd;
};

static int panel_glp_power_on(struct owl_panel *panel)
{
	struct panel_glp_data *glp = panel->pdata;
	
	/* assert  */
	if (dm_gpio_is_valid(&glp->reset_gpio))
		dm_gpio_set_value(&glp->reset_gpio, 1);
	
	/* deassert */
	if (dm_gpio_is_valid(&glp->reset_gpio))
		dm_gpio_set_value(&glp->reset_gpio, 0);

	mdelay(100);
	
	/* assert  */
	if (dm_gpio_is_valid(&glp->reset_gpio))
		dm_gpio_set_value(&glp->reset_gpio, 1);

	udelay(20);
	
	if (dm_gpio_is_valid(&glp->power_gpio))
		dm_gpio_set_value(&glp->power_gpio, 1);

	/* deassert */
	if (dm_gpio_is_valid(&glp->reset_gpio))
		dm_gpio_set_value(&glp->reset_gpio, 0);

	mdelay(200);
	return 0;
}

static int panel_glp_power_off(struct owl_panel *panel)
{
	struct panel_glp_data *glp = panel->pdata;
	if (dm_gpio_is_valid(&glp->power_gpio))
		dm_gpio_set_value(&glp->power_gpio, 0);

	return 0;
}

/*
 * command buffer format:
 *	buffer 3 ~ (MIPI_MAX_PARS - 1) ---> parameters
 *	buffer 2---> address
 *	buffer 1---> number of parameters
 *	buffer 0---> cmd delay
 *
 */
static int panel_glp_enable(struct owl_panel *panel)
{
	struct panel_glp_data *glp = panel->pdata;
	struct owl_display_ctrl *ctrl = panel->ctrl;
	uint8_t *buffer, buffer_size;
	int i, index, len, entry, ret;
	char entryname[64];

	debug("%s\n", __func__);

	index = 0;
	do {
		snprintf(entryname, sizeof(entryname), "glp_init_cmd-%u", index);
		entry = fdtdec_lookup_phandle(glp->blob, glp->node, entryname);
		debug("entry = %d\n", entry);
		if (entry < 0) {
			debug("no etry for %s\n", entryname);
			break;
		} else {
			glp->init_cmd.address = fdtdec_get_int(glp->blob, entry, "address", 0);
			debug("address 0x%x\n",glp->init_cmd.address);

			if (fdt_getprop(glp->blob, entry, "parameters", &len)) {
				ret = fdtdec_get_byte_array(glp->blob, entry, "parameters",
						glp->init_cmd.parameters, len);
				if (ret < 0)
					error("The requested node or property does not exist!\n");
				glp->init_cmd.n_parameters = len;
				debug("parameters lens %d\n", len);
			} else {
				glp->init_cmd.n_parameters = 0;
			}
			
			glp->init_cmd.delay = fdtdec_get_int(glp->blob, entry, "delay", 0);
			debug("delay %d\n",glp->init_cmd.delay);

			index++;
		}

		buffer_size = 3 +glp->init_cmd.n_parameters;
		buffer = malloc(buffer_size);
		if(!buffer) {
			error("malloc buffer failed!\n");
			return -1;
		}

		buffer[0] =glp->init_cmd.delay;
		buffer[1] =glp->init_cmd.n_parameters;
		buffer[2] =glp->init_cmd.address;

		debug("cmd buffer size %d\n", buffer_size);
		debug("delay %d, n_parameters %d, address 0x%x\n",
			buffer[0], buffer[1], buffer[2]);

		if (glp->init_cmd.n_parameters > 0)
			memcpy(&buffer[3], &glp->init_cmd.parameters[0],
				glp->init_cmd.n_parameters);

		/* send glp initail command */
		if (ctrl->ops && ctrl->ops->aux_write)
			ctrl->ops->aux_write(ctrl, (char *)buffer, buffer_size);

		free(buffer);
	
	} while (1);
	
	return 0;
}

static int panel_glp_disable(struct owl_panel *panel)
{
	struct panel_glp_data *glp = panel->pdata;

	return 0;
}



struct owl_panel_ops owl_panel_glp_ops = {
	.power_on = panel_glp_power_on,
	.power_off = panel_glp_power_off,

	.enable = panel_glp_enable,
	.disable = panel_glp_disable,
};

static struct owl_panel owl_panel_glp = {
	.desc = {
		.name = "panel_glp",
		.type = OWL_DISPLAY_TYPE_LCD,
		.ops = &owl_panel_glp_ops,
	},
};

static int panel_parse_info(const void *blob, int node,
				struct owl_panel *panel, struct panel_glp_data *glp)
{
	uint32_t byte_lens;
	int entry, index, ret, len, i;
	char entryname[64];
	struct dsi_cmd *cmd = NULL;
	void *temp = NULL;
	int val;

	debug("%s\n", __func__);

	/*
	 * parse glp panel power on gpio,  It is not necessary!!!
	 *
	 * */
	/* parse power gpio ... */
	if (gpio_request_by_name_nodev(blob, node, "power-gpio", 0,
				       &glp->power_gpio, GPIOD_IS_OUT) < 0)
		debug("%s: fdtdec_decode_power-gpio failed\n", __func__);
	/* parse reset gpio ... */
	if (gpio_request_by_name_nodev(blob, node, "reset-gpio", 0,
				       &glp->reset_gpio, GPIOD_IS_OUT) < 0)
		debug("%s: fdtdec_decode_reset-gpio failed\n", __func__);

	return 0;
}

int owl_panel_glp_init(const void *blob)
{
	int node;

	int ret = 0;
	struct panel_glp_data *glp;

	/* 
	 * DTS match
	 */
	node = fdt_node_offset_by_compatible(blob, 0, "actions,panel-glp");
	if (node < 0) {
		debug("no match in DTS\n");
		return 0;
	}
	debug("%s\n", __func__);

	glp = malloc(sizeof(*glp));
	if (!glp) {
		error("malloc glp failed\n");
		return 0;
	}
	
	glp->blob = blob;
	glp->node = node;

	/*
	 * parse private panel info
	 * */
	ret = panel_parse_info(blob, node, &owl_panel_glp, glp);

	ret = owl_panel_parse_panel_info(blob, node, &owl_panel_glp);
	if (ret < 0) {
		error("failed to parse timings\n");
		return ret;
	}

	ret = owl_panel_register(&owl_panel_glp);
	if (ret < 0) {
		error("failed to register panel\n");
		return ret;
	}

	owl_panel_glp.pdata = glp;

	return 0;
}
