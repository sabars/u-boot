/*
 *
 * DSI contrllor driver.
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
#define pr_fmt(fmt) "owl_gmp: " fmt

#define DSS_SUBSYS_NAME "PANEL_MIPI"

#include <dss.h>
#include <malloc.h>
#include <errno.h>
#include <asm/io.h>
#include <fdtdec.h>
#define MIPI_MAX_PARS	100
#define MIPI_DSC_SET_ADRR_MODE	0x36

struct dsi_cmd {
	uint8_t		data_type;
	uint8_t		address;
	uint8_t		parameters[MIPI_MAX_PARS];
	uint8_t		n_parameters;
	uint8_t		delay;
};

struct panel_gmp_data {
	struct gpio_desc		power_gpio;
	struct gpio_desc		power1_gpio;
	struct gpio_desc		reset_gpio;
	/* Specific data can be added here */

	struct dsi_cmd			*cmd_list;
	uint32_t			n_cmd_list;
};

static int panel_mipi_power_on(struct owl_panel *panel)
{
	struct panel_gmp_data *gmp = panel->pdata;

	debug("%s, ... ...\n", __func__);
	/* assert  */
	if (dm_gpio_is_valid(&gmp->reset_gpio))
		dm_gpio_set_value(&gmp->reset_gpio, 1);
	mdelay(10);

	/* power on */
	if (dm_gpio_is_valid(&gmp->power1_gpio))
		dm_gpio_set_value(&gmp->power1_gpio, 1);
	if (dm_gpio_is_valid(&gmp->power_gpio))
		dm_gpio_set_value(&gmp->power_gpio, 1);
	mdelay(10);

	/* deassert */
	if (dm_gpio_is_valid(&gmp->reset_gpio))
		dm_gpio_set_value(&gmp->reset_gpio, 0);
	return 0;
}

static int panel_mipi_power_off(struct owl_panel *panel)
{
	struct panel_gmp_data *gmp = panel->pdata;

	if (dm_gpio_is_valid(&gmp->power_gpio))
		dm_gpio_set_value(&gmp->power_gpio, 0);
	return 0;
}

/*
 * command buffer format:
 *	buffer 4 ~ (MIPI_MAX_PARS - 1) ---> parameters
 *	buffer 3---> address
 *	buffer 2---> number of parameters
 *	buffer 1---> dcs data type
 *	buffer 0---> cmd delay
 *
 */
static int panel_mipi_enable(struct owl_panel *panel)
{
	struct panel_gmp_data *gmp = panel->pdata;
	struct owl_display_ctrl *ctrl = panel->ctrl;
	uint8_t *buffer, buffer_size;
	struct dsi_cmd *tmp_cmd = NULL;
	int i;

	debug("%s\n", __func__);

	tmp_cmd = gmp->cmd_list;

	for (i = 0; i < gmp->n_cmd_list; i++) {

		buffer_size = 4 + tmp_cmd[i].n_parameters;
		buffer = malloc(buffer_size);
		if(!buffer) {
			error("malloc buffer failed!\n");
			return -1;
		}

		buffer[0] = tmp_cmd[i].delay;
		buffer[1] = tmp_cmd[i].data_type;
		buffer[2] = tmp_cmd[i].n_parameters;
		buffer[3] = tmp_cmd[i].address;

		debug("cmd buffer size %d\n", buffer_size);
		debug("data type 0x%x, n_parameters %d, address 0x%x\n",
			buffer[1], buffer[2], buffer[3]);

		memcpy(&buffer[4], &tmp_cmd[i].parameters,
					tmp_cmd[i].n_parameters);

		/* send mipi initail command */
		if (ctrl->ops && ctrl->ops->aux_write)
			ctrl->ops->aux_write(ctrl, (char *)buffer, buffer_size);

		free(buffer);
	}

	return 0;
}

static int panel_mipi_disable(struct owl_panel *panel)
{
	struct panel_gmp_data *gmp = panel->pdata;

	return 0;
}

struct owl_panel_ops owl_panel_gmp_ops = {
	.power_on = panel_mipi_power_on,
	.power_off = panel_mipi_power_off,

	.enable = panel_mipi_enable,
	.disable = panel_mipi_disable,
};

static struct owl_panel owl_panel_mipi = {
	.desc = {
		.name = "mipi_panel",
		.type = OWL_DISPLAY_TYPE_DSI,
		.ops = &owl_panel_gmp_ops,
	},
};

static int panel_parse_info(const void *blob, int node,
	struct owl_panel *panel, struct panel_gmp_data *gmp)
{
	uint32_t byte_lens;
	int entry, index, ret, len, i;
	char entryname[64];
	struct dsi_cmd *cmd = NULL;
	void *temp = NULL;
	int val;

	debug("%s\n", __func__);

	/* parse mipi init cmd ... */
	index = 0;
	do {
		snprintf(entryname, sizeof(entryname), "mipi_init_cmd-%u", index);
		entry = fdtdec_lookup_phandle(blob, node, entryname);
		debug("entry = %d\n", entry);
		if (entry < 0) {
			debug("no etry for %s\n", entryname);
			break;
		} else {

			temp = realloc(cmd, (index + 1) * sizeof(*cmd));
			if (!temp) {
				return -ENOMEM;
			}

			cmd[index].data_type = fdtdec_get_int(blob, entry, "data_type", 0);
			debug("data_type 0x%x\n", cmd[index].data_type);
			cmd[index].address = fdtdec_get_int(blob, entry, "address", 0);
			debug("address 0x%x\n", cmd[index].address);

			if (!fdt_getprop(blob, entry, "parameters", &len))
				return -1;
			ret = fdtdec_get_byte_array(blob, entry, "parameters",
								&cmd[index].parameters, len);
			if (ret < 0)
				error("The requested node or property does not exist!\n");
			cmd[index].n_parameters = len;
			debug("parameters lens %d\n", len);

			cmd[index].delay = fdtdec_get_int(blob, entry, "delay", 0);
			debug("delay %d\n", cmd[index].delay);

			index++;
		}
	} while (1);

	gmp->cmd_list = cmd;
	gmp->n_cmd_list = index;
	debug("n_cmd_list 0x%x\n", gmp->n_cmd_list);

	/* parse mipi disable cmd ... TODO */

	/*
	 * parse mipi panel power on gpio,  It is not necessary!!!
	 *
	 * */
	/* parse power gpio ... */
	if (gpio_request_by_name_nodev(blob, node, "power-gpio", 0,
				       &gmp->power_gpio, GPIOD_IS_OUT) < 0)
		debug("%s: fdtdec_decode_power-gpio failed\n", __func__);
	/* parse power1 gpio ... */
	if (gpio_request_by_name_nodev(blob, node, "power1-gpio", 0,
				       &gmp->power1_gpio, GPIOD_IS_OUT) < 0)
		debug("%s: fdtdec_decode_power1-gpio failed\n", __func__);
	/* parse reset gpio ... */
	if (gpio_request_by_name_nodev(blob, node, "reset-gpio", 0,
				       &gmp->reset_gpio, GPIOD_IS_OUT) < 0)
		debug("%s: fdtdec_decode_reset-gpio failed\n", __func__);

	return 0;
}
int owl_panel_gmp_init(const void *blob)
{
	int node;

	int ret = 0;
	struct panel_gmp_data *gmp;
	debug("%s\n", __func__);

	/*
	 * DTS match
	 */
	node = fdt_node_offset_by_compatible(blob, 0, "actions,panel-gmp");
	if (node < 0) {
		debug("dsi gmp no match in DTS\n");
		return 0;
	}
	debug("%s\n", __func__);

	gmp = malloc(sizeof(*gmp));
	if (!gmp) {
		error("malloc gmp failed\n");
		return 0;
	}
	/*
	 * parse private panel info
	 * */
	ret = panel_parse_info(blob, node, &owl_panel_mipi, gmp);

	ret = owl_panel_parse_panel_info(blob, node, &owl_panel_mipi);
	if (ret < 0) {
		error("failed to parse timings\n");
		return ret;
	}

	ret = owl_panel_register(&owl_panel_mipi);
	if (ret < 0) {
		error("failed to register panel\n");
		return ret;
	}

	owl_panel_mipi.pdata = gmp;

	return 0;
}
