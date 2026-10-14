// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 MediaTek Inc.
 */
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/mm_types.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif
#include <linux/module.h>
#include <linux/sched/clock.h> /* local_clock() */
#include <linux/umh.h>		/* call_usermodehelper() */
#include <linux/kthread.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include "ccci_config.h"
#include "ccci_common_config.h"
#include <linux/arm-smccc.h>
#include <linux/soc/mediatek/ccci_mtk_sip_svc.h>

#define TRNG_MAGIC		0x74726e67
#ifdef FEATURE_INFORM_NFC_VSIM_CHANGE
#include <mach/mt6605.h>
#endif
#ifdef FEATURE_RF_CLK_BUF
#include <mtk-clkbuf-bridge.h>
#endif

#include "ccci_core.h"
#include "ccci_auxadc.h"
#include "ccci_bm.h"
#include "ccci_modem.h"
#include "port_rpc.h"
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/fcntl.h>
#include <linux/err.h>

/* PEARL-TEST: override the DRDI RF set index for RPC replies (-1 = use the device tree value) */
static int pearl_rf_set_idx = -1;
module_param(pearl_rf_set_idx, int, 0644);
MODULE_PARM_DESC(pearl_rf_set_idx,
	"PEARL test override for mediatek,md_drdi_rf_set_idx (-1 = use DT)");
#define MAX_QUEUE_LENGTH 16

static struct gpio_item gpio_mapping_table[] = {
	{"GPIO_FDD_Band_Support_Detection_1",
		"GPIO_FDD_BAND_SUPPORT_DETECT_1ST_PIN",},
	{"GPIO_FDD_Band_Support_Detection_2",
		"GPIO_FDD_BAND_SUPPORT_DETECT_2ND_PIN",},
	{"GPIO_FDD_Band_Support_Detection_3",
		"GPIO_FDD_BAND_SUPPORT_DETECT_3RD_PIN",},
	{"GPIO_FDD_Band_Support_Detection_4",
		"GPIO_FDD_BAND_SUPPORT_DETECT_4TH_PIN",},
	{"GPIO_FDD_Band_Support_Detection_5",
		"GPIO_FDD_BAND_SUPPORT_DETECT_5TH_PIN",},
	{"GPIO_FDD_Band_Support_Detection_6",
		"GPIO_FDD_BAND_SUPPORT_DETECT_6TH_PIN",},
	{"GPIO_FDD_Band_Support_Detection_7",
		"GPIO_FDD_BAND_SUPPORT_DETECT_7TH_PIN",},
	{"GPIO_FDD_Band_Support_Detection_8",
		"GPIO_FDD_BAND_SUPPORT_DETECT_8TH_PIN",},
	{"GPIO_FDD_Band_Support_Detection_9",
		"GPIO_FDD_BAND_SUPPORT_DETECT_9TH_PIN",},
	{"GPIO_FDD_Band_Support_Detection_A",
		"GPIO_FDD_BAND_SUPPORT_DETECT_ATH_PIN",},
	{"GPIO_RF_PWREN_RST_PIN",
		"GPIO_RF_PWREN_RST_PIN",},
};

static int get_md_gpio_val(unsigned int num)
{
	/* do not read the level back: reading the GPIO triggers a pinctrl access
	 * (see the get_gpio_id_from_dt comment). Return 0; the MODEM only treats
	 * it as the SIM presence state.
	 */
	return 0;
}

static int get_md_adc_val(__attribute__((unused))unsigned int num)
{
	int val = ccci_get_adc_val();

	return val;
}


static int get_td_eint_info(char *eint_name, unsigned int len)
{
	return -1;
}

static int get_md_adc_info(__attribute__((unused))char *adc_name,
			   __attribute__((unused))unsigned int len)
{
	int num = ccci_get_adc_num();

	CCCI_NORMAL_LOG(0, RPC, "ADC channel num:%d\n", num);
	return num;
}

static char *md_gpio_name_convert(char *gpio_name, unsigned int len)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(gpio_mapping_table); i++) {
		if (!strncmp(gpio_name, gpio_mapping_table[i].gpio_name_from_md,
			len))
			return gpio_mapping_table[i].gpio_name_from_dts;
	}

	return NULL;
}

static int get_gpio_id_from_dt(struct device_node *node,
	char *gpio_name, int *md_view_id)
{
	int md_view_gpio_id = -1;
	int ret;

	/* For new API, there is a shift between AP GPIO ID and MD GPIO ID.
	 * PEARL: the DT property is <&pio 42 0>, whose index 1 is the pin number
	 * the MODEM expects (Android logs "get_num:42" for this same node).
	 *
	 * Only the device tree is read here, on purpose: requesting or reading
	 * the GPIO would go through the MT6895 pinctrl, whose registers live in
	 * the SCP shared window (0x10005000) and hang the AP while SCP is down.
	 */
	ret = of_property_read_u32_index(node, gpio_name, 1, &md_view_gpio_id);
	if (ret)
		return ret;
	*md_view_id = md_view_gpio_id;

	return md_view_gpio_id;
}

static int get_md_gpio_info(char *gpio_name,
	unsigned int len, int *md_view_gpio_id)
{
	struct device_node *node = of_find_compatible_node(NULL, NULL,
		"mediatek,gpio_usage_mapping");
	int gpio_id = -1;
	char *name;

	if (len >= 4096) {
		CCCI_NORMAL_LOG(0, RPC,
			"MD GPIO name length abnoremal(%d)\n", len);
		return gpio_id;
	}

	if (!node) {
		CCCI_NORMAL_LOG(0, RPC,
			"MD_USE_GPIO is not set in device tree,need to check?\n");
		return gpio_id;
	}

	name = md_gpio_name_convert(gpio_name, len);
	if (name) {
		gpio_id = get_gpio_id_from_dt(node, name, md_view_gpio_id);
		return gpio_id;
	}
	if (gpio_name[len-1] != 0) {
		name = kmalloc(len + 1, GFP_KERNEL);
		if (name) {
			memcpy(name, gpio_name, len);
			name[len] = 0;
			gpio_id = get_gpio_id_from_dt(node, name,
				md_view_gpio_id);
			kfree(name);
			return gpio_id;
		}
		CCCI_BOOTUP_LOG(0, RPC,
			"alloc memory fail for gpio with size:%d\n", len);
		return gpio_id;
	}
	gpio_id = get_gpio_id_from_dt(node, gpio_name, md_view_gpio_id);
	return gpio_id;
}

static int get_dram_type_clk(int *clk, int *type)
{
	return -1;
}

static struct eint_struct md_eint_struct[] = {
	/* ID of MD get, property name,  cell index read from property */
	{SIM_EINT_NUM, "interrupts", 0,},
	{SIM_EINT_DEBOUNCE, "debounce", 1,},
	{SIM_EINT_POLA, "interrupts", 1,},
	{SIM_EINT_SENS, "interrupts", 1,},
	{SIM_EINT_SOCKE, "sockettype", 1,},
	{SIM_EINT_DEDICATEDEN, "dedicated", 1,},
	{SIM_EINT_SRCPIN, "src_pin", 1,},
	{SIM_HOT_PLUG_EINT_MAX, "invalid_type", 0xFF,},
};

static struct eint_node_name md_eint_node[] = {
	{"MD1_SIM1_HOT_PLUG_EINT", 1, 1,},
	{"MD1_SIM2_HOT_PLUG_EINT", 1, 2,},
	{"MD1_SIM3_HOT_PLUG_EINT", 1, 3,},
	{"MD1_SIM4_HOT_PLUG_EINT", 1, 4,},
	/* {"MD1_SIM5_HOT_PLUG_EINT", 1, 5, }, */
	/* {"MD1_SIM6_HOT_PLUG_EINT", 1, 6, }, */
	/* {"MD1_SIM7_HOT_PLUG_EINT", 1, 7, }, */
	/* {"MD1_SIM8_HOT_PLUG_EINT", 1, 8, }, */
	/* {"MD2_SIM1_HOT_PLUG_EINT", 2, 1, }, */
	/* {"MD2_SIM2_HOT_PLUG_EINT", 2, 2, }, */
	/* {"MD2_SIM3_HOT_PLUG_EINT", 2, 3, }, */
	/* {"MD2_SIM4_HOT_PLUG_EINT", 2, 4, }, */
	/* {"MD2_SIM5_HOT_PLUG_EINT", 2, 5, }, */
	/* {"MD2_SIM6_HOT_PLUG_EINT", 2, 6, }, */
	/* {"MD2_SIM7_HOT_PLUG_EINT", 2, 7, }, */
	/* {"MD2_SIM8_HOT_PLUG_EINT", 2, 8, }, */
	{NULL,},
};

struct eint_node_struct eint_node_prop = {
	0,
	md_eint_node,
	md_eint_struct,
};

static int get_eint_attr_val(int md_id, struct device_node *node, int index)
{
	int value;
	int ret = 0, type;

	/* unit of AP eint is us, but unit of MD eint is ms.
	 * So need covertion here.
	 */
	int covert_AP_to_MD_unit = 1000;

	for (type = 0; type < SIM_HOT_PLUG_EINT_MAX; type++) {
		ret = of_property_read_u32_index(node,
			md_eint_struct[type].property,
			md_eint_struct[type].index, &value);
		if (ret != 0) {
			md_eint_struct[type].value_sim[index] =
			ERR_SIM_HOT_PLUG_QUERY_TYPE;
			CCCI_NORMAL_LOG(md_id, RPC, "%s:  not found\n",
			md_eint_struct[type].property);
			ret = ERR_SIM_HOT_PLUG_QUERY_TYPE;
			continue;
		}
		/* special case: polarity's position == sensitivity's start[ */
		if (type == SIM_EINT_POLA) {
			switch (value) {
			case IRQ_TYPE_EDGE_RISING:
			case IRQ_TYPE_EDGE_FALLING:
			case IRQ_TYPE_LEVEL_HIGH:
			case IRQ_TYPE_LEVEL_LOW:
				md_eint_struct[SIM_EINT_POLA].value_sim[index]
					= (value & 0x5) ? 1 : 0;
				/* 1/4:
				 * IRQ_TYPE_EDGE_RISING/
				 * IRQ_TYPE_LEVEL_HIGH Set 1
				 */
				md_eint_struct[SIM_EINT_SENS].value_sim[index]
					= (value & 0x3) ? 1 : 0;
				/* 1/2:
				 * IRQ_TYPE_EDGE_RISING/
				 * IRQ_TYPE_LEVEL_FALLING Set 1
				 */
				break;
			default:	/* invalid */
				md_eint_struct[SIM_EINT_POLA].value_sim[index]
					= -1;
				md_eint_struct[SIM_EINT_SENS].value_sim[index]
					= -1;
				CCCI_ERROR_LOG(md_id, RPC,
					"invalid value, please check dtsi!\n");
				break;
			}
			type++;
		} else if (type == SIM_EINT_DEBOUNCE) {
			/* debounce time should divide by 1000 due
			 * to different unit in AP and MD.
			 */
			md_eint_struct[type].value_sim[index] =
				value/covert_AP_to_MD_unit;
		} else
			md_eint_struct[type].value_sim[index] = value;
	}
	return ret;
}

void get_dtsi_eint_node(int md_id)
{
	static int init; /*default is 0*/
	int i;
	struct device_node *node = NULL;

	if (init)
		return;
	init = 1;
	for (i = 0; i < MD_SIM_MAX; i++) {
		if (eint_node_prop.name[i].node_name == NULL) {
			CCCI_INIT_LOG(md_id, RPC, "node %d is NULL\n", i);
			break;
		}
		node = of_find_node_by_name(NULL,
			eint_node_prop.name[i].node_name);
		if (node != NULL) {
			eint_node_prop.ExistFlag |= (1U << i);
			get_eint_attr_val(md_id, node, i);
		} else {
			CCCI_INIT_LOG(md_id, RPC, "%s: node %d no found\n",
				     eint_node_prop.name[i].node_name, i);
		}
	}
}

int get_eint_attr_DTSVal(int md_id, const char *name, unsigned int name_len,
			unsigned int type, char *result, unsigned int *len)
{
	int i, sim_value;
	int *sim_info = (int *)result;

	if ((name == NULL) || (result == NULL) || (len == NULL))
		return ERR_SIM_HOT_PLUG_NULL_POINTER;
	if (type >= SIM_HOT_PLUG_EINT_MAX)
		return ERR_SIM_HOT_PLUG_QUERY_TYPE;

	for (i = 0; i < MD_SIM_MAX; i++) {
		if ((eint_node_prop.ExistFlag & (1U << i)) == 0)
			continue;
		if (!(strncmp(name,
			eint_node_prop.name[i].node_name, name_len))) {
			sim_value =
			eint_node_prop.eint_value[type].value_sim[i];
			*len = sizeof(sim_value);
			memcpy(sim_info, &sim_value, *len);
			CCCI_BOOTUP_LOG(md_id, RPC,
			"md_eint:%s, sizeof: %d, sim_info: %d, %d\n",
			eint_node_prop.eint_value[type].property,
			*len, *sim_info,
			eint_node_prop.eint_value[type].value_sim[i]);
			if (sim_value >= 0)
				return 0;
		}
	}
	return ERR_SIM_HOT_PLUG_QUERY_STRING;
}

static int get_eint_attr(int md_id, char *name, unsigned int name_len,
			unsigned int type, char *result, unsigned int *len)
{
	return get_eint_attr_DTSVal(md_id, name, name_len, type, result, len);
}

static void get_md_dtsi_val(struct ccci_rpc_md_dtsi_input *input,
	struct ccci_rpc_md_dtsi_output *output)
{
	int ret = -1;
	int value = 0;
	struct device_node *node =
	of_find_compatible_node(NULL, NULL, "mediatek,md_attr_node");

	/* PEARL-TEST: check the test override parameters first */
	if (pearl_rf_set_idx >= 0 &&
	    strncmp(input->strName, "mediatek,md_drdi_rf_set_idx",
		    strlen("mediatek,md_drdi_rf_set_idx")) == 0) {
		output->retValue = (unsigned int)pearl_rf_set_idx;
		CCCI_ERROR_LOG(-1, RPC, "PEARL-TEST: rf_set_idx -> %d\n",
			pearl_rf_set_idx);
		return;
	}

	if (node == NULL) {
		CCCI_INIT_LOG(-1, RPC, "%s: No node: %s\n", __func__,
			input->strName);
		CCCI_NORMAL_LOG(-1, RPC, "%s: No node: %s\n", __func__,
			input->strName);
		return;
	}

	switch (input->req) {
	case RPC_REQ_PROP_VALUE:
		ret = of_property_read_u32(node, input->strName, &value);
		if (ret == 0)
			output->retValue = value;
		break;
	}
	CCCI_INIT_LOG(-1, RPC, "%s %d, %s -- 0x%x\n", __func__,
		input->req, input->strName, output->retValue);
	CCCI_NORMAL_LOG(-1, RPC, "%s %d, %s -- 0x%x\n", __func__,
		input->req, input->strName, output->retValue);
}

static void get_md_dtsi_debug(void)
{
	struct ccci_rpc_md_dtsi_input input;
	struct ccci_rpc_md_dtsi_output output;
	int ret;

	input.req = RPC_REQ_PROP_VALUE;
	output.retValue = 0;
	ret = snprintf(input.strName, sizeof(input.strName), "%s",
		"mediatek,md_drdi_rf_set_idx");
	if (ret <= 0 || ret >= sizeof(input.strName)) {
		CCCI_ERROR_LOG(-1, RPC, "%s:snprintf input.strName fail\n",
			__func__);
		return;
	}
	get_md_dtsi_val(&input, &output);
}

static void ccci_rpc_get_gpio_adc(struct ccci_rpc_gpio_adc_intput *input,
	struct ccci_rpc_gpio_adc_output *output)
{
	int num;
	unsigned int val, i, md_val = -1;

	if ((input->reqMask & (RPC_REQ_GPIO_PIN | RPC_REQ_GPIO_VALUE)) ==
		(RPC_REQ_GPIO_PIN | RPC_REQ_GPIO_VALUE)) {
		for (i = 0; i < GPIO_MAX_COUNT; i++) {
			if (input->gpioValidPinMask & (1 << i)) {
				num = get_md_gpio_info(input->gpioPinName[i],
						strlen(input->gpioPinName[i]),
						&md_val);
				if (num >= 0) {
					output->gpioPinNum[i] = md_val;
					val = get_md_gpio_val(num);
					output->gpioPinValue[i] = val;
				}
			}
		}
	} else {
		if (input->reqMask & RPC_REQ_GPIO_PIN) {
			for (i = 0; i < GPIO_MAX_COUNT; i++) {
				if (input->gpioValidPinMask & (1 << i)) {
					num = get_md_gpio_info(
					input->gpioPinName[i],
					strlen(input->gpioPinName[i]), &md_val);
					if (num >= 0)
						output->gpioPinNum[i] = md_val;
				}
			}
		}
		if (input->reqMask & RPC_REQ_GPIO_VALUE) {
			for (i = 0; i < GPIO_MAX_COUNT; i++) {
				if (input->gpioValidPinMask & (1 << i)) {
					val = get_md_gpio_val(
					input->gpioPinNum[i]);
					output->gpioPinValue[i] = val;
				}
			}
		}
	}
	if ((input->reqMask & (RPC_REQ_ADC_PIN | RPC_REQ_ADC_VALUE)) ==
		(RPC_REQ_ADC_PIN | RPC_REQ_ADC_VALUE)) {
		num = get_md_adc_info(input->adcChName,
				strlen(input->adcChName));

		if (num >= 0) {
			output->adcChNum = num;
			output->adcChMeasSum = 0;
			for (i = 0; i < input->adcChMeasCount; i++) {
				val = get_md_adc_val(num);
				output->adcChMeasSum += val;
			}
			CCCI_NORMAL_LOG(0, RPC,
					"%s, reqMask:%d, adcChmeasCount:%u, adcChMeasSum:%u\n",
					__func__, input->reqMask, i, output->adcChMeasSum);
		}
	} else {
		if (input->reqMask & RPC_REQ_ADC_PIN) {
			num = get_md_adc_info(input->adcChName,
					strlen(input->adcChName));
			if (num >= 0)
				output->adcChNum = num;
		}
		if (input->reqMask & RPC_REQ_ADC_VALUE) {
			output->adcChMeasSum = 0;
			for (i = 0; i < input->adcChMeasCount; i++) {
				val = get_md_adc_val(input->adcChNum);
				output->adcChMeasSum += val;
			}
			CCCI_NORMAL_LOG(0, RPC,
					"%s, reqMask:%d, adcChmeasCount:%u, adcChMeasSum:%u\n",
					__func__, input->reqMask, i, output->adcChMeasSum);
		}
	}
}

static void ccci_rpc_get_gpio_adc_v2(struct ccci_rpc_gpio_adc_intput_v2 *input,
	struct ccci_rpc_gpio_adc_output_v2 *output)
{
	int num, md_val = -1;
	unsigned int val, i;

	if ((input->reqMask & (RPC_REQ_GPIO_PIN | RPC_REQ_GPIO_VALUE)) ==
		(RPC_REQ_GPIO_PIN | RPC_REQ_GPIO_VALUE)) {
		for (i = 0; i < GPIO_MAX_COUNT_V2; i++) {
			if (input->gpioValidPinMask & (1 << i)) {
				num = get_md_gpio_info(input->gpioPinName[i],
						strlen(input->gpioPinName[i]),
						&md_val);
				if (num >= 0) {
					output->gpioPinNum[i] = md_val;
					val = get_md_gpio_val(num);
					output->gpioPinValue[i] = val;
				}
			}
		}
	} else {
		if (input->reqMask & RPC_REQ_GPIO_PIN) {
			for (i = 0; i < GPIO_MAX_COUNT_V2; i++) {
				if (input->gpioValidPinMask & (1 << i)) {
					num = get_md_gpio_info(
						input->gpioPinName[i],
						strlen(input->gpioPinName[i]),
						&md_val);
					if (num >= 0)
						output->gpioPinNum[i] = md_val;
				}
			}
		}
		if (input->reqMask & RPC_REQ_GPIO_VALUE) {
			for (i = 0; i < GPIO_MAX_COUNT_V2; i++) {
				if (input->gpioValidPinMask & (1 << i)) {
					val = get_md_gpio_val(
							input->gpioPinNum[i]);
					output->gpioPinValue[i] = val;
				}
			}
		}
	}
	if ((input->reqMask & (RPC_REQ_ADC_PIN | RPC_REQ_ADC_VALUE)) ==
		(RPC_REQ_ADC_PIN | RPC_REQ_ADC_VALUE)) {
		num = get_md_adc_info(input->adcChName,
				strlen(input->adcChName));
		if (num >= 0) {
			output->adcChNum = num;
			output->adcChMeasSum = 0;
			for (i = 0; i < input->adcChMeasCount; i++) {
				val = get_md_adc_val(num);
				output->adcChMeasSum += val;
			}
			CCCI_NORMAL_LOG(0, RPC,
					"%s, reqMask:%d, adcChmeasCount:%u, adcChMeasSum:%u\n",
					__func__, input->reqMask, i, output->adcChMeasSum);
		}
	} else {
		if (input->reqMask & RPC_REQ_ADC_PIN) {
			num = get_md_adc_info(input->adcChName,
					strlen(input->adcChName));
			if (num >= 0)
				output->adcChNum = num;
		}
		if (input->reqMask & RPC_REQ_ADC_VALUE) {
			output->adcChMeasSum = 0;
			for (i = 0; i < input->adcChMeasCount; i++) {
				val = get_md_adc_val(input->adcChNum);
				output->adcChMeasSum += val;
			}
			CCCI_NORMAL_LOG(0, RPC,
					"%s, reqMask:%d, adcChmeasCount:%u, adcChMeasSum:%u\n",
					__func__, input->reqMask, i, output->adcChMeasSum);
		}
	}
}

static int ccci_rpc_remap_queue(int md_id, struct ccci_rpc_queue_mapping *remap)
{
	struct port_t *port;

	port = port_get_by_minor(md_id, remap->net_if + CCCI_NET_MINOR_BASE);

	if (!port) {
		CCCI_ERROR_LOG(md_id, RPC, "can't find ccmni for netif: %d\n",
			remap->net_if);
		return -1;
	}

	if (remap->lhif_q == LHIF_HWQ_AP_UL_Q0) {
		/*normal queue*/
		port->txq_index = 0;
		port->txq_exp_index = 0xF0 | 0x1;
		CCCI_NORMAL_LOG(md_id, RPC, "remap port %s Tx to cldma%d\n",
			port->name, port->txq_index);
	} else if (remap->lhif_q == LHIF_HWQ_AP_UL_Q1) {
		/*IMS queue*/
		port->txq_index = 3;
		port->txq_exp_index = 0xF0 | 0x3;
		CCCI_NORMAL_LOG(md_id, RPC, "remap port %s Tx to cldma%d\n",
			port->name, port->txq_index);
	} else
		CCCI_ERROR_LOG(md_id, RPC, "invalid remap for q%d\n",
			remap->lhif_q);

	return 0;
}

/* ===== PEARL: AMMS DRDI control (kernel-side replacement for ccci_rpcd) =====
 *
 * Protocol reconstructed from vendor/bin/ccci_rpcd disassembly plus
 * byte-level verification on a real yuechu device; see
 * E:\pearl\notes\rpcd\PROTOCOL.md and amms_drdi_protocol.h.
 *
 * Request (op_id=0x4014, para_num=1, para[0].len=188):
 *   +0x00 u8 cmd          1=INIT / 2=DRDI_COPY
 *   +0x01 u8 seq_id       echoed into the reply
 *   +0x04 u8 ver          INIT must be ==3 ; COPY must be ==1
 *   +0x05 u8 set_total_num <= 15
 *   +0x08 INIT table: {u32 offset; u32 len;} stride 8   (relative to the md1drdi data area)
 *   +0x08 COPY table: {u32 src; u32 dst; u32 len;} stride 12
 *                 src is relative to the md1drdi data area, dst is an offset inside the 64KiB DRDI smem
 *
 * Reply (op_id=0xFFFF4014, para_num=2, 44 bytes total):
 *   para[0] = {u32 len=4, u32 ret_code(0/0xFFFFFFFF)}
 *   para[1] = {u32 len=8, 8-byte status}
 *   status: {stats, seq_id, rsv[2], ver, copystat, drdiinfostat, rsv}
 */
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/vmalloc.h>

#define PEARL_AMMS_MAX_SET	15
#define PEARL_AMMS_REQ_SIZE	188	/* 8 + 15*12; the kernel hard-checks 0xBC */
#define PEARL_AMMS_CMD_INIT	1
#define PEARL_AMMS_CMD_COPY	2
#define PEARL_AMMS_INIT_VER	3
#define PEARL_AMMS_COPY_VER	1
#define PEARL_AMMS_MAX_COPY_LEN	0x10000
#define PEARL_AMMS_MD_NUM	2

#define PEARL_MD1IMG_PATH	"/dev/disk/by-partlabel/md1img_a"
/* PEARL-175: during an early insmod /dev/disk/by-partlabel/ does not exist yet
 * (udev has not run), so DRDI data loading failed and the MD asserted at
 * dmmgr_amms_v2.c:577. Switched to multiple paths + retries. */
static const char *pearl175_paths[] = {
	"/dev/disk/by-partlabel/md1img_a",
	"/dev/sdb24",
	"/dev/block/by-name/md1img_a",
	"/dev/disk/by-name/md1img_a",
};

static int pearl175_open_md1img(struct file **out)
{
	int i, ret = -ENOENT;

	for (i = 0; i < (int)ARRAY_SIZE(pearl175_paths); i++) {
		struct file *f = filp_open(pearl175_paths[i], O_RDONLY | O_LARGEFILE, 0);

		if (!IS_ERR(f)) {
			pr_info("PEARL-175: md1img opened successfully via %s\n", pearl175_paths[i]);
			*out = f;
			return 0;
		}
		ret = PTR_ERR(f);
		pr_info("PEARL-175: open %s fail %d\n", pearl175_paths[i], ret);
	}
	return ret;
}

#define PEARL_SEG_MAGIC		0x58881688
#define PEARL_SEG_HDR_LEN	0x200	/* default segment header hdrlen; the data area follows */
#define PEARL_DRDI_SEG_NAME	"md1drdi"
#define PEARL_SCAN_CHUNK		(4 * 1024 * 1024)
#define PEARL_SCAN_BLOCKS	40

struct pearl_amms_req {
	u8 cmd;
	u8 seq_id;
	u8 rsv0[2];
	u8 ver;
	u8 set_total_num;
	u8 rsv1[2];
	u8 tbl[180];
} __packed;

struct pearl_amms_rsp {
	u8 stats;		/* 0 = success, 0xFF = failure */
	u8 seq_id;		/* = req.seq_id */
	u8 rsv2[2];
	u8 ver;
	u8 copystat;
	u8 drdiinfostat;
	u8 rsv3;
} __packed;

struct pearl_amms_set_init {
	u32 off;
	u32 len;
} __packed;

struct pearl_amms_set_copy {
	u32 src;
	u32 dst;
	u32 len;
} __packed;

static unsigned int pearl_amms_dump_req = 1;
module_param(pearl_amms_dump_req, uint, 0644);
MODULE_PARM_DESC(pearl_amms_dump_req, "PEARL: dump every AMMS DRDI request");

static unsigned int pearl_amms_do_copy = 1;
module_param(pearl_amms_do_copy, uint, 0644);
MODULE_PARM_DESC(pearl_amms_do_copy, "PEARL: perform the AMMS DRDI data copy");

static unsigned int pearl_amms_fail_ok = 0;
module_param(pearl_amms_fail_ok, uint, 0644);
MODULE_PARM_DESC(pearl_amms_fail_ok,
	"PEARL: force error reply (for A/B testing the reply path)");

/* state left by the INIT request (ccci_rpcd keeps it in a global; the COPY entry count comes from it) */
static unsigned int pearl_amms_set_total[PEARL_AMMS_MD_NUM];
static int pearl_amms_copy_done[PEARL_AMMS_MD_NUM];
static unsigned int pearl_amms_req_cnt[PEARL_AMMS_MD_NUM];

/* md1drdi segment data area (starts at segment header +0x200); offset/src in requests are relative to it */
static void *pearl_drdi_data;
/* PEARL-176: wait for AMMS/DRDI data to be ready before auto-starting the MD */
int pearl176_drdi_ready(void)
{
	return pearl_drdi_data != NULL;
}
EXPORT_SYMBOL(pearl176_drdi_ready);

static unsigned int pearl_drdi_len;
static unsigned int pearl_drdi_seg_off;

static u32 pearl_le32(const unsigned char *p)
{
	return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) |
		((u32)p[3] << 24);
}

/* locate the md1drdi segment in the md1img partition and read it into memory (called once at insmod) */
static int pearl_drdi_load_image(void)
{
	struct file *f;
	unsigned char *buf;
	loff_t pos;
	unsigned int i, blk, got, seg_len = 0;
	int ret = 0;

	if (pearl_drdi_data)
		return 0;

	ret = pearl175_open_md1img(&f);
	if (ret) {
		pr_err("PEARL-175: failed to open md1img via all paths, ret=%d\n", ret);
		return ret;
	}
	buf = vmalloc(PEARL_SCAN_CHUNK + PEARL_SEG_HDR_LEN);
	if (!buf) {
		ret = -ENOMEM;
		goto out_close;
	}
	for (blk = 0; blk < PEARL_SCAN_BLOCKS && !seg_len; blk++) {
		pos = (loff_t)blk * PEARL_SCAN_CHUNK;
		got = kernel_read(f, buf, PEARL_SCAN_CHUNK, &pos);
		if ((int)got <= 0)
			break;
		for (i = 0; i + 16 <= got; i += 8) {
			if (pearl_le32(buf + i) != PEARL_SEG_MAGIC)
				continue;
			if (memcmp(buf + i + 8, PEARL_DRDI_SEG_NAME, 8))
				continue;
			seg_len = pearl_le32(buf + i + 4);
			pearl_drdi_seg_off = (unsigned int)pos - got + i;
			break;
		}
	}
	if (!seg_len) {
		pr_err("PEARL-AMMS: %s segment not found\n", PEARL_DRDI_SEG_NAME);
		ret = -ENOENT;
		goto out_free;
	}
	pearl_drdi_data = vmalloc(seg_len);
	if (!pearl_drdi_data) {
		ret = -ENOMEM;
		goto out_free;
	}
	pos = (loff_t)pearl_drdi_seg_off + PEARL_SEG_HDR_LEN;
	got = kernel_read(f, pearl_drdi_data, seg_len, &pos);
	if ((int)got != (int)seg_len) {
		pr_err("PEARL-AMMS: read drdi seg short %u/%u\n", got, seg_len);
		vfree(pearl_drdi_data);
		pearl_drdi_data = NULL;
		ret = -EIO;
		goto out_free;
	}
	pearl_drdi_len = seg_len;
	pr_info("PEARL-AMMS: md1drdi loaded seg_off=0x%x data=0x%x len=0x%x\n",
		pearl_drdi_seg_off, pearl_drdi_seg_off + PEARL_SEG_HDR_LEN,
		pearl_drdi_len);
out_free:
	vfree(buf);
out_close:
	filp_close(f, NULL);
	return ret;
}

/* ===== PEARL: NVRAM cache shared-memory fill =====
 * Mobian has no Android nvram service; SMEM_USER_MD_NVRAM_CACHE (AP view
 * 0x8a180000, 1.5MB) measures all-zero. The MODEM's RF calibration data
 * comes from NVRAM, and reading all zeros asserts in mml1_rf_error_check,
 * so fill the shared region with the nvram partition contents before the
 * MODEM reads it (at the AMMS init request). Source, offset and length are
 * all configurable for experiments.
 */
#define PEARL_NVRAM_SRC_DEFAULT	"/dev/disk/by-partlabel/nvram"

static char *pearl_nvram_src = PEARL_NVRAM_SRC_DEFAULT;
module_param(pearl_nvram_src, charp, 0444);
MODULE_PARM_DESC(pearl_nvram_src, "PEARL: NVRAM source for the MD cache region");

static unsigned int pearl_nvram_skip;
module_param(pearl_nvram_skip, uint, 0444);
MODULE_PARM_DESC(pearl_nvram_skip, "PEARL: source offset");

static unsigned int pearl_nvram_len;
module_param(pearl_nvram_len, uint, 0444);
MODULE_PARM_DESC(pearl_nvram_len, "PEARL: bytes to copy (0 = whole region)");

static unsigned int pearl_nvram_fill = 1;
module_param(pearl_nvram_fill, uint, 0644);
MODULE_PARM_DESC(pearl_nvram_fill, "PEARL: fill NVRAM cache region at AMMS init");

static void *pearl_nvram_data;
static unsigned int pearl_nvram_data_len;
static unsigned int pearl_nvram_done[PEARL_AMMS_MD_NUM];

/* read the NVRAM source into memory at insmod time */
static int pearl_nvram_load(void)
{
	struct file *f;
	loff_t pos;
	unsigned int want = 0x200000;	/* read 2MB first; enough to cover the 1.5MB cache region */
	int got;

	if (pearl_nvram_data)
		return 0;
	f = filp_open(pearl_nvram_src, O_RDONLY | O_LARGEFILE, 0);
	if (IS_ERR(f)) {
		pr_err("PEARL-AMMS: open nvram src %s fail %ld\n",
			pearl_nvram_src, PTR_ERR(f));
		return PTR_ERR(f);
	}
	pearl_nvram_data = vmalloc(want);
	if (!pearl_nvram_data) {
		filp_close(f, NULL);
		return -ENOMEM;
	}
	pos = pearl_nvram_skip;
	got = kernel_read(f, pearl_nvram_data, want, &pos);
	filp_close(f, NULL);
	if (got <= 0) {
		pr_err("PEARL-AMMS: read nvram src fail %d\n", got);
		vfree(pearl_nvram_data);
		pearl_nvram_data = NULL;
		return -EIO;
	}
	pearl_nvram_data_len = got;
	pr_info("PEARL-AMMS: nvram src %s skip=0x%x read=0x%x\n",
		pearl_nvram_src, pearl_nvram_skip, pearl_nvram_data_len);
	return 0;
}

/* just before the MODEM reads NVRAM, write the data into its cache shared region.
 * mark=false is for the initial fill at insmod time (which the MD boot flow may
 * still clear).
 */
static void pearl_nvram_fill_cache(int md_id, int mark)
{
	struct ccci_smem_region *r;
	void __iomem *dst;
	bool own = false;
	unsigned int len, i, nz_before = 0, nz_after = 0;
	u8 *rb;

	if (!pearl_nvram_fill)
		return;
	if (mark && pearl_nvram_done[md_id & 1])
		return;
	if (!pearl_nvram_data && pearl_nvram_load())
		return;
	r = ccci_md_get_smem_by_user_id(md_id, SMEM_USER_MD_NVRAM_CACHE);
	if (!r || !r->size) {
		CCCI_ERROR_LOG(md_id, RPC,
			"PEARL-AMMS: no NVRAM cache region (%p)\n", r);
		return;
	}
	len = pearl_nvram_len ? pearl_nvram_len : r->size;
	if (len > r->size)
		len = r->size;
	if (len > pearl_nvram_data_len)
		len = pearl_nvram_data_len;

	dst = r->base_ap_view_vir;
	if (!dst) {
		dst = ioremap_wc(r->base_ap_view_phy, r->size);
		own = true;
	}
	if (!dst) {
		CCCI_ERROR_LOG(md_id, RPC, "PEARL-AMMS: map nvram cache fail\n");
		return;
	}
	rb = vmalloc(len);
	if (rb) {
		/* look at the current contents before filling: tells whether the insmod-time fill was cleared */
		memcpy_fromio(rb, dst, len);
		for (i = 0; i < len; i++)
			if (rb[i])
				nz_before++;
		CCCI_ERROR_LOG(md_id, RPC,
			"PEARL-AMMS: NVRAM cache before fill: len=0x%x nonzero=%u first=%*ph\n",
			len, nz_before, 16, rb);
	}
	memcpy_toio(dst, pearl_nvram_data, len);
	if (rb) {
		memcpy_fromio(rb, dst, len);
		for (i = 0; i < len; i++)
			if (rb[i])
				nz_after++;
		CCCI_ERROR_LOG(md_id, RPC,
			"PEARL-AMMS: NVRAM cache filled%s len=0x%x nonzero=%u first=%*ph\n",
			mark ? "" : "(insmod)", len, nz_after, 16, rb);
		vfree(rb);
	}
	if (own)
		iounmap(dst);
	if (mark)
		pearl_nvram_done[md_id & 1] = 1;
}

/*
 * PEARL-180: md1img lives on a block partition whose /dev node is created by
 * udev, but ccci is built-in and probes at ~2.7 s, long before udev runs, so
 * the first pearl_drdi_load_image() always fails with -ENOENT. The AMMS INIT
 * lazy-retry only helps once the MD is already up, which never happens because
 * MD start is gated on pearl176_drdi_ready(). Retry the load from a deferred
 * work every 500 ms until it succeeds, so the image is resident by the time the
 * MD auto-start gate (PEARL-176B) evaluates it.
 */
static void pearl_drdi_retry_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(pearl_drdi_retry_dwork, pearl_drdi_retry_work);

static void pearl_drdi_retry_work(struct work_struct *work)
{
	if (pearl_drdi_data)
		return;
	if (!pearl_drdi_load_image())
		pr_info("PEARL-180: md1drdi loaded on retry (len=%u)\n",
			pearl_drdi_len);
	else
		schedule_delayed_work(&pearl_drdi_retry_dwork,
				      msecs_to_jiffies(500));
}

/*
 * PEARL: Android's ccci_mdinit does "wait for NVRAM ready -> then DO_START_MD";
 * we used to do "start first -> fill NVRAM at AMMS init", so the modem could
 * read 0 early in boot. Move the preparation ahead of the start command here
 * (the lazy-load fallback is kept, so a not-yet-ready partition is not missed).
 */
void pearl_prepare_before_md_start(unsigned char md_id)
{
	int ret;

	ret = pearl_drdi_load_image();
	if (!pearl_drdi_data)
		pr_err("PEARL-MD-START: md1drdi not ready yet (ret=%d), lazy path will retry\n",
		       ret);

	pearl_nvram_fill_cache(md_id, 2);
	pr_err("PEARL-MD-START: nvram cache prepared before MD start (len=%u)\n",
	       pearl_nvram_data_len);
}
EXPORT_SYMBOL(pearl_prepare_before_md_start);

/*
 * PEARL: FS (ccci_fs, ch14/ch15) server
 *
 * On Android this channel is served by the userspace ccci_fsd; newer MTK
 * merged it into ccci_mdinit (measured on yuechu: /proc/<ccci_mdinit>/fd/10 ->
 * /dev/ccci_fs, ccci_fsd is not started at all) and the modem still boots to
 * ready. On Mobian no process opens /dev/ccci_fs, so the op=0x1001
 * (FS_CCCI_Open) request sent after HS1 never gets a reply and 43s later
 * MD_BOOT_HS2_FAIL -- the only remaining failure cause after the early
 * anomaly (pattern A) disappeared.
 *
 * Message format (reverse-engineered from yuechu device strace; see notes/rpcd/FS_PROTOCOL.md):
 *   [struct ccci_header 16B][u32 op][u32 nblocks]
 *   nblocks * { u32 len; u8 data[len], 4-byte aligned }
 * Reply: channel changed to CCCI_FS_TX, op ORed with 0xFFFF0000, seq echoed back unchanged.
 *
 * Block structure per op (measured on yuechu + pearl):
 *   0x1001 Open        req {len: path(UTF-16LE)}{4: mode}   rep {4: handle}
 *   0x1002 Seek        req {4: handle}{4: off}{4: whence}   rep {4: new_pos}
 *   0x1004 Write       req {4: handle}{len: data}{4: off}   rep {4: status}
 *   0x1005 Close       req {4: handle}                      rep {4: status}
 *   0x1009 GetFileSize req {4: handle}                      rep {4: status}{4: size}
 *
 * This is a "mini filesystem in memory": files persist by name (across
 * open/close) and Write really stores data. The modem can therefore append
 * to nv_boot_trace just like on Android, and /proc/pearl_fs lets us read the
 * boot trace the modem wrote itself.
 * Measured lesson: an early version cleared size on Close; when the modem
 * reopened and got 0 from Seek(END) it hammered Seek with the same seq and
 * finally asserted at dev_fs.c:224.
 */
#define PEARL_FS_MAX_MSG	8192
/* PEARL-FSFIX-63: sane upper bound (64MB) when descriptor w4 is used as the CMPT_WRITE offset. */
#define PEARL_FS_MAX_WRITE_OFF	(64U * 1024U * 1024U)
/*
 * Upper bound of the "data block" in a reply message.
 * The first statement of ccci_alloc_skb() is
 * `if (size > SKB_4K || size < 0) goto err_exit;` (ccci_bm.c), and
 * SKB_4K = CCCI_MTU + 128 = 3584 (ccci_bm.h:23; ccci_common_config.h also
 * says "3584 ==SKB_4K") -- the whole reply packet must not exceed 3584.
 * The reply header (16 header + 4 op/nblk + per-block length fields + our
 * earlier blocks) is about 56 bytes, so the data block is at most 3528;
 * this computes it directly as SKB_4K - 64.
 * Measured pitfall: when the modem wanted to read 4096 bytes of MC04_010 we
 * replied 4152 bytes; alloc skb failed outright, the reply was lost and the
 * modem waited forever -> MD_BOOT_HS2_FAIL.
 * Reply block 3 is the "actual length"; short reads are allowed by the
 * protocol (the modem asks again).
 */
/* PEARL-FSFIX-64: the modem requires the length reported in the reply to
 * equal w8 of the request. Measured on MC06_009: w8=4566 > 3420; the old
 * PEARL_FS_DATA_MAX truncation made dev_fs.c's CMPT_R validation fail with
 * 260, and finally lid_error_handle.c:239 asserted. Raise the data block cap
 * to what the whole-packet buffer can hold (PEARL_FS_MAX_MSG-64).
 */
#define PEARL_FS_DATA_MAX_BIG	(PEARL_FS_MAX_MSG - 64)
/* PEARL-FSMPFIX-73: data cap for a single CMPT_READ.
 *
 * The old implementation clamped want to PEARL_FS_DATA_MAX_BIG(8128), so any
 * read of a "whole record > 8128" was **silently truncated**: the modem
 * requested pl+40 (NR06_010 pl=61600 -> want=61640), the kernel returned only
 * 8128 bytes, far less than requested, so
 *   [E][ID:0x985][ret:260]read data from file/cache fail[record_idx:1][section_count:0]
 * and subsequently lid_error_handle.c asserted.
 * Measured LID container layout: file = 160(header) + rc*(pl+40),
 * record = pl + 8 sec_factor + 32 chksum -- so "read length = pl+40" is
 * required by the protocol. Raise the cap to the databuf capacity here;
 * oversized reads still use the vendor PEARL-FS-FRAG continuation shape.
 */
#define PEARL_FS_READ_MAX	(96U * 1024U)
/* Hard FS packet limit: ccci_bm.h notes that ccci_fsd caps the payload at
 * CCCI_MTU, plus ccci_header(16) and op_id(4) as headers, so a single packet
 * totals 3456+16+4 = 3476. The modem's own write requests are exactly 3476;
 * a CMPT_READ reply totals 56+data, and data above 3420 overflows the modem's
 * 3476-byte buffer and corrupts the payload of following frames.
 * (Proven in the 2026-09-25 storm: after a 3576 reply the modem sent
 * all-zero 0x1024 payloads.)*/
#define PEARL_FS_PKT_MAX	(CCCI_MTU + sizeof(struct ccci_header) + sizeof(unsigned int))
#define PEARL_FS_DATA_MAX	(PEARL_FS_PKT_MAX - 56)
/* PEARL-FSFIX-68: payload cap of a continuation (pure data fragment).
 * Continuation = ccci_header(16) + op(4) + payload -- no nblk field, data at +20.
 * Whole packet <= PEARL_FS_PKT_MAX(3476) => payload <= 3456.
 * The vendor rpcd@0x7794-0x77b8 / ccci_fsd@0xd1e0 both use this shape
 * (when remaining >= 0xd81 set bit31, clamp payload to 0xd80(3456), whole
 * packet 0xd94(3476)). The old FSFIX-64c implementation wrote an extra
 * nblk=0 (24-byte header), so after the modem stripped a 20-byte header the
 * continuation data was shifted by 4 bytes.
 */
#define PEARL_FS_CONT_MAX	(PEARL_FS_PKT_MAX - 20)
#define PEARL_FS_MAX_BLK	8
/* PEARL-FSFIX-67: the old value 24/16 is too small. The modem's NVRAM LID
 * enumeration touches 288 different files by name (see the mini dump stats
 * in notes/dev/md66-findings.txt §20.4); when the table fills up,
 * pearl_fs_file_new() returns -1 and OPEN replies status=1, failing the
 * whole enumeration.
 */
#define PEARL_FS_MAX_FILE	512
#define PEARL_FS_MAX_HANDLE	64
#define PEARL_FS_MAX_CAP	(256 * 1024)
#define PEARL_FS_OP_OPEN	0x1001
#define PEARL_FS_OP_SEEK	0x1002
#define PEARL_FS_OP_READ	0x1003
#define PEARL_FS_OP_WRITE	0x1004
#define PEARL_FS_OP_CLOSE	0x1005
#define PEARL_FS_OP_CLOSE_ALL	0x1006
#define PEARL_FS_OP_CREATE_DIR	0x1007	/* FS_CCCI_CreateDir */
#define PEARL_FS_OP_FILE_SIZE	0x1009
#define PEARL_FS_OP_CMPT_READ	0x1022
/* PEARL-FS-NVBOOTUP: ops the modem needs for its NVRAM first-boot-up. */
#define PEARL_FS_OP_RESTORE     0x1021
#define PEARL_FS_OP_MOVE        0x100c
#define PEARL_FS_OP_CMPT_WRITE  0x1024	/* whole-file read (with a status bitmap)*/
#define PEARL_FS_OP_FIND_FIRST	0x1012
#define PEARL_FS_OP_FIND_NEXT	0x1013
#define PEARL_FS_OP_FIND_CLOSE	0x1014
/* PEARL-FSFIX-69: the last batch of requests after the modem reaches
 * md_state=3 (HS1 done) are these two ops (querying mdota config file
 * attributes). They were unimplemented, fell into default and got -1001,
 * which the modem treated as fatal => all channels went silent for 36s =>
 * MD_BOOT_HS2_FAIL. The vendor ccci_fsd returns a legal errno
 * (error=2/ENOENT) for equally missing files and continues booting.
 */
#define PEARL_FS_OP_GET_ATTR	0x1010	/* FS_CCCI_GetAttributes */

/* PEARL-205: 1 = kernel intercepts and emulates FS (default 0; leave it to the userspace owner) */
unsigned int pearl_fs_emul;
module_param(pearl_fs_emul, uint, 0644);
MODULE_PARM_DESC(pearl_fs_emul, "1=kernel emulates FS (default 0, handled by the userspace owner)");
static unsigned int pearl202_enoent = 2;
module_param(pearl202_enoent, uint, 0644);
MODULE_PARM_DESC(pearl202_enoent, "reply error code for missing files (contract: 2 = ENOENT)");
#define PEARL_FS_OP_FILE_DETAIL	0x1025	/* FS_CCCI_GetFileDetail */

/* 0 = no reply (for A/B control); 1/2 = reserved degraded modes; >=2 normal replies */
static int pearl_fs_mode = 2;	/* PEARL: 2=normal (rescue default-off removed) */
module_param(pearl_fs_mode, int, 0644);

/* PEARL-RPC-KERNEL-69: reply value for 0x4010 SAR_TABLE_IDX_QUERY.
 * The vendor ccci_rpcd replies {value:0, ret:0} even when
 * mtk_sar_table_id_get fails. Adjustable without reflashing:
 * /sys/module/port_rpc/parameters/pearl_sar_table_id
 */
static int pearl_sar_table_id;
module_param(pearl_sar_table_id, int, 0644);
static int pearl_sar_rsp_args = 2;	/* 2 = {value,ret} (vendor 40 bytes) */
module_param(pearl_sar_rsp_args, int, 0644);
/* PEARL-RPC-KERNEL-69: value returned for 0x400F QUERY_AP_SYS_PROPERTY.
 * Measured on the vendor build: key<ro.product.vendor.name> -> value<yuechu>.
 */
static char pearl_ap_sys_prop_val[32] = "pearl";
module_param_string(ap_sys_prop_val, pearl_ap_sys_prop_val,
		    sizeof(pearl_ap_sys_prop_val), 0644);

/* PEARL-FSFIX-65: OPEN (0x1001) reply shape switch (runtime-writable, no reflash).
 *   0 = keep current nblk=1 {4: handle}
 *   1 = nblk=2 {4: 0}{4: handle} (the device shape inferred in FS_PROTOCOL.md §8)
 * The ret in the modem log "O: <path>, flag <mode>, ret <n>" comes from reply
 * blk0; for Close(0x1005)/GetFileSize(0x1009) blk0 is always the "result code
 * (0=success)", and only OPEN puts the handle in blk0 -- both shapes are kept
 * for on-site comparison:
 *   /sys/module/ccci_md_all/parameters/pearl_fs_open_rsp2
 */
static unsigned int pearl_fs_open_rsp2;
module_param(pearl_fs_open_rsp2, uint, 0644);
MODULE_PARM_DESC(pearl_fs_open_rsp2,
		 "PEARL FS(ccci_fs): OPEN reply shape 0={handle}, 1={0}{handle}");
MODULE_PARM_DESC(pearl_fs_mode, "PEARL FS(ccci_fs): 0=off, 2=normal");

/* ================ PEARL-FSFIX-66: config-file-driven reply variants ================
 *
 * Goal: cut the cost of "trying another reply shape" from "reflash + user
 * present" to "edit one line + soft reboot". Basis: /mnt/nvdata (Z:) is
 * already mounted before the modem sends its first FS request (~7.9 s)
 * (proven by Z:\BITMAP writing to disk successfully), so the on-disk config
 * file can be read safely here.
 *
 * File: /mnt/nvdata/md/pearl_fs.cfg (fallback /mnt/protect1/md/pearl_fs.cfg)
 * Format: key=value per line, # starts a comment; lazily re-read on every FS
 *       job, re-parsed and logged as one PEARL-FS-CFG line only when the
 *       content changes. Deleting a line restores its default.
 *
 * Options (all default to FSFIX-65's old behavior, so behavior is neutral
 * without a config file):
 *   open_rsp2 = 0|1                OPEN reply {handle} / {0}{handle}
 *   read_hdr1 = zero|got|want|<u32> second u32 of CMPT_READ reply blk0
 *                                  (old value always 0; exp=0 in the modem
 *                                    trace `read len(exp/r):0:44` matches it,
 *                                    needs measurement to rule out)
 *   read_out  = got|want            whether CMPT_READ reply blk2 reports got or want
 *   read_roff = auto|zero|<u32>     ignore descriptor w5 or force the offset
 *   read_want = auto|<u32>          ignore descriptor w8 or force the length
 *   write_two = <u32>               CMPT_WRITE reply blk1 (old value hardcoded 2)
 *   read_dump = 0|1                 dump the first 48 data bytes of the first 6 CMPT_READs
 */
#define PEARL_FS_CFG_NPATH	2
#define PEARL_FS_CFG_MAX	1024

static const char *pearl_fs_cfg_paths[PEARL_FS_CFG_NPATH] = {
	"/mnt/nvdata/md/pearl_fs.cfg",
	"/mnt/protect1/md/pearl_fs.cfg",
};

static char pearl_fs_cfg_raw[PEARL_FS_CFG_MAX];
static char pearl_fs_cfg_applied[PEARL_FS_CFG_MAX];
static unsigned int pearl_fs_cfg_gen;
static unsigned int pearl_fs_cfg_open_rsp2;
static unsigned int pearl_fs_read_hdr1_mode;	/* 0=zero 1=got 2=want 3=const */
static unsigned int pearl_fs_read_hdr1_const;
static unsigned int pearl_fs_read_out_mode;	/* 0=got 1=want */
static unsigned int pearl_fs_read_roff_mode;	/* 0=auto 1=const */
static unsigned int pearl_fs_read_roff_const;
static unsigned int pearl_fs_read_want_mode;	/* 0=auto 1=const */
static unsigned int pearl_fs_read_want_const;
static unsigned int pearl_fs_write_two = 2;
/* PEARL-190 fwd: two helper functions defined later in this file, used here first */
static unsigned int pearl_fs_put_block(unsigned char *dst, unsigned int off,
				       const void *data, unsigned int len);
static int pearl_fs_map_path(const char *mpath, char *out, unsigned int outlen);

/* ===== PEARL-190: directory enumeration (FindFirst/FindNext) ===== */
#define PEARL190_MAX_ENT 32
struct pearl190_dirent {
	unsigned char name[64];
};
static struct pearl190_dirent pearl190_ents[PEARL190_MAX_ENT];
static unsigned int pearl190_cnt, pearl190_idx;
static char pearl190_dir[256];

struct pearl190_ctx {
	struct dir_context ctx;
	int n;
};

static bool pearl190_actor(struct dir_context *ctx, const char *name, int nlen,
			   loff_t off, u64 ino, unsigned int d_type)
{
	struct pearl190_ctx *c = container_of(ctx, struct pearl190_ctx, ctx);

	if (c->n >= PEARL190_MAX_ENT || nlen <= 0 || nlen >= 64)
		return true;
	if (!strcmp(name, ".") || !strcmp(name, ".."))
		return true;
	memcpy(pearl190_ents[c->n].name, name, nlen);
	pearl190_ents[c->n].name[nlen] = 0;
	c->n++;
	return true;
}

/* first=1 rescans the directory; returns 0 on success (reply written), -1 when no more entries */
static unsigned int pearl201_find_h = 1;   /* PEARL-201: FIND_FIRST handle */
/* PEARL-202: value of the GETATTR single-block reply (contract {int val}) */
static unsigned int pearl202_attr_val;
module_param(pearl202_attr_val, uint, 0644);
MODULE_PARM_DESC(pearl202_attr_val, "value of the GETATTR(0x1010) reply (contract: single int block)");
static unsigned int pearl190_op;   /* PEARL-201 */
static int pearl190_find(int first, const char *name, unsigned char *reply,
			 unsigned int *ppos, unsigned int *pnblk,
			 unsigned int *pout)
{
	struct file *df;
	struct pearl190_ctx c;
	char lpath[256];
	unsigned int pos = *ppos;

	if (first) {
		if (!name[0])
			return -1;
		pearl_fs_map_path(name, lpath, sizeof(lpath));
		strscpy(pearl190_dir, lpath, sizeof(pearl190_dir));
		memset(&c, 0, sizeof(c));
		c.ctx.actor = pearl190_actor;
		df = filp_open(lpath, O_RDONLY | O_DIRECTORY, 0);
		if (IS_ERR(df))
			return -1;
		iterate_dir(df, &c.ctx);
		filp_close(df, NULL);
		pearl190_cnt = c.n;
		pearl190_idx = 0;
		pr_err("PEARL-190: enumerated %s -> %u entries\n", lpath, pearl190_cnt);
	}
	if (pearl190_idx >= pearl190_cnt)
		return -1;
	{
		/*
		 * PEARL-201: per the repo contract, FIND_FIRST/FIND_NEXT reply with 3 blocks:
		 *   {int handle} {4B entry} {name}
		 */
		unsigned int st = (pearl190_op == PEARL_FS_OP_FIND_FIRST) ?
			(unsigned int)pearl201_find_h : 0;
		unsigned int entry = pearl190_idx;
		unsigned int nlen = strlen(pearl190_ents[pearl190_idx].name);

		pos = pearl_fs_put_block(reply, pos, &st, 4);
		pos = pearl_fs_put_block(reply, pos, &entry, 4);
		pos = pearl_fs_put_block(reply, pos,
					 pearl190_ents[pearl190_idx].name, nlen);
		*pnblk = 3;
		*pout = nlen;
	}
	pearl190_idx++;
	*ppos = pos;
	return 0;
}

static void pearl190_find_close(void)
{
	pearl190_cnt = 0;
	pearl190_idx = 0;
}

/* PEARL-188: whether the modem may move staged files into NVRAM (allowed by default, 1) */
static unsigned int pearl_fs_move_nvram = 1;
/* PEARL-189: value carried in the 0x1016/0x1017 replies (default 0 = success) */
static unsigned int pearl189_val = 0;
/* PEARL-195: value carried in the 0x101a/0x101c replies */
static unsigned int pearl195_val = 0;
/* PEARL-201: reply values corrected per the firmware-verified contract in mtk-ccci-userspace */
static unsigned int pearl201_unlock_val = 1;   /* UNLOCK_ALL -> 1 */
static unsigned int pearl201_drive_val = 1;    /* GET_DRIVE / SET_DISK_FLAG -> 1 */
/* PEARL-198: 1=reply -9(ENOENT) instead of 1 when the file is missing (matches the reference) */
static unsigned int pearl198_enable = 1;
module_param(pearl198_enable, uint, 0644);
MODULE_PARM_DESC(pearl198_enable, "1=reply -9(ENOENT) for missing files (reference behavior)");
module_param(pearl195_val, uint, 0644);
MODULE_PARM_DESC(pearl195_val, "value carried in the 0x101a/0x101c replies (default 0)");
module_param(pearl189_val, uint, 0644);
MODULE_PARM_DESC(pearl189_val, "value carried in the 0x1016/0x1017 replies (default 0)");
module_param(pearl_fs_move_nvram, uint, 0644);
MODULE_PARM_DESC(pearl_fs_move_nvram, "1=allow moves into NVRAM (normal modem flow)");
static unsigned int pearl_fs_write_off24;   /* PEARL-187 */
static unsigned int woff_dummy;             /* PEARL-187 */
static unsigned int pearl_fs_read_dump = 1;
static unsigned int pearl_fs_read_dump_cnt;
/* PEARL-FSFIX-69: reply shapes and status values for 0x1010 / 0x1025
 * (runtime-switchable, see getattr_* / detail_* in pearl_fs.cfg).
 * shape 0=nblk1{st} 1=nblk2{st,attr} 2=nblk1{0} 3=nblk2{0,attr} 9=rotate 0..3
 */
static unsigned int pearl_fs_getattr_rsp = 1;
static unsigned int pearl_fs_getattr_st = 0xFFFFFFFEU;	/* -2 = ENOENT */
static unsigned int pearl_fs_getattr_attr;
static unsigned int pearl_fs_detail_rsp = 1;
static unsigned int pearl_fs_detail_st = 0xFFFFFFFEU;	/* -2 = ENOENT */
static unsigned int pearl_fs_detail_attr;
static unsigned int pearl_fs_getattr_rot;

/* PEARL-FSFIX-68: fragment-protocol shape switch for large CMPT_READ replies.
 *   1 = vendor shape (default): continuation = ccci_header(16) + op(4) + data
 *       (offset 20), total = 20 + n; the head packet's blk3 "declared length"
 *       reports the full out (the modem memcpys by blk3.len and continuations
 *       supply the excess).
 *   0 = old FSFIX-64c shape: continuation has an extra nblk=0 (offset 24),
 *       blk3 reports only the first fragment 3420.
 * Runtime-switchable: write frag68=0 in /mnt/nvdata/md/pearl_fs.cfg to go back.
 * Basis in the PEARL-FSFIX-68 note at the top of the file (rpcd/ccci_fsd
 * disassembly + the modem parser).
 */
static unsigned int pearl_fs_frag68 = 1;

/* PEARL-FSFIX-67: whether X:\nv_config maps to /mnt/nvdata/AllMap.
 *   0 = no mapping (default) -- OPEN/CMPT_READ use the normal drive path;
 *       /mnt/protect1/md/nv_config does not exist => ENOENT(-9), same as
 *       the vendor build.
 *   1 = map to AllMap (the old FSFIX-66-and-earlier behavior).
 *
 * Vendor ccci_fsd boot log measured (notes/dev/fsd-orig-seq.txt):
 *   O: X:/nv_config, flag 0x500, ret -9      <-- the vendor build fails too
 *   O: X:/MT00A001, flag 0x700, ret 2        <-- only then the full LID enumeration
 * Making it succeed => the modem thinks "a valid LID config already exists"
 * => skips the LID enumeration => nvram_lid_cache stays empty => reading LID
 * 0xF00A gives section_count=0 / exp=0 => asserts at lid_error_handle.c with
 * para0=0x228d para1=0xf00a para2=0x2240.
 * Runtime-switchable: write nvcfg=1 in /mnt/nvdata/md/pearl_fs.cfg to go back.
 */
static unsigned int pearl_fs_nvcfg;

/* PEARL-FSMPFIX-75: CCISM handshake runtime-switchable (shares
 * /mnt/nvdata/md/pearl_fs.cfg with the FS variants).
 *   ccism_auto   = 0|1   whether to auto-send CCISM_SHM_INIT(0x119) while
 *                        waiting for HS2, default 1
 *   ccism_ms     = <u32> auto-send delay in ms, default 2000
 *   ccism_11b    = 0|1   whether to also send CCISM_SHM_INIT_DONE(0x11B), default 0
 *   ccism_11b_ms = <u32> delay of 0x11B relative to the planned time in ms, default 3000
 */
static unsigned int pearl_fs_ccism_auto = 1;
static unsigned int pearl_fs_ccism_ms = 2000;
static unsigned int pearl_fs_ccism_11b;
static unsigned int pearl_fs_ccism_11b_ms = 3000;

/* minimal integer parser (supports a 0x prefix, stops at the first non-digit); returns whether a number was parsed */
static int pearl_fs_cfg_atoi(const char *s, unsigned int *out)
{
	unsigned int base = 10, v = 0, n = 0;

	if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
		base = 16;
		s += 2;
	}
	for (;; s++) {
		unsigned int d;

		if (*s >= '0' && *s <= '9')
			d = *s - '0';
		else if (base == 16 && *s >= 'a' && *s <= 'f')
			d = *s - 'a' + 10;
		else if (base == 16 && *s >= 'A' && *s <= 'F')
			d = *s - 'A' + 10;
		else
			break;
		v = v * base + d;
		n++;
	}
	*out = v;
	return n ? 1 : 0;
}

/* get the value of key=value and strip trailing CR/whitespace */
static char *pearl_fs_cfg_val(char *line)
{
	char *eq = strchr(line, '=');
	char *v, *e;

	if (eq == NULL)
		return NULL;
	v = eq + 1;
	while (*v == ' ' || *v == '\t')
		v++;
	e = v + strlen(v);
	while (e > v && (e[-1] == '\r' || e[-1] == '\n' ||
			 e[-1] == ' ' || e[-1] == '\t'))
		*--e = 0;
	return v;
}

/* whether line starts with "key=" */
static int pearl_fs_cfg_key(const char *line, const char *key)
{
	unsigned int n = strlen(key);

	/* PEARL-FSFIX-67: tolerate `key = value` (spaces around the equals sign).
	 * The old implementation required a tight `key=`, an easy pitfall when
	 * hand-writing configs (hit during the e1 round).
	 */
	if (strncmp(line, key, n) != 0)
		return 0;
	while (line[n] == ' ' || line[n] == '\t')
		n++;
	return (line[n] == '=');
}

static void pearl_fs_cfg_parse(void)
{
	char *p = pearl_fs_cfg_raw;
	char *line;

	while (p != NULL && *p) {
		line = strsep(&p, "\n");
		if (line == NULL)
			break;
		while (*line == ' ' || *line == '\t')
			line++;
		if (*line == 0 || *line == '#' || *line == '\r')
			continue;
		{
			char *v = pearl_fs_cfg_val(line);
			unsigned int x;

			if (v == NULL)
				continue;
			if (pearl_fs_cfg_key(line, "open_rsp2")) {
				pearl_fs_cfg_open_rsp2 = (v[0] == '1');
			} else if (pearl_fs_cfg_key(line, "read_dump")) {
				pearl_fs_read_dump = (v[0] != '0');
			} else if (pearl_fs_cfg_key(line, "nvcfg")) {
				/* PEARL-FSFIX-67: nvcfg=1 restores the FSFIX-66 old behavior (map to AllMap) */
				pearl_fs_nvcfg = (v[0] == '1');
			} else if (pearl_fs_cfg_key(line, "frag68")) {
				/* PEARL-FSFIX-68: frag68=0 restores FSFIX-64c's 24-byte continuation header */
				pearl_fs_frag68 = (v[0] != '0');
			} else if (pearl_fs_cfg_key(line, "write_two")) {
				if (pearl_fs_cfg_atoi(v, &x))
					pearl_fs_write_two = x;
			} else if (pearl_fs_cfg_key(line, "read_out")) {
				pearl_fs_read_out_mode = (v[0] == 'w') ? 1 : 0;
			} else if (pearl_fs_cfg_key(line, "read_hdr1")) {
				if (v[0] == 'z')
					pearl_fs_read_hdr1_mode = 0;
				else if (v[0] == 'g')
					pearl_fs_read_hdr1_mode = 1;
				else if (v[0] == 'w')
					pearl_fs_read_hdr1_mode = 2;
				else if (pearl_fs_cfg_atoi(v, &x)) {
					pearl_fs_read_hdr1_mode = 3;
					pearl_fs_read_hdr1_const = x;
				}
			} else if (pearl_fs_cfg_key(line, "read_roff")) {
				if (v[0] == 'a') {
					pearl_fs_read_roff_mode = 0;
				} else if (v[0] == 'z') {
					pearl_fs_read_roff_mode = 1;
					pearl_fs_read_roff_const = 0;
				} else if (pearl_fs_cfg_atoi(v, &x)) {
					pearl_fs_read_roff_mode = 1;
					pearl_fs_read_roff_const = x;
				}
			} else if (pearl_fs_cfg_key(line, "read_want")) {
				if (v[0] == 'a') {
					pearl_fs_read_want_mode = 0;
				} else if (pearl_fs_cfg_atoi(v, &x)) {
					pearl_fs_read_want_mode = 1;
					pearl_fs_read_want_const = x;
				}
			} else if (pearl_fs_cfg_key(line, "getattr_rsp")) {
				/* PEARL-FSFIX-69 */
				if (pearl_fs_cfg_atoi(v, &x))
					pearl_fs_getattr_rsp = x;
			} else if (pearl_fs_cfg_key(line, "getattr_st")) {
				if (pearl_fs_cfg_atoi(v, &x))
					pearl_fs_getattr_st = x;
			} else if (pearl_fs_cfg_key(line, "getattr_attr")) {
				if (pearl_fs_cfg_atoi(v, &x))
					pearl_fs_getattr_attr = x;
			} else if (pearl_fs_cfg_key(line, "detail_rsp")) {
				if (pearl_fs_cfg_atoi(v, &x))
					pearl_fs_detail_rsp = x;
			} else if (pearl_fs_cfg_key(line, "detail_st")) {
				if (pearl_fs_cfg_atoi(v, &x))
					pearl_fs_detail_st = x;
			} else if (pearl_fs_cfg_key(line, "ccism_auto")) {
				pearl_fs_ccism_auto = (v[0] != '0');
			} else if (pearl_fs_cfg_key(line, "ccism_ms")) {
				if (pearl_fs_cfg_atoi(v, &x))
					pearl_fs_ccism_ms = x;
			} else if (pearl_fs_cfg_key(line, "ccism_11b")) {
				pearl_fs_ccism_11b = (v[0] == '1');
			} else if (pearl_fs_cfg_key(line, "ccism_11b_ms")) {
				if (pearl_fs_cfg_atoi(v, &x))
					pearl_fs_ccism_11b_ms = x;
			} else if (pearl_fs_cfg_key(line, "detail_attr")) {
				if (pearl_fs_cfg_atoi(v, &x))
					pearl_fs_detail_attr = x;
			}
		}
	}
}

/* lazy config re-read: keep defaults when there is no file; skip re-parsing when unchanged */
static void pearl_fs_cfg_refresh(void)
{
	struct file *f;
	loff_t pos = 0;
	int n = 0, i;

	for (i = 0; i < PEARL_FS_CFG_NPATH && n <= 0; i++) {
		f = filp_open(pearl_fs_cfg_paths[i], O_RDONLY, 0);
		if (IS_ERR(f))
			continue;
		n = kernel_read(f, pearl_fs_cfg_raw,
				PEARL_FS_CFG_MAX - 1, &pos);
		filp_close(f, NULL);
	}
	if (n <= 0)
		return;
	pearl_fs_cfg_raw[n] = 0;
	if (strcmp(pearl_fs_cfg_raw, pearl_fs_cfg_applied) == 0)
		return;
	memcpy(pearl_fs_cfg_applied, pearl_fs_cfg_raw, n + 1);
	/* restore defaults before applying: deleting a line restores the old behavior */
	pearl_fs_cfg_open_rsp2 = 0;
	pearl_fs_read_hdr1_mode = 0;
	pearl_fs_read_hdr1_const = 0;
	pearl_fs_read_out_mode = 0;
	pearl_fs_read_roff_mode = 0;
	pearl_fs_read_roff_const = 0;
	pearl_fs_read_want_mode = 0;
	pearl_fs_read_want_const = 0;
	pearl_fs_write_two = 2;
	pearl_fs_read_dump = 1;
	pearl_fs_nvcfg = 0;	/* PEARL-FSFIX-67: do not map nv_config by default */
	pearl_fs_frag68 = 1;	/* PEARL-FSFIX-68: use the vendor fragment shape by default */
	pearl_fs_getattr_rsp = 1;	/* PEARL-FSFIX-69 */
	pearl_fs_getattr_st = 0xFFFFFFFEU;
	pearl_fs_getattr_attr = 0;
	pearl_fs_detail_rsp = 1;
	pearl_fs_detail_st = 0xFFFFFFFEU;
	pearl_fs_detail_attr = 0;
	pearl_fs_ccism_auto = 1;	/* PEARL-FSMPFIX-75 */
	pearl_fs_ccism_ms = 2000;
	pearl_fs_ccism_11b = 0;
	pearl_fs_ccism_11b_ms = 3000;
	pearl_fs_cfg_parse();
	pearl_fs_cfg_gen++;
	pr_info("PEARL-FS-CFG: gen=%u nvcfg=%u rsp2=%u hdr1=%u/%u out=%u roff=%u/%u want=%u/%u two=%u dump=%u frag68=%u ccism=%u/%u/%u/%u\n",
		pearl_fs_cfg_gen, pearl_fs_nvcfg, pearl_fs_cfg_open_rsp2,
		pearl_fs_read_hdr1_mode, pearl_fs_read_hdr1_const,
		pearl_fs_read_out_mode,
		pearl_fs_read_roff_mode, pearl_fs_read_roff_const,
		pearl_fs_read_want_mode, pearl_fs_read_want_const,
		pearl_fs_write_two, pearl_fs_read_dump, pearl_fs_frag68,
		pearl_fs_ccism_auto, pearl_fs_ccism_ms,
		pearl_fs_ccism_11b, pearl_fs_ccism_11b_ms);
}

/* PEARL-FSMPFIX-75: lets ccci_fsm_scp.c read the runtime-switchable CCISM
 * options. Each call refreshes first (no re-parse when unchanged), so
 * "edit cfg + soft reboot" takes effect immediately. */
unsigned int pearl_ccism_cfg_auto(void)
{
	pearl_fs_cfg_refresh();
	return pearl_fs_ccism_auto;
}
EXPORT_SYMBOL(pearl_ccism_cfg_auto);

unsigned int pearl_ccism_cfg_ms(void)
{
	pearl_fs_cfg_refresh();
	return pearl_fs_ccism_ms;
}
EXPORT_SYMBOL(pearl_ccism_cfg_ms);

unsigned int pearl_ccism_cfg_11b(void)
{
	pearl_fs_cfg_refresh();
	return pearl_fs_ccism_11b;
}
EXPORT_SYMBOL(pearl_ccism_cfg_11b);

unsigned int pearl_ccism_cfg_11b_ms(void)
{
	pearl_fs_cfg_refresh();
	return pearl_fs_ccism_11b_ms;
}
EXPORT_SYMBOL(pearl_ccism_cfg_11b_ms);

/* evidence: whether the cfg was actually read, and which generation */
unsigned int pearl_ccism_cfg_gen(void)
{
	pearl_fs_cfg_refresh();
	return pearl_fs_cfg_gen;
}
EXPORT_SYMBOL(pearl_ccism_cfg_gen);


static atomic_t pearl_fs_msg_cnt = ATOMIC_INIT(0);
static atomic_t pearl_fs_mpdump_cnt = ATOMIC_INIT(0);

/* PEARL-FS-MP: capture raw bytes of multi-packet writes (max 4 per shape, up to 320 bytes each) */
static void pearl_fs_dump_req(const char *tag, const unsigned char *msg,
	unsigned int len)
{
	unsigned int n = min(len, 320u);
	unsigned int off;

	if (atomic_inc_return(&pearl_fs_mpdump_cnt) > 12)
		return;
	pr_err("PEARL-FS-MP: %s len=%u w0=0x%08x w1=%u seq=0x%04x op=0x%04x nblk=%u\n",
		tag, len, *(unsigned int *)msg, *(unsigned int *)(msg + 4),
		(*(unsigned int *)(msg + 8) >> 16) & 0xffff,
		*(unsigned int *)(msg + 16), *(unsigned int *)(msg + 20));
	for (off = 0; off < n; off += 32) {
		unsigned int k = min(32u, n - off);
		char buf[3 * 32 + 1];
		unsigned int i2;
		unsigned char ch;

		for (i2 = 0; i2 < k; i2++) {
			ch = msg[off + i2];
			buf[i2 * 3] = "0123456789abcdef"[ch >> 4];
			buf[i2 * 3 + 1] = "0123456789abcdef"[ch & 15];
			buf[i2 * 3 + 2] = ' ';
		}
		buf[k * 3] = 0;
		pr_err("PEARL-FS-MP: %04x: %s\n", off, buf);
	}
}
/* PEARL-FSMPFIX-71: raw-byte dump of multi-packet heads (max 8 times, first
 * 224 bytes each). On the reference device the desc of 34 single-packet
 * 0x1024s is always w4=0xa0(160), but our multi-packet heads read w4=0, so
 * the raw bytes must confirm which descriptor word the offset really is in. */
static atomic_t pearl_fs_hd_cnt = ATOMIC_INIT(0);

static void pearl_fs_dump_hex(const char *tag, const unsigned char *msg,
			      unsigned int len, unsigned int limit)
{
	unsigned int off, n = min(len, limit);

	pr_err("PEARL-FS-HEX: %s len=%u\n", tag, len);
	for (off = 0; off < n; off += 32) {
		unsigned int k = min(32u, n - off);
		unsigned int i2;
		char buf[3 * 32 + 1];

		for (i2 = 0; i2 < k; i2++) {
			unsigned char ch = msg[off + i2];

			buf[i2 * 3] = "0123456789abcdef"[ch >> 4];
			buf[i2 * 3 + 1] = "0123456789abcdef"[ch & 15];
			buf[i2 * 3 + 2] = ' ';
		}
		buf[k * 3] = 0;
		pr_err("PEARL-FS-HEX: %04x: %s\n", off, buf);
	}
}
static DEFINE_MUTEX(pearl_fs_lock);

struct pearl_fs_file {
	int used;
	char name[96];
	unsigned char *data;
	unsigned int size;
	unsigned int cap;
};

struct pearl_fs_handle {
	int used;
	int file;
	unsigned int pos;
	struct file *fp;	/* real file handle (under /mnt/nvdata) */
};

static struct pearl_fs_file pearl_fs_files[PEARL_FS_MAX_FILE];
static struct pearl_fs_handle pearl_fs_handles[PEARL_FS_MAX_HANDLE];

static struct pearl_fs_job {
	struct list_head node;
	unsigned char md_id;
	unsigned int len;
	unsigned char data[];
};

/*
 * PEARL-FS-QUEUE: this used to keep only one job slot; new requests arriving
 * while the previous one was running were silently dropped and the modem could
 * only wait for a timeout (measured: after the last FS request it idled ~5s
 * and entered lid_error_handle). Changed to a linked-list queue + a single
 * serial worker, no more drops.
 */
static LIST_HEAD(pearl_fs_pending);
static DEFINE_SPINLOCK(pearl_fs_q_lock);
static atomic_t pearl_fs_q_depth = ATOMIC_INIT(0);
static struct work_struct pearl_fs_work;
#define PEARL_FS_Q_MAX 64

static int pearl_fs_file_find(const char *name)
{
	int i;

	for (i = 0; i < PEARL_FS_MAX_FILE; i++)
		if (pearl_fs_files[i].used &&
		    strcmp(pearl_fs_files[i].name, name) == 0)
			return i;
	return -1;
}

static int pearl_fs_file_new(const char *name)
{
	int i;

	for (i = 0; i < PEARL_FS_MAX_FILE; i++) {
		if (!pearl_fs_files[i].used) {
			pearl_fs_files[i].used = 1;
			strscpy(pearl_fs_files[i].name, name,
				sizeof(pearl_fs_files[i].name));
			pearl_fs_files[i].data = NULL;
			pearl_fs_files[i].size = 0;
			pearl_fs_files[i].cap = 0;
			return i;
		}
	}
	return -1;
}

static int pearl_fs_file_reserve(struct pearl_fs_file *f, unsigned int need)
{
	unsigned int cap;
	unsigned char *p;

	if (need <= f->cap)
		return 0;
	if (need > PEARL_FS_MAX_CAP)
		return -1;
	for (cap = 4096; cap < need; cap <<= 1)
		;
	p = kmalloc(cap, GFP_KERNEL);
	if (p == NULL)
		return -1;
	if (f->data != NULL) {
		memcpy(p, f->data, f->size);
		kfree(f->data);
	}
	f->data = p;
	f->cap = cap;
	return 0;
}

static int pearl_fs_handle_alloc(int file)
{
	int i;

	for (i = 0; i < PEARL_FS_MAX_HANDLE; i++) {
		if (!pearl_fs_handles[i].used) {
			pearl_fs_handles[i].used = 1;
			pearl_fs_handles[i].file = file;
			pearl_fs_handles[i].pos = 0;
			return i + 1;
		}
	}
	return 0;
}

/* ---- /proc/pearl_fs: read out the files the modem wrote in ---- */
static int pearl_fs_proc_show(struct seq_file *m, void *v)
{
	int i, n;

	mutex_lock(&pearl_fs_lock);
	for (i = 0; i < PEARL_FS_MAX_FILE; i++) {
		if (!pearl_fs_files[i].used)
			continue;
		seq_printf(m, "=== [%d] %s  size=%u cap=%u ===\n", i,
			pearl_fs_files[i].name, pearl_fs_files[i].size,
			pearl_fs_files[i].cap);
		if (pearl_fs_files[i].data != NULL) {
			n = pearl_fs_files[i].size;
			if (n > 65536)
				n = 65536;
			/* output as text, non-printable characters replaced with '.' */
			{
				char *buf = kmalloc(n + 1, GFP_KERNEL);
				int k;

				if (buf != NULL) {
					for (k = 0; k < n; k++) {
						unsigned char c =
							pearl_fs_files[i].data[k];

						buf[k] = (c >= 0x20 && c < 0x7f)
							? (char)c : '.';
					}
					buf[n] = 0;
					seq_puts(m, buf);
					kfree(buf);
				}
			}
			seq_puts(m, "\n");
		}
	}
	mutex_unlock(&pearl_fs_lock);
	return 0;
}

static int pearl_fs_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, pearl_fs_proc_show, NULL);
}

static const struct proc_ops pearl_fs_proc_fops = {
	.proc_open = pearl_fs_proc_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static void pearl_fs_proc_init(void)
{
	proc_create("pearl_fs", 0444, NULL, &pearl_fs_proc_fops);
}

/* ---- message construction ---- */
/*
 * Hard cap of the reply buffer. pearl_fs_put_block() used to have no bounds
 * check at all: CMPTREAD would write a data block of up to 4096 bytes after
 * pos had already used ~40, totaling 4140 > reply[4096] => smashing the stack
 * of pearl_fs_process_job(). Once the offset was fixed the modem began
 * reading full records (larger lengths), triggering it: the kernel ran away
 * -> TZ watchdog reset -> boot loop. Now it truncates (keeping 4-byte
 * alignment) and reports an error instead of ever overflowing.
 */
static unsigned int pearl_fs_put_block(unsigned char *dst, unsigned int off,
	const void *data, unsigned int len)
{
	unsigned int alen = (len + 3) & ~3U;

	if (off + sizeof(unsigned int) + alen > PEARL_FS_MAX_MSG) {
		unsigned int room = 0;

		if (PEARL_FS_MAX_MSG > off + sizeof(unsigned int))
			room = PEARL_FS_MAX_MSG - off - sizeof(unsigned int);
		room &= ~3U;
		pr_err("PEARL-FS: reply overflow off=%u len=%u cap=%u -> truncate %u\n",
			off, len, (unsigned int)PEARL_FS_MAX_MSG, room);
		len = room;
		alen = room;
	}
	*(unsigned int *)(dst + off) = len;
	off += sizeof(unsigned int);
	if (len)
		memcpy(dst + off, data, len);
	if (alen != len)
		memset(dst + off + len, 0, alen - len);
	return off + alen;
}

static void pearl_fs_wcs2cs(const unsigned char *wcs, unsigned int len,
	char *out, unsigned int out_len)
{
	unsigned int i;

	out[0] = 0;
	for (i = 0; i + 1 < len && (i / 2) + 1 < out_len; i += 2) {
		unsigned int c = wcs[i] | (wcs[i + 1] << 8);

		if (c == 0)
			break;
		out[i / 2] = (c < 0x80) ? (char)c : '?';
	}
	out[(i / 2) < out_len ? (i / 2) : (out_len - 1)] = 0;
}

static void pearl_fs_send(unsigned char md_id, unsigned char *msg,
	unsigned int len)
{
	struct port_t *port;
	struct sk_buff *skb;
	void *ptr;

	port = port_get_by_channel(md_id, CCCI_FS_TX);
	if (port == NULL) {
		pr_err("PEARL-FS: cannot find CCCI_FS_TX port\n");
		return;
	}
	/* PEARL-FSFIX-64: large replies (the modem reading a 4566-byte record)
	 * must go out as a whole packet. ccci_alloc_skb()'s first statement is
	 * `if (size > SKB_4K) goto err_exit;`, so packets > SKB_4K are allocated
	 * directly with __dev_alloc_skb(). ccci_free_skb() reads head_magic at
	 * skb->head + NET_SKB_PAD - sizeof(buf_ctrl) to decide the policy; clear
	 * it explicitly here so it != CCCI_BUF_MAGIC, taking the FREE branch
	 * (dev_kfree_skb_any) instead of being recycled as a pool object.
	 */
	if (len > SKB_4K) {
		struct ccci_buffer_ctrl *bc;

		pr_err("PEARL-FS: reply %u > SKB_4K(%u), using big skb\n",
			len, (unsigned int)SKB_4K);
		skb = __dev_alloc_skb(len, GFP_KERNEL);
		if (skb == NULL) {
			pr_err("PEARL-FS: big alloc skb(%u) fail\n", len);
			return;
		}
		bc = (struct ccci_buffer_ctrl *)(skb->head + NET_SKB_PAD -
						 sizeof(*bc));
		memset(bc, 0, sizeof(*bc));
	} else {
		skb = ccci_alloc_skb(len, 1, 1);
	}
	if (skb == NULL) {
		pr_err("PEARL-FS: alloc skb(%u) fail\n", len);
		return;
	}
	ptr = skb_put(skb, len);
	memcpy(ptr, msg, len);
	if (port_send_skb_to_md(port, skb, 1) != 0) {
		pr_err("PEARL-FS: send reply fail\n");
		ccci_free_skb(skb);
	}
}


/*
 * Modem path -> Linux path.
 *   Z:\NVRAM\CALIBRAT\ML09_001 -> /mnt/nvdata/md/NVRAM/CALIBRAT/ML09_001
 * The real root of X:\ is not fully determined yet, so probe the candidate roots in order and use whichever opens.
 */
/*
 * PEARL-FS-DRIVES: each drive letter must map to its own real storage.
 *
 * Measured on the real device (mounting protect1/protect2 and looking):
 *   X:\LD36_003 / X:\ER1D_002 / X:\MTBT_000 / X:\AA01_010 / X:\MC00_007 ...
 *     -> /mnt/protect1/md/ (35 files; protect2/md is the second copy, 32 files)
 *   Z:\NVRAM\CALIBRAT\CC00_001 -> /mnt/nvdata/md/NVRAM/CALIBRAT/CC00_001
 *
 * This used to strip the drive letter and probe candidate roots "whichever
 * opens", so every X: read landed on the empty /mnt/nvcfg/ directory on the
 * root filesystem. The modem could not see its own LID storage,
 * nvram_get_dev_boot_times() failed to read LID 0xF00A (bt stuck at 1), and
 * finally lid_error_handle.c asserted and HS2 never happened.
 */
static const struct {
	char drive;
	const char *root;
} pearl_fs_drives[] = {
	{ 'X', "/mnt/protect1/md/" },
	{ 'Y', "/mnt/protect2/md/" },
	{ 'Z', "/mnt/nvdata/md/" },
	/* S: is the modem's OTA drive. The modem creates S:\mdota with
	 * op=0x1007; the string inside the mcf_ota_a partition is
	 * /mnt/vendor/mdota, so the root of S: is /mnt/vendor/ (the mdota
	 * subdirectory is mounted from that partition).
	 */
	{ 'S', "/mnt/vendor/" },
};

/* probe candidates for an unknown drive letter (keeps the old behavior) */
static const char *pearl_fs_roots[] = {
	"/mnt/protect1/md/",
	"/mnt/protect2/md/",
	"/mnt/nvdata/md/",
	"/mnt/vendor/",
};

/* PEARL-FS-NVCFG (corrected by FSFIX-67):
 * An old comment asserted "this open must succeed"; that was **wrong**. The
 * authoritative vendor reference (notes/dev/fsd-orig-seq.txt, from
 * notes/yuechu/lc_boot.txt) shows:
 *   O: X:/nv_config, flag 0x500, ret -9      <-- the vendor build fails too
 *   O: X:/MT00A001, flag 0x700, ret 2        <-- only then the full LID enumeration
 * Passing off the APCFG index AllMap as nv_config to make it succeed only
 * made the modem skip the LID enumeration and leave nvram_lid_cache empty,
 * ending in an assert on LID 0xF00A. The table is now disabled by default
 * (see pearl_fs_nvcfg) and kept only as an escape hatch.
 */
static const struct {
	const char *modem_path;
	const char *linux_path;
} pearl_fs_path_map[] = {
	/* PEARL-FS-NVCFG-ON: this open has to SUCCEED. The modem's own
	 * trace shows the failure is fatal:
	 *   [C][OP:FS_OP_OPEN][file:X:\nv_config]
	 *   [E][OP:FS_OP_OPEN][hd:-9][ret:-9]
	 *   [C][ID:0xFFFFFFFF][ret:0][bt:1][init_index:0]
	 * after which it restarts its NVRAM init from the top and loops
	 * forever without ever finishing HS2. /mnt/nvdata/AllMap is the real
	 * 24272-byte NVRAM index table and is what nv_config has to be; the
	 * no-O_CREAT rule still applies, so it is served but never faked.
	 */
	{ "X:\\nv_config", "/mnt/nvdata/AllMap" },
};

static int pearl_fs_map_path(const char *mpath, char *out, unsigned int outlen)
{
	const char *rest = mpath;
	unsigned int i, k = 0;
	char cand[256];
	int r;
	char drv = 0;

	/* PEARL-FSFIX-67: skip the whole table when nvcfg=0 (default).
	 * X:\nv_config then goes through the normal drive path ->
	 * /mnt/protect1/md/nv_config; the file does not exist -> ENOENT(-9),
	 * exactly like the vendor ccci_fsd.
	 */
	for (i = 0; pearl_fs_nvcfg && i < ARRAY_SIZE(pearl_fs_path_map); i++) {
		if (strcmp(mpath, pearl_fs_path_map[i].modem_path) == 0) {
			struct file *f = filp_open(pearl_fs_path_map[i].linux_path,
						   O_RDONLY, 0);

			if (!IS_ERR(f)) {
				filp_close(f, NULL);
				strscpy(out, pearl_fs_path_map[i].linux_path, outlen);
				return 0;
			}
			pr_err("PEARL-FS: nv_config map target missing: %s\n",
			       pearl_fs_path_map[i].linux_path);
		}
	}

	if (mpath[0] != 0 && mpath[1] == ':') {
		drv = mpath[0];
		rest = mpath + 2;
		if (rest[0] == '\\' || rest[0] == '/')
			rest++;
	}

	/* known drive letter: prepend its own root directly, regardless of whether the file exists (write paths must land right too) */
	for (i = 0; i < ARRAY_SIZE(pearl_fs_drives); i++) {
		if (drv == 0 || drv != pearl_fs_drives[i].drive)
			continue;
		k = 0;
		k += scnprintf(cand + k, sizeof(cand) - k, "%s",
			pearl_fs_drives[i].root);
		{
			const char *p = rest;

			while (*p != 0 && k + 1 < sizeof(cand)) {
				cand[k++] = (*p == '\\') ? '/' : *p;
				p++;
			}
		}
		cand[k] = 0;
		strscpy(out, cand, outlen);
		return 0;
	}

	for (r = 0; r < ARRAY_SIZE(pearl_fs_roots); r++) {
		k = 0;
		k += scnprintf(cand + k, sizeof(cand) - k, "%s",
			pearl_fs_roots[r]);
		{
			const char *p = rest;

			while (*p != 0 && k + 1 < sizeof(cand)) {
				cand[k++] = (*p == '\\') ? '/' : *p;
				p++;
			}
		}
		cand[k] = 0;
		/* probe directly with filp_open: if it exists, it opens */
		{
			struct file *f = filp_open(cand, O_RDONLY, 0);

			if (!IS_ERR(f)) {
				filp_close(f, NULL);
				strscpy(out, cand, outlen);
				return 0;
			}
		}
	}
	/* none exist: return the first candidate; the caller treats it as an empty file */
	k = 0;
	k += scnprintf(cand + k, sizeof(cand) - k, "%s", pearl_fs_roots[0]);
	{
		const char *p = rest;

		while (*p != 0 && k + 1 < sizeof(cand)) {
			cand[k++] = (*p == '\\') ? '/' : *p;
			p++;
		}
	}
	cand[k] = 0;
	strscpy(out, cand, outlen);
	return -1;
}

/* read up to maxlen bytes from offset start; return the bytes actually read, -1 on failure */
static int pearl_fs_read_file(const char *lpath, unsigned char *out,
	unsigned int maxlen, loff_t start)
{
	struct file *f;
	loff_t pos = start;
	int ret;

	f = filp_open(lpath, O_RDONLY, 0);
	if (IS_ERR(f))
		return -1;
	ret = kernel_read(f, out, maxlen, &pos);
	filp_close(f, NULL);
	return (ret < 0) ? -1 : ret;
}

/*
 * Whole-file copy: must be chunked.
 * Move used to read the entire file into the 4096-byte databuf and write it
 * out, truncating LID files >4KB (measured MC01_006/MC09_000 5388 -> 4096),
 * which corrupted the modem's backup copies.
 * Returns the number of bytes copied, or a negative errno on failure.
 */
#define PEARL_FS_COPY_CHUNK	65536

static int pearl_fs_copy_file(const char *src, const char *dst,
	unsigned int *written)
{
	struct file *in, *outf;
	loff_t rp = 0, wp = 0;
	unsigned char *buf;
	int n, total = 0;

	*written = 0;
	buf = kmalloc(PEARL_FS_COPY_CHUNK, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	in = filp_open(src, O_RDONLY, 0);
	if (IS_ERR(in)) {
		kfree(buf);
		return PTR_ERR(in);
	}
	outf = filp_open(dst, O_RDWR | O_CREAT | O_TRUNC, 0660);
	if (IS_ERR(outf)) {
		int err = PTR_ERR(outf);

		filp_close(in, NULL);
		kfree(buf);
		return err;
	}

	for (;;) {
		n = kernel_read(in, buf, PEARL_FS_COPY_CHUNK, &rp);
		if (n < 0) {
			total = -EIO;
			break;
		}
		if (n == 0)
			break;
		if (kernel_write(outf, buf, n, &wp) != n) {
			total = -EIO;
			break;
		}
		total += n;
	}

	if (total >= 0)
		vfs_fsync(outf, 0);
	filp_close(outf, NULL);
	filp_close(in, NULL);
	kfree(buf);
	if (total >= 0)
		*written = (unsigned int)total;
	return total;
}

/*
 * Keep a backup before the modem first rewrites an NVRAM file.
 * The backup goes in the /mnt/nvdata/ root (not /mnt/nvdata/md) because Z:
 * maps only to md/, so the backup never appears in the tree the modem sees.
 * An existing backup is not overwritten.
 */
static void pearl_fs_backup_once(const char *path)
{
	char bak[256];
	const char *base;

	if (!strstr(path, "NVRAM"))
		return;
	base = strrchr(path, '/');
	base = base ? base + 1 : path;
	if (scnprintf(bak, sizeof(bak), "/mnt/nvdata/pearl-nvrambak-%s",
		      base) >= (int)sizeof(bak))
		return;

	{
		struct file *f = filp_open(bak, O_RDONLY, 0);

		if (!IS_ERR(f)) {
			filp_close(f, NULL);
			return;	/* already backed up */
		}
	}
	{
		unsigned int got = 0;
		int r = pearl_fs_copy_file(path, bak, &got);

		pr_info("PEARL-FS: nvram backup %s -> %s (%d bytes, ret=%d)\n",
			path, bak, got, r);
	}
}

/* PEARL-FS-MP: reassembly state for multi-packet 0x1024 writes (single serial
 * worker, no locking needed).
 * Measured frame shapes (2026-09-25 capture):
 *   head w0=0x80000000: op+nblk=3+{name}{desc44}{blk2.len=total(deliberately out of bounds)}+first data block
 *   middle w0=0x80000000: op+nblk=0+pure data
 *   end w0=0x00000000: op+nblk=0+pure data tail block
 * Total data length = blk2.len; when full or on the end packet, write to disk
 * and send one success reply. */
static struct {
	int		active;
	unsigned int	total, got;
	unsigned char	*buf;
	unsigned int	wsteps, wbaddr, woff;
	unsigned char	desc[44];
	char		path[256];
} pearl_fs_mpw;

/* PEARL-FS-MP-CONT-64: when a multi-packet stream delivers only got(<total),
 * the modem tops up the remaining bytes with a separate 0x1024 (measured
 * X:\MC06_009: 4724 + 2 = 4726). That packet's w4=0 is not a continuation
 * offset; we must continue from the "end position of the previous
 * multi-packet write". Otherwise the 2 bytes land at offset 0, overwriting
 * the file header's "LI" with 0, and the modem then fails NVRAM record
 * validation (lid_error_handle.c:239 assert, para1=0x1006) and resets. */
static char pearl_fs_cont_path[256];
static unsigned int pearl_fs_cont_off, pearl_fs_cont_len;

/* PEARL-FSMPFIX-72: multi-packet write "continuation segment" state.
 * The modem splits a large container into multiple multi-packet writes and
 * distinguishes segments by descriptor w1:
 *   w1 == 0  -> first segment, target offset = w4
 *   w1 != 0  -> continuation segment, target offset = where the previous
 *               segment finished
 * (measured NR06_010 four segments w1 = 0,0x61,0x61,0x61; w4 always 0.) */
static char pearl_fs_mpw_chain_path[256];	/* mapped path the previous segment wrote to */
static unsigned int pearl_fs_mpw_chain_end;	/* end offset the previous segment wrote to */

static void pearl_fs_mpw_reset(void)
{
	pearl_fs_mpw.active = 0;
	kfree(pearl_fs_mpw.buf);
	pearl_fs_mpw.buf = NULL;
}

static void pearl_fs_mpw_finish(struct pearl_fs_job *job, int complete)
{
	unsigned char rrep[128];
	unsigned int pos = 24;
	unsigned int hdr[2], out = 0;
	int wret = -1;
	struct file *wf;
	loff_t wpos = pearl_fs_mpw.woff;
	char lpath[256];	/* PEARL-FSMPFIX-71: the mapped real Linux path */

	if (!complete) {
		pr_err("PEARL-FS-MP: tail short (%u/%u), write what we got\n",
			pearl_fs_mpw.got, pearl_fs_mpw.total);
		/* PEARL-FS-MP-CONT-64: remember the continuation position; the modem's separate top-up packet will take over */
		strscpy(pearl_fs_cont_path, pearl_fs_mpw.path,
			sizeof(pearl_fs_cont_path));
		pearl_fs_cont_off = pearl_fs_mpw.woff + pearl_fs_mpw.got;
		pearl_fs_cont_len = pearl_fs_mpw.total - pearl_fs_mpw.got;
		pr_info("PEARL-FS-MP: expect continuation at off=%u len=%u\n",
			pearl_fs_cont_off, pearl_fs_cont_len);
	}
	/* PEARL-FSMPFIX-71: pearl_fs_mpw.path holds the modem's raw CCCI path
	 * ("Z:\NVRAM\NVD_DATA\NR06_010"). A direct filp_open would create a
	 * literal file name "/Z:\NVRAM\NVD_DATA\NR06_010" in the root directory
	 * and the real NVRAM file /mnt/nvdata/md/NVRAM/NVD_DATA/NR06_010 would
	 * never be updated -- the modem's read-back after writing still sees the
	 * old content and LID validation fails (lid_error_handle.c assert,
	 * para1=0x985 para2=0x104). The normal CMPT_WRITE path always used
	 * pearl_fs_map_path(); only this spot missed it and must be fixed.
	 */
	pearl_fs_map_path(pearl_fs_mpw.path, lpath, sizeof(lpath));
	print_hex_dump(KERN_ERR, "PEARL-FS-MP-BUF64: ", DUMP_PREFIX_OFFSET,
		16, 1, pearl_fs_mpw.buf,
		pearl_fs_mpw.got < 64 ? pearl_fs_mpw.got : 64, false);
	pearl_fs_backup_once(lpath);
	wf = filp_open(lpath, O_RDWR | O_CREAT, 0660);
	if (IS_ERR(wf)) {
		pr_err("PEARL-FS: mpwrite open %s -> %s fail %ld\n",
			pearl_fs_mpw.path, lpath, PTR_ERR(wf));
	} else {
		wret = kernel_write(wf, pearl_fs_mpw.buf, pearl_fs_mpw.got, &wpos);
		filp_close(wf, NULL);
	}
	if (wret >= 0 && (unsigned int)wret == pearl_fs_mpw.got) {
		loff_t fsz = -1;
		struct file *vf = filp_open(lpath, O_RDONLY, 0);

		if (!IS_ERR(vf)) {
			fsz = i_size_read(file_inode(vf));
			filp_close(vf, NULL);
		}
		pr_info("PEARL-FS: mpwrite %s -> %s off=%u %u bytes ok (fsize=%lld)\n",
			pearl_fs_mpw.path, lpath, pearl_fs_mpw.woff,
			pearl_fs_mpw.got, (long long)fsz);
		/* PEARL-FSMPFIX-72: remember this segment's end position for the next one to continue from */
		strscpy(pearl_fs_mpw_chain_path, lpath,
			sizeof(pearl_fs_mpw_chain_path));
		pearl_fs_mpw_chain_end = pearl_fs_mpw.woff + pearl_fs_mpw.got;
	} else {
		pr_err("PEARL-FS: mpwrite %s -> %s wrote %d of %u\n",
			pearl_fs_mpw.path, lpath, wret, pearl_fs_mpw.got);
		/* break the chain on failure, so the next segment does not land at a wrong offset */
		pearl_fs_mpw_chain_path[0] = 0;
		pearl_fs_mpw_chain_end = 0;
	}

	memcpy(rrep, job->data, sizeof(struct ccci_header));
	rrep[8] = CCCI_FS_TX;
	*(unsigned int *)(rrep + 16) = PEARL_FS_OP_CMPT_WRITE | 0xFFFF0000U;
	*(unsigned int *)(rrep + 20) = 3;
	hdr[0] = pearl_fs_mpw.wsteps ? pearl_fs_mpw.wsteps : 0x1d;
	hdr[1] = (wret >= 0) ? 0 : 1;
	pos = pearl_fs_put_block(rrep, pos, hdr, sizeof(hdr));
	/* PEARL-FS-CMPTW-RSP-64: on the reference device blk1 of all 52 0x1024
	 * replies is always 0x2, not the buffer address from the request. */
	{
		unsigned int two = 2;

		pos = pearl_fs_put_block(rrep, pos, &two, 4);
	}
	out = (wret >= 0) ? pearl_fs_mpw.got : 0;
	pos = pearl_fs_put_block(rrep, pos, &out, 4);
	*(unsigned int *)(rrep + 4) = pos;
	pearl_fs_send(job->md_id, rrep, pos);
	pearl_fs_mpw_reset();
}

static void pearl_fs_process_job(struct pearl_fs_job *job)
{
	unsigned char *req = job->data;
	unsigned int req_len = job->len;
	static unsigned char reply[PEARL_FS_MAX_MSG];	/* single worker, no reentrancy */
	const unsigned char *blk[PEARL_FS_MAX_BLK];
	unsigned int blk_len[PEARL_FS_MAX_BLK];
	unsigned int op, req_blk, i, off, pos, nblk = 0;
	/* PEARL-FSFIX-64c: fragment state for large CMPT_READ replies (valid within one call) */
	unsigned int frag_rest = 0, frag_pos = 0;
	unsigned int status = 0, out = 0, handle = 0, mode = 0;
	unsigned int blk3val = 0;
	/* anti-livelock: back off when the same request repeats, see the note at the end of the file */
	static struct {
		unsigned int op, reqlen, words[2];
		unsigned int streak;
	} last_req;
	static unsigned char databuf[PEARL_FS_READ_MAX];	/* PEARL-FSMPFIX-73: holds whole-record reads */
	/* PEARL-FSFIX-64c: continuation-packet buffer for fragmented sends (single worker, no reentrancy) */
	static unsigned char fragbuf[PEARL_FS_MAX_MSG];
	char name[96];
	int idx, hidx, cnt, j;

	/* PEARL-FSFIX-66: lazily re-read the on-disk config before each job */
	pearl_fs_cfg_refresh();

	if (req_len < 24) {
		pr_err("PEARL-FS: request too short (%u)\n", req_len);
		goto out;
	}
	op = *(unsigned int *)(req + 16);
	req_blk = *(unsigned int *)(req + 20);
	off = 24;
	for (i = 0; i < req_blk && i < PEARL_FS_MAX_BLK; i++) {
		unsigned int l;

		if (off + sizeof(unsigned int) > req_len)
			break;
		l = *(unsigned int *)(req + off);
		off += sizeof(unsigned int);
		if (l > req_len - off)
			break;
		blk[i] = req + off;
		blk_len[i] = l;
		off += (l + 3) & ~3U;
	}
	name[0] = 0;
	if (i >= 2 && op == PEARL_FS_OP_OPEN && blk_len[1] >= 4)
		mode = *(unsigned int *)blk[1];
	if ((op == PEARL_FS_OP_OPEN || op == PEARL_FS_OP_CMPT_READ ||
	     op == PEARL_FS_OP_RESTORE || op == PEARL_FS_OP_CMPT_WRITE ||
	     op == PEARL_FS_OP_MOVE || op == PEARL_FS_OP_GET_ATTR ||
	     op == PEARL_FS_OP_FIND_FIRST ||   /* PEARL-190: directory enumeration path */
	     op == PEARL_FS_OP_FILE_DETAIL) && i >= 1)	/* PEARL-FSFIX-69 */
		pearl_fs_wcs2cs(blk[0], blk_len[0], name, sizeof(name));
	else if (i >= 1 && blk_len[0] >= 4)
		handle = *(unsigned int *)blk[0];

	mutex_lock(&pearl_fs_lock);
	hidx = -1;
	if (handle >= 1 && handle <= PEARL_FS_MAX_HANDLE &&
	    pearl_fs_handles[handle - 1].used)
		hidx = handle - 1;

	memcpy(reply, req, sizeof(struct ccci_header));
	reply[8] = CCCI_FS_TX;	/* channel low byte; byte 9 is already 0 */
	*(unsigned int *)(reply + 16) = op | 0xFFFF0000U;
	pos = 24;
	/* PEARL-FS-MP: multi-packet 0x1024 write dispatch (before the normal switch) */
	/* PEARL-FSMPFIX-70: fix recognition of multi-packet 0x1024 continuations.
	 *
	 * Measured: the modem's multi-packet 0x1024 write is split into 3 parts
	 * (Z:\NVRAM\NVD_DATA\ER1B_152, 9632 bytes):
	 *   head : [16B ccci_header][u32 op][u32 nblk][blocks...][data]  hdr[0] bit31=1
	 *   cont : [16B ccci_header][u32 op][raw data @ offset 20]      hdr[0] bit31=1
	 *   tail : same as cont, but hdr[0] bit31=0
	 * A continuation has **no nblk field**, so the 4 bytes of data at offset
	 * 20 are read as nblk (a random value) and block parsing necessarily
	 * fails (i < 2 or no valid path).
	 *
	 * The old condition accepted only "bit31 set" or "req_blk==0", so the
	 * second continuation (bit31=0, random non-zero nblk) fell into case
	 * PEARL_FS_OP_CMPT_WRITE and was "mp/anomaly drop"ped; the multi-packet
	 * write never reached its declared total (measured 9624/9632). The modem
	 * then reported CMPTW fail[fs_ret:-1001] -> LID 0xEE01 write failed
	 * -> lid_error_handle.c:239 assert -> md_state 3->5.
	 *
	 * New condition: whenever a multi-packet stream is active and the packet is not a "head packet with a valid path", it is a continuation.
	 */
	if (op == PEARL_FS_OP_CMPT_WRITE &&
	    ((*(unsigned int *)req & 0x80000000U) ||
	     (pearl_fs_mpw.active && !(i >= 2 && name[0])) ||
	     (req_blk == 0 && !name[0]))) {
		unsigned int inpk;

		if (!(*(unsigned int *)req & 0x80000000U)) {
			/* end packet: bit31 cleared, nblk=0, no name */
			if (!pearl_fs_mpw.active) {
				pearl_fs_dump_req("mp-tail-orphan", req, req_len);
				goto mp_skip;
			}
			inpk = req_len - 20;
			if (inpk && pearl_fs_mpw.buf) {
				unsigned int room = pearl_fs_mpw.total -
						    pearl_fs_mpw.got;
				unsigned int n = min(inpk, room);

				memcpy(pearl_fs_mpw.buf + pearl_fs_mpw.got,
				       req + 20, n);
				pearl_fs_mpw.got += n;
			}
			pearl_fs_mpw_finish(job,
				pearl_fs_mpw.got >= pearl_fs_mpw.total);
			goto mp_skip;
		}
		if (i >= 2 && name[0]) {
			/* head packet: name + descriptor + blk2.len (=total, out of bounds) + first data block */
			unsigned int total = *(unsigned int *)(req + off - 4);

			/* PEARL-FS-MP-TOTAL-64: on the reference device blk2.len is
			 * the "length of this packet's fragment" (464), while descriptor
			 * w7(+28) is the total length of the whole write (15884). Take
			 * the larger of the two, or large files would be treated as only
			 * as long as their first block. */
			if (blk_len[1] >= 32) {
				unsigned int w7 = *(unsigned int *)(blk[1] + 28);

				if (w7 > total && w7 <= (1024u * 1024u))
					total = w7;
			}

			if (pearl_fs_mpw.active) {
				pr_err("PEARL-FS-MP: new head, restart (old %u/%u)\n",
					pearl_fs_mpw.got, pearl_fs_mpw.total);
				pearl_fs_mpw_reset();
			}
			if (total == 0 || total > (1024u * 1024u)) {
				pr_err("PEARL-FS-MP: bad total %u, drop\n", total);
				goto mp_skip;
			}
			pearl_fs_mpw.buf = kmalloc(total, GFP_KERNEL);
			if (pearl_fs_mpw.buf == NULL) {
				pr_err("PEARL-FS-MP: kmalloc %u fail, drop\n", total);
				goto mp_skip;
			}
			pearl_fs_mpw.active = 1;
			pearl_fs_mpw.total = total;
			pearl_fs_mpw.got = 0;
			pearl_fs_mpw.wsteps = (blk_len[1] >= 8) ?
				*(unsigned int *)blk[1] : 0;
			pearl_fs_mpw.wbaddr = (blk_len[1] >= 20) ?
				*(unsigned int *)(blk[1] + 16) : 0;
			/* PEARL-FS-MP-OFF-64: the write offset is descriptor w4(+16),
			 * consistent with the normal 0x1024 path (FSFIX-63) and the 52
			 * reference-device samples. The old code took w5(+20), always 0,
			 * which wrote the whole file at offset 0. */
			pearl_fs_mpw.woff = (blk_len[1] >= 20) ?
				*(unsigned int *)(blk[1] + 16) : 0;
			if (pearl_fs_mpw.woff > PEARL_FS_MAX_WRITE_OFF) {
				pr_err("PEARL-FS-MP: woff 0x%x not an offset, using 0\n",
					pearl_fs_mpw.woff);
				pearl_fs_mpw.woff = 0;
			}
			/* PEARL-FSMPFIX-72: first-segment/continuation decision.
			 * Measured across four segments: descriptor w1 =
			 * 0x00,0x61,0x61,0x61, w4 always 0, w6 (source buffer pointer)
			 * steps by exactly 0x4000 = segment length. w1==0 means first. */
			{
				unsigned int w1v = (blk_len[1] >= 8) ?
					*(unsigned int *)(blk[1] + 4) : 0;
				char clpath[256];

				pearl_fs_map_path(name, clpath, sizeof(clpath));
				if (w1v == 0 ||
				    strcmp(pearl_fs_mpw_chain_path, clpath) != 0) {
					/* a new logical write segment: use descriptor w4 as the offset */
					pearl_fs_mpw_chain_path[0] = 0;
					pearl_fs_mpw_chain_end = 0;
				} else if (pearl_fs_mpw_chain_end) {
					loff_t cfsz = -1;
					struct file *cvf = filp_open(clpath, O_RDONLY, 0);

					if (!IS_ERR(cvf)) {
						cfsz = i_size_read(file_inode(cvf));
						filp_close(cvf, NULL);
					}
					if (cfsz >= 0 &&
					    (loff_t)pearl_fs_mpw_chain_end <= cfsz) {
						pr_err("PEARL-FS-MP: chain %s off %u -> %u (fsize=%lld)\n",
							name, pearl_fs_mpw.woff,
							pearl_fs_mpw_chain_end,
							(long long)cfsz);
						pearl_fs_mpw.woff =
							pearl_fs_mpw_chain_end;
					}
				}
			}
			if (blk_len[1] >= 44)
				memcpy(pearl_fs_mpw.desc, blk[1], 44);
			strscpy(pearl_fs_mpw.path, name, sizeof(pearl_fs_mpw.path));
			inpk = req_len - off;
			if (inpk > total)
				inpk = total;
			memcpy(pearl_fs_mpw.buf, req + off, inpk);
			pearl_fs_mpw.got = inpk;
			/* PEARL-FSMPFIX-71: print the head packet descriptor and raw bytes */
			{
				char dbuf[11 * 10];
				unsigned int q = 0, w;

				for (w = 0; w < 11; w++) {
					if ((w + 1) * 4 > blk_len[1])
						break;
					q += scnprintf(dbuf + q, sizeof(dbuf) - q,
						"%08x ", *(unsigned int *)(blk[1] + w * 4));
				}
				dbuf[q] = 0;
				pr_err("PEARL-FS-MP-HD: i=%u blk0=%u blk1=%u blk2=%u req=%u off=%u desc=%s\n",
					i, blk_len[0], blk_len[1],
					(i >= 3) ? blk_len[2] : 0, req_len, off, dbuf);
			}
			if (atomic_inc_return(&pearl_fs_hd_cnt) <= 8)
				pearl_fs_dump_hex("mp-head", req, req_len, 224);
			pr_info("PEARL-FS-MP: head %s total=%u first=%u\n",
				name, total, inpk);
			if (pearl_fs_mpw.got >= total)
				pearl_fs_mpw_finish(job, 1);
			goto mp_skip;
		}
		/* middle packet: bit31 set, nblk=0, pure data */
		if (!pearl_fs_mpw.active) {
			pearl_fs_dump_req("mp-cont-orphan", req, req_len);
			goto mp_skip;
		}
		inpk = req_len - 20;
		if (pearl_fs_mpw.buf) {
			unsigned int room = pearl_fs_mpw.total - pearl_fs_mpw.got;
			unsigned int n = min(inpk, room);

			if (n) {
				memcpy(pearl_fs_mpw.buf + pearl_fs_mpw.got,
				       req + 20, n);
				pearl_fs_mpw.got += n;
			}
			pr_info("PEARL-FS-MP: cont +%u (%u/%u)\n",
				n, pearl_fs_mpw.got, pearl_fs_mpw.total);
		}
		goto mp_skip;
	}
	switch (op) {
	case PEARL_FS_OP_OPEN:
		/*
		 * PEARL-198: when the file does not actually exist, do not invent an empty in-memory entry.
		 *
		 * Reference-device behavior (yuechu log):
		 *   O: X:/nv_config, flag 0x500, ret -9        <- ENOENT
		 * We previously returned status=1, which the modem logged as
		 * ret:4097 (0x1001), so "file does not exist" was taken as "exists
		 * but empty". The security module custom_nvram_sec then got empty
		 * data when reading entries like X:\MC**, failed validation and
		 * asserted with para0 = -1001.
		 *
		 * Now: after mapping to the local path, if the file is missing and
		 * this open is a read-only probe, return ENOENT(-9) directly,
		 * matching the reference; write/create mode semantics are unchanged.
		 */
		if (pearl198_enable) {
			char lp198[256];
			struct file *tf198;

			if (pearl_fs_map_path(name, lp198, sizeof(lp198)) == 0) {
				tf198 = filp_open(lp198, O_RDONLY, 0);
				if (IS_ERR(tf198)) {
					/*
					 * PEARL-202: the contract is ENOENT = 2 (measured
					 * on mdinit: the vendor ccci_fsd also reports
					 * error=2 for these files). We previously returned
					 * -9 (which is EBADF), so the modem decoded the
					 * wrong error code.
					 */
					status = pearl202_enoent;
					handle = 0;
					pos = pearl_fs_put_block(reply, pos,
								 &status, 4);
					nblk = 1;
					pr_err("PEARL-198: open %s does not exist -> -9 (ENOENT)\n",
					       name);
					break;
				}
				filp_close(tf198, NULL);
			}
		}
		idx = pearl_fs_file_find(name);
		if (idx < 0)
			idx = pearl_fs_file_new(name);
		if (idx < 0) {
			status = 1;
			handle = 0;
		} else {
			char lpath[256];
			struct file *f;
			struct pearl_fs_handle *h;

			pearl_fs_map_path(name, lpath, sizeof(lpath));

			/*
			 * PEARL-FS-NOCREAT: never fabricate a file for a plain
			 * open. The modem probes names that are meant to be
			 * absent -- a healthy yuechu log shows
			 *   ccci_fsd: O: X:/nv_config, flag 0x500, ret -9
			 * So open read-only first, and only create when the
			 * modem itself asks to write (it opens nv_boot_trace and
			 * nv_mini_dump with mode 0x10400 and expects them to
			 * appear). The 0x100 bit marks a read-only probe; any
			 * other mode keeps the old create behaviour.
			 */
			/* PEARL-FS-OPENMODE: decide the access mode up front.
			 * Opening read-only first and retrying only on error
			 * looked safe but is not: for a file that already
			 * exists the read-only open succeeds, the handle never
			 * gets FMODE_WRITE, and the modem's later write trips
			 * WARN_ON_ONCE in __kernel_write_iter
			 * (fs/read_write.c:608) and fails with -EBADF.
			 */
			/* PEARL-FSFIX-65: decode the access mode per measured
			 * semantics. The 4 measured modes are self-consistent:
			 *   0x10400 Z:\nv_boot_trace / nv_mini_dump  read|write|create
			 *   0x500   X:\nv_config / Z:\FATD7C68128.log read|write
			 *   0x900   Z:\NVRAM / Z:\NVRAM\NVD_DATA\     read|dir
			 * => 0x100=read, 0x400=write, 0x800=dir, 0x10000=create.
			 * The old test `mode & 0x100` treated 0x10400 as a read-only
			 * probe and opened the modem's nv_boot_trace / nv_mini_dump
			 * O_RDONLY, so its following 0x1004 Write via kernel_write()
			 * necessarily failed with -EBADF, and modem traces and crash
			 * dumps stopped being written (mtime stuck at 9/25, blinding
			 * the assert scene).
			 */
			{
				unsigned int oflags;

				if (mode & 0x400U)
					oflags = O_RDWR |
						 ((mode & 0x10000U) ? O_CREAT : 0);
				else
					oflags = O_RDONLY;
				pr_info("PEARL-FS: open %s mode=0x%x oflags=0x%x\n",
					name, mode, oflags);
				f = filp_open(lpath, oflags, 0660);
			}

			/* PEARL-FS-NOCREAT-FIX: the reply block must always be
			 * written, so failures fall through instead of breaking
			 * out of the switch here.
			 */
			if (IS_ERR(f)) {
				pr_err("PEARL-FS: open %s -> %s fail %ld\n",
					name, lpath, PTR_ERR(f));
				status = 1;
				handle = (unsigned int)-9;
			} else {
				/* Allocate the handle only once the file is really
				 * open, otherwise a failing open leaks a handle slot
				 * and the modem retries until the table is full.
				 */
				handle = pearl_fs_handle_alloc(idx);
				if (handle == 0) {
					status = 1;
					filp_close(f, NULL);
				} else {
					h = &pearl_fs_handles[handle - 1];
					h->fp = f;
					h->pos = 0;
					pearl_fs_files[h->file].size =
						(unsigned int)
						i_size_read(file_inode(f));
				}
			}
		}
		/* PEARL-FSFIX-65: the reply shape is switchable, see pearl_fs_open_rsp2. */
		/* PEARL-FSFIX-66: the config file's open_rsp2=1 also enables it */
		if (pearl_fs_open_rsp2 || pearl_fs_cfg_open_rsp2) {
			unsigned int ok = status ? 1U : 0U;

			pos = pearl_fs_put_block(reply, pos, &ok, 4);
			pos = pearl_fs_put_block(reply, pos, &handle, 4);
			nblk = 2;
		} else {
			pos = pearl_fs_put_block(reply, pos, &handle, 4);
			nblk = 1;
		}
		break;
	case PEARL_FS_OP_MOVE:
	{
		/* PEARL-FS-NVBOOTUP: 0x100c, blk0 = source, blk1 = target,
		 * blk2 = flags. The modem stages files on Y: and moves them
		 * into X:, its persistent NVRAM store.
		 */
		char spath[256], dpath[256], tname[96];
		int got;

		tname[0] = 0;
		if (i >= 2)
			pearl_fs_wcs2cs(blk[1], blk_len[1], tname, sizeof(tname));

		if (!name[0] || !tname[0]) {
			pr_err("PEARL-FS: move bad req (src=%d dst=%d)\n",
			       name[0] ? 1 : 0, tname[0] ? 1 : 0);
			status = 1;
			pos = pearl_fs_put_block(reply, 24, &status, 4);
			nblk = 1;
			break;
		}

		pearl_fs_map_path(name, spath, sizeof(spath));
		pearl_fs_map_path(tname, dpath, sizeof(dpath));

		/*
		 * PEARL-188: this used to reject every move into NVRAM. But the
		 * modem's normal flow is "stage on Y:, then move into X: (persistent
		 * NVRAM storage)"; with that rejected its LID persistence failed,
		 * and 1.9s after READY it asserted at
		 * mcu/custom/service/nvram/custom_nvram_sec.c (para0 = 0xfffffc17 =
		 * -1001). Allow it by default; a module parameter reverts it.
		 */
		if (!pearl_fs_move_nvram &&
		    (strstr(spath, "NVRAM") || strstr(dpath, "NVRAM"))) {
			pr_err("PEARL-FS: move refused (NVRAM, knob off) %s -> %s\n",
			       spath, dpath);
			status = 1;
			pos = pearl_fs_put_block(reply, 24, &status, 4);
			nblk = 1;
			break;
		}

		got = pearl_fs_copy_file(spath, dpath, &out);
		if (got < 0) {
			if (got == -ENOENT) {
				/* Source absent: best-effort tidy-up, ack it
				 * rather than abort the whole NVRAM init.
				 */
				pr_info("PEARL-FS: move src missing %s (-> %s), acked\n",
					name, tname);
				status = 0;
			} else {
				pr_err("PEARL-FS: move %s -> %s fail %d\n",
				       name, tname, got);
				status = 1;
			}
			out = 0;
			pos = pearl_fs_put_block(reply, 24, &out, 4);
			nblk = 1;
			break;
		}

		pr_info("PEARL-FS: move %s -> %s %d bytes ok (chunked)\n",
			name, tname, got);
		pos = pearl_fs_put_block(reply, 24, &out, 4);
		nblk = 1;
		break;
	}

	case PEARL_FS_OP_CREATE_DIR:
	{
		/*
		 * op table (Android ccci_fsd jump table): 0x1007 = FS_CCCI_CreateDir.
		 * The modem uses it to create S:\mdota (the OTA work directory).
		 * Reply success even if the directory already exists, because the
		 * modem calls it on every boot.
		 * lookup_one_len was removed from the kernel and kern_path_create
		 * is not exported, so delegate to /bin/mkdir -p (-p returns 0 for
		 * an existing directory).
		 */
		char dpath[256];
		char *argv[5];
		char *envp[3];
		int r;

		/* 0x1007 has only 1 block, the path is in blk[0]
		 * (only nblk=2 ops like 0x1001/0x1022 use blk[1]). */
		if (!name[0] && req_blk >= 1 && blk_len[0] >= 4)
			pearl_fs_wcs2cs(blk[0], blk_len[0], name, sizeof(name));

		pearl_fs_map_path(name, dpath, sizeof(dpath));
		if (!name[0]) {
			pr_err("PEARL-FS: mkdir bad req (no name)\n");
			status = 1;
			pos = pearl_fs_put_block(reply, 24, &status, 4);
			nblk = 1;
			break;
		}

		argv[0] = "/bin/mkdir";
		argv[1] = "-p";
		argv[2] = dpath;
		argv[3] = NULL;
		argv[4] = NULL;
		envp[0] = "HOME=/";
		envp[1] = "PATH=/sbin:/bin:/usr/sbin:/usr/bin";
		envp[2] = NULL;

		r = call_usermodehelper(argv[0], argv, envp, UMH_WAIT_PROC);
		status = (r == 0) ? 0 : 1;
		pr_info("PEARL-FS: mkdir %s -> %s ret=%d status=%u\n",
			name, dpath, r, status);
		pos = pearl_fs_put_block(reply, 24, &status, 4);
		nblk = 1;
		break;
	}

	case PEARL_FS_OP_RESTORE:
		/* PEARL-FS-NVBOOTUP: 0x1021, {path} + two 4-byte params.
		 * The modem calls this as part of its normal NVRAM first
		 * boot up (the trace shows it on every boot, not only when
		 * RestoreFlag is armed). Nothing here needs restoring, so
		 * report success instead of the generic -41 so the modem can
		 * carry on. Non-destructive on purpose.
		 */
		pr_info("PEARL-FS: restore %s -> ok\n", name);
		status = 0;
		pos = pearl_fs_put_block(reply, 24, &status, 4);
		nblk = 1;
		break;

	case PEARL_FS_OP_CMPT_WRITE:
	{
		/* PEARL-FS-CMPTW: 0x1024, request = {path} + 44-byte descriptor + data(N).
		 * This is the modem writing an NVRAM LID file (blk[2] starts with "LID\0").
		 *
		 * The reply layout must have the same shape as CMPT_READ (minus
		 * the data block) -- per notes/rpcd/FS_PROTOCOL.md section 6,
		 * yuechu device strace:
		 *   0x1022 -> nblocks=4 {8}{4}{4}{112}
		 *   0x1024 -> nblocks=3 {8}{4}{4}
		 * With only 1 block {4: out} returned, the modem parses block0 as
		 * {steps,status} and reads -1001 (exactly the error code of the
		 * unimplemented op in this file's default branch), so it judges the
		 * write failed: dev_fs_write() returns 8768 -> nvram_get_dev_boot_times()
		 * fails to read the LID -> bt stays at 1 -> lid_error_handle.c:228
		 * assert and HS2 never happens.
		 */
		char wpath[256];
		const unsigned char *data = NULL;
		unsigned int dlen = 0;
		unsigned int wsteps = 0, wbaddr = 0, woff = 0;
		struct file *wf;
		loff_t wpos;
		int wret;

		/* the descriptor has the same layout as CMPT_READ: w0=step bitmap, +16=buffer address */
		if (i >= 2 && blk_len[1] >= 8)
			wsteps = *(unsigned int *)blk[1];
		if (i >= 2 && blk_len[1] >= 20)
			wbaddr = *(unsigned int *)(blk[1] + 16);
		/*
		 * PEARL-FSFIX-63: the write offset is descriptor w4 (byte +16), not w5.
		 * Two measured examples (PEARL-FS-REQ raw bytes):
		 *   Z:\BITMAP    w4=0x0    w7=1932
		 *   X:\MTBT_000  w4=0xa0   w7=44
		 * The old code took it from +20 (w5 is always 0), so MTBT_000's new
		 * record was written at offset 0 instead of 160 -- the old and new
		 * records ended up swapped in the file and the modem failed NVRAM
		 * record validation (CMPT_R ... read len(exp/r):0:44 ->
		 * dev_fs_read() ret:260 -> lid_error_handle.c:239 assert).
		 *
		 * Guard: real NVRAM files are at most tens of KB, so the offset
		 * never exceeds 64MB. If w4 is out of that range it is not an
		 * offset (probably a buffer address); fall back to the old
		 * behavior with a warning, to avoid writing at astronomical offsets.
		 */
		if (i >= 2 && blk_len[1] >= 20) {
			unsigned int cand = *(unsigned int *)(blk[1] + 16);

			if (cand <= PEARL_FS_MAX_WRITE_OFF)
				woff = cand;
			else
				pr_err("PEARL-FS: cmptwrite %s w4=0x%x not an offset, using 0\n",
				       name, cand);
		}
		/* PEARL-FS-MP-CONT-64: when the previous multi-packet write delivered
		 * only half, the modem tops up the remaining bytes with a separate
		 * 0x1024 whose w4 is 0 (not a continuation offset). Continue from the
		 * end position left by the multi-packet write, or the file header
		 * gets overwritten. */
		if (pearl_fs_cont_path[0] &&
		    strcmp(pearl_fs_cont_path, name) == 0) {
			woff = pearl_fs_cont_off;
			pr_info("PEARL-FS: cmptwrite %s mp-continuation off=%u len=%u\n",
				name, woff, pearl_fs_cont_len);
			pearl_fs_cont_path[0] = 0;
			pearl_fs_cont_off = 0;
			pearl_fs_cont_len = 0;
		}
		/* PEARL-FSMPFIX-74: for single-packet writes with w4==0, dump the descriptor first (w1=continuation flag). */
		if (woff == 0 && i >= 2 && blk_len[1] >= 40) {
			pr_err("PEARL-FS-CMPTW-DESC: %s blk1len=%u desc=%08x %08x %08x %08x %08x %08x %08x %08x %08x %08x\n",
			       name, blk_len[1],
			       *(unsigned int *)(blk[1] + 0),
			       *(unsigned int *)(blk[1] + 4),
			       *(unsigned int *)(blk[1] + 8),
			       *(unsigned int *)(blk[1] + 12),
			       *(unsigned int *)(blk[1] + 16),
			       *(unsigned int *)(blk[1] + 20),
			       *(unsigned int *)(blk[1] + 24),
			       *(unsigned int *)(blk[1] + 28),
			       *(unsigned int *)(blk[1] + 32),
			       *(unsigned int *)(blk[1] + 36));
		}
		/* PEARL-FSMPFIX-74: when a single-packet write is the "continuation
		 * segment" of a previous multi-packet write, its offset must continue
		 * from where the multi-packet write ended.
		 *
		 * Measured: NR08_004's record is split by the modem into 3
		 * multi-packet writes (off=160/16544/32928, 16384 each) + 1
		 * single-packet write (3292 bytes, w4=0). That single packet should
		 * have landed at 0xa0+49152=49312 but went to offset 0 because of
		 * w4=0, zeroing the LID header 0..160 and the first 3132 bytes of
		 * the record ("header all zeros").
		 * The end of the multi-packet chain is recorded in
		 * pearl_fs_mpw_chain_end by FSMPFIX-72.
		 */
		if (woff == 0 && pearl_fs_mpw_chain_path[0] &&
		    pearl_fs_mpw_chain_end) {
			char mpath[256];

			pearl_fs_map_path(name, mpath, sizeof(mpath));
			if (strcmp(pearl_fs_mpw_chain_path, mpath) == 0) {
				woff = pearl_fs_mpw_chain_end;
				pr_info("PEARL-FS: cmptwrite %s mp-chain-cont off=%u\n",
					name, woff);
			}
		}
		wpos = woff;
		if (i >= 3) {
			data = blk[2];
			dlen = blk_len[2];
		}

		status = 1;    /* fail by default; the success branch clears it to 0 */
		if (!name[0] || !data || dlen == 0) {
			/* PEARL-FS-MP: a multi-packet write head packet (name +
			 * descriptor present, data block out of bounds) or a remnant
			 * whose remaining shape was swallowed upstream. Replying ERROR
			 * would be treated as retryable and back-to-back resends would
			 * scramble the protocol; drop silently here, not even one reply. */
			pr_err("PEARL-FS: cmptwrite mp/anomaly drop (name=%d data=%d len=%u)\n",
			       name[0] ? 1 : 0, data ? 1 : 0, dlen);
			pearl_fs_dump_req("mp-head", req, req_len);
			goto cmptw_drop;
		}

		pearl_fs_map_path(name, wpath, sizeof(wpath));

		/*
		 * This used to reject every write containing "NVRAM". During boot
		 * the modem writes back its own NVRAM entries like
		 * Z:\NVRAM\NVD_DATA\MC04_010 (43968 bytes, chunked at ~3372 bytes,
		 * op=0x1024 req=3476); when rejected it treated the failure as
		 * retryable and resent back-to-back indefinitely (seq 44..48 with
		 * identical lengths, queued depth rising continuously), and the
		 * request+log flood starved userspace (load 10+) until a hard reset.
		 * That is the modem's own storage, whose read path we already
		 * serve, so allow writes too; pearl_fs_backup_once() keeps a backup
		 * before the first rewrite.
		 */
		print_hex_dump(KERN_ERR, "PEARL-FS-CMPTW-DATA64: ", DUMP_PREFIX_OFFSET,
			16, 1, data, dlen < 64 ? dlen : 64, false);
		pearl_fs_backup_once(wpath);

		wf = filp_open(wpath, O_RDWR | O_CREAT, 0660);
		if (IS_ERR(wf)) {
			pr_err("PEARL-FS: cmptwrite open %s -> %s fail %ld\n",
			       name, wpath, PTR_ERR(wf));
			goto cmptw_reply;
		}
		wret = kernel_write(wf, data, dlen, &wpos);
		filp_close(wf, NULL);
		if (wret < 0 || (unsigned int)wret != dlen) {
			pr_err("PEARL-FS: cmptwrite %s wrote %d of %u\n",
			       name, wret, dlen);
			goto cmptw_reply;
		}
		{
			struct file *vf = filp_open(wpath, O_RDONLY, 0);
			loff_t fsz = -1;

			if (!IS_ERR(vf)) {
				fsz = i_size_read(file_inode(vf));
				filp_close(vf, NULL);
			}
			pr_info("PEARL-FS: cmptwrite %s off=%u %u bytes ok (fsize=%lld baddr=0x%x)\n",
				name, woff, dlen, (long long)fsz, wbaddr);
		}
		out = dlen;
		status = 0;

cmptw_reply:
		{
			unsigned int hdr[2];
			/* PEARL-FSFIX-66: overridable via write_two (still 2 by default) */
			unsigned int two = pearl_fs_write_two;

			hdr[0] = wsteps ? wsteps : 0x1d;
			hdr[1] = status;
			pos = pearl_fs_put_block(reply, pos, hdr, sizeof(hdr));
			/* PEARL-FS-CMPTW-RSP-64: blk1 of all 52 reference replies is always 0x2 */
			pos = pearl_fs_put_block(reply, pos, &two, 4);
		}
		pos = pearl_fs_put_block(reply, pos, &out, 4);
		nblk = 3;
		break;

cmptw_drop:
		/* PEARL-FSMPFIX-70: this must really "not reply".
		 * The old code broke out of the switch and still sent a malformed
		 * 24-byte, nblk=0 reply; bytes at offset>=24 in the reply buffer were
		 * leftovers from the previous reply, and when the modem parsed the
		 * CMPT_WRITE reply in the 3-block shape it read the stale -1001
		 * (0xFFFFFC17) and judged the NVRAM write CMPTW fail[fs_ret:-1001].
		 * goto mp_skip only unlocks, without sending. */
		goto mp_skip;
	}

	case PEARL_FS_OP_SEEK:
	{
		int whence = 0;
		unsigned int seek_off = 0;
		struct pearl_fs_file *f;

		if (hidx < 0) {
			status = 1;
			break;
		}
		f = &pearl_fs_files[pearl_fs_handles[hidx].file];
		if (i >= 2 && blk_len[1] >= 4)
			seek_off = *(unsigned int *)blk[1];
		if (i >= 3 && blk_len[2] >= 4)
			whence = (int)*(unsigned int *)blk[2];
		if (pearl_fs_handles[hidx].fp != NULL) {
			loff_t np = vfs_llseek(pearl_fs_handles[hidx].fp,
				seek_off, whence);

			if (np < 0) {
				status = 1;
				break;
			}
			pearl_fs_handles[hidx].pos = (unsigned int)np;
		} else if (whence == 1) {
			pearl_fs_handles[hidx].pos += seek_off;
		} else if (whence == 2) {
			pearl_fs_handles[hidx].pos = f->size + seek_off;
		} else {
			pearl_fs_handles[hidx].pos = seek_off;
		}
		out = pearl_fs_handles[hidx].pos;
		pos = pearl_fs_put_block(reply, pos, &out, 4);
		nblk = 1;
		break;
	}
	case PEARL_FS_OP_WRITE:
	{
		unsigned int wlen = (i >= 2) ? blk_len[1] : 0;
		unsigned int woff;
		struct pearl_fs_file *f;

		if (hidx < 0) {
			/*
			 * PEARL-187: measured 0x1004 requests sent by the modem after
			 * READY have no block table (nblk=0) and no handle (h=0), with
			 * data right after the 24-byte header: req=760. The old code
			 * replied status=1 here, and the modem treats the reply value
			 * as an error code -- 1 shows up as -1001, so
			 * custom_nvram_sec.c asserted and READY lasted only 1.9 seconds.
			 *
			 * Handle three cases in order:
			 *  1) the word at offset 24 is a valid handle -> use it (data
			 *     starts at 28)
			 *  2) an active multi-packet continuation stream exists ->
			 *     append to that file
			 *  3) neither -> reply success (an unperformed write is a
			 *     no-op), no more 1
			 */
			int fixed = 0;

			if (req_len >= 32) {
				unsigned int h2 = *(unsigned int *)(req + 24);

				if (h2 >= 1 && h2 <= PEARL_FS_MAX_HANDLE &&
				    pearl_fs_handles[h2 - 1].used) {
					hidx = h2 - 1;
					pearl_fs_handles[hidx].pos += 0;
					pearl_fs_write_off24 = 28;
					fixed = 1;
					pr_err("PEARL-187: 0x1004 using handle at offset 24 h=%u\n", h2);
				}
			}
			if (!fixed && pearl_fs_cont_path[0]) {
				/* append to the existing file: data starts at 24 */
				char lpath[256];
				struct file *cf;
				loff_t cpos = pearl_fs_cont_off;

				pearl_fs_map_path(pearl_fs_cont_path, lpath, sizeof(lpath));
				cf = filp_open(lpath, O_WRONLY | O_CREAT | O_LARGEFILE, 0644);
				if (!IS_ERR(cf)) {
					ssize_t wr = kernel_write(cf, req + 24,
								  req_len - 24, &cpos);

					if (wr > 0)
						pearl_fs_cont_off = (unsigned int)cpos;
					filp_close(cf, NULL);
					pr_err("PEARL-187: 0x1004 append %s off=%u len=%u wrote=%zd\n",
					       pearl_fs_cont_path, (unsigned)woff_dummy,
					       req_len - 24, wr);
					status = 0;
					out = (wr > 0) ? (unsigned int)wr : 0;
					fixed = 1;
				}
			}
			if (!fixed) {
				pr_err("PEARL-187: 0x1004 no handle/no continuation stream -> no-op success len=%u\n",
				       req_len);
				status = 0;
				out = 0;
			}
			pos = pearl_fs_put_block(reply, pos, &status, 4);
			nblk = 1;
			break;
		}
		if (pearl_fs_write_off24) {
			/* PEARL-187: data starts at offset 28 (offset 24 is the handle word) */
			woff_dummy = 0;
		}
		f = &pearl_fs_files[pearl_fs_handles[hidx].file];
		/*
		 * PEARL-FS-WOFF: use the "current file position", not block3.
		 * The modem does Seek(SEEK_END) followed by Write (POSIX
		 * semantics); measured: treating block3 as the offset writes at 0 --
		 * nv_boot_trace never grows (the write succeeds but the file size
		 * does not change). block3 is logged only.
		 */
		woff = pearl_fs_handles[hidx].pos;
		if (i >= 3 && blk_len[2] >= 4)
			blk3val = *(unsigned int *)blk[2];
		if (pearl_fs_handles[hidx].fp != NULL &&
		    strstr(f->name, "NVRAM") == NULL) {
			loff_t wpos = woff;
			ssize_t wr = kernel_write(pearl_fs_handles[hidx].fp,
				blk[1], wlen, &wpos);

			if (wr < 0)
				status = 1;
			else
				out = (unsigned int)wr;
			pearl_fs_handles[hidx].pos = woff + wlen;
			if (woff + wlen > f->size)
				f->size = woff + wlen;
		} else if (pearl_fs_file_reserve(f, woff + wlen) == 0) {
			memcpy(f->data + woff, blk[1], wlen);
			if (woff + wlen > f->size)
				f->size = woff + wlen;
			pearl_fs_handles[hidx].pos = woff + wlen;
			out = wlen;
		} else {
			status = 1;
		}
		pos = pearl_fs_put_block(reply, pos, &status, 4);
		nblk = 1;
		break;
	}
	case PEARL_FS_OP_READ:
	{
		unsigned int rlen = 0, roff;
		struct pearl_fs_file *f;
		/*
		 * PEARL-204: this used to be a 2048-byte stack buffer tmp[2048];
		 * when the modem wanted to read a 39231-byte certificate blob it
		 * was silently truncated to 2KB, and the security module then
		 * failed validation and asserted custom_nvram_sec (-1001). Use the
		 * same big buffer databuf as CMPT_READ (PEARL_FS_READ_MAX).
		 */
		unsigned char *tmp = databuf;

		if (hidx < 0) {
			status = 1;
		} else {
			f = &pearl_fs_files[pearl_fs_handles[hidx].file];
			roff = pearl_fs_handles[hidx].pos;
			if (i >= 2 && blk_len[1] >= 4)
				rlen = *(unsigned int *)blk[1];
			if (rlen == 0 || rlen > PEARL_FS_READ_MAX)
				rlen = PEARL_FS_READ_MAX;
			memset(tmp, 0, sizeof(tmp));
			if (pearl_fs_handles[hidx].fp != NULL) {
				loff_t rpos = roff;
				ssize_t rd = kernel_read(
					pearl_fs_handles[hidx].fp, tmp, rlen,
					&rpos);

				if (rd < 0) {
					status = 1;
					rd = 0;
				}
				pearl_fs_handles[hidx].pos = roff + rd;
				out = rd;
			} else {
				if (roff >= f->size)
					rlen = 0;
				else if (roff + rlen > f->size)
					rlen = f->size - roff;
				if (rlen > 0)
					memcpy(tmp, f->data + roff, rlen);
				pearl_fs_handles[hidx].pos = roff + rlen;
				out = rlen;
			}
		}
		/*
		 * PEARL-203: contract (mtk-ccci-userspace / modem firmware):
		 *   READ -> {int st} {uint nread} {payload}   three blocks
		 * We previously returned only two blocks {st}{payload} -- the modem
		 * took the first 4 payload bytes as "bytes read", the length became
		 * garbage, and the security module then failed validation and
		 * asserted custom_nvram_sec (-1001).
		 */
		pos = pearl_fs_put_block(reply, pos, &status, 4);
		pos = pearl_fs_put_block(reply, pos, &out, 4);
		/*
		 * PEARL-204: large replies go through fragmentation (same path as
		 * CMPT_READ): the head packet carries only PEARL_FS_DATA_MAX bytes
		 * with bit31 set, and the rest is continued from databuf by the
		 * fragment code at the end. Without this, the modem takes the head
		 * packet as the complete message, sees a block length larger than
		 * the packet and fails the read (the "CMPTREAD ret:-1001" recorded
		 * in the repo is this symptom).
		 */
		if (out > PEARL_FS_DATA_MAX) {
			pos = pearl_fs_put_block(reply, pos, databuf,
						 PEARL_FS_DATA_MAX);
			frag_pos = PEARL_FS_DATA_MAX;
			frag_rest = out - PEARL_FS_DATA_MAX;
			pr_err("PEARL-204: READ %u bytes -> send fragmented (first %u, rest %u)\n",
			       out, PEARL_FS_DATA_MAX, frag_rest);
		} else {
			pos = pearl_fs_put_block(reply, pos, tmp, out);
		}
		nblk = 3;
		break;
	}
	case PEARL_FS_OP_FILE_SIZE:
		if (hidx < 0) {
			status = 1;
			out = 0;
		} else if (pearl_fs_handles[hidx].fp != NULL) {
			out = (unsigned int)i_size_read(
				file_inode(pearl_fs_handles[hidx].fp));
			pearl_fs_files[pearl_fs_handles[hidx].file].size = out;
		} else {
			out = pearl_fs_files[pearl_fs_handles[hidx].file].size;
		}
		pos = pearl_fs_put_block(reply, pos, &status, 4);
		pos = pearl_fs_put_block(reply, pos, &out, 4);
		nblk = 2;
		break;
	case PEARL_FS_OP_CLOSE:
		if (hidx >= 0) {
			if (pearl_fs_handles[hidx].fp != NULL)
				filp_close(pearl_fs_handles[hidx].fp, NULL);
			memset(&pearl_fs_handles[hidx], 0,
				sizeof(pearl_fs_handles[hidx]));
		} else {
			status = 1;
		}
		pos = pearl_fs_put_block(reply, pos, &status, 4);
		nblk = 1;
		break;
	case PEARL_FS_OP_CLOSE_ALL:
		for (idx = 0; idx < PEARL_FS_MAX_HANDLE; idx++)
			if (pearl_fs_handles[idx].fp != NULL)
				filp_close(pearl_fs_handles[idx].fp, NULL);
		memset(pearl_fs_handles, 0, sizeof(pearl_fs_handles));
		pos = pearl_fs_put_block(reply, pos, &status, 4);
		nblk = 1;
		break;
	case PEARL_FS_OP_CMPT_READ:
	{
		/*
		 * Measured (4 consistent yuechu strace samples):
		 *   request blk1 = {len: path(UTF-16LE)}
		 *   request blk2 = 40-byte descriptor:
		 *     w0=step bitmap  w1=state  w2/w3=flags  w4=buffer address
		 *     w5..w7=?     w8=length to read  w9=?
		 *   reply nblk=4: {8: [w0][state]} {4: w4} {4: actual length} {data}
		 * Processing chain = Open + GetFileSize + Seek + Read + Close (read the whole file).
		 */
		unsigned int steps = 0, astat = 0, bufaddr = 0, want = 0;
		unsigned int want_raw = 0;	/* PEARL-FSMPFIX-73: raw request length before clamping */
		unsigned int roff = 0;	/* descriptor w5: read offset */
		char lpath[256];
		int got = -1;

		if (i >= 2 && blk_len[1] >= 8) {
			steps = *(unsigned int *)blk[1];
			astat = 0;
		}
		if (i >= 2 && blk_len[1] >= 40) {
			bufaddr = *(unsigned int *)(blk[1] + 16);
			roff = *(unsigned int *)(blk[1] + 20);
			want = *(unsigned int *)(blk[1] + 32);
		} else if (i >= 3 && blk_len[2] >= 4) {
			want = *(unsigned int *)blk[2];
		}
		/* PEARL-FSFIX-66: offset/length can be overridden by config
		 * (default auto = use descriptor values). Failure chain B's chksum
		 * error requires the "returned bytes" to match exactly what the
		 * modem expects; if w5 is not really an offset, read_roff=zero
		 * immediately changes the outcome.
		 */
		if (pearl_fs_read_roff_mode)
			roff = pearl_fs_read_roff_const;
		if (pearl_fs_read_want_mode)
			want = pearl_fs_read_want_const;
		want_raw = want;	/* PEARL-FSMPFIX-73: raw request length before clamping */
		if (want == 0 || want > sizeof(databuf))
			want = sizeof(databuf);
		/* PEARL-FSFIX-64: the old value PEARL_FS_DATA_MAX(3420) truncated
		 * MC06_009's 4566 and made the modem's CMPT_R length check fail
		 * (260). PEARL-FSMPFIX-73: raised the cap from
		 * PEARL_FS_DATA_MAX_BIG(8128) to the databuf capacity -- NR06_010
		 * wants to read pl+40=61640 and the old cap truncated it silently;
		 * the excess still goes through PEARL-FS-FRAG continuations (head
		 * packet 3420 + several continuations).
		 */
		if (want > PEARL_FS_READ_MAX)
			want = PEARL_FS_READ_MAX;
		memset(databuf, 0, sizeof(databuf));
		pearl_fs_map_path(name, lpath, sizeof(lpath));
		got = pearl_fs_read_file(lpath, databuf, want, (loff_t)roff);
		/* PEARL-FSMPFIX-73: on large reads, dump the 40-byte descriptor
		 * raw, to confirm how many bytes the modem's w8 actually requests
		 * (8128 is the kernel clamp value, 61640 is the real pl+40).
		 */
		if (want_raw > PEARL_FS_DATA_MAX_BIG && i >= 2 && blk_len[1] >= 40) {
			pr_err("PEARL-FS-CMPTR-DESC: %s blk1len=%u desc=%08x %08x %08x %08x %08x %08x %08x %08x %08x %08x\n",
			       name, blk_len[1],
			       *(unsigned int *)(blk[1] + 0),
			       *(unsigned int *)(blk[1] + 4),
			       *(unsigned int *)(blk[1] + 8),
			       *(unsigned int *)(blk[1] + 12),
			       *(unsigned int *)(blk[1] + 16),
			       *(unsigned int *)(blk[1] + 20),
			       *(unsigned int *)(blk[1] + 24),
			       *(unsigned int *)(blk[1] + 28),
			       *(unsigned int *)(blk[1] + 32),
			       *(unsigned int *)(blk[1] + 36));
		}
		pr_info("PEARL-FS-CMPTR: %s off=%u want_raw=%u want=%u bufaddr=0x%x got=%d\n",
			name, roff, want_raw, want, bufaddr, got);
		if (got < 0) {
			pr_err("PEARL-FS: CMPTREAD miss %s -> %s\n",
				name, lpath);
			got = 0;
		}
		/* PEARL-FSFIX-66: for the first 6 CMPT_READs, dump the first 48
		 * bytes of returned data, to compare byte-for-byte with on-disk
		 * bytes (the modem raises chksum error only on mismatch).
		 */
		if (pearl_fs_read_dump && pearl_fs_read_dump_cnt < 6) {
			pearl_fs_read_dump_cnt++;
			print_hex_dump(KERN_ERR, "PEARL-FS-RD: ",
				DUMP_PREFIX_OFFSET, 16, 1, databuf,
				got > 48 ? 48 : (unsigned int)got, false);
		}
		out = got;
		/* step bitmap: 0x1d = open|seek|read|close all done (per Android observations) */
		if (steps == 0)
			steps = 0x1d;
		{
			unsigned int hdr[2];

			hdr[0] = steps;
			/* PEARL-FSFIX-66: hdr[1] semantics configurable (default 0 =
			 * FSFIX-65 old behavior). The exp=0 in the modem trace
			 * `read len(exp/r):0:44` matches the old value; if exp is
			 * really "length to read this time", it should be got/want.
			 */
			switch (pearl_fs_read_hdr1_mode) {
			case 1:
				hdr[1] = (unsigned int)out;	/* got */
				break;
			case 2:
				hdr[1] = want;
				break;
			case 3:
				hdr[1] = pearl_fs_read_hdr1_const;
				break;
			default:
				hdr[1] = astat;
				break;
			}
			pos = pearl_fs_put_block(reply, pos, hdr, sizeof(hdr));
		}
		pos = pearl_fs_put_block(reply, pos, &bufaddr, 4);
		/* PEARL-FSFIX-66: whether blk2 reports got or want is configurable
		 * (default got). Note: the data block length is always the real out;
		 * this switch only changes the reported number.
		 */
		{
			unsigned int rlen = (pearl_fs_read_out_mode == 1) ? want : out;

			pos = pearl_fs_put_block(reply, pos, &rlen, 4);
		}
		/* PEARL-FSFIX-68: blk3's "declared length" must report the full out.
		 *
		 * The modem MD-side parser (CCCI_FS_OP_Wrapper@0x3154b2) does
		 *   memcpy(dst = s3[3].ptr, src = rx + 0x38, n = blk3.len)
		 * -- how much is copied is decided entirely by blk3.len. FSFIX-64c
		 * reported 3420, so the tail of the record area (sec_factor at
		 * relative offset 4526 in the record area, chksum right after it)
		 * was never copied into the destination buffer and stayed at the
		 * old value 0:
		 *   [E][ID:0x1006][ret:4288]R fail sec_factor error
		 *   [U]00000000 00000000   (expected 06100000 01000000)
		 * The extra bytes are supplied by continuations: the modem strips
		 * the 20-byte header off each continuation and appends it after the
		 * head packet, so rx + 0x38 + 3420 meets the continuation data exactly.
		 */
		if (out > PEARL_FS_DATA_MAX) {
			frag_pos = PEARL_FS_DATA_MAX;
			frag_rest = out - PEARL_FS_DATA_MAX;
			if (pearl_fs_frag68) {
				/* carry only the first fragment, but report out in the length field */
				*(unsigned int *)(reply + pos) = out;
				pos += sizeof(unsigned int);
				memcpy(reply + pos, databuf,
				       PEARL_FS_DATA_MAX);
				/* 3420 is already a multiple of 4, no padding needed */
				pos += PEARL_FS_DATA_MAX;
			} else {
				pos = pearl_fs_put_block(reply, pos, databuf,
							 PEARL_FS_DATA_MAX);
			}
		} else {
			pos = pearl_fs_put_block(reply, pos, databuf, out);
		}
		nblk = 4;
		break;
	}
	case PEARL_FS_OP_GET_ATTR:
	case PEARL_FS_OP_FILE_DETAIL:
	{
		/* PEARL-FSFIX-69: the last batch of requests after the modem reaches
		 * md_state=3 (HS1 done) are these two ops -- querying the attributes
		 * of mdota config files (MTK_RAWOTA_DEFAULT.mcfrawota etc.). They
		 * used to be unimplemented, fell into default and got
		 * -1001(0xFFFFFC17), which the modem treated as fatal, after which
		 * **all channels** went silent for 36s => MD_BOOT_HS2_FAIL.
		 *
		 * Decisive comparison: the vendor ccci_fsd also reports
		 * error=2(ENOENT) for these files and, after querying dozens of SAR
		 * configs, continues normally and the modem reaches READY
		 * (notes/dev/fsd-orig-seq.txt). => the difference is "a legal errno
		 * vs -1001", not whether the file exists.
		 *
		 * The shape is not confirmed by measurement, so it is runtime-selectable (/mnt/nvdata/md/pearl_fs.cfg):
		 *   shape 0 = nblk=1 {u32 st}
		 *   shape 1 = nblk=2 {u32 st}{u32 attr}
		 *   shape 2 = nblk=1 {u32 0}          (treat as "exists, attr 0")
		 *   shape 3 = nblk=2 {u32 0}{u32 attr}
		 *   shape 9 = rotate 0..3 across requests (try several per flash)
		 */
		unsigned int is_attr = (op == PEARL_FS_OP_GET_ATTR);
		unsigned int shape = is_attr ? pearl_fs_getattr_rsp
					     : pearl_fs_detail_rsp;
		unsigned int st = is_attr ? pearl_fs_getattr_st
					  : pearl_fs_detail_st;
		unsigned int at = is_attr ? pearl_fs_getattr_attr
					  : pearl_fs_detail_attr;

		if (shape == 9) {	/* rotate: try 0..3 all in one flash */
			shape = pearl_fs_getattr_rot++ & 3U;
			pr_err("PEARL-FS: %s rotate -> shape %u\n",
			       is_attr ? "getattr" : "filedetail", shape);
		}
		if (shape == 2 || shape == 3)
			st = 0;	/* treat as "file exists, attr 0" */
		pos = pearl_fs_put_block(reply, pos, &st, 4);
		nblk = 1;
		if (shape == 1 || shape == 3) {
			pos = pearl_fs_put_block(reply, pos, &at, 4);
			nblk = 2;
		}
		status = st;
		out = at;
		pr_err("PEARL-FS: %s name=%s rsp=%u -> nblk=%u st=0x%08x attr=%u\n",
		       is_attr ? "getattr(0x1010)" : "filedetail(0x1025)",
		       name, is_attr ? pearl_fs_getattr_rsp
				     : pearl_fs_detail_rsp, nblk, st, at);
		break;
	}
	/*
	 * PEARL-189: 0x1016 / 0x1017 -- the modem sends these two "no-argument"
	 * requests after READY and before writing custom NVRAM (req=24 bytes,
	 * nblk=0):
	 *   #339 op=0x1016 -> the old code hit default and returned -1001
	 *   #340 op=0x1017 -> same
	 *   mcu/custom/service/nvram/custom_nvram_sec.c
	 * After getting -1001 the modem immediately asserted at
	 *   mcu/custom/service/nvram/custom_nvram_sec.c
	 * (para0 = 0xfffffc17) and READY lasted only 1.9 seconds.
	 * Treat them as "query" requests and reply success; the value is
	 * adjustable via a module parameter (to keep narrowing down what it wants).
	 */
	case 0x1017:
	{
		unsigned int v = pearl189_val;

		status = 0;
		pos = pearl_fs_put_block(reply, pos, &status, 4);
		pos = pearl_fs_put_block(reply, pos, &v, 4);
		nblk = 2;
		pr_err("PEARL-189: op=0x%04x -> reply success (v=%u)\n", op, v);
		break;
	}
	/*
	 * PEARL-190: 0x1012/0x1013/0x1014 -- FindFirst/FindNext/FindClose.
	 * After READY the modem mkdirs Q:\cacerts\ and Q:\cacerts\crl\, then
	 * enumerates the certificate directory twice with 0x1012; the old code
	 * returned -1001, after which CCIF went silent for 2 seconds and the MD
	 * raised a HIF exception (the direct cause of unstable READY).
	 *
	 * Minimal usable implementation here: cache the directory's file names,
	 * FindFirst returns the first entry, FindNext the following ones in
	 * order, FindClose releases. Reply shape = {status}{name}.
	 */
	case PEARL_FS_OP_FIND_FIRST:
	case PEARL_FS_OP_FIND_NEXT:
	{
		int r;

		pearl190_op = op;
		r = pearl190_find(op == PEARL_FS_OP_FIND_FIRST,
				  name, reply, &pos, &nblk, &out);
		if (r < 0) {
			status = 0;	/* no more files: reply success with an empty name */
			pos = pearl_fs_put_block(reply, pos, &status, 4);
			nblk = 1;
		}
		pr_err("PEARL-190: op=0x%04x name=%s -> nblk=%u out=%u\n",
		       op, name, nblk, out);
		break;
	}
	case PEARL_FS_OP_FIND_CLOSE:
	{
		status = 0;
		pearl190_find_close();
		pos = pearl_fs_put_block(reply, pos, &status, 4);
		nblk = 1;
		break;
	}
	/*
	 * PEARL-195: 0x101a / 0x101c -- measured as the last three FS requests
	 * after READY, before the crash:
	 *   [8.772] #311 op=0x101a req=48 -> old code returned -1001
	 *   [8.772] #312 op=0x101a req=48 -> same
	 *   [8.830] #322 op=0x101c req=24 -> same
	 * Then FS went silent for 2 seconds and the MD asserted at
	 * mcu/custom/service/nvram/custom_nvram_sec.c with para0 = 0xfffffc17
	 * (= -1001) -- fully matching these two unimplemented ops.
	 * req=48 with one block also appeared among 0x1004 writes, so treat
	 * them as "query/set" and reply success; the value is adjustable to
	 * keep narrowing down what it wants.
	 */
	case 0x101a:	/* FS_OP_GET_DRIVE */
	case 0x101c:	/* FS_OP_SET_DISK_FLAG */
	{
		/*
		 * PEARL-201: per the contract verified from modem firmware in
		 * mtk-ccci-userspace:
		 *   GET_DRIVE     -> single block {int val}
		 *   SET_DISK_FLAG -> single block {int val}
		 * We previously returned **two blocks** {status, value}, which the
		 * modem misparses when expecting a single block.
		 */
		int v = (int)pearl201_drive_val;

		pos = pearl_fs_put_block(reply, pos, &v, 4);
		nblk = 1;
		pr_err("PEARL-201: op=0x%04x -> single block {int %d}\n", op, v);
		break;
	}
	case 0x1016:	/* FS_OP_UNLOCK_ALL */
	{
		/*
		 * **CRITICAL** implementation verified from firmware 0x194bc in
		 * the repo:
		 *     UNLOCKALL -> single block {int 1}
		 * custom_nvram_sec takes any other value as unlock failure and
		 * asserts -1001. We previously returned two blocks {0,0} -- both
		 * the value and the block count were wrong.
		 */
		int v = (int)pearl201_unlock_val;

		pos = pearl_fs_put_block(reply, pos, &v, 4);
		nblk = 1;
		pr_err("PEARL-201: UNLOCK_ALL(0x1016) -> single block {int %d}\n", v);
		break;
	}
	default:
		/* unimplemented ops (FindFirst/GetAttributes/Restore/...): return MTK's
		 * generic error code, not 1 -- the modem treats the reply value as an
		 * error code (measured: it shows our 1 as -1001).
		 */
		status = 0xFFFFFC17;	/* -1001, matching what the modem trace shows */
		pos = pearl_fs_put_block(reply, pos, &status, 4);
		nblk = 1;
		break;
	}
	*(unsigned int *)(reply + 20) = nblk;
	*(unsigned int *)(reply + 4) = pos;
	mutex_unlock(&pearl_fs_lock);

	cnt = atomic_inc_return(&pearl_fs_msg_cnt);
	/* PEARL-FSMPFIX-70: window 96 -> 400. The old value buried all requests
	 * after #96, making it impossible to tell whether new missing ops appear
	 * after the multi-packet writes. */
	if (cnt <= 400)
		pr_err("PEARL-FS: #%d op=0x%04x seq=%u req=%u nblk=%u h=%u mode=0x%x st=%u out=%u b3=0x%x name=%s -> rep=%u\n",
			cnt, op, ((struct ccci_header *)req)->seq_num, req_len,
			req_blk, handle, mode, status, out, blk3val, name, pos);
	/* PEARL-FSMPFIX-70: log every non-zero reply status separately, not
	 * subject to the cnt limit. 0xFFFFFFFE(-2/ENOENT) is the normal return
	 * of getattr/filedetail for missing files, not a failure, so exclude it. */
	if (status != 0 && status != 0xFFFFFFFEu)
		pr_err("PEARL-FS-FAIL: #%d op=0x%04x seq=%u req=%u st=%u out=%u name=%s\n",
			cnt, op, ((struct ccci_header *)req)->seq_num, req_len,
			status, out, name);
	/* frames with nblk==0 (modem retry frames) are dumped too; they are
	 * needed most the first time they appear.
	 * PEARL-FSFIX-65: window 128 -> 256. The data block right after the
	 * 11×u32 descriptor of a 0x1024 head packet was cut off entirely at
	 * 128 bytes.
	 */
	if (cnt <= 400 || req_blk == 0)
		print_hex_dump(KERN_ERR, "PEARL-FS-REQ: ", DUMP_PREFIX_OFFSET,
			16, 1, req, req_len < 256 ? req_len : 256, false);
	/* PEARL-FSFIX-65: also dump our reply (before sending) for the first
	 * 32 entries. The ret in the modem log "O: <path>, flag <mode>, ret <n>"
	 * comes from reply blk0; without seeing what we sent, the OPEN reply
	 * shape cannot be determined.
	 */
	if (cnt <= 400)
		print_hex_dump(KERN_ERR, "PEARL-FS-REP: ", DUMP_PREFIX_OFFSET,
			16, 1, reply, pos < 128 ? pos : 128, false);

	/*
	 * Anti-livelock: the modem immediately resends the same request when it
	 * receives a failure reply. The more consecutive repeats, the longer the
	 * backoff before replying (capped at 200ms), so the request+log flood
	 * does not starve userspace. Note: this only delays the reply, never
	 * fakes success.
	 */
	{
		struct {
			unsigned int op, reqlen, words[2];
		} cur;
		unsigned int i2;

		cur.op = op;
		cur.reqlen = req_len;
		cur.words[0] = (i >= 1 && blk_len[0] >= 4) ?
			*(unsigned int *)blk[0] : 0;
		cur.words[1] = (i >= 2 && blk_len[1] >= 4) ?
			*(unsigned int *)blk[1] : 0;

		if (cur.op == last_req.op && cur.reqlen == last_req.reqlen &&
		    cur.words[0] == last_req.words[0] &&
		    cur.words[1] == last_req.words[1]) {
			last_req.streak++;
		} else {
			last_req.op = cur.op;
			last_req.reqlen = cur.reqlen;
			last_req.words[0] = cur.words[0];
			last_req.words[1] = cur.words[1];
			last_req.streak = 1;
		}
		i2 = last_req.streak;
		if (i2 > 8) {
			unsigned int backoff = (i2 - 8) * 10;

			if (backoff > 200)
				backoff = 200;
			if (i2 == 9 || (i2 % 64) == 0)
				pr_err_ratelimited("PEARL-FS: repeated request op=0x%04x len=%u streak=%u, backoff %ums\n",
					op, req_len, i2, backoff);
			/* pearl_fs_lock was already released here (see the mutex_unlock
			 * above), and there is only one worker, so just sleep without
			 * touching the lock again.
			 */
			msleep(backoff);
		}
	}

	/* PEARL-FSFIX-68: fragmented sending of large replies (vendor shape).
	 *
	 * Head packet: full reply (ccci_header 16 + op 4 + nblk 4 + 4 blocks +
	 *       first fragment), word0 bit31 set means "more fragments follow",
	 *       total 3476 = 20 + 3456.
	 * Continuation: ccci_header(16) + op|0xFFFF0000 + data (offset 20),
	 *       total 20 + n, last packet clears bit31. **No nblk field** --
	 *       this is exactly where FSFIX-64c was wrong.
	 *
	 * The receiving side (modem) reassembles by "strip the 20-byte header,
	 * append the remaining payload", so continuation data sits right after
	 * the head packet's data in the reassembly buffer:
	 *   rx+0x38 + 3420 == rx+3476 == start of the continuation payload.
	 *
	 * With pearl_fs_frag68 == 0 it falls back to FSFIX-64c's 24-byte continuation header (A/B only).
	 */
	if (frag_rest) {
		unsigned int flen, hdrlen;

		*(unsigned int *)reply |= 0x80000000U;
		hdrlen = pearl_fs_frag68 ? 20U : 24U;
		pr_err("PEARL-FS-FRAG: head pkt=%u data=%u rest=%u cont_hdr=%u\n",
			(unsigned int)pos, (unsigned int)PEARL_FS_DATA_MAX,
			frag_rest, hdrlen);
		pearl_fs_send(job->md_id, reply, pos);
		while (frag_rest) {
			unsigned int n = (frag_rest > PEARL_FS_CONT_MAX) ?
				PEARL_FS_CONT_MAX : frag_rest;

			memcpy(fragbuf, reply, 16);
			fragbuf[8] = CCCI_FS_TX;
			*(unsigned int *)fragbuf = (frag_rest > n) ?
				0x80000000U : 0U;
			*(unsigned int *)(fragbuf + 16) = op | 0xFFFF0000U;
			if (pearl_fs_frag68) {
				memcpy(fragbuf + 20, databuf + frag_pos, n);
			} else {
				*(unsigned int *)(fragbuf + 20) = 0;
				memcpy(fragbuf + 24, databuf + frag_pos, n);
			}
			flen = hdrlen + n;
			*(unsigned int *)(fragbuf + 4) = flen;
			pr_err("PEARL-FS-FRAG: cont pkt=%u off=%u rest=%u more=%u\n",
				flen, frag_pos, frag_rest,
				(frag_rest > n) ? 1U : 0U);
			pearl_fs_send(job->md_id, fragbuf, flen);
			frag_pos += n;
			frag_rest -= n;
		}
		goto out;
	}
	pearl_fs_send(job->md_id, reply, pos);
	goto out;

mp_skip:
	mutex_unlock(&pearl_fs_lock);
out:
	j = 0;
	(void)j;
}

/* the single worker drains the queue serially; jobs are freed here */
static void pearl_fs_work_fn(struct work_struct *work)
{
	struct pearl_fs_job *job;
	unsigned long flags;

	for (;;) {
		spin_lock_irqsave(&pearl_fs_q_lock, flags);
		if (list_empty(&pearl_fs_pending)) {
			spin_unlock_irqrestore(&pearl_fs_q_lock, flags);
			break;
		}
		job = list_first_entry(&pearl_fs_pending,
				struct pearl_fs_job, node);
		list_del(&job->node);
		spin_unlock_irqrestore(&pearl_fs_q_lock, flags);

		pearl_fs_process_job(job);
		atomic_dec(&pearl_fs_q_depth);
		kfree(job);
	}
}

int pearl_fs_handle_rx(unsigned char md_id, const unsigned char *msg,
	unsigned int len)
{
	struct pearl_fs_job *job;
	unsigned long flags;
	int depth;

	/*
	 * PEARL-205: by default do **not** intercept FS in the kernel -- leave it
	 * to the userspace owner (mtk-ccci-userspace's mdinit.py) via
	 * /dev/ccci_fs. Kernel emulation and the owner would race for the same
	 * requests, so it must be one or the other. Set pearl_fs_emul to 1 to
	 * return to kernel emulation.
	 */
	if (!pearl_fs_emul)
		return 0;	/* not handled, leave it to the upper layer */

	if (pearl_fs_mode == 0 || len < 24 || len > PEARL_FS_MAX_MSG)
		return 0;
	/* PEARL-FS-MP: bit31 fragment frames must be queued for the worker to
	 * reassemble (see the mp dispatch at the top of process_job); swallowing
	 * them here would leave the reassembler without data forever. */
	/* may be called in interrupt context: only copy + enqueue, the worker processes */
	job = kmalloc(sizeof(*job) + len, GFP_ATOMIC);
	if (job == NULL)
		return -ENOMEM;
	job->md_id = md_id;
	job->len = len;
	memcpy(job->data, msg, len);
	spin_lock_irqsave(&pearl_fs_q_lock, flags);
	if (atomic_read(&pearl_fs_q_depth) >= PEARL_FS_Q_MAX) {
		spin_unlock_irqrestore(&pearl_fs_q_lock, flags);
		pr_err("PEARL-FS-DROP: queue full (%d), len=%u op=0x%04x\n",
			PEARL_FS_Q_MAX, len, *(unsigned int *)(msg + 16));
		kfree(job);
		return 0;
	}
	list_add_tail(&job->node, &pearl_fs_pending);
	depth = atomic_inc_return(&pearl_fs_q_depth);
	spin_unlock_irqrestore(&pearl_fs_q_lock, flags);
	if (depth > 1)
		pr_err("PEARL-FS: queued depth=%d len=%u op=0x%04x seq=%u\n",
			depth, len, *(unsigned int *)(msg + 16),
			((struct ccci_header *)msg)->seq_num);
	schedule_work(&pearl_fs_work);
	return 1;
}
EXPORT_SYMBOL(pearl_fs_handle_rx);

static int __init pearl_fs_init(void)
{
	INIT_WORK(&pearl_fs_work, pearl_fs_work_fn);
	pearl_fs_proc_init();
	/* PEARL-FSFIX-62: build tag. The label fall-through introduced by
	 * 61316821e3 made the switch (op) unreachable code, so the MD's FS
	 * requests were never answered and HS2 always timed out. That revision
	 * moved the label back to the right place.
	 *
	 * PEARL-FSFIX-63: build tag. CMPT_WRITE now takes the write offset from
	 * descriptor w4 (the old code took w5, which is always 0, so MTBT_000's
	 * new record was written at offset 0 instead of 160, and the modem
	 * asserted after NVRAM record validation failed).
	 *
	 * PEARL-FSFIX-65: build tag. The OPEN access-mode test changed from
	 * `mode & 0x100` to `mode & 0x400` (write) / `mode & 0x10000` (create).
	 * The old test treated the 0x10400 of nv_boot_trace / nv_mini_dump as a
	 * read-only probe and opened O_RDONLY, so the modem's following 0x1004
	 * Write necessarily failed with -EBADF => modem traces and crash dumps
	 * stopped updating after 9/25, leaving only the AP side at assert time.
	 * Also widened the request dump window to 256 bytes and added the reply
	 * dump (PEARL-FS-REP).
	 *
	 * PEARL-FSFIX-68: build tag. The fragment protocol of large CMPT_READ
	 * replies changed to the vendor shape:
	 *   continuation = ccci_header(16) + op(4) + data (offset 20, no nblk=0),
	 *   total 20+n; the head packet's blk3 declared length reports the full
	 *   out (the modem memcpys by it).
	 * Basis: vendor rpcd (ARM64, notes/rpcd/rpcd_disasm_full.txt@0x7794) and
	 * ccci_fsd (ARM, notes/dev/fsd.asm@0xd1e0) both use a 20-byte
	 * continuation header + bit31=more + payload cap 3456; the modem parser
	 * CCCI_FS_OP_Wrapper@0x3154b2 copies from rx+0x38 by blk3.len only.
	 *
	 * PEARL-FSFIX-69: build tag. Added FS_CCCI_GetAttributes (0x1010) and
	 * FS_CCCI_GetFileDetail (0x1025). The last 3 requests after the modem
	 * reaches md_state=3 are 0x1010 (querying the attributes of
	 * MTK_RAWOTA_DEFAULT.mcfrawota under S:\mdota / T:\custom /
	 * T:\mtk_default); they used to fall into default and get -1001, which
	 * the modem treated as fatal, then the whole link went silent for 36s
	 * => MD_BOOT_HS2_FAIL. The vendor fsd returns a legal errno
	 * (error=2/ENOENT) for equally missing files and continues booting.
	 * The reply shape/state values can be switched at runtime via the
	 * getattr_rsp/getattr_st/getattr_attr, detail_rsp/detail_st/detail_attr
	 * keys in /mnt/nvdata/md/pearl_fs.cfg (shape 0..3, 9=rotate).
	 */
	pr_info("PEARL-FS: server init, build tag PEARL-FSMPFIX-75\n");
	return 0;
}
late_initcall(pearl_fs_init);


static void pearl_amms_dump(int md_id, const unsigned char *p,
	unsigned int len, const char *tag)
{
	unsigned int k;

	CCCI_ERROR_LOG(md_id, RPC, "PEARL-AMMS %s len=%u\n", tag, len);
	if (!pearl_amms_dump_req)
		return;
	for (k = 0; k + 16 <= len; k += 16)
		CCCI_ERROR_LOG(md_id, RPC,
			"PEARL-AMMS %s %03u: %08x %08x %08x %08x\n", tag, k,
			*(u32 *)(p + k), *(u32 *)(p + k + 4),
			*(u32 *)(p + k + 8), *(u32 *)(p + k + 12));
	if (k < len) {
		u32 w[4] = {0, 0, 0, 0};
		unsigned int i;

		for (i = 0; i < 4 && (k + i * 4) < len; i++)
			memcpy(&w[i], p + k + i * 4,
				min_t(unsigned int, 4, len - k - i * 4));
		CCCI_ERROR_LOG(md_id, RPC,
			"PEARL-AMMS %s %03u: %08x %08x %08x %08x\n",
			tag, k, w[0], w[1], w[2], w[3]);
	}
}

/* COPY: move md1drdi data into the 64KiB DRDI smem per {src,dst,len} */
static int pearl_amms_copy_sets(int md_id, int slot, struct pearl_amms_req *req)
{
	struct ccci_smem_region *smem;
	struct pearl_amms_set_copy *cs;
	void __iomem *dst_base;
	unsigned int i, num = pearl_amms_set_total[slot];
	int ret = 0;

	if (!num || num > PEARL_AMMS_MAX_SET) {
		CCCI_ERROR_LOG(md_id, RPC,
			"PEARL-AMMS: no valid set_total_num (%u), use req value %u\n",
			num, req->set_total_num);
		num = req->set_total_num;
	}
	smem = ccci_md_get_smem_by_user_id(md_id, SMEM_USER_MD_DRDI);
	if (!smem || !pearl_drdi_data) {
		CCCI_ERROR_LOG(md_id, RPC,
			"PEARL-AMMS: smem=%p drdi=%p, cannot copy\n",
			smem, pearl_drdi_data);
		return -ENODEV;
	}
	/* PEARL-AMMS: DRDI smem must be non-cached -- see the notes in
	 * scripts/patch-amms-drdi-wc.py. The vendor lists SMEM_USER_MD_DRDI in
	 * md1_6297_noncacheable_fat[], and userspace ccci_rpcd maps it with
	 * pgprot_noncached too. memremap(MEMREMAP_WB) yields a cacheable linear
	 * mapping, so the MD reading DRAM over the bus would see stale data (and
	 * create cacheable/non-cacheable aliases of the same physical page).
	 * Use ioremap_wc uniformly here, matching the NVRAM cache path in this file.
	 */
	dst_base = ioremap_wc(smem->base_ap_view_phy, smem->size);
	if (!dst_base) {
		CCCI_ERROR_LOG(md_id, RPC, "PEARL-AMMS: ioremap_wc smem fail\n");
		return -ENOMEM;
	}
	/* The COPY table starts at req+0x08, stride 12: {src, dst, len}
	 * (ccci_rpcd disassembly: x9=req+0x10 points at len, src=[x9-8], dst=[x9-4])
	 */
	cs = (struct pearl_amms_set_copy *)req->tbl;
	for (i = 0; i < num && i < PEARL_AMMS_MAX_SET; i++) {
		u32 src = cs[i].src, dst = cs[i].dst, len = cs[i].len;

		if (!len)
			continue;
		if (len > PEARL_AMMS_MAX_COPY_LEN || src + len > pearl_drdi_len ||
		    dst + len > smem->size) {
			CCCI_ERROR_LOG(md_id, RPC,
				"PEARL-AMMS copy set(%u) bad: src=0x%x dst=0x%x len=0x%x (img 0x%x smem 0x%x)\n",
				i, src, dst, len, pearl_drdi_len, smem->size);
			ret = -ERANGE;
			continue;
		}
		memcpy_toio(dst_base + dst, pearl_drdi_data + src, len);
		CCCI_ERROR_LOG(md_id, RPC,
			"PEARL-AMMS copy set(%u) from 0x%x to smem+0x%x len=0x%x\n",
			i, src, dst, len);
	}
	/* write ordering: make sure the copy lands in DRAM before the reply is sent (MD and AP are not cache-coherent) */
	wmb();
	iounmap(dst_base);
	return ret;
}

/* handle one AMMS DRDI request: fill opkt[], return the parameter count (the reply always has 2 parameters) */
/* ===== PEARL-MDLOG-AMMS-BEGIN ===== */
/* PEARL-MDLOG: the "first AMMS trigger" the task asked for -- AMMS means the
 * modem is up and talking, so send the log-enable message once here (only
 * once; also gated by the pearl_mdlog_auto switch). */
extern int pearl_mdlog_send_armed(unsigned int msg, unsigned int resv,
	int blocking, const char *why);
extern void pearl_mdlog_kick(void);

static void pearl_mdlog_amms_kick(void)
{
	/* PEARL-MDLOG-EXC: must not send here.
	 *
	 * The AMMS request lands exactly in the HS1/HS2 window of the modem boot
	 * handshake (~7.7-8.1s), where vendor port_proxy.c explicitly forbids all
	 * non-FS/RPC port traffic; experiment M proved that sending 0x0C then
	 * makes the modem deterministically assert at HS1+5.43s
	 * (ccismcore_ccci.c:1326). Log pulling is left to pearl_mdlog_kick() on
	 * entering EXCEPTION.
	 */
}
/* ===== PEARL-MDLOG-AMMS-END ===== */








static int pearl_amms_handle(struct port_t *port, struct rpc_buffer *rpc_buf,
	struct rpc_pkt *pkt, int pkt_num, struct rpc_pkt *opkt, u32 *tmp_data)
{
	int md_id = port->md_id;
	int slot = md_id & 1;
	struct pearl_amms_req *req;
	struct pearl_amms_rsp *rsp;
	struct pearl_amms_set_init *is;
	u32 *ret_code = &tmp_data[0];
	int i, copy_ret = 0;
	bool fail = false;

	pearl_mdlog_amms_kick();

	if (pkt_num < 1 || pkt[0].len < PEARL_AMMS_REQ_SIZE) {
		CCCI_ERROR_LOG(md_id, RPC,
			"PEARL-AMMS bad request pkt_num=%d len=%u (need %u)\n",
			pkt_num, pkt_num > 0 ? pkt[0].len : 0,
			PEARL_AMMS_REQ_SIZE);
		tmp_data[0] = FS_PARAM_ERROR;
		opkt[0].len = sizeof(u32);
		opkt[0].buf = (void *)&tmp_data[0];
		return 1;
	}
	req = (struct pearl_amms_req *)pkt[0].buf;
	rsp = (struct pearl_amms_rsp *)&tmp_data[1];
	memset(rsp, 0, sizeof(*rsp));
	rsp->seq_id = req->seq_id;
	*ret_code = 0;
	pearl_amms_req_cnt[slot]++;
	/* match ccci_rpcd: clear the "more fragments" bit in data[0] */
	rpc_buf->header.data[0] &= ~0x80000000U;

	pearl_amms_dump(md_id, (const unsigned char *)req, pkt[0].len,
		(req->cmd == PEARL_AMMS_CMD_INIT) ? "init" : "copy");
	CCCI_ERROR_LOG(md_id, RPC,
		"PEARL-AMMS cmd=%u seq=%u ver=%u set_total=%u cnt=%u\n",
		req->cmd, req->seq_id, req->ver, req->set_total_num,
		pearl_amms_req_cnt[slot]);

	switch (req->cmd) {
	case PEARL_AMMS_CMD_INIT:
		/*
		 * PEARL-AMMS-RESET: INIT means "the AMMS session of this MD boot
		 * begins". pearl_amms_copy_done[] is a kernel static cleared only at
		 * module load, **not reset on an MD restart**. So when restarting the
		 * MD via ccci-mdctl, a copy_done==1 left over from the previous boot
		 * was taken as "DRDI already copied", the INIT reply said
		 * copystat=0xFF, and the MD then sent **not a single COPY**.
		 *
		 * Measured (same kernel #51):
		 *   full boot    : 15 AMMS messages = 1 INIT + 14 COPY, copystat=0x0
		 *   mdctl restart:  1 AMMS message  = 1 INIT +  0 COPY, copystat=0xFF
		 * => an mdctl restart is **not equivalent** to a full boot; the
		 *   injection experiment conclusions based on it ("injecting messages
		 *   has no effect") must be rechecked after this fix.
		 *
		 * Clear per boot cycle, so the INIT reply truthfully reflects "not copied yet this round"
		 * (on a full boot it is already 0, so full boots are unaffected).
		 */
		pearl_amms_copy_done[slot] = 0;
		/*
		 * PEARL: with built-in CCCI, port_rpc_init returns at ~2.7s, when
		 * /dev/disk/by-partlabel does not exist yet (udev has not run);
		 * pearl_drdi_load_image() cannot open the md1img partition and
		 * pearl_drdi_data stays NULL, so the AMMS init could only reply
		 * 0xFFFFFFFF. Make it lazy-load like the NVRAM cache: read again
		 * when actually needed.
		 */
		if (!pearl_drdi_data)
			pearl_drdi_load_image();
		/* the MODEM is about to read RF/NVRAM data: fill the NVRAM cache region first */
		pearl_nvram_fill_cache(md_id, 1);
		pearl_amms_set_total[slot] = req->set_total_num;
		is = (struct pearl_amms_set_init *)req->tbl;
		for (i = 0; i < PEARL_AMMS_MAX_SET; i++)
			if (is[i].len)
				CCCI_ERROR_LOG(md_id, RPC,
					"PEARL-AMMS set[%d] off=0x%x len=0x%x\n",
					i, is[i].off, is[i].len);
		if (req->ver != PEARL_AMMS_INIT_VER) {
			CCCI_ERROR_LOG(md_id, RPC,
				"PEARL-AMMS init version(%u) error\n", req->ver);
			fail = true;
		} else if (req->set_total_num > PEARL_AMMS_MAX_SET) {
			CCCI_ERROR_LOG(md_id, RPC,
				"PEARL-AMMS init set_total_num(%u) error\n",
				req->set_total_num);
			fail = true;
		} else if (!pearl_drdi_data) {
			CCCI_ERROR_LOG(md_id, RPC,
				"PEARL-AMMS init: no drdi image\n");
			fail = true;
		} else {
			for (i = 0; i < req->set_total_num; i++) {
				if (is[i].off + is[i].len > pearl_drdi_len) {
					CCCI_ERROR_LOG(md_id, RPC,
						"PEARL-AMMS init set[%d] out of range off=0x%x len=0x%x (img 0x%x)\n",
						i, is[i].off, is[i].len,
						pearl_drdi_len);
					fail = true;
					break;
				}
			}
		}
		rsp->ver = PEARL_AMMS_INIT_VER;
		if (fail) {
			rsp->stats = 0xFF;
			rsp->drdiinfostat = 0xFF;
			*ret_code = 0xFFFFFFFF;
		} else {
			rsp->stats = 0;
			rsp->copystat =
				(pearl_amms_copy_done[slot] == 1) ? 0xFF : 0;
			rsp->drdiinfostat = 0;
		}
		break;

	case PEARL_AMMS_CMD_COPY:
		pearl_amms_copy_done[slot] = (req->ver == PEARL_AMMS_COPY_VER);
		rsp->ver = (req->ver == PEARL_AMMS_COPY_VER) ? 0 : 0xFF;
		if (pearl_amms_do_copy)
			copy_ret = pearl_amms_copy_sets(md_id, slot, req);
		/* exactly like rpcd: stats=0 and ret=0 when copy_done is not -1 */
		rsp->stats = 0;
		*ret_code = 0;
		if (copy_ret)
			CCCI_ERROR_LOG(md_id, RPC,
				"PEARL-AMMS copy error %d (still reply success like rpcd)\n",
				copy_ret);
		break;

	default:
		CCCI_ERROR_LOG(md_id, RPC, "PEARL-AMMS unknown cmd %u\n",
			req->cmd);
		rsp->stats = 0xFF;
		*ret_code = 0xFFFFFFFF;
		break;
	}

	if (pearl_amms_fail_ok) {
		*ret_code = 0xFFFFFFFF;
		rsp->stats = 0xFF;
	}
	CCCI_ERROR_LOG(md_id, RPC,
		"PEARL-AMMS reply ret=0x%x stats=0x%x seq=%u ver=%u copystat=0x%x drdiinfo=0x%x\n",
		*ret_code, rsp->stats, rsp->seq_id, rsp->ver, rsp->copystat,
		rsp->drdiinfostat);

	opkt[0].len = sizeof(u32);
	opkt[0].buf = (void *)ret_code;
	opkt[1].len = sizeof(struct pearl_amms_rsp);
	opkt[1].buf = (void *)rsp;
	return 2;
}

static void ccci_rpc_work_helper(struct port_t *port, struct rpc_pkt *pkt,
	struct rpc_buffer *p_rpc_buf, unsigned int tmp_data[])
{
	/*
	 * tmp_data[] is used to make sure memory address is valid
	 * after this function return, be careful with the size!
	 */
	int pkt_num = p_rpc_buf->para_num;
	int md_id = port->md_id;
	int md_val = -1;

	CCCI_DEBUG_LOG(md_id, RPC, "%s++ %d\n", __func__,
		p_rpc_buf->para_num);
	tmp_data[0] = 0;
	switch (p_rpc_buf->op_id) {
	/* call EINT API to get TDD EINT configuration for modem EINT initial */
	case IPC_RPC_GET_TDD_EINT_NUM_OP:
	case IPC_RPC_GET_GPIO_NUM_OP:
	case IPC_RPC_GET_ADC_NUM_OP:
		{
			int get_num = 0;
			unsigned char *name = NULL;
			unsigned int length = 0;

			if (pkt_num < 2 || pkt_num > RPC_MAX_ARG_NUM) {
				CCCI_ERROR_LOG(md_id, RPC,
				"invalid parameter for [0x%X]: pkt_num=%d!\n",
				p_rpc_buf->op_id, pkt_num);
				tmp_data[0] = FS_PARAM_ERROR;
				goto err1;
			}
			length = pkt[0].len;
			if (length < 1) {
				CCCI_ERROR_LOG(md_id, RPC,
				"invalid parameter for [0x%X]: pkt_num=%d, name_len=%d!\n",
				p_rpc_buf->op_id, pkt_num, length);
				tmp_data[0] = FS_PARAM_ERROR;
				goto err1;
			}

			name = kmalloc(length, GFP_KERNEL);
			if (name == NULL) {
				CCCI_ERROR_LOG(md_id, RPC,
				"Fail alloc Mem for [0x%X]!\n",
				p_rpc_buf->op_id);
				tmp_data[0] = FS_ERROR_RESERVED;
				goto err1;
			} else {
				memcpy(name, (unsigned char *)(pkt[0].buf),
				length);

				if (p_rpc_buf->op_id ==
					IPC_RPC_GET_TDD_EINT_NUM_OP) {
					get_num = get_td_eint_info(name,
								length);
					if (get_num < 0)
						get_num = FS_FUNC_FAIL;
				} else if (p_rpc_buf->op_id ==
						IPC_RPC_GET_GPIO_NUM_OP) {
					get_num = get_md_gpio_info(name,
								length,
								&md_val);
					if (get_num < 0)
						get_num = FS_FUNC_FAIL;
					else
						get_num = md_val;
				} else if (p_rpc_buf->op_id ==
						IPC_RPC_GET_ADC_NUM_OP) {
					get_num = get_md_adc_info(name,
								length);
					if (get_num < 0)
						get_num = FS_FUNC_FAIL;
				}

				CCCI_NORMAL_LOG(md_id, RPC,
					"[0x%08X]: name:%s, len=%d, get_num:%d\n",
					p_rpc_buf->op_id, name,
					length, get_num);
				pkt_num = 0;

				/* NOTE: tmp_data[1] not [0] */
				tmp_data[1] = (unsigned int)get_num;
				/* get_num may be invalid after
				 * exit this function
				 */
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)(&tmp_data[1]);
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)(&tmp_data[1]);
				kfree(name);
			}
			break;

 err1:
			pkt_num = 0;
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			break;
		}

	case IPC_RPC_GET_EMI_CLK_TYPE_OP:
		{
			int dram_type = 0;
			int dram_clk = 0;

			if (pkt_num != 0) {
				CCCI_ERROR_LOG(md_id, RPC,
				"invalid parameter for [0x%X]: pkt_num=%d!\n",
				p_rpc_buf->op_id, pkt_num);
				tmp_data[0] = FS_PARAM_ERROR;
				goto err2;
			}

			if (get_dram_type_clk(&dram_clk, &dram_type)) {
				tmp_data[0] = FS_FUNC_FAIL;
				goto err2;
			} else {
				tmp_data[0] = 0;
				CCCI_NORMAL_LOG(md_id, RPC,
				"[0x%08X]: dram_clk: %d, dram_type:%d\n",
				p_rpc_buf->op_id, dram_clk, dram_type);
			}

			tmp_data[1] = (unsigned int)dram_type;
			tmp_data[2] = (unsigned int)dram_clk;

			pkt_num = 0;
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)(&tmp_data[0]);
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)(&tmp_data[1]);
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)(&tmp_data[2]);
			break;

 err2:
			pkt_num = 0;
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			break;
		}

	case IPC_RPC_GET_EINT_ATTR_OP:
		{
			char *eint_name = NULL;
			unsigned int name_len = 0;
			unsigned int type = 0;
			char *res = NULL;
			unsigned int res_len = 0;
			int ret = 0;

			if (pkt_num < 3 || pkt_num > RPC_MAX_ARG_NUM) {
				CCCI_ERROR_LOG(md_id, RPC,
				"invalid parameter for [0x%X]: pkt_num=%d!\n",
				p_rpc_buf->op_id, pkt_num);
				tmp_data[0] = FS_PARAM_ERROR;
				goto err3;
			}
			name_len = pkt[0].len;
			if (name_len < 1) {
				CCCI_ERROR_LOG(md_id, RPC,
				"invalid parameter for [0x%X]: pkt_num=%d, name_len=%d!\n",
				p_rpc_buf->op_id, pkt_num, name_len);
				tmp_data[0] = FS_PARAM_ERROR;
				goto err3;
			}

			eint_name = kmalloc(name_len, GFP_KERNEL);
			if (eint_name == NULL) {
				CCCI_ERROR_LOG(md_id, RPC,
				"Fail alloc Mem for [0x%X]!\n",
				p_rpc_buf->op_id);
				tmp_data[0] = FS_ERROR_RESERVED;
				goto err3;
			} else {
				memcpy(eint_name, (unsigned char *)(pkt[0].buf),
				name_len);
			}

			type = *(unsigned int *)(pkt[2].buf);
			res = (unsigned char *)&(p_rpc_buf->para_num) +
					4 * sizeof(unsigned int);
			ret = get_eint_attr(md_id, eint_name, name_len, type,
					res, &res_len);
			if (ret == 0) {
				tmp_data[0] = ret;
				pkt_num = 0;
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				pkt[pkt_num].len = res_len;
				pkt[pkt_num++].buf = (void *)res;
				CCCI_DEBUG_LOG(md_id, RPC,
					"[0x%08X] OK: name:%s, len:%d, type:%d, res:%d, res_len:%d\n",
					p_rpc_buf->op_id, eint_name, name_len,
					type, *res, res_len);
				kfree(eint_name);
			} else {
				tmp_data[0] = ret;
				CCCI_DEBUG_LOG(md_id, RPC,
					"[0x%08X] fail: name:%s, len:%d, type:%d, ret:%d\n",
					p_rpc_buf->op_id, eint_name, name_len,
					type, ret);
				kfree(eint_name);
				goto err3;
			}
			break;

 err3:
			pkt_num = 0;
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			break;
		}
#ifdef FEATURE_RF_CLK_BUF
	case IPC_RPC_GET_RF_CLK_BUF_OP:
		{
			u16 count = 0;
			struct ccci_rpc_clkbuf_result *clkbuf;
			CLK_BUF_SWCTRL_STATUS_T swctrl_status[CLKBUF_MAX_COUNT];
			struct ccci_rpc_clkbuf_input *clkinput;
			u32 AfcDac;
			int ret = 0;

			if (pkt_num != 1) {
				CCCI_ERROR_LOG(md_id, RPC,
					"invalid parameter for [0x%X]: pkt_num=%d!\n",
					p_rpc_buf->op_id, pkt_num);
				tmp_data[0] = FS_PARAM_ERROR;
				pkt_num = 0;
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				break;
			}
			clkinput = (struct ccci_rpc_clkbuf_input *)pkt[0].buf;
			AfcDac = clkinput->AfcCwData;
			count = clkinput->CLKBuf_Num;
			pkt_num = 0;
			tmp_data[0] = 0;
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			pkt[pkt_num].len =
				sizeof(struct ccci_rpc_clkbuf_result);
			pkt[pkt_num++].buf = (void *)&tmp_data[1];
			clkbuf = (struct ccci_rpc_clkbuf_result *)&tmp_data[1];
			if (count != CLKBUF_MAX_COUNT) {
				CCCI_ERROR_LOG(md_id, RPC,
				"IPC_RPC_GET_RF_CLK_BUF, wrong count %d/%d\n",
				count, CLKBUF_MAX_COUNT);
				clkbuf->CLKBuf_Count = 0xFF;
				memset(&clkbuf->CLKBuf_Status, 0,
					sizeof(clkbuf->CLKBuf_Status));
			} else if (is_clk_buf_from_pmic()) {
				clkbuf->CLKBuf_Count = CLKBUF_MAX_COUNT;
				memset(&clkbuf->CLKBuf_Status, 0,
					sizeof(clkbuf->CLKBuf_Status));
				memset(&clkbuf->CLKBuf_SWCtrl_Status, 0,
					sizeof(clkbuf->CLKBuf_SWCtrl_Status));
				memset(&clkbuf->ClkBuf_Driving, 0,
					sizeof(clkbuf->ClkBuf_Driving));
			} else {
				unsigned int vals_drv[CLKBUF_MAX_COUNT] = {
					2, 2, 2, 2};
				u32 vals[CLKBUF_MAX_COUNT] = {0, 0, 0, 0};
				struct device_node *node;

				node = of_find_compatible_node(NULL, NULL,
						"mediatek,rf_clock_buffer");
				if (node) {
					ret = of_property_read_u32_array(node,
						"mediatek,clkbuf-config", vals,
						CLKBUF_MAX_COUNT);

					if (ret)
						CCCI_ERROR_LOG(md_id, RPC,
							"%s get property fail\n",
							__func__);

				} else {
					CCCI_ERROR_LOG(md_id, RPC,
					"%s can't find compatible node\n",
					__func__);
				}
				clkbuf->CLKBuf_Count = CLKBUF_MAX_COUNT;
				clkbuf->CLKBuf_Status[0] = vals[0];
				clkbuf->CLKBuf_Status[1] = vals[1];
				clkbuf->CLKBuf_Status[2] = vals[2];
				clkbuf->CLKBuf_Status[3] = vals[3];
				clk_buf_get_swctrl_status(swctrl_status);
				clk_buf_get_rf_drv_curr(vals_drv);
				clk_buf_save_afc_val(AfcDac);
				clkbuf->CLKBuf_SWCtrl_Status[0] =
					swctrl_status[0];
				clkbuf->CLKBuf_SWCtrl_Status[1] =
					swctrl_status[1];
				clkbuf->CLKBuf_SWCtrl_Status[2] =
					swctrl_status[2];
				clkbuf->CLKBuf_SWCtrl_Status[3] =
					swctrl_status[3];
				clkbuf->ClkBuf_Driving[0] = vals_drv[0];
				clkbuf->ClkBuf_Driving[1] = vals_drv[1];
				clkbuf->ClkBuf_Driving[2] = vals_drv[2];
				clkbuf->ClkBuf_Driving[3] = vals_drv[3];
				CCCI_NORMAL_LOG(md_id, RPC,
					"RF_CLK_BUF*_DRIVING_CURR %d, %d, %d, %d, AfcDac: %d\n",
					vals_drv[0], vals_drv[1], vals_drv[2],
					vals_drv[3], AfcDac);
			}
			CCCI_DEBUG_LOG(md_id, RPC,
				"IPC_RPC_GET_RF_CLK_BUF count=%x\n",
				clkbuf->CLKBuf_Count);
			break;
		}
#endif
	case IPC_RPC_GET_GPIO_VAL_OP:
	case IPC_RPC_GET_ADC_VAL_OP:
		{
			unsigned int num = 0;
			int val = 0;

			if (pkt_num != 1) {
				CCCI_ERROR_LOG(md_id, RPC,
					"invalid parameter for [0x%X]: pkt_num=%d!\n",
					p_rpc_buf->op_id, pkt_num);
				tmp_data[0] = FS_PARAM_ERROR;
				goto err4;
			}

			num = *(unsigned int *)(pkt[0].buf);
			if (p_rpc_buf->op_id == IPC_RPC_GET_GPIO_VAL_OP)
				val = get_md_gpio_val(num);
			else if (p_rpc_buf->op_id == IPC_RPC_GET_ADC_VAL_OP)
				val = get_md_adc_val(num);
			tmp_data[0] = val;
			CCCI_DEBUG_LOG(md_id, RPC, "[0x%X]: num=%d, val=%d!\n",
				p_rpc_buf->op_id, num, val);

 err4:
			pkt_num = 0;
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			break;
		}

	case IPC_RPC_GET_GPIO_ADC_OP:
		{
			struct ccci_rpc_gpio_adc_intput *input;
			struct ccci_rpc_gpio_adc_output *output;
			struct ccci_rpc_gpio_adc_intput_v2 *input_v2;
			struct ccci_rpc_gpio_adc_output_v2 *output_v2;
			unsigned int pkt_size;

			if (pkt_num != 1) {
				CCCI_ERROR_LOG(md_id, RPC,
					"invalid parameter for [0x%X]: pkt_num=%d!\n",
					p_rpc_buf->op_id, pkt_num);
				tmp_data[0] = FS_PARAM_ERROR;
				pkt_num = 0;
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				break;
			}
			pkt_size = pkt[0].len;
			if (pkt_size ==
				sizeof(struct ccci_rpc_gpio_adc_intput)) {
				input =
				(struct ccci_rpc_gpio_adc_intput *)(pkt[0].buf);
				pkt_num = 0;
				tmp_data[0] = 0;
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				pkt[pkt_num].len =
				sizeof(struct ccci_rpc_gpio_adc_output);
				pkt[pkt_num++].buf = (void *)&tmp_data[1];
				output =
				(struct ccci_rpc_gpio_adc_output *)&tmp_data[1];
				/* 0xF for failure */
				memset(output, 0xF,
				sizeof(struct ccci_rpc_gpio_adc_output));
				CCCI_BOOTUP_LOG(md_id, RPC,
					"IPC_RPC_GET_GPIO_ADC_OP request=%x\n",
					input->reqMask);
				ccci_rpc_get_gpio_adc(input, output);
			} else if (pkt_size ==
				sizeof(struct ccci_rpc_gpio_adc_intput_v2)) {
				input_v2 =
				(struct ccci_rpc_gpio_adc_intput_v2 *)
				(pkt[0].buf);
				pkt_num = 0;
				tmp_data[0] = 0;
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				pkt[pkt_num].len =
				sizeof(struct ccci_rpc_gpio_adc_output_v2);
				pkt[pkt_num++].buf = (void *)&tmp_data[1];
				output_v2 =
				(struct ccci_rpc_gpio_adc_output_v2 *)
				&tmp_data[1];
				/* 0xF for failure */
				memset(output_v2, 0xF,
				sizeof(struct ccci_rpc_gpio_adc_output_v2));
				CCCI_BOOTUP_LOG(md_id, RPC,
					"IPC_RPC_GET_GPIO_ADC_OP request=%x\n",
					input_v2->reqMask);
				ccci_rpc_get_gpio_adc_v2(input_v2, output_v2);
			} else {
				CCCI_ERROR_LOG(md_id, RPC,
					"can't recognize pkt size%d!\n",
					pkt_size);
				tmp_data[0] = FS_PARAM_ERROR;
				pkt_num = 0;
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
			}
			break;
		}

#ifdef FEATURE_INFORM_NFC_VSIM_CHANGE
	case IPC_RPC_USIM2NFC_OP:
		{
			struct ccci_rpc_usim2nfs *input, *output;

			if (pkt_num != 1) {
				CCCI_ERROR_LOG(md_id, RPC,
					"invalid parameter for [0x%X]: pkt_num=%d!\n",
					p_rpc_buf->op_id, pkt_num);
				tmp_data[0] = FS_PARAM_ERROR;
				pkt_num = 0;
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				break;
			}
			input = (struct ccci_rpc_usim2nfs *)(pkt[0].buf);
			pkt_num = 0;
			tmp_data[0] = 0;
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			pkt[pkt_num].len = sizeof(struct ccci_rpc_usim2nfs);
			pkt[pkt_num++].buf = (void *)&tmp_data[1];
			output = (struct ccci_rpc_usim2nfs *)&tmp_data[1];
			output->lock_vsim1 = input->lock_vsim1;
			CCCI_DEBUG_LOG(md_id, RPC,
				"IPC_RPC_USIM2NFC_OP request=%x\n",
				input->lock_vsim1);
			/* lock_vsim1==1, NFC not power VSIM;
			 * lock_vsim==0, NFC power VSIM
			 */
			inform_nfc_vsim_change(md_id, 1, input->lock_vsim1);
			break;
		}
#endif
	case IPC_RPC_CCCI_LHIF_MAPPING:
		{
			struct ccci_rpc_queue_mapping *remap;

			if (pkt_num != 1) {
				CCCI_ERROR_LOG(md_id, RPC,
					"invalid parameter for [0x%X]: pkt_num=%d!\n",
					p_rpc_buf->op_id, pkt_num);
				tmp_data[0] = FS_PARAM_ERROR;
				pkt_num = 0;
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				break;
			}

			CCCI_NORMAL_LOG(md_id, RPC,
				"op_id[0x%X]: pkt_num=%d, pkt[0] len %u!\n",
				p_rpc_buf->op_id, pkt_num, pkt[0].len);

			remap = (struct ccci_rpc_queue_mapping *)(pkt[0].buf);
			ccci_rpc_remap_queue(md_id, remap);
			pkt_num = 0;
			tmp_data[0] = 0;
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];

			break;
		}
	case IPC_RPC_DTSI_QUERY_OP:
		{
			struct ccci_rpc_md_dtsi_input *input;
			struct ccci_rpc_md_dtsi_output *output;

			if (pkt_num != 1) {
				CCCI_ERROR_LOG(md_id, RPC,
					"invalid parameter for [0x%X]: pkt_num=%d!\n",
					p_rpc_buf->op_id, pkt_num);
				tmp_data[0] = FS_PARAM_ERROR;
				pkt_num = 0;
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				break;
			}
			input = (struct ccci_rpc_md_dtsi_input *)(pkt[0].buf);
			pkt_num = 0;
			tmp_data[0] = 0;
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			pkt[pkt_num].len =
				sizeof(struct ccci_rpc_md_dtsi_output);
			pkt[pkt_num++].buf = (void *)&tmp_data[1];
			output = (struct ccci_rpc_md_dtsi_output *)&tmp_data[1];
			/* 0xF for failure */
			memset(output, 0xF,
				sizeof(struct ccci_rpc_md_dtsi_output));
			get_md_dtsi_val(input, output);
			break;
		}
	case IPC_RPC_QUERY_CARD_TYPE:
		CCCI_NORMAL_LOG(md_id, RPC,
			"enter QUERY CARD_TYPE operation in ccci_rpc_work\n");
		break;
	case IPC_RPC_TRNG:
		{
			struct arm_smccc_res res = {0};

			if (pkt_num != 1) {
				CCCI_ERROR_LOG(md_id, RPC,
				"invalid parameter for [0x%X]: pkt_num=%d!\n",
					     p_rpc_buf->op_id, pkt_num);
				tmp_data[0] = FS_PARAM_ERROR;
				pkt_num = 0;
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				pkt[pkt_num].len = sizeof(unsigned int);
				pkt[pkt_num++].buf = (void *)&tmp_data[0];
				break;
			}
			arm_smccc_smc(MTK_SIP_KERNEL_GET_RND,
				TRNG_MAGIC, 0, 0, 0, 0, 0, 0, &res);
			pkt_num = 0;
			tmp_data[0] = 0;
			tmp_data[1] = res.a0;
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[1];
			break;

		}
	case IPC_RPC_AMMS_DRDI_CONTROL:
		/* PEARL: kernel-side implementation of Android ccci_rpcd's AMMS DRDI reply */
		{
			struct rpc_pkt opkt[RPC_MAX_ARG_NUM];
			int xa_i;

			pkt_num = pearl_amms_handle(port, p_rpc_buf, pkt, pkt_num,
				opkt, (u32 *)tmp_data);
			for (xa_i = 0; xa_i < pkt_num; xa_i++)
				pkt[xa_i] = opkt[xa_i];
		}
		break;
	case IPC_RPC_IT_OP:
		{
			int i;

			CCCI_NORMAL_LOG(md_id, RPC,
				"[RPCIT] enter IT operation in ccci_rpc_work\n");
			/* exam input parameters in pkt */
			for (i = 0; i < pkt_num; i++) {
				CCCI_NORMAL_LOG(md_id, RPC,
					"len=%d val=%X\n", pkt[i].len,
					*((unsigned int *)pkt[i].buf));
			}
			tmp_data[0] = 1;
			tmp_data[1] = 0xA5A5;
			pkt_num = 0;
			CCCI_NORMAL_LOG(md_id, RPC,
				"[RPCIT] prepare output parameters\n");
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			CCCI_NORMAL_LOG(md_id, RPC,
				"[RPCIT] LV[%d]  len= 0x%08X, value= 0x%08X\n",
				0, pkt[0].len, *((unsigned int *)pkt[0].buf));
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[1];
			CCCI_NORMAL_LOG(md_id, RPC,
			"[RPCIT] LV[%d]  len= 0x%08X, value= 0x%08X\n",
			1, pkt[1].len, *((unsigned int *)pkt[1].buf));
			break;
		}

	case IPC_RPC_SAR_TABLE_IDX_QUERY_OP:
		/* PEARL-RPC-KERNEL-69: answered by ccci_rpcd on the vendor build;
		 * (notes/yuechu/lc_boot.txt):
		 *   Read 32 bytes  CCCI_H(0x0)(0x20)(0x80260020)  <- 1 four-byte parameter
		 *   IPC_RPC_SAR_TABLE_IDX_QUERY_OP:mtk_sar_table_id_get fail
		 *   IPC_RPC_SAR_TABLE_IDX_QUERY_OP, value: 0, ret: 0
		 *   Write 40 bytes CCCI_H(0x0)(0x28)(0x80260021)  <- 2 four-byte parameters
		 * => reply = {u32 value; u32 ret} = {0, 0} (0/0 is returned even when the lookup fails).
		 */
		pkt_num = 0;
		tmp_data[0] = (unsigned int)pearl_sar_table_id;
		tmp_data[1] = 0;
		pkt[pkt_num].len = sizeof(unsigned int);
		pkt[pkt_num++].buf = (void *)&tmp_data[0];
		if (pearl_sar_rsp_args >= 2) {
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[1];
		}
		pr_err("PEARL-RPC-KERNEL-69: SAR_TABLE_IDX_QUERY -> value=%d ret=0 args=%d\n",
		       pearl_sar_table_id, pkt_num);
		break;
	case IPC_RPC_QUERY_AP_SYS_PROPERTY:
		/* PEARL-RPC-KERNEL-70: the vendor ccci_rpcd 0x400f reply has **two
		 * blocks**: {4B int32 property_get return value}{strlen(value)+1
		 * bytes value+NUL}. Version 69 returned only the second (the value
		 * string); the MD-side RPC layer parses "result code + data" as two
		 * sections, so a single-block frame is judged a failure:
		 * custom_nvram_get_product() got -1001 and immediately asserted at
		 * custom/service/nvram/custom_nvram_sec.c:1208 (device measured
		 * para0=0xfffffc17). nanoMIPS disassembly confirms:
		 *   0x5486ee ADDIUPC a1,"ro.product.vendor.name"
		 *   0x54870c ADDIU   a0,zero,0x400f    -> IPC_RPC_General_Query
		 *   0x548716 BLTC    zero,a0,<success> ; ret<=0 -> assert line 0x4b8
		 * the two-block shape matches 0x4010/0x4005/0x400e (kernel measured para=2).
		 */
		pkt_num = 0;
		{
			unsigned int n = strlen(pearl_ap_sys_prop_val) + 1;
			char *val = (char *)&tmp_data[1];

			if (n > sizeof(pearl_ap_sys_prop_val))
				n = sizeof(pearl_ap_sys_prop_val);
			tmp_data[0] = n - 1;	/* property_get() return value */
			memcpy(val, pearl_ap_sys_prop_val, n);
			pkt[pkt_num].len = sizeof(unsigned int);
			pkt[pkt_num++].buf = (void *)&tmp_data[0];
			pkt[pkt_num].len = n;
			pkt[pkt_num++].buf = (void *)val;
		}
		pr_err("PEARL-RPC-KERNEL-70: QUERY_AP_SYS_PROPERTY -> ret=%u value=%s blocks=%d\n",
		       tmp_data[0], pearl_ap_sys_prop_val, pkt_num);
		break;
	case IPC_RPC_SAVE_MD_CAPID:
		/* PEARL-RPC-KERNEL-69: the vendor rpcd records md_capid/md_aac;
		 * Mobian has no rpcd => return a success code, at least so the modem
		 * does not stall. */
		pkt_num = 0;
		tmp_data[0] = 0;
		pkt[pkt_num].len = sizeof(unsigned int);
		pkt[pkt_num++].buf = (void *)&tmp_data[0];
		pr_err("PEARL-RPC-KERNEL-69: SAVE_MD_CAPID -> ack\n");
		break;
	default:
		CCCI_NORMAL_LOG(md_id, RPC,
		"[Error]Unknown Operation ID (0x%08X)\n",
		p_rpc_buf->op_id);
		tmp_data[0] = FS_NO_OP;
		pkt_num = 0;
		pkt[pkt_num].len = sizeof(int);
		pkt[pkt_num++].buf = (void *)&tmp_data[0];
		break;
	}

	p_rpc_buf->para_num = pkt_num;
	CCCI_DEBUG_LOG(md_id, RPC, "%s-- %d\n", __func__,
		p_rpc_buf->para_num);
}

static void rpc_msg_handler(struct port_t *port, struct sk_buff *skb)
{
	int md_id = port->md_id;
	struct rpc_buffer *rpc_buf = (struct rpc_buffer *)skb->data;
	int i, data_len, AlignLength, ret;
	struct rpc_pkt pkt[RPC_MAX_ARG_NUM];
	char *ptr = NULL, *ptr_base = NULL;
	/* unsigned int tmp_data[128]; */
	/* size of tmp_data should be >= any RPC output result */
	unsigned int *tmp_data =
		kmalloc(128*sizeof(unsigned int), GFP_ATOMIC);

	if (tmp_data == NULL) {
		CCCI_ERROR_LOG(md_id, RPC,
			"RPC request buffer fail 128*sizeof(unsigned int)\n");
		goto err_out;
	}
	/* sanity check */
	if (skb->len > RPC_MAX_BUF_SIZE) {
		CCCI_ERROR_LOG(md_id, RPC,
				"invalid RPC buffer size 0x%x/0x%x\n",
				skb->len, RPC_MAX_BUF_SIZE);
		goto err_out;
	}
	if (rpc_buf->header.reserved < 0 ||
		rpc_buf->header.reserved > RPC_REQ_BUFFER_NUM ||
	    rpc_buf->para_num < 0 ||
		rpc_buf->para_num > RPC_MAX_ARG_NUM) {
		CCCI_ERROR_LOG(md_id, RPC,
			"invalid RPC index %d/%d\n",
			rpc_buf->header.reserved, rpc_buf->para_num);
		goto err_out;
	}
	/* parse buffer */
	ptr_base = ptr = rpc_buf->buffer;
	data_len = sizeof(rpc_buf->op_id) + sizeof(rpc_buf->para_num);
	for (i = 0; i < rpc_buf->para_num; i++) {
		pkt[i].len = *((unsigned int *)ptr);
		if (pkt[i].len >= skb->len) {
			CCCI_ERROR_LOG(md_id, RPC,
				"invalid packet length in parse %u\n",
				pkt[i].len);
			goto err_out;
		}
		if ((data_len + sizeof(pkt[i].len) + pkt[i].len) >
			RPC_MAX_BUF_SIZE) {
			CCCI_ERROR_LOG(md_id, RPC,
				"RPC buffer overflow in parse %zu\n",
				data_len + sizeof(pkt[i].len) + pkt[i].len);
			goto err_out;
		}
		ptr += sizeof(pkt[i].len);
		pkt[i].buf = ptr;
		AlignLength = ((pkt[i].len + 3) >> 2) << 2;
		ptr += AlignLength;	/* 4byte align */
		data_len += (sizeof(pkt[i].len) + AlignLength);
	}
	if ((ptr - ptr_base) > RPC_MAX_BUF_SIZE) {
		CCCI_ERROR_LOG(md_id, RPC,
			"RPC overflow in parse 0x%p\n",
			(void *)(ptr - ptr_base));
		goto err_out;
	}
	/* PEARL-RPCREQ: log op_id/parameter count of every RPC request */
	pr_err("PEARL-RPCREQ op=0x%x para=%d len=%u seq=%u resv=%d\n",
		rpc_buf->op_id, rpc_buf->para_num, skb->len,
		rpc_buf->header.seq_num, rpc_buf->header.reserved);
	/* handle RPC request */
	ccci_rpc_work_helper(port, pkt, rpc_buf, tmp_data);
	/* write back to modem */
	/* update message */
	rpc_buf->op_id |= RPC_API_RESP_ID;
	data_len = sizeof(rpc_buf->op_id) + sizeof(rpc_buf->para_num);
	ptr = rpc_buf->buffer;
	for (i = 0; i < rpc_buf->para_num; i++) {
		if ((data_len + sizeof(pkt[i].len) + pkt[i].len) >
			RPC_MAX_BUF_SIZE) {
			CCCI_ERROR_LOG(md_id, RPC,
				"RPC overflow in write %zu\n",
				data_len + sizeof(pkt[i].len) + pkt[i].len);
			goto err_out;
		}

		*((unsigned int *)ptr) = pkt[i].len;
		ptr += sizeof(pkt[i].len);
		data_len += sizeof(pkt[i].len);
		/* 4byte aligned */
		AlignLength = ((pkt[i].len + 3) >> 2) << 2;
		data_len += AlignLength;

		if (ptr != pkt[i].buf)
			memcpy(ptr, pkt[i].buf, pkt[i].len);
		else
			CCCI_DEBUG_LOG(md_id, RPC,
				"same addr, no copy, op_id=0x%x\n",
				rpc_buf->op_id);

		ptr += AlignLength;
	}
	/* resize skb */
	data_len += sizeof(struct ccci_header);
	if (data_len > skb->len)
		skb_put(skb, data_len - skb->len);
	else if (data_len < skb->len)
		skb_trim(skb, data_len);
	/* update CCCI header */
	rpc_buf->header.channel = CCCI_RPC_TX;
	rpc_buf->header.data[1] = data_len;
	CCCI_DEBUG_LOG(md_id, RPC,
		"Write %d/%d, %08X, %08X, %08X, %08X, op_id=0x%x\n",
		skb->len, data_len, rpc_buf->header.data[0],
		rpc_buf->header.data[1], rpc_buf->header.channel,
		rpc_buf->header.reserved, rpc_buf->op_id);
	/* PEARL-RPCREP: reply summary */
	pr_err("PEARL-RPCREP op=0x%x para=%d len=%d seq=%u\n",
		rpc_buf->op_id, rpc_buf->para_num, data_len,
		rpc_buf->header.seq_num);
	/* switch to Tx request */
	ret = port_send_skb_to_md(port, skb, 1);
	if (ret) {
		pr_err("PEARL-RPCTXFAIL op=0x%x ret=%d\n",
			rpc_buf->op_id, ret);
		goto err_out;
	}
	kfree(tmp_data);
	return;

 err_out:
	pr_err("PEARL-RPCERR len=%u para=%d resv=%d\n",
		skb->len, rpc_buf->para_num, rpc_buf->header.reserved);
	kfree(tmp_data);
	ccci_free_skb(skb);
}

/*
 * define character device operation for rpc_u
 */
static int port_rpc_dev_mmap(struct file *fp, struct vm_area_struct *vma)
{
	struct port_t *port = fp->private_data;
	int md_id, len, ret;
	unsigned long pfn;
	struct ccci_smem_region *amms_smem = NULL;

	if (port == NULL) {
		CCCI_ERROR_LOG(-1, RPC, "%s:port is NULL\n", __func__);
		return -1;
	}

	md_id = port->md_id;
	if (port->rx_ch != CCCI_RPC_RX)
		return -EFAULT;

	amms_smem = ccci_md_get_smem_by_user_id(md_id,
		SMEM_USER_MD_DRDI);
	if (!amms_smem) {
		CCCI_ERROR_LOG(md_id, RPC, "%s:%d:ccci_md_get_smem_by_user_id fail\n",
			__func__, __LINE__);
		return -1;
	}

	if (amms_smem->size != BANK4_DRDI_SMEM_SIZE)
		CCCI_ERROR_LOG(md_id, RPC, "%s:%d:SMEM_USER_MD_DRDI size invalid(0x%x)\n",
			__func__, __LINE__, amms_smem->size);
	amms_smem->size &= ~(PAGE_SIZE - 1);
	CCCI_NORMAL_LOG(md_id, RPC,
			"remap drdi smem addr:0x%llx len:%d  map-len:%lx\n",
			(unsigned long long)amms_smem->base_ap_view_phy,
			amms_smem->size, vma->vm_end - vma->vm_start);
	if ((vma->vm_end - vma->vm_start) != amms_smem->size) {
		CCCI_ERROR_LOG(md_id, RPC,
			"smem size error:%s,vm_start=0x%llx,vm_end=0x%llx,smem_size=0x%x\n",
			port->name, vma->vm_start, vma->vm_end, amms_smem->size);
		return -EINVAL;
	}

	len = amms_smem->size;
	pfn = amms_smem->base_ap_view_phy;
	pfn >>= PAGE_SHIFT;
	/* ensure that memory does not get swapped to disk */
	vm_flags_set(vma, VM_IO);
	/* ensure non-cacheable */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	ret = remap_pfn_range(vma, vma->vm_start, pfn,
				len, vma->vm_page_prot);
	if (ret) {
		CCCI_ERROR_LOG(md_id, RPC,
			"drdi_smem remap failed %d/%lx, 0x%llx -> 0x%llx\n",
			ret, pfn,
			(unsigned long long)amms_smem->base_ap_view_phy,
			(unsigned long long)vma->vm_start);
		return -EAGAIN;
	}

	return 0;
}

static const struct file_operations rpc_dev_fops = {
	.owner = THIS_MODULE,
	.open = &port_dev_open, /*use default API*/
	.read = &port_dev_read, /*use default API*/
	.write = &port_dev_write, /*use default API*/
	.release = &port_dev_close,/*use default API*/
	.mmap = &port_rpc_dev_mmap,/*use internal API*/
};
static int port_rpc_init(struct port_t *port)
{
	struct cdev *dev = NULL;
	int ret = 0;
	static int first_init = 1;

	CCCI_DEBUG_LOG(port->md_id, RPC,
		"rpc port %s is initializing\n", port->name);
	port->rx_length_th = MAX_QUEUE_LENGTH;
	port->skb_from_pool = 1;
	port->interception = 0;
	if (port->flags & PORT_F_WITH_CHAR_NODE) {
		dev = kmalloc(sizeof(struct cdev), GFP_KERNEL);
		if (unlikely(!dev)) {
			CCCI_ERROR_LOG(port->md_id, CHAR,
				"alloc rpc char dev fail!!\n");
			return -1;
		}
		cdev_init(dev, &rpc_dev_fops);
		dev->owner = THIS_MODULE;
		ret = cdev_add(dev, MKDEV(port->major,
			port->minor_base + port->minor), 1);
		ret = ccci_register_dev_node(port->name, port->major,
			port->minor_base + port->minor);
		port->flags |= PORT_F_ADJUST_HEADER;
	} else {
		port->skb_handler = &rpc_msg_handler;
		kthread_run(port_kthread_handler, port, "%s", port->name);
	}

	if (first_init) {
		get_dtsi_eint_node(port->md_id);
		get_md_dtsi_debug();
		pearl_drdi_load_image();	/* PEARL: data source for the AMMS DRDI copy */
		schedule_delayed_work(&pearl_drdi_retry_dwork,
				      msecs_to_jiffies(500));	/* PEARL-180: retry until udev creates the node */
		first_init = 0;
		pr_info("PEARL-RPC-KERNEL-69: init (0x400F/0x4010/0x4015 handled in kernel, sar_table_id=%d args=%d prop=%s)\n",
			pearl_sar_table_id, pearl_sar_rsp_args,
			pearl_ap_sys_prop_val);
	}
	return 0;
}

int port_rpc_recv_match(struct port_t *port, struct sk_buff *skb)
{
	int md_id = port->md_id;
	int is_userspace_msg = 0;
	struct ccci_header *ccci_h = (struct ccci_header *)skb->data;
	struct rpc_buffer *rpc_buf = (struct rpc_buffer *)skb->data;

	if (ccci_h->channel == CCCI_RPC_RX) {
		switch (rpc_buf->op_id) {
#ifdef CONFIG_MTK_TC1_FEATURE
		/* LGE specific OP ID */
		case RPC_CCCI_LGE_FAC_READ_SIM_LOCK_TYPE:
		case RPC_CCCI_LGE_FAC_READ_FUSG_FLAG:
		case RPC_CCCI_LGE_FAC_CHECK_UNLOCK_CODE_VALIDNESS:
		case RPC_CCCI_LGE_FAC_CHECK_NETWORK_CODE_VALIDNESS:
		case RPC_CCCI_LGE_FAC_WRITE_SIM_LOCK_TYPE:
		case RPC_CCCI_LGE_FAC_READ_IMEI:
		case RPC_CCCI_LGE_FAC_WRITE_IMEI:
		case RPC_CCCI_LGE_FAC_READ_NETWORK_CODE_LIST_NUM:
		case RPC_CCCI_LGE_FAC_READ_NETWORK_CODE:
		case RPC_CCCI_LGE_FAC_WRITE_NETWORK_CODE_LIST_NUM:
		case RPC_CCCI_LGE_FAC_WRITE_UNLOCK_CODE_VERIFY_FAIL_COUNT:
		case RPC_CCCI_LGE_FAC_READ_UNLOCK_CODE_VERIFY_FAIL_COUNT:
		case RPC_CCCI_LGE_FAC_WRITE_UNLOCK_FAIL_COUNT:
		case RPC_CCCI_LGE_FAC_READ_UNLOCK_FAIL_COUNT:
		case RPC_CCCI_LGE_FAC_WRITE_UNLOCK_CODE:
		case RPC_CCCI_LGE_FAC_VERIFY_UNLOCK_CODE:
		case RPC_CCCI_LGE_FAC_WRITE_NETWORK_CODE:
		case RPC_CCCI_LGE_FAC_INIT_SIM_LOCK_DATA:
			is_userspace_msg = 1;
#endif
			break;

		case IPC_RPC_QUERY_AP_SYS_PROPERTY:
		case IPC_RPC_SAR_TABLE_IDX_QUERY_OP:
		case IPC_RPC_SAVE_MD_CAPID:
			/* PEARL-RPC-KERNEL-69: on the vendor build these three ops are
			 * (notes/yuechu/lc_boot.txt:
			 *   "IPC_RPC_SAR_TABLE_IDX_QUERY_OP, value: %d, ret: %d"
			 *   "IPC_RPC_QUERY_AP_SYS_PROPERTY, key<%s>, value<%s>").
			 * answered by userspace ccci_rpcd (see notes/yuechu/lc_boot.txt).
			 * Mobian has no ccci_rpcd => the frames are delivered to the
			 * /dev/ccci_rpc char node and never answered => the modem waits
			 * forever => MD_BOOT_HS2_FAIL.
			 * Measured: at 8.0415s one frame ch=32 len=32 produced no log at
			 * all (swallowed). Handle it like AMMS DRDI (0x4014): in the kernel.
			 */
			is_userspace_msg = 0;
			pr_err("PEARL-RPC-KERNEL-69: op=0x%x handled in kernel (no ccci_rpcd)\n",
			       rpc_buf->op_id);
			break;
		case IPC_RPC_AMMS_DRDI_CONTROL:
			/* PEARL: no ccci_rpcd daemon on Mobian; keep DRDI
			 * requests in the kernel RPC handler. */
			is_userspace_msg = 0;
			break;
		default:
			is_userspace_msg = 0;
			break;
		}
	}
	if (is_userspace_msg &&
		(port->flags & PORT_F_WITH_CHAR_NODE)) {
		/*userspace msg, so need match userspace port*/
		CCCI_DEBUG_LOG(md_id, RPC, "userspace rpc msg 0x%x on %s\n",
						rpc_buf->op_id, port->name);
	} else {
		/*kernel msg, so need match kernel port*/
		if (is_userspace_msg == 0 &&
			!(port->flags & PORT_F_WITH_CHAR_NODE)) {
			CCCI_DEBUG_LOG(md_id, RPC,
				"kernelspace rpc msg 0x%x on %s\n",
				rpc_buf->op_id, port->name);
		} else {
			CCCI_DEBUG_LOG(md_id, RPC,
				"port_rpc cfg error, need check:msg 0x%x on %s\n",
				rpc_buf->op_id, port->name);
			return 0;
		}
	}
	return 1;
}

struct port_ops rpc_port_ops = {
	.init = &port_rpc_init,
	.recv_match = &port_rpc_recv_match,
	.recv_skb = &port_recv_skb,
};

