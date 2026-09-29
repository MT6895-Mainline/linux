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

/* PEARL-TEST：覆盖 RPC 回答的 DRDI 射频配置集索引（-1 表示沿用设备树） */
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
	/* 不回读电平：读 GPIO 会触发 pinctrl 访问（见 get_gpio_id_from_dt 注释）。
	 * 返回 0，MODEM 仅把它当作 SIM 在位状态。
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

	/* PEARL-TEST：先看测试覆盖参数 */
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
 * 协议由 vendor/bin/ccci_rpcd 反汇编 + yuechu 真机字节级验证还原，
 * 详见 E:\pearl\notes\rpcd\PROTOCOL.md 与 amms_drdi_protocol.h。
 *
 * 请求 (op_id=0x4014, para_num=1, para[0].len=188)：
 *   +0x00 u8 cmd          1=INIT / 2=DRDI_COPY
 *   +0x01 u8 seq_id       回显到应答
 *   +0x04 u8 ver          INIT 必须==3 ; COPY 必须==1
 *   +0x05 u8 set_total_num <= 15
 *   +0x08 INIT 表: {u32 offset; u32 len;} stride 8   (相对 md1drdi 数据区)
 *   +0x08 COPY 表: {u32 src; u32 dst; u32 len;} stride 12
 *                 src 相对 md1drdi 数据区，dst 是 64KiB DRDI smem 内偏移
 *
 * 应答 (op_id=0xFFFF4014, para_num=2, 共 44 字节)：
 *   para[0] = {u32 len=4, u32 ret_code(0/0xFFFFFFFF)}
 *   para[1] = {u32 len=8, 8 字节状态}
 *   状态: {stats, seq_id, rsv[2], ver, copystat, drdiinfostat, rsv}
 */
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/vmalloc.h>

#define PEARL_AMMS_MAX_SET	15
#define PEARL_AMMS_REQ_SIZE	188	/* 8 + 15*12，内核硬校验 0xBC */
#define PEARL_AMMS_CMD_INIT	1
#define PEARL_AMMS_CMD_COPY	2
#define PEARL_AMMS_INIT_VER	3
#define PEARL_AMMS_COPY_VER	1
#define PEARL_AMMS_MAX_COPY_LEN	0x10000
#define PEARL_AMMS_MD_NUM	2

#define PEARL_MD1IMG_PATH	"/dev/disk/by-partlabel/md1img_a"
#define PEARL_SEG_MAGIC		0x58881688
#define PEARL_SEG_HDR_LEN	0x200	/* 段头 hdrlen 默认值，数据区在其后 */
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
	u8 stats;		/* 0 = 成功, 0xFF = 失败 */
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

/* 从 INIT 请求留下的状态（ccci_rpcd 用全局变量保存，COPY 表项数取自它） */
static unsigned int pearl_amms_set_total[PEARL_AMMS_MD_NUM];
static int pearl_amms_copy_done[PEARL_AMMS_MD_NUM];
static unsigned int pearl_amms_req_cnt[PEARL_AMMS_MD_NUM];

/* md1drdi 段数据区（段头 +0x200 起），请求里的 offset/src 以此为基准 */
static void *pearl_drdi_data;
static unsigned int pearl_drdi_len;
static unsigned int pearl_drdi_seg_off;

static u32 pearl_le32(const unsigned char *p)
{
	return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) |
		((u32)p[3] << 24);
}

/* 从 md1img 分区里找出 md1drdi 段并读入内存（insmod 时调用一次） */
static int pearl_drdi_load_image(void)
{
	struct file *f;
	unsigned char *buf;
	loff_t pos;
	unsigned int i, blk, got, seg_len = 0;
	int ret = 0;

	if (pearl_drdi_data)
		return 0;

	f = filp_open(PEARL_MD1IMG_PATH, O_RDONLY | O_LARGEFILE, 0);
	if (IS_ERR(f)) {
		ret = PTR_ERR(f);
		pr_err("PEARL-AMMS: open %s fail %d\n", PEARL_MD1IMG_PATH, ret);
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

/* ===== PEARL: NVRAM cache 共享内存填充 =====
 * Mobian 没有 Android 的 nvram 服务，SMEM_USER_MD_NVRAM_CACHE（AP 视图
 * 0x8a180000，1.5MB）实测全为 0。MODEM 的 RF 校准数据来自 NVRAM，
 * 读到全零就会在 mml1_rf_error_check 断言，所以这里在 MODEM 读它之前
 * （AMMS init 请求时刻）把 nvram 分区内容灌进该共享区。
 * 源、偏移、长度都可配，便于实验。
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

/* insmod 时把 NVRAM 源读进内存 */
static int pearl_nvram_load(void)
{
	struct file *f;
	loff_t pos;
	unsigned int want = 0x200000;	/* 先读 2MB，够覆盖 1.5MB 的 cache 区 */
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

/* MODEM 即将读 NVRAM 之前，把数据写进它的 cache 共享区。
 * mark=false 用于 insmod 时先填一次（此时可能还会被 MD 启动流程清掉）。
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
		/* 填充前先看当前内容：判断 insmod 时填的是否已被清掉 */
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
 * PEARL: Android 的 ccci_mdinit 是「等 NVRAM 就绪 -> 再 DO_START_MD」，
 * 我们以前是「先启动 -> AMMS init 时才填 NVRAM」，基带启动早期可能读到 0。
 * 这里把准备动作提前到启动命令之前（懒加载兜底保留，分区没就绪也不会漏）。
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
 * PEARL: FS(ccci_fs, ch14/ch15) 服务端
 *
 * Android 上这条通道由用户态 ccci_fsd 服务；新版 MTK 已把它并进 ccci_mdinit
 * （实测 yuechu：/proc/<ccci_mdinit>/fd/10 -> /dev/ccci_fs，ccci_fsd 根本没被启动），
 * 基带照样能 boot 到 ready。Mobian 侧没有任何进程打开 /dev/ccci_fs，于是基带在
 * HS1 之后发的 op=0x1001(FS_CCCI_Open) 请求永远等不到回复，43s 后
 * MD_BOOT_HS2_FAIL —— 这是早期异常(A 形态)消失之后剩下的唯一失败原因。
 *
 * 报文格式（yuechu 真机 strace 反解，详见 notes/rpcd/FS_PROTOCOL.md）：
 *   [struct ccci_header 16B][u32 op][u32 nblocks]
 *   nblocks * { u32 len; u8 data[len]，按 4 字节对齐 }
 * 回复：channel 改成 CCCI_FS_TX、op 或上 0xFFFF0000、seq 原样回填。
 *
 * 各 op 的块结构（yuechu + pearl 实测）：
 *   0x1001 Open        req {len: path(UTF-16LE)}{4: mode}   rep {4: handle}
 *   0x1002 Seek        req {4: handle}{4: off}{4: whence}   rep {4: new_pos}
 *   0x1004 Write       req {4: handle}{len: data}{4: off}   rep {4: status}
 *   0x1005 Close       req {4: handle}                      rep {4: status}
 *   0x1009 GetFileSize req {4: handle}                      rep {4: status}{4: size}
 *
 * 这是"内存里的迷你文件系统"：文件按名字持久（跨 open/close），Write 真存数据。
 * 基带因此能像在 Android 上一样追加写 nv_boot_trace，而我们用
 * /proc/pearl_fs 就能把基带自己写的 boot trace 读出来。
 * 实测教训：早期版本 Close 时清掉 size，基带重开后再 Seek(END) 拿到 0，
 * 就会用同一个 seq 疯狂重发 Seek，最后在 dev_fs.c:224 断言。
 */
#define PEARL_FS_MAX_MSG	8192
/* PEARL-FSFIX-63: CMPT_WRITE 描述符 w4 当写偏移用时的合理上限（64MB）。 */
#define PEARL_FS_MAX_WRITE_OFF	(64U * 1024U * 1024U)
/*
 * 回复报文里"数据块"的上限。
 * ccci_alloc_skb() 的第一句是 `if (size > SKB_4K || size < 0) goto err_exit;`
 * （ccci_bm.c），而 SKB_4K = CCCI_MTU + 128 = 3584（ccci_bm.h:23，
 * ccci_common_config.h 里也写着 "3584 ==SKB_4K"）—— 回复整包不能超过 3584。
 * 回复头（16 header + 4 op/nblk + 各块长度字段 + 我们前面几个块）约 56 字节，
 * 所以数据块最多 3528；这里按 SKB_4K - 64 直接算。
 * 实测踩坑：基带要读 4096 字节的 MC04_010 时我们回了 4152 字节，
 * alloc skb 直接失败，应答丢失，基带死等 → MD_BOOT_HS2_FAIL。
 * 回复块 3 就是"实际长度"，短读是协议允许的（基带会再要一次）。
 */
/* PEARL-FSFIX-64: 基带要求应答里报的长度 == 请求的 w8。
 * MC06_009 实测 w8=4566 > 3420，旧的 PEARL_FS_DATA_MAX 截断会让
 * dev_fs.c 的 CMPT_R 校验失败并返回 260，最后 lid_error_handle.c:239 断言。
 * 这里把数据块上限抬到整包缓冲能容纳的最大值（PEARL_FS_MAX_MSG-64）。
 */
#define PEARL_FS_DATA_MAX_BIG	(PEARL_FS_MAX_MSG - 64)
/* PEARL-FSMPFIX-73: CMPT_READ 单次可读的数据上限。
 *
 * 旧实现把 want 夹到 PEARL_FS_DATA_MAX_BIG(8128)，于是任何"整记录 > 8128"
 * 的读都被【静默截断】：基带请求 pl+40（NR06_010 pl=61600 -> want=61640），
 * 内核只回 8128 字节，基带拿到的远少于它要的，于是
 *   [E][ID:0x985][ret:260]read data from file/cache fail[record_idx:1][section_count:0]
 * 进而 lid_error_handle.c 断言。
 * LID 容器实测布局：文件 = 160(头) + rc*(pl+40)，记录 = pl + 8 sec_factor
 * + 32 chksum —— 所以"读长度 = pl+40"是协议要求的。这里把上限抬到 databuf
 * 容量，超长仍按原厂 PEARL-FS-FRAG 续包形状发送。
 */
#define PEARL_FS_READ_MAX	(96U * 1024U)
/* FS 包硬上限：ccci_bm.h 注明 ccci_fsd 以 CCCI_MTU 为载荷上限，再把
 * ccci_header(16) 和 op_id(4) 当头，故单包总数 = 3456+16+4 = 3476。
 * modem 自己的写请求就是顶格 3476；CMPT_READ 回复总长 = 56+数据，
 * 数据超 3420 就会让 modem 侧按 3476 的缓冲读溢出，后续帧载荷被冲毁。
 * （2026-09-25 风暴实证：3576 回复后 modem 连发全零载荷 0x1024。）*/
#define PEARL_FS_PKT_MAX	(CCCI_MTU + sizeof(struct ccci_header) + sizeof(unsigned int))
#define PEARL_FS_DATA_MAX	(PEARL_FS_PKT_MAX - 56)
/* PEARL-FSFIX-68: 续包（纯数据片）载荷上限。
 * 续包 = ccci_header(16) + op(4) + 载荷 —— 没有 nblk 字段，数据在 +20。
 * 整包 <= PEARL_FS_PKT_MAX(3476) => 载荷 <= 3456。
 * 原厂 rpcd@0x7794-0x77b8 / ccci_fsd@0xd1e0 都是这个形状
 * （remaining >= 0xd81 时置 bit31、载荷夹到 0xd80(3456)、整包 0xd94(3476)）。
 * FSFIX-64c 旧实现多写了一个 nblk=0（24 字节头），基带按 20 字节剥头后
 * 续包数据整体错位 4 字节。
 */
#define PEARL_FS_CONT_MAX	(PEARL_FS_PKT_MAX - 20)
#define PEARL_FS_MAX_BLK	8
/* PEARL-FSFIX-67: 原值 24/16 太小。基带的 NVRAM LID 枚举要按名字碰
 * 288 个不同文件（见 notes/dev/md66-findings.txt §20.4 的 mini dump 统计），
 * 表满时 pearl_fs_file_new() 返回 -1、OPEN 回 status=1，枚举会整片失败。
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
#define PEARL_FS_OP_CMPT_WRITE  0x1024	/* 整文件读（带状态位图）*/
#define PEARL_FS_OP_FIND_FIRST	0x1012
#define PEARL_FS_OP_FIND_NEXT	0x1013
#define PEARL_FS_OP_FIND_CLOSE	0x1014
/* PEARL-FSFIX-69: 基带 md_state=3（HS1 完成）之后的最后一批请求就是这两个 op
 * （查 mdota 配置文件属性）。以前没有实现，落进 default 回 -1001，基带把它
 * 当致命错误 ⇒ 之后所有通道静默 36s ⇒ MD_BOOT_HS2_FAIL。原厂 ccci_fsd 对同样
 * 缺失的文件回合法 errno（error=2/ENOENT）并继续启动。
 */
#define PEARL_FS_OP_GET_ATTR	0x1010	/* FS_CCCI_GetAttributes */
#define PEARL_FS_OP_FILE_DETAIL	0x1025	/* FS_CCCI_GetFileDetail */

/* 0 = 不响应（A/B 对照用）；1/2 = 预留的降级模式；>=2 正常应答 */
static int pearl_fs_mode = 2;	/* PEARL: 2=normal (rescue default-off removed) */
module_param(pearl_fs_mode, int, 0644);

/* PEARL-RPC-KERNEL-69: 0x4010 SAR_TABLE_IDX_QUERY 的应答值。
 * 原厂 ccci_rpcd 即使 mtk_sar_table_id_get 失败也回 {value:0, ret:0}。
 * 免刷机可调：/sys/module/port_rpc/parameters/pearl_sar_table_id
 */
static int pearl_sar_table_id;
module_param(pearl_sar_table_id, int, 0644);
static int pearl_sar_rsp_args = 2;	/* 2 = {value,ret}（原厂 40 字节） */
module_param(pearl_sar_rsp_args, int, 0644);
/* PEARL-RPC-KERNEL-69: 0x400F QUERY_AP_SYS_PROPERTY 回的值。
 * 原厂实测 key<ro.product.vendor.name> -> value<yuechu>。
 */
static char pearl_ap_sys_prop_val[32] = "pearl";
module_param_string(ap_sys_prop_val, pearl_ap_sys_prop_val,
		    sizeof(pearl_ap_sys_prop_val), 0644);

/* PEARL-FSFIX-65: OPEN(0x1001) 应答形状开关（运行时可写，免刷机切换）。
 *   0 = 保持现状 nblk=1 {4: handle}
 *   1 = nblk=2 {4: 0}{4: handle}（FS_PROTOCOL.md §8 推断的真机形状）
 * 基带日志 "O: <path>, flag <mode>, ret <n>" 里的 ret 取自应答 blk0；
 * Close(0x1005)/GetFileSize(0x1009) 的 blk0 都是"结果码(0=成功)"，
 * 只有 OPEN 把句柄放在了 blk0 —— 两种形状都保留，现场对比：
 *   /sys/module/ccci_md_all/parameters/pearl_fs_open_rsp2
 */
static unsigned int pearl_fs_open_rsp2;
module_param(pearl_fs_open_rsp2, uint, 0644);
MODULE_PARM_DESC(pearl_fs_open_rsp2,
		 "PEARL FS(ccci_fs): OPEN reply shape 0={handle}, 1={0}{handle}");
MODULE_PARM_DESC(pearl_fs_mode, "PEARL FS(ccci_fs): 0=off, 2=normal");

/* ================ PEARL-FSFIX-66: 配置文件驱动的应答变体 ================
 *
 * 目的：把"换一种应答形状"的成本从"刷机 + 用户在场"降到"改一行 + 软件重启"。
 * 依据：/mnt/nvdata（Z:）在基带发出第一条 FS 请求（约 7.9 s）之前就已挂载
 * （Z:\BITMAP 能成功写盘为证），所以这里可以放心读盘上的配置文件。
 *
 * 文件：/mnt/nvdata/md/pearl_fs.cfg（备选 /mnt/protect1/md/pearl_fs.cfg）
 * 格式：每行 key=value，# 起注释；每处理一个 FS job 惰性重读一次，
 *       内容变化才重解析并打一行 PEARL-FS-CFG 日志。删掉某一行即回默认。
 *
 * 可调项（默认全部 = FSFIX-65 的旧行为，故"没有配置文件"时行为中性）：
 *   open_rsp2 = 0|1                OPEN 应答 {handle} / {0}{handle}
 *   read_hdr1 = zero|got|want|<u32> CMPT_READ 应答 blk0 的第二个 u32
 *                                  （旧值恒 0；基带 trace `read len(exp/r):0:44`
 *                                    的 exp=0 与之吻合，需实测排除）
 *   read_out  = got|want            CMPT_READ 应答 blk2 报 got 还是 want
 *   read_roff = auto|zero|<u32>     忽略描述符 w5 或强制偏移
 *   read_want = auto|<u32>          忽略描述符 w8 或强制长度
 *   write_two = <u32>               CMPT_WRITE 应答 blk1（旧值硬编码 2）
 *   read_dump = 0|1                 CMPT_READ 前 6 次 dump 返回数据头 48 字节
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
static unsigned int pearl_fs_read_dump = 1;
static unsigned int pearl_fs_read_dump_cnt;
/* PEARL-FSFIX-69: 0x1010 / 0x1025 的应答形状与状态值（运行时可切，见
 * pearl_fs.cfg 的 getattr_* / detail_*）。
 * shape 0=nblk1{st} 1=nblk2{st,attr} 2=nblk1{0} 3=nblk2{0,attr} 9=轮转 0..3
 */
static unsigned int pearl_fs_getattr_rsp = 1;
static unsigned int pearl_fs_getattr_st = 0xFFFFFFFEU;	/* -2 = ENOENT */
static unsigned int pearl_fs_getattr_attr;
static unsigned int pearl_fs_detail_rsp = 1;
static unsigned int pearl_fs_detail_st = 0xFFFFFFFEU;	/* -2 = ENOENT */
static unsigned int pearl_fs_detail_attr;
static unsigned int pearl_fs_getattr_rot;

/* PEARL-FSFIX-68: CMPT_READ 大应答的分片协议形状开关。
 *   1 = 原厂形状（默认）：续包 = ccci_header(16) + op(4) + 数据（偏移 20），
 *       整包 = 20 + n；同时头包 blk3 的"声明长度"报完整的 out
 *       （基带就是按 blk3.len memcpy，超出的部分由续包补齐）。
 *   0 = FSFIX-64c 旧形状：续包多一个 nblk=0（偏移 24），blk3 只报首片 3420。
 * 运行时可改：/mnt/nvdata/md/pearl_fs.cfg 里写 frag68=0 即回旧形状。
 * 依据见文件头 PEARL-FSFIX-68 说明（rpcd/ccci_fsd 反汇编 + 基带解析器）。
 */
static unsigned int pearl_fs_frag68 = 1;

/* PEARL-FSFIX-67: X:\nv_config 是否映射到 /mnt/nvdata/AllMap。
 *   0 = 不映射（默认）—— OPEN/CMPT_READ 走正常盘符路径，
 *       /mnt/protect1/md/nv_config 不存在 ⟹ ENOENT(-9)，与原厂一致。
 *   1 = 映射到 AllMap（FSFIX-66 及以前的旧行为）。
 *
 * 原厂 ccci_fsd 启动日志实测（notes/dev/fsd-orig-seq.txt）：
 *   O: X:/nv_config, flag 0x500, ret -9      <-- 原厂就是失败的
 *   O: X:/MT00A001, flag 0x700, ret 2        <-- 之后才全量枚举 LID
 * 我们让它成功 ⟹ 基带认为"已有合法 LID 配置" ⟹ 跳过 LID 枚举 ⟹
 * nvram_lid_cache 为空 ⟹ 读 LID 0xF00A 时 section_count=0 / exp=0 ⟹
 * 断言 lid_error_handle.c para0=0x228d para1=0xf00a para2=0x2240。
 * 运行时可改：/mnt/nvdata/md/pearl_fs.cfg 里写 nvcfg=1 即回旧行为。
 */
static unsigned int pearl_fs_nvcfg;

/* PEARL-FSMPFIX-75: CCISM 握手运行时可切（与 FS 变体共用 /mnt/nvdata/md/pearl_fs.cfg）。
 *   ccism_auto   = 0|1    HS2 等待期是否自动发 CCISM_SHM_INIT(0x119)，默认 1
 *   ccism_ms     = <u32>  自动发送延迟 ms，默认 2000
 *   ccism_11b    = 0|1    是否补发 CCISM_SHM_INIT_DONE(0x11B)，默认 0
 *   ccism_11b_ms = <u32>  0x11B 相对规划时刻的延迟 ms，默认 3000
 */
static unsigned int pearl_fs_ccism_auto = 1;
static unsigned int pearl_fs_ccism_ms = 2000;
static unsigned int pearl_fs_ccism_11b;
static unsigned int pearl_fs_ccism_11b_ms = 3000;

/* 极简整数解析（支持 0x 前缀，遇非数字停止）；返回是否解析到数字 */
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

/* 取 key=value 的 value，并去掉行尾 CR/空白 */
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

/* line 是否以 "key=" 开头 */
static int pearl_fs_cfg_key(const char *line, const char *key)
{
	unsigned int n = strlen(key);

	/* PEARL-FSFIX-67: 容忍 `key = value`（等号两侧可有空格）。
	 * 旧实现要求 `key=` 紧贴，手写配置极易踩坑（e1 那轮就中过）。
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
				/* PEARL-FSFIX-67: nvcfg=1 回 FSFIX-66 旧行为（映射 AllMap） */
				pearl_fs_nvcfg = (v[0] == '1');
			} else if (pearl_fs_cfg_key(line, "frag68")) {
				/* PEARL-FSFIX-68: frag68=0 回 FSFIX-64c 的 24 字节续包头 */
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

/* 惰性重读配置：无文件则保持默认；内容未变则不重解析 */
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
	/* 先恢复默认再套用：删掉一行就回旧行为 */
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
	pearl_fs_nvcfg = 0;	/* PEARL-FSFIX-67: 默认不映射 nv_config */
	pearl_fs_frag68 = 1;	/* PEARL-FSFIX-68: 默认用原厂分片形状 */
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

/* PEARL-FSMPFIX-75: 供 ccci_fsm_scp.c 读取 CCISM 运行时可切项。
 * 每次都先 refresh（内容没变就不会重解析），保证"改 cfg + 软重启"即生效。 */
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

/* 证据：cfg 到底有没有被读到、读到第几代 */
unsigned int pearl_ccism_cfg_gen(void)
{
	pearl_fs_cfg_refresh();
	return pearl_fs_cfg_gen;
}
EXPORT_SYMBOL(pearl_ccism_cfg_gen);


static atomic_t pearl_fs_msg_cnt = ATOMIC_INIT(0);
static atomic_t pearl_fs_mpdump_cnt = ATOMIC_INIT(0);

/* PEARL-FS-MP: 抓多包写入的原始字节（每形状限 4 次，每次最多 320 字节） */
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
/* PEARL-FSMPFIX-71: 多包头原始字节转储（限 8 次，每次前 224 字节）。
 * 参考机 34 条单包 0x1024 的 desc 恒为 w4=0xa0(160)，但我方多包头读到
 * w4=0，必须用原始字节确认偏移到底落在描述符哪个字里。 */
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
	struct file *fp;	/* 真实文件句柄（/mnt/nvdata 下） */
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
 * PEARL-FS-QUEUE: 以前只留一个 job 槽位，上一单没跑完时新请求会被静默丢弃，
 * 基带只能等超时（实测最后一条 FS 请求之后干等约 5 秒就进 lid_error_handle）。
 * 改成链表队列 + 单个 worker 串行处理，不再丢包。
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

/* ---- /proc/pearl_fs：把基带写进来的文件读出来 ---- */
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
			/* 按文本输出，不可打印字符替换成 '.' */
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

/* ---- 报文构造 ---- */
/*
 * 回复缓冲区的硬上限。pearl_fs_put_block() 过去完全没有边界检查：
 * CMPTREAD 在 pos 已经用到 ~40 字节之后还会写最多 4096 字节的数据块，
 * 合计 4140 > reply[4096] ⇒ 踩爆 pearl_fs_process_job() 的栈。
 * 偏移修对后基带开始读取完整记录（更大的长度），于是触发，
 * 内核跑飞 → TZ 看门狗复位 → 启动循环。
 * 现在装不下就截断（保持 4 字节对齐）并报错，绝不越界。
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
	/* PEARL-FSFIX-64: 大应答（基带读 4566 字节记录）必须整包发出。
	 * ccci_alloc_skb() 首句就是 `if (size > SKB_4K) goto err_exit;`，
	 * 所以 > SKB_4K 的包改用 __dev_alloc_skb() 直配。
	 * ccci_free_skb() 会读 skb->head + NET_SKB_PAD - sizeof(buf_ctrl)
	 * 处的 head_magic 来决定 policy；这里显式清零，使其 != CCCI_BUF_MAGIC，
	 * 从而走 FREE 分支（dev_kfree_skb_any），不会被误当成池对象回收。
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
 * 基带路径 -> Linux 路径。
 *   Z:\NVRAM\CALIBRAT\ML09_001 -> /mnt/nvdata/md/NVRAM/CALIBRAT/ML09_001
 * X:\ 的真实根还没完全确定，所以按候选根依次探测，谁能打开就用谁。
 */
/*
 * PEARL-FS-DRIVES: 盘符必须各自映到自己的真实存储。
 *
 * 真机实测（把 protect1/protect2 挂起来看）：
 *   X:\LD36_003 / X:\ER1D_002 / X:\MTBT_000 / X:\AA01_010 / X:\MC00_007 ...
 *     -> /mnt/protect1/md/（35 个文件；protect2/md 是第二份，32 个）
 *   Z:\NVRAM\CALIBRAT\CC00_001 -> /mnt/nvdata/md/NVRAM/CALIBRAT/CC00_001
 *
 * 以前这里把盘符一刀切掉、再"谁能打开就用谁"地探测候选根，于是 X: 的读都落到
 * 根文件系统上的空目录 /mnt/nvcfg/。基带看不到自己的 LID 存储，
 * nvram_get_dev_boot_times() 读 LID 0xF00A 失败（bt 停在 1），
 * 最后 lid_error_handle.c 断言、HS2 永不发出。
 */
static const struct {
	char drive;
	const char *root;
} pearl_fs_drives[] = {
	{ 'X', "/mnt/protect1/md/" },
	{ 'Y', "/mnt/protect2/md/" },
	{ 'Z', "/mnt/nvdata/md/" },
	/* S: 是基带的 OTA 盘。基带用 op=0x1007 建 S:\mdota，
	 * mcf_ota_a 分区里的字符串就是 /mnt/vendor/mdota，所以 S: 的根是
	 * /mnt/vendor/（mdota 子目录由该分区挂载出来）。
	 */
	{ 'S', "/mnt/vendor/" },
};

/* 未知盘符时的探测候选（保持旧行为） */
static const char *pearl_fs_roots[] = {
	"/mnt/protect1/md/",
	"/mnt/protect2/md/",
	"/mnt/nvdata/md/",
	"/mnt/vendor/",
};

/* PEARL-FS-NVCFG（FSFIX-67 更正）：
 * 旧注释断言"这个 open 必须成功"，那是**错的**。原厂权威参照
 * （notes/dev/fsd-orig-seq.txt，来自 notes/yuechu/lc_boot.txt）显示：
 *   O: X:/nv_config, flag 0x500, ret -9      <-- 原厂就是失败的
 *   O: X:/MT00A001, flag 0x700, ret 2        <-- 之后才全量枚举 LID
 * 我们把 APCFG 索引 AllMap 冒充成 nv_config 让它成功，反而让基带
 * 跳过 LID 枚举、nvram_lid_cache 留空，最终在 LID 0xF00A 上断言。
 * 现在默认关闭本表（见 pearl_fs_nvcfg），仅作逃生舱保留。
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

	/* PEARL-FSFIX-67: nvcfg=0（默认）时整表跳过。
	 * X:\nv_config 于是走正常盘符路径 -> /mnt/protect1/md/nv_config，
	 * 该文件不存在 -> ENOENT(-9)，与原厂 ccci_fsd 的行为完全一致。
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

	/* 已知盘符：直接拼到它自己的根，不依赖文件是否存在（写路径也要落对） */
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
		/* 直接用 filp_open 探测：存在就能打开 */
		{
			struct file *f = filp_open(cand, O_RDONLY, 0);

			if (!IS_ERR(f)) {
				filp_close(f, NULL);
				strscpy(out, cand, outlen);
				return 0;
			}
		}
	}
	/* 都不存在：返回第一个候选，调用方按"空文件"处理 */
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

/* 从 start 偏移读最多 maxlen 字节；返回实际读到的字节数，失败返回 -1 */
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
 * 整文件复制：必须分块。
 * 以前 Move 用 4096 字节的 databuf 一次读完再写，>4KB 的 LID 文件会被截断
 * （实测 MC01_006/MC09_000 5388 -> 4096），基带的备份副本因此损坏。
 * 返回复制的字节数，失败返回负 errno。
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
 * 基带第一次改写某个 NVRAM 文件前留一份备份。
 * 备份放在 /mnt/nvdata/ 根（不是 /mnt/nvdata/md），因为 Z: 只映射到 md/，
 * 所以备份不会出现在基带看到的目录树里。已存在则不重复备份。
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
			return;	/* 已经备份过 */
		}
	}
	{
		unsigned int got = 0;
		int r = pearl_fs_copy_file(path, bak, &got);

		pr_info("PEARL-FS: nvram backup %s -> %s (%d bytes, ret=%d)\n",
			path, bak, got, r);
	}
}

/* PEARL-FS-MP: 多包 0x1024 写入的重组状态（单 worker 串行，无需加锁）。
 * 实测帧型（2026-09-25 抓包）：
 *   头包 w0=0x80000000：op+nblk=3+{name}{desc44}{blk2.len=总长(故意越界)}+数据首块
 *   中间包 w0=0x80000000：op+nblk=0+纯数据
 *   结束包 w0=0x00000000：op+nblk=0+纯数据尾块
 * 数据总长 = blk2.len；收满或见到结束包即写盘并回一条成功应答。 */
static struct {
	int		active;
	unsigned int	total, got;
	unsigned char	*buf;
	unsigned int	wsteps, wbaddr, woff;
	unsigned char	desc[44];
	char		path[256];
} pearl_fs_mpw;

/* PEARL-FS-MP-CONT-64: 多包流只送到 got(<total) 时，余下字节由基带用一条
 * 独立 0x1024 补上（实测 X:\MC06_009：4724 + 2 = 4726）。那条包的 w4=0
 * 并不是续写偏移，必须由我们按"上一个多包写的结束位置"接上；否则 2 字节
 * 会落在偏移 0、把文件头的 "LI" 覆盖成 0，基带随后校验 NVRAM 记录失败
 * （lid_error_handle.c:239 断言、para1=0x1006）并复位基带。 */
static char pearl_fs_cont_path[256];
static unsigned int pearl_fs_cont_off, pearl_fs_cont_len;

/* PEARL-FSMPFIX-72: 多包写"续段"状态。
 * 基带把一个大容器拆成多个多包写，段与段之间用描述符 w1 区分：
 *   w1 == 0  -> 首段，目标偏移 = w4
 *   w1 != 0  -> 续段，目标偏移 = 上一段写完的位置
 * （实测 NR06_010 四段 w1 = 0,0x61,0x61,0x61；w4 恒为 0。） */
static char pearl_fs_mpw_chain_path[256];	/* 上一段写到的映射后路径 */
static unsigned int pearl_fs_mpw_chain_end;	/* 上一段写完的结束偏移 */

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
	char lpath[256];	/* PEARL-FSMPFIX-71: 映射后的真实 Linux 路径 */

	if (!complete) {
		pr_err("PEARL-FS-MP: tail short (%u/%u), write what we got\n",
			pearl_fs_mpw.got, pearl_fs_mpw.total);
		/* PEARL-FS-MP-CONT-64: 记下续写位置，等基带那条独立的补写包接力 */
		strscpy(pearl_fs_cont_path, pearl_fs_mpw.path,
			sizeof(pearl_fs_cont_path));
		pearl_fs_cont_off = pearl_fs_mpw.woff + pearl_fs_mpw.got;
		pearl_fs_cont_len = pearl_fs_mpw.total - pearl_fs_mpw.got;
		pr_info("PEARL-FS-MP: expect continuation at off=%u len=%u\n",
			pearl_fs_cont_off, pearl_fs_cont_len);
	}
	/* PEARL-FSMPFIX-71: pearl_fs_mpw.path 存的是基带原始 CCCI 路径
	 * （"Z:\NVRAM\NVD_DATA\NR06_010"）。直接 filp_open 会在根目录造出
	 * 字面文件名 "/Z:\NVRAM\NVD_DATA\NR06_010"，真实 NVRAM 文件
	 * /mnt/nvdata/md/NVRAM/NVD_DATA/NR06_010 永远不更新 —— 基带写完再
	 * 回读拿到的还是旧内容，LID 校验失败（lid_error_handle.c 断言、
	 * para1=0x985 para2=0x104）。正常 CMPT_WRITE 路径一直用
	 * pearl_fs_map_path()，只有这里漏了，必须补上。
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
		/* PEARL-FSMPFIX-72: 记住这一段的结束位置，供下一段接力 */
		strscpy(pearl_fs_mpw_chain_path, lpath,
			sizeof(pearl_fs_mpw_chain_path));
		pearl_fs_mpw_chain_end = pearl_fs_mpw.woff + pearl_fs_mpw.got;
	} else {
		pr_err("PEARL-FS: mpwrite %s -> %s wrote %d of %u\n",
			pearl_fs_mpw.path, lpath, wret, pearl_fs_mpw.got);
		/* 失败就断链，避免把下一段接到错误的位置上 */
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
	/* PEARL-FS-CMPTW-RSP-64: 参考机 52 条 0x1024 应答的 blk1 恒为 0x2，
	 * 并不是请求里的缓冲区地址。 */
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
	static unsigned char reply[PEARL_FS_MAX_MSG];	/* 单一 worker，不会重入 */
	const unsigned char *blk[PEARL_FS_MAX_BLK];
	unsigned int blk_len[PEARL_FS_MAX_BLK];
	unsigned int op, req_blk, i, off, pos, nblk = 0;
	/* PEARL-FSFIX-64c: CMPT_READ 大应答的分片状态（同一次调用内有效）*/
	unsigned int frag_rest = 0, frag_pos = 0;
	unsigned int status = 0, out = 0, handle = 0, mode = 0;
	unsigned int blk3val = 0;
	/* 防活锁：连续重复同一个请求时退避，见文件末尾注释 */
	static struct {
		unsigned int op, reqlen, words[2];
		unsigned int streak;
	} last_req;
	static unsigned char databuf[PEARL_FS_READ_MAX];	/* PEARL-FSMPFIX-73: 容纳整记录读 */
	/* PEARL-FSFIX-64c: 分片发送用的续包缓冲（单 worker，不会重入）*/
	static unsigned char fragbuf[PEARL_FS_MAX_MSG];
	char name[96];
	int idx, hidx, cnt, j;

	/* PEARL-FSFIX-66: 每次处理 job 前惰性重读盘上配置 */
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
	reply[8] = CCCI_FS_TX;	/* channel 低字节；第 9 字节本来就是 0 */
	*(unsigned int *)(reply + 16) = op | 0xFFFF0000U;
	pos = 24;
	/* PEARL-FS-MP: 多包 0x1024 写入分发（在普通 switch 之前） */
	/* PEARL-FSMPFIX-70: 多包 0x1024 续包识别修正。
	 *
	 * 实测基带的多包 0x1024 写入分 3 段（Z:\NVRAM\NVD_DATA\ER1B_152，9632 字节）：
	 *   head : [16B ccci_header][u32 op][u32 nblk][块…][数据]   hdr[0] bit31=1
	 *   cont : [16B ccci_header][u32 op][原始数据 @ 偏移 20]    hdr[0] bit31=1
	 *   tail : 同 cont，但 hdr[0] bit31=0
	 * 续包【没有 nblk 字段】，偏移 20 处的 4 字节数据会被当成 nblk（随机值），
	 * 块解析必然失败（i < 2 或没有合法路径）。
	 *
	 * 旧条件只认 "bit31 置位" 或 "req_blk==0"，于是第二段续包
	 * （bit31=0、nblk 随机非 0）掉进 case PEARL_FS_OP_CMPT_WRITE 被
	 * "mp/anomaly drop"，多包写永远凑不满声明总长（实测 9624/9632）。
	 * 基带随后报 CMPTW fail[fs_ret:-1001] → LID 0xEE01 写失败
	 * → lid_error_handle.c:239 断言 → md_state 3→5。
	 *
	 * 新条件：只要有活跃的多包流，且本包不是"带合法路径的头包"，就是续包。
	 */
	if (op == PEARL_FS_OP_CMPT_WRITE &&
	    ((*(unsigned int *)req & 0x80000000U) ||
	     (pearl_fs_mpw.active && !(i >= 2 && name[0])) ||
	     (req_blk == 0 && !name[0]))) {
		unsigned int inpk;

		if (!(*(unsigned int *)req & 0x80000000U)) {
			/* 结束包：bit31 清零、nblk=0、无名字 */
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
			/* 头包：名字 + 描述符 + blk2.len(=总长,越界) + 数据首块 */
			unsigned int total = *(unsigned int *)(req + off - 4);

			/* PEARL-FS-MP-TOTAL-64: 参考机实测 blk2.len 是"本包片段长"
			 * （464），而描述符 w7(+28) 才是整个写入的总长（15884）。
			 * 取两者较大者，否则大文件会被误当成只有首块那么长。 */
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
			/* PEARL-FS-MP-OFF-64: 写偏移在描述符 w4(+16)，与普通
			 * 0x1024 路径（FSFIX-63）以及参考机 52 条实测一致。
			 * 旧代码取 w5(+20)，恒为 0，会把整个文件写到偏移 0。 */
			pearl_fs_mpw.woff = (blk_len[1] >= 20) ?
				*(unsigned int *)(blk[1] + 16) : 0;
			if (pearl_fs_mpw.woff > PEARL_FS_MAX_WRITE_OFF) {
				pr_err("PEARL-FS-MP: woff 0x%x not an offset, using 0\n",
					pearl_fs_mpw.woff);
				pearl_fs_mpw.woff = 0;
			}
			/* PEARL-FSMPFIX-72: 首段/续段判定。
			 * 实测四段描述符 w1 = 0x00,0x61,0x61,0x61，w4 恒为 0，
			 * w6（源缓冲指针）步进恒为 0x4000 = 段长。w1==0 即首段。 */
			{
				unsigned int w1v = (blk_len[1] >= 8) ?
					*(unsigned int *)(blk[1] + 4) : 0;
				char clpath[256];

				pearl_fs_map_path(name, clpath, sizeof(clpath));
				if (w1v == 0 ||
				    strcmp(pearl_fs_mpw_chain_path, clpath) != 0) {
					/* 新的一段逻辑写：偏移就用描述符 w4 */
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
			/* PEARL-FSMPFIX-71: 打印头包描述符与原始字节 */
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
		/* 中间包：bit31 置位、nblk=0、纯数据 */
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
			/* PEARL-FSFIX-65: 访问模式按实测语义解码。4 种实测 mode 自洽：
			 *   0x10400 Z:\nv_boot_trace / nv_mini_dump  读|写|create
			 *   0x500   X:\nv_config / Z:\FATD7C68128.log 读|写
			 *   0x900   Z:\NVRAM / Z:\NVRAM\NVD_DATA\     读|目录
			 * ⇒ 0x100=读、0x400=写、0x800=目录、0x10000=create。
			 * 旧判据 `mode & 0x100` 把 0x10400 当只读探测，把基带要写的
			 * nv_boot_trace / nv_mini_dump 开成 O_RDONLY，于是它随后的
			 * 0x1004 Write 走 kernel_write() 必然 -EBADF，基带 trace 与
			 * 崩溃转储从此写不出来（mtime 停在 9/25，断言现场失明）。
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
		/* PEARL-FSFIX-65: 应答形状可切换，见 pearl_fs_open_rsp2。 */
		/* PEARL-FSFIX-66: 配置文件 open_rsp2=1 同样可开 */
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

		if (strstr(spath, "NVRAM") || strstr(dpath, "NVRAM")) {
			pr_err("PEARL-FS: move refused (NVRAM) %s -> %s\n",
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
		 * op 表（Android ccci_fsd 跳转表）：0x1007 = FS_CCCI_CreateDir。
		 * 基带用它建 S:\mdota（OTA 工作目录）。目录已存在也回成功，
		 * 因为基带每次启动都会调用。
		 * lookup_one_len 已从内核移除、kern_path_create 未导出，
		 * 所以交给 /bin/mkdir -p（-p 对已存在的目录返回 0）。
		 */
		char dpath[256];
		char *argv[5];
		char *envp[3];
		int r;

		/* 0x1007 只有 1 个块，路径就在 blk[0]
		 * （0x1001/0x1022 之类 nblk=2 的才是 blk[1]）。
		 */
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
		/* PEARL-FS-CMPTW: 0x1024，请求 = {path} + 44 字节描述符 + 数据(N)。
		 * 这是基带在写一个 NVRAM LID 文件（blk[2] 以 "LID\0" 开头）。
		 *
		 * 回复布局必须与 CMPT_READ 同形状（去掉数据块）——依据
		 * notes/rpcd/FS_PROTOCOL.md 第 6 节 yuechu 真机 strace：
		 *   0x1022 -> nblocks=4 {8}{4}{4}{112}
		 *   0x1024 -> nblocks=3 {8}{4}{4}
		 * 只回 1 块 {4: out} 时基带把块0当 {steps,status} 解析，读到 -1001
		 * （正是本文件 default 分支那个未实现 op 的错误码），于是判定写失败，
		 * dev_fs_write() 返回 8768 → nvram_get_dev_boot_times() 读 LID 失败
		 * → bt 停在 1 → lid_error_handle.c:228 断言、HS2 永不发出。
		 */
		char wpath[256];
		const unsigned char *data = NULL;
		unsigned int dlen = 0;
		unsigned int wsteps = 0, wbaddr = 0, woff = 0;
		struct file *wf;
		loff_t wpos;
		int wret;

		/* 描述符与 CMPT_READ 同布局：w0=步骤位图，+16=缓冲地址 */
		if (i >= 2 && blk_len[1] >= 8)
			wsteps = *(unsigned int *)blk[1];
		if (i >= 2 && blk_len[1] >= 20)
			wbaddr = *(unsigned int *)(blk[1] + 16);
		/*
		 * PEARL-FSFIX-63: 写偏移在描述符的 w4（字节 +16），不是 w5。
		 * 实测两例（PEARL-FS-REQ 原始字节）：
		 *   Z:\BITMAP    w4=0x0    w7=1932
		 *   X:\MTBT_000  w4=0xa0   w7=44
		 * 旧代码从 +20 取（w5 恒为 0），于是把 MTBT_000 的新记录写到了
		 * 偏移 0 而不是 160 —— 文件里新旧记录正好写反，基带校验 NVRAM
		 * 记录失败（CMPT_R ... read len(exp/r):0:44 -> dev_fs_read()
		 * ret:260 -> lid_error_handle.c:239 断言）。
		 *
		 * 护栏：真实 NVRAM 文件最大也就几十 KB，偏移不会超过 64MB。
		 * 若 w4 超出这个范围，说明它不是偏移（可能是缓冲区地址），
		 * 这时退回旧行为并告警，避免把文件写到天文数字的偏移上。
		 */
		if (i >= 2 && blk_len[1] >= 20) {
			unsigned int cand = *(unsigned int *)(blk[1] + 16);

			if (cand <= PEARL_FS_MAX_WRITE_OFF)
				woff = cand;
			else
				pr_err("PEARL-FS: cmptwrite %s w4=0x%x not an offset, using 0\n",
				       name, cand);
		}
		/* PEARL-FS-MP-CONT-64: 上一个多包写只送了一半时，基带会用一条
		 * 独立 0x1024 补上剩余字节，但那条包的 w4 是 0（不是续写偏移）。
		 * 这里用多包写留下的结束位置接力，否则文件头会被覆盖。 */
		if (pearl_fs_cont_path[0] &&
		    strcmp(pearl_fs_cont_path, name) == 0) {
			woff = pearl_fs_cont_off;
			pr_info("PEARL-FS: cmptwrite %s mp-continuation off=%u len=%u\n",
				name, woff, pearl_fs_cont_len);
			pearl_fs_cont_path[0] = 0;
			pearl_fs_cont_off = 0;
			pearl_fs_cont_len = 0;
		}
		/* PEARL-FSMPFIX-74: w4==0 的单包写先 dump 描述符（w1=续段标志）。 */
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
		/* PEARL-FSMPFIX-74: 单包写作为上一段多包写的"续段"时，偏移必须
		 * 接在多包写结束的位置上。
		 *
		 * 实测：NR08_004 的记录被基带拆成 3 条多包写（off=160/16544/32928，
		 * 各 16384）+ 1 条单包写（3292 字节，w4=0）。那条单包写本应落在
		 * 0xa0+49152=49312，却因为 w4=0 落到偏移 0，把 LID 头 0..160 和
		 * 记录前 3132 字节一起清零（"头全 0"）。
		 * 多包写链的结束位置已由 FSMPFIX-72 记在 pearl_fs_mpw_chain_end。
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

		status = 1;    /* 默认失败；成功分支再清 0 */
		if (!name[0] || !data || dlen == 0) {
			/* PEARL-FS-MP: 多包写入头包（名字+描述符在、数据块越界）或
			 * 已被上层吞掉剩余形状的残片。回 ERROR 会被 modem 当可重试，
			 * 背靠背重发搅乱协议；这里静默丢弃，一次回复都不给。 */
			pr_err("PEARL-FS: cmptwrite mp/anomaly drop (name=%d data=%d len=%u)\n",
			       name[0] ? 1 : 0, data ? 1 : 0, dlen);
			pearl_fs_dump_req("mp-head", req, req_len);
			goto cmptw_drop;
		}

		pearl_fs_map_path(name, wpath, sizeof(wpath));

		/*
		 * 以前这里一律拒绝含 "NVRAM" 的写入。基带启动时会回写
		 * Z:\NVRAM\NVD_DATA\MC04_010 这类自己的 NVRAM 项（43968 字节，
		 * 按 ~3372 字节分块，op=0x1024 req=3476），被拒后它把失败当可重试、
		 * 背靠背无限重发（seq 44..48 长度一模一样，queued depth 连续递增），
		 * 请求+日志洪水把 CPU 吃干（load 10+）、用户态饿死、最后硬复位。
		 * 那是基带自己的存储，读路径我们一直在服务，写入同样放开；
		 * 首次改写前 pearl_fs_backup_once() 会留一份备份。
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
			/* PEARL-FSFIX-66: 可用 write_two 覆盖（默认仍 2） */
			unsigned int two = pearl_fs_write_two;

			hdr[0] = wsteps ? wsteps : 0x1d;
			hdr[1] = status;
			pos = pearl_fs_put_block(reply, pos, hdr, sizeof(hdr));
			/* PEARL-FS-CMPTW-RSP-64: 参考机 52 条应答的 blk1 恒为 0x2 */
			pos = pearl_fs_put_block(reply, pos, &two, 4);
		}
		pos = pearl_fs_put_block(reply, pos, &out, 4);
		nblk = 3;
		break;

cmptw_drop:
		/* PEARL-FSMPFIX-70: 这里必须真的"不回复"。
		 * 旧代码 break 出 switch 后仍会照常发一条 24 字节、nblk=0 的畸形
		 * 应答；reply 缓冲里 offset>=24 还是上一条应答的残留字节，
		 * 基带按 3 块形状解析 CMPT_WRITE 应答时会读到残留的 -1001
		 * (0xFFFFFC17)，把一次 NVRAM 写入判成 CMPTW fail[fs_ret:-1001]。
		 * goto mp_skip 只解锁、不发送。 */
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
			status = 1;
			pos = pearl_fs_put_block(reply, pos, &status, 4);
			nblk = 1;
			break;
		}
		f = &pearl_fs_files[pearl_fs_handles[hidx].file];
		/*
		 * PEARL-FS-WOFF: 用"文件当前位置"而不是 block3。
		 * 基带的动作是 Seek(SEEK_END) 之后再 Write（POSIX 语义），
		 * 实测把 block3 当偏移会写到 0 —— 表现是 nv_boot_trace
		 * 永远不增长（我们写成功了但文件大小不变）。block3 只记日志。
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
		unsigned char tmp[2048];

		if (hidx < 0) {
			status = 1;
		} else {
			f = &pearl_fs_files[pearl_fs_handles[hidx].file];
			roff = pearl_fs_handles[hidx].pos;
			if (i >= 2 && blk_len[1] >= 4)
				rlen = *(unsigned int *)blk[1];
			if (rlen > sizeof(tmp))
				rlen = sizeof(tmp);
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
		pos = pearl_fs_put_block(reply, pos, &status, 4);
		pos = pearl_fs_put_block(reply, pos, tmp, out);
		nblk = 2;
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
		 * 实测（yuechu strace 4 个样本一致）：
		 *   请求 blk1 = {len: path(UTF-16LE)}
		 *   请求 blk2 = 40 字节描述符:
		 *     w0=步骤位图  w1=状态  w2/w3=flags  w4=缓冲地址
		 *     w5..w7=?     w8=要读取的长度  w9=?
		 *   回复 nblk=4: {8: [w0][状态]} {4: w4} {4: 实际长度} {数据}
		 * 处理链 = Open + GetFileSize + Seek + Read + Close（读整个文件）。
		 */
		unsigned int steps = 0, astat = 0, bufaddr = 0, want = 0;
		unsigned int want_raw = 0;	/* PEARL-FSMPFIX-73: 夹断前的原始请求长度 */
		unsigned int roff = 0;	/* 描述符 w5：读取偏移 */
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
		/* PEARL-FSFIX-66: 偏移/长度可被配置覆盖（默认 auto = 用描述符值）。
		 * 故障链 B 的 chksum error 要求"返回的字节"和基带期望的完全一致；
		 * 若 w5 其实不是偏移，read_roff=zero 会立刻改变结局。
		 */
		if (pearl_fs_read_roff_mode)
			roff = pearl_fs_read_roff_const;
		if (pearl_fs_read_want_mode)
			want = pearl_fs_read_want_const;
		want_raw = want;	/* PEARL-FSMPFIX-73: 夹断前的原始请求长度 */
		if (want == 0 || want > sizeof(databuf))
			want = sizeof(databuf);
		/* PEARL-FSFIX-64: 旧值 PEARL_FS_DATA_MAX(3420) 会把 MC06_009
		 * 的 4566 截断，导致基带 CMPT_R 长度校验失败(260)。
		 * PEARL-FSMPFIX-73: 上限从 PEARL_FS_DATA_MAX_BIG(8128) 抬到
		 * databuf 容量 —— NR06_010 要读 pl+40=61640，旧上限静默截断；
		 * 超长部分仍走 PEARL-FS-FRAG 续包（头包 3420 + 若干续包）。
		 */
		if (want > PEARL_FS_READ_MAX)
			want = PEARL_FS_READ_MAX;
		memset(databuf, 0, sizeof(databuf));
		pearl_fs_map_path(name, lpath, sizeof(lpath));
		got = pearl_fs_read_file(lpath, databuf, want, (loff_t)roff);
		/* PEARL-FSMPFIX-73: 大读时把 40 字节描述符原样 dump 出来，
		 * 用来确认基带 w8 到底请求了多少字节（8128 是内核夹断值，
		 * 61640 才是 pl+40）。
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
		/* PEARL-FSFIX-66: 前 6 次 CMPT_READ dump 返回数据头 48 字节，
		 * 用于和盘上字节逐字节比对（基带只在数据不符时 chksum error）。
		 */
		if (pearl_fs_read_dump && pearl_fs_read_dump_cnt < 6) {
			pearl_fs_read_dump_cnt++;
			print_hex_dump(KERN_ERR, "PEARL-FS-RD: ",
				DUMP_PREFIX_OFFSET, 16, 1, databuf,
				got > 48 ? 48 : (unsigned int)got, false);
		}
		out = got;
		/* 步骤位图：0x1d = open|seek|read|close 都做过（照 Android 观测值） */
		if (steps == 0)
			steps = 0x1d;
		{
			unsigned int hdr[2];

			hdr[0] = steps;
			/* PEARL-FSFIX-66: hdr[1] 语义可配置（默认 0 = FSFIX-65 旧行为）。
			 * 基带 trace `read len(exp/r):0:44` 里的 exp=0 与旧值吻合；
			 * 若 exp 实为"本次应读长度"，则应回 got/want。
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
		/* PEARL-FSFIX-66: blk2 报 got 还是 want 可配置（默认 got）。
		 * 注意：数据块长度始终是真实读到的 out，本开关只改"报数"。
		 */
		{
			unsigned int rlen = (pearl_fs_read_out_mode == 1) ? want : out;

			pos = pearl_fs_put_block(reply, pos, &rlen, 4);
		}
		/* PEARL-FSFIX-68: blk3 的"声明长度"必须报完整的 out。
		 *
		 * 基带 MD 侧解析器（CCCI_FS_OP_Wrapper@0x3154b2）是
		 *   memcpy(dst = s3[3].ptr, src = rx + 0x38, n = blk3.len)
		 * —— 拷多少完全由 blk3.len 决定。FSFIX-64c 报 3420，
		 * 于是记录区尾部（sec_factor 在记录区相对偏移 4526、
		 * chksum 紧随其后）根本没被拷进目标缓冲，留在旧值 0：
		 *   [E][ID:0x1006][ret:4288]R fail sec_factor error
		 *   [U]00000000 00000000   (期望 06100000 01000000)
		 * 多出来的字节由续包补齐：基带把续包剥掉 20 字节头后追加在
		 * 头包之后，所以 rx + 0x38 + 3420 正好接上续包的数据。
		 */
		if (out > PEARL_FS_DATA_MAX) {
			frag_pos = PEARL_FS_DATA_MAX;
			frag_rest = out - PEARL_FS_DATA_MAX;
			if (pearl_fs_frag68) {
				/* 只装首片，但长度字段报 out */
				*(unsigned int *)(reply + pos) = out;
				pos += sizeof(unsigned int);
				memcpy(reply + pos, databuf,
				       PEARL_FS_DATA_MAX);
				/* 3420 已是 4 的倍数，无需补零 */
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
		/* PEARL-FSFIX-69: 基带 md_state=3（HS1 完成）之后的最后一批请求就是
		 * 这两个 op —— 查 mdota 配置文件（MTK_RAWOTA_DEFAULT.mcfrawota 等）
		 * 的属性。以前没有实现，落进 default 回 -1001(0xFFFFFC17)，基带把它
		 * 当致命错误，之后【所有通道】静默 36s ⇒ MD_BOOT_HS2_FAIL。
		 *
		 * 决定性对照：原厂 ccci_fsd 对这些文件同样报 error=2(ENOENT)，
		 * 而且连查了几十个 SAR 配置之后正常继续、基带 READY
		 * （notes/dev/fsd-orig-seq.txt）。⇒ 差别在"回的是合法 errno 还是
		 * -1001"，不在文件是否存在。
		 *
		 * 形状未实测确认，故运行时可选（/mnt/nvdata/md/pearl_fs.cfg）：
		 *   shape 0 = nblk=1 {u32 st}
		 *   shape 1 = nblk=2 {u32 st}{u32 attr}
		 *   shape 2 = nblk=1 {u32 0}          （当成"存在、属性 0"）
		 *   shape 3 = nblk=2 {u32 0}{u32 attr}
		 *   shape 9 = 每次请求在 0..3 间轮转（一次刷机多试几种）
		 */
		unsigned int is_attr = (op == PEARL_FS_OP_GET_ATTR);
		unsigned int shape = is_attr ? pearl_fs_getattr_rsp
					     : pearl_fs_detail_rsp;
		unsigned int st = is_attr ? pearl_fs_getattr_st
					  : pearl_fs_detail_st;
		unsigned int at = is_attr ? pearl_fs_getattr_attr
					  : pearl_fs_detail_attr;

		if (shape == 9) {	/* 轮转：一次刷机把 0..3 都试一遍 */
			shape = pearl_fs_getattr_rot++ & 3U;
			pr_err("PEARL-FS: %s rotate -> shape %u\n",
			       is_attr ? "getattr" : "filedetail", shape);
		}
		if (shape == 2 || shape == 3)
			st = 0;	/* 当成"文件存在、属性 0" */
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
	default:
		/* 未实现的 op（FindFirst/GetAttributes/Restore/...）：回 MTK 的通用错误码，
		 * 而不是 1 —— 基带把回复里的值当错误码用（实测它把我们的 1 显示成 -1001）。
		 */
		status = 0xFFFFFC17;	/* -1001，与基带 trace 里看到的一致 */
		pos = pearl_fs_put_block(reply, pos, &status, 4);
		nblk = 1;
		break;
	}
	*(unsigned int *)(reply + 20) = nblk;
	*(unsigned int *)(reply + 4) = pos;
	mutex_unlock(&pearl_fs_lock);

	cnt = atomic_inc_return(&pearl_fs_msg_cnt);
	/* PEARL-FSMPFIX-70: 窗口 96 -> 400。旧值把 #96 之后的请求全部埋掉，
	 * 无法判断多包写之后还有没有新的缺失 op。 */
	if (cnt <= 400)
		pr_err("PEARL-FS: #%d op=0x%04x seq=%u req=%u nblk=%u h=%u mode=0x%x st=%u out=%u b3=0x%x name=%s -> rep=%u\n",
			cnt, op, ((struct ccci_header *)req)->seq_num, req_len,
			req_blk, handle, mode, status, out, blk3val, name, pos);
	/* PEARL-FSMPFIX-70: 任何非 0 的应答状态都单独记一条，不受 cnt 限流。
	 * 0xFFFFFFFE(-2/ENOENT) 是 getattr/filedetail 查缺失文件的正常返回，
	 * 不算失败，排除。 */
	if (status != 0 && status != 0xFFFFFFFEu)
		pr_err("PEARL-FS-FAIL: #%d op=0x%04x seq=%u req=%u st=%u out=%u name=%s\n",
			cnt, op, ((struct ccci_header *)req)->seq_num, req_len,
			status, out, name);
	/* nblk==0 的帧（基带重试帧）也 dump 出来，第一次见时最需要。
	 * PEARL-FSFIX-65: 窗口 128 -> 256。基带 0x1024 头包的 11×u32 描述符
	 * 之后紧跟数据首块，128 字节会把数据首块整段截掉。
	 */
	if (cnt <= 400 || req_blk == 0)
		print_hex_dump(KERN_ERR, "PEARL-FS-REQ: ", DUMP_PREFIX_OFFSET,
			16, 1, req, req_len < 256 ? req_len : 256, false);
	/* PEARL-FSFIX-65: 前 32 条同时 dump 我们的应答（发送之前）。
	 * 基带日志 "O: <path>, flag <mode>, ret <n>" 的 ret 取自应答 blk0，
	 * 看不到自己发了什么就无法判定 OPEN 应答形状。
	 */
	if (cnt <= 400)
		print_hex_dump(KERN_ERR, "PEARL-FS-REP: ", DUMP_PREFIX_OFFSET,
			16, 1, reply, pos < 128 ? pos : 128, false);

	/*
	 * 防活锁：基带在收到失败回复时会立刻重发同一个请求。连续重复越多，
	 * 回复前的退避越长（上限 200ms），免得请求+日志洪水把用户态饿死。
	 * 注意：只是延后回复，绝不假回成功。
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
			/* pearl_fs_lock 在此处早已释放（见上面的 mutex_unlock），
			 * 而且只有一个 worker，所以直接睡，不要再动锁。
			 */
			msleep(backoff);
		}
	}

	/* PEARL-FSFIX-68: 大应答分片发送（原厂形状）。
	 *
	 * 头包：完整应答（ccci_header 16 + op 4 + nblk 4 + 4 个块 + 首片数据），
	 *       word0 bit31 置位表示"还有分片"，整包 3476 = 20 + 3456。
	 * 续包：ccci_header(16) + op|0xFFFF0000 + 数据（偏移 20），整包 20 + n，
	 *       末包 bit31 清零。**没有 nblk 字段** —— 这正是 FSFIX-64c 的错处。
	 *
	 * 接收侧（基带）按 "剥掉 20 字节头、把剩余 payload 追加" 重组，
	 * 所以续包的数据在重组缓冲里紧接头包数据之后：
	 *   rx+0x38 + 3420 == rx+3476 == 续包 payload 起点。
	 *
	 * 若 pearl_fs_frag68 == 0 则退回 FSFIX-64c 的 24 字节续包头（仅用于 A/B）。
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

/* 单个 worker 串行把队列排干；job 由这里释放 */
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

	if (pearl_fs_mode == 0 || len < 24 || len > PEARL_FS_MAX_MSG)
		return 0;
	/* PEARL-FS-MP: bit31 分片帧必须入队交给 worker 重组（见 process_job
	 * 开头的 mp 分发）；在这里吞掉的话重组器就永远收不到数据了。 */
	/* 可能在中断上下文被调用：只做拷贝 + 入队，处理交给 worker */
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
	/* PEARL-FSFIX-62: 构建标记。61316821e3 引入的 label fall-through
	 * 让 switch (op) 成了不可达代码，MD 的 FS 请求因此从不被应答，
	 * HS2 永远超时。那一版把标签挪回正确位置。
	 *
	 * PEARL-FSFIX-63: 构建标记。CMPT_WRITE 的写偏移改从描述符 w4 取
	 * （旧代码从 w5 取，而 w5 恒为 0，导致 MTBT_000 的新记录被写到
	 * 偏移 0 而不是 160，基带校验 NVRAM 记录失败后断言）。
	 *
	 * PEARL-FSFIX-65: 构建标记。OPEN 的访问模式判据由 `mode & 0x100`
	 * 改为 `mode & 0x400`（写）/ `mode & 0x10000`（create）。旧判据把
	 * nv_boot_trace / nv_mini_dump 的 0x10400 当只读探测、开成 O_RDONLY，
	 * 基带随后的 0x1004 Write 必然 -EBADF ⇒ 基带 trace 与崩溃转储自
	 * 9/25 起再未更新，断言现场只剩 AP 侧。同时把请求 dump 窗口放宽到
	 * 256 字节并新增应答 dump（PEARL-FS-REP）。
	 *
	 * PEARL-FSFIX-68: 构建标记。CMPT_READ 大应答的分片协议改成原厂形状：
	 *   续包 = ccci_header(16) + op(4) + 数据（偏移 20，不再有 nblk=0），
	 *   整包 20+n；头包 blk3 的声明长度报完整 out（基带按它 memcpy）。
	 * 依据：原厂 rpcd（ARM64，notes/rpcd/rpcd_disasm_full.txt@0x7794）与
	 * ccci_fsd（ARM，notes/dev/fsd.asm@0xd1e0）都用 20 字节续包头 +
	 * bit31=more + 载荷上限 3456；基带解析器 CCCI_FS_OP_Wrapper@0x3154b2
	 * 只按 blk3.len 从 rx+0x38 拷贝。
	 *
	 * PEARL-FSFIX-69: 构建标记。补上 FS_CCCI_GetAttributes(0x1010) 与
	 * FS_CCCI_GetFileDetail(0x1025)。基带 md_state=3 之后的最后 3 条请求
	 * 就是 0x1010（查 S:\mdota / T:\custom / T:\mtk_default 下的
	 * MTK_RAWOTA_DEFAULT.mcfrawota 属性）；以前落进 default 回 -1001，
	 * 基带当致命错误后全链路静默 36s ⇒ MD_BOOT_HS2_FAIL。原厂 fsd 对同样
	 * 缺失的文件回合法 errno（error=2/ENOENT）并继续启动。
	 * 应答形状/状态值可用 /mnt/nvdata/md/pearl_fs.cfg 的
	 * getattr_rsp/getattr_st/getattr_attr、detail_rsp/detail_st/detail_attr
	 * 运行时切换（shape 0..3，9=轮转）。
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

/* COPY：按 {src,dst,len} 把 md1drdi 数据搬进 64KiB DRDI smem */
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
	/* PEARL-AMMS: DRDI smem 必须非 cache —— 见 scripts/patch-amms-drdi-wc.py 的说明。
	 * 厂商把 SMEM_USER_MD_DRDI 放在 md1_6297_noncacheable_fat[]，userspace
	 * ccci_rpcd 也用 pgprot_noncached 映射它。memremap(MEMREMAP_WB) 会拿到
	 * cacheable 线性映射，MD 走总线读 DRAM 会读到旧数据（并造成同一物理页
	 * 的 cacheable/non-cacheable 别名）。这里统一用 ioremap_wc，与同文件
	 * NVRAM cache 路径一致。
	 */
	dst_base = ioremap_wc(smem->base_ap_view_phy, smem->size);
	if (!dst_base) {
		CCCI_ERROR_LOG(md_id, RPC, "PEARL-AMMS: ioremap_wc smem fail\n");
		return -ENOMEM;
	}
	/* COPY 表从 req+0x08 开始，stride 12：{src, dst, len}
	 * （ccci_rpcd 反汇编：x9=req+0x10 指向 len，src=[x9-8], dst=[x9-4]）
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
	/* 写序：确保拷贝在应答发出前落到 DRAM（MD 与 AP 不缓存一致） */
	wmb();
	iounmap(dst_base);
	return ret;
}

/* 处理一条 AMMS DRDI 请求：写 opkt[]，返回参数个数（应答固定 2 个参数） */
/* ===== PEARL-MDLOG-AMMS-BEGIN ===== */
/* PEARL-MDLOG: 任务要求的「首次 AMMS 触发」——AMMS 说明基带已经起来并在通信，
 * 此时顺手把日志使能消息发一次（只发一次；同样受 pearl_mdlog_auto 开关约束）。 */
extern int pearl_mdlog_send_armed(unsigned int msg, unsigned int resv,
	int blocking, const char *why);
extern void pearl_mdlog_kick(void);

static void pearl_mdlog_amms_kick(void)
{
	/* PEARL-MDLOG-EXC: 这里不能发。
	 *
	 * AMMS 请求正好出现在基带启动握手的 HS1/HS2 窗口里（约 7.7-8.1s），
	 * 而厂商 port_proxy.c 在这个窗口明确禁止一切非 FS/RPC 端口流量；实验 M
	 * 证明此时发 0x0C 会让基带固定在 HS1+5.43s 断言（ccismcore_ccci.c:1326）。
	 * 拉日志改由进入 EXCEPTION 时的 pearl_mdlog_kick() 负责。
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
	/* 与 ccci_rpcd 一致：清掉 data[0] 的"还有分片"位 */
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
		 * PEARL-AMMS-RESET: INIT 表示"MD 这一轮启动的 AMMS 会话开始"。
		 * pearl_amms_copy_done[] 是内核静态量，只在模块加载时清零，
		 * **不会随 MD 重启复位**。于是用 ccci-mdctl 重启 MD 时，会把
		 * 上一轮启动留下的 copy_done==1 当成"DRDI 已经拷过了"，
		 * 在 INIT 应答里回 copystat=0xFF，MD 于是**一条 COPY 都不发**。
		 *
		 * 实测（同一内核 #51）：
		 *   整机启动 : AMMS 15 条 = 1 INIT + 14 COPY，copystat=0x0
		 *   mdctl 重启: AMMS  1 条 = 1 INIT +  0 COPY，copystat=0xFF
		 * ⇒ mdctl 重启与整机启动**不等价**，此前基于它的注入实验结论
		 *   （"灌消息无效"）需要在修好之后再复核。
		 *
		 * 按启动周期清零，让 INIT 应答如实反映"本轮还没拷"。
		 * 整机启动时本就是 0，此改动对整机启动无影响。
		 */
		pearl_amms_copy_done[slot] = 0;
		/*
		 * PEARL: CCCI 内建后 port_rpc_init 在 ~2.7s 就返回了，那一刻
		 * /dev/disk/by-partlabel 尚未建立（udev 还没跑），
		 * pearl_drdi_load_image() 打不开 md1img 分区，pearl_drdi_data
		 * 恒为 NULL，AMMS init 只能回 0xFFFFFFFF。这里改成与 NVRAM
		 * 缓存同样的懒加载：真正要用的时候再读一次。
		 */
		if (!pearl_drdi_data)
			pearl_drdi_load_image();
		/* MODEM 马上要读 RF/NVRAM 数据了：先把 NVRAM cache 区灌好 */
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
		/* 完全照 rpcd 行为：copy_done 不为 -1 时 stats=0，ret=0 */
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
		/* PEARL: 内核侧实现 Android ccci_rpcd 的 AMMS DRDI 应答 */
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
		/* PEARL-RPC-KERNEL-69: 原厂由 ccci_rpcd 应答，实测逐字节形状
		 * （notes/yuechu/lc_boot.txt）：
		 *   Read 32 bytes  CCCI_H(0x0)(0x20)(0x80260020)  ← 1 个 4 字节参数
		 *   IPC_RPC_SAR_TABLE_IDX_QUERY_OP:mtk_sar_table_id_get fail
		 *   IPC_RPC_SAR_TABLE_IDX_QUERY_OP, value: 0, ret: 0
		 *   Write 40 bytes CCCI_H(0x0)(0x28)(0x80260021)  ← 2 个 4 字节参数
		 * ⇒ 应答 = {u32 value; u32 ret} = {0, 0}（取值失败也回 0/0）。
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
		/* PEARL-RPC-KERNEL-69: 原厂回属性值
		 * （实测 logcat: key<ro.product.vendor.name>, value<yuechu>）。
		 */
		pkt_num = 0;
		{
			unsigned int n = strlen(pearl_ap_sys_prop_val) + 1;

			if (n > sizeof(pearl_ap_sys_prop_val))
				n = sizeof(pearl_ap_sys_prop_val);
			memcpy(tmp_data, pearl_ap_sys_prop_val, n);
			pkt[pkt_num].len = n;
			pkt[pkt_num++].buf = (void *)tmp_data;
		}
		pr_err("PEARL-RPC-KERNEL-69: QUERY_AP_SYS_PROPERTY -> %s\n",
		       pearl_ap_sys_prop_val);
		break;
	case IPC_RPC_SAVE_MD_CAPID:
		/* PEARL-RPC-KERNEL-69: 原厂 rpcd 记 md_capid/md_aac；
		 * Mobian 无 rpcd ⇒ 回一个成功码，至少让基带不卡。
		 */
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
	/* PEARL-RPCREQ: 记录每个 RPC 请求的 op_id/参数个数 */
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
	/* PEARL-RPCREP: 回复内容摘要 */
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
		pearl_drdi_load_image();	/* PEARL: AMMS DRDI copy 的数据源 */
		first_init = 0;
		pr_info("PEARL-RPC-KERNEL-69: init (0x400F/0x4010/0x4015 收到内核侧, sar_table_id=%d args=%d prop=%s)\n",
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
			/* PEARL-RPC-KERNEL-69: 原厂这三个 op 由 userspace ccci_rpcd 应答
			 * （notes/yuechu/lc_boot.txt：
			 *   "IPC_RPC_SAR_TABLE_IDX_QUERY_OP, value: %d, ret: %d"
			 *   "IPC_RPC_QUERY_AP_SYS_PROPERTY, key<%s>, value<%s>"）。
			 * Mobian 上没有 ccci_rpcd ⇒ 帧被投给 /dev/ccci_rpc 字符节点
			 * 而永远无人应答 ⇒ 基带死等 ⇒ MD_BOOT_HS2_FAIL。
			 * 实测：8.0415s 有一帧 ch=32 len=32 完全无日志（被吞）。
			 * 与 AMMS DRDI(0x4014) 同样处理：收到内核侧。
			 */
			is_userspace_msg = 0;
			pr_err("PEARL-RPC-KERNEL-69: op=0x%x 转内核处理（无 ccci_rpcd）\n",
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

