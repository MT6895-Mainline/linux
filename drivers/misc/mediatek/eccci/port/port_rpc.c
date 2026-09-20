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

/* XAGA-TEST：覆盖 RPC 回答的 DRDI 射频配置集索引（-1 表示沿用设备树） */
static int xaga_rf_set_idx = -1;
module_param(xaga_rf_set_idx, int, 0644);
MODULE_PARM_DESC(xaga_rf_set_idx,
	"XAGA test override for mediatek,md_drdi_rf_set_idx (-1 = use DT)");
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
	 * XAGA: the DT property is <&pio 42 0>, whose index 1 is the pin number
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

	/* XAGA-TEST：先看测试覆盖参数 */
	if (xaga_rf_set_idx >= 0 &&
	    strncmp(input->strName, "mediatek,md_drdi_rf_set_idx",
		    strlen("mediatek,md_drdi_rf_set_idx")) == 0) {
		output->retValue = (unsigned int)xaga_rf_set_idx;
		CCCI_ERROR_LOG(-1, RPC, "XAGA-TEST: rf_set_idx -> %d\n",
			xaga_rf_set_idx);
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

/* ===== XAGA: AMMS DRDI control (kernel-side replacement for ccci_rpcd) =====
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

#define XAGA_AMMS_MAX_SET	15
#define XAGA_AMMS_REQ_SIZE	188	/* 8 + 15*12，内核硬校验 0xBC */
#define XAGA_AMMS_CMD_INIT	1
#define XAGA_AMMS_CMD_COPY	2
#define XAGA_AMMS_INIT_VER	3
#define XAGA_AMMS_COPY_VER	1
#define XAGA_AMMS_MAX_COPY_LEN	0x10000
#define XAGA_AMMS_MD_NUM	2

#define XAGA_MD1IMG_PATH	"/dev/disk/by-partlabel/md1img_a"
#define XAGA_SEG_MAGIC		0x58881688
#define XAGA_SEG_HDR_LEN	0x200	/* 段头 hdrlen 默认值，数据区在其后 */
#define XAGA_DRDI_SEG_NAME	"md1drdi"
#define XAGA_SCAN_CHUNK		(4 * 1024 * 1024)
#define XAGA_SCAN_BLOCKS	40

struct xaga_amms_req {
	u8 cmd;
	u8 seq_id;
	u8 rsv0[2];
	u8 ver;
	u8 set_total_num;
	u8 rsv1[2];
	u8 tbl[180];
} __packed;

struct xaga_amms_rsp {
	u8 stats;		/* 0 = 成功, 0xFF = 失败 */
	u8 seq_id;		/* = req.seq_id */
	u8 rsv2[2];
	u8 ver;
	u8 copystat;
	u8 drdiinfostat;
	u8 rsv3;
} __packed;

struct xaga_amms_set_init {
	u32 off;
	u32 len;
} __packed;

struct xaga_amms_set_copy {
	u32 src;
	u32 dst;
	u32 len;
} __packed;

static unsigned int xaga_amms_dump_req = 1;
module_param(xaga_amms_dump_req, uint, 0644);
MODULE_PARM_DESC(xaga_amms_dump_req, "XAGA: dump every AMMS DRDI request");

static unsigned int xaga_amms_do_copy = 1;
module_param(xaga_amms_do_copy, uint, 0644);
MODULE_PARM_DESC(xaga_amms_do_copy, "XAGA: perform the AMMS DRDI data copy");

static unsigned int xaga_amms_fail_ok = 0;
module_param(xaga_amms_fail_ok, uint, 0644);
MODULE_PARM_DESC(xaga_amms_fail_ok,
	"XAGA: force error reply (for A/B testing the reply path)");

/* 从 INIT 请求留下的状态（ccci_rpcd 用全局变量保存，COPY 表项数取自它） */
static unsigned int xaga_amms_set_total[XAGA_AMMS_MD_NUM];
static int xaga_amms_copy_done[XAGA_AMMS_MD_NUM];
static unsigned int xaga_amms_req_cnt[XAGA_AMMS_MD_NUM];

/* md1drdi 段数据区（段头 +0x200 起），请求里的 offset/src 以此为基准 */
static void *xaga_drdi_data;
static unsigned int xaga_drdi_len;
static unsigned int xaga_drdi_seg_off;

static u32 xaga_le32(const unsigned char *p)
{
	return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) |
		((u32)p[3] << 24);
}

/* 从 md1img 分区里找出 md1drdi 段并读入内存（insmod 时调用一次） */
static int xaga_drdi_load_image(void)
{
	struct file *f;
	unsigned char *buf;
	loff_t pos;
	unsigned int i, blk, got, seg_len = 0;
	int ret = 0;

	if (xaga_drdi_data)
		return 0;

	f = filp_open(XAGA_MD1IMG_PATH, O_RDONLY | O_LARGEFILE, 0);
	if (IS_ERR(f)) {
		ret = PTR_ERR(f);
		pr_err("XAGA-AMMS: open %s fail %d\n", XAGA_MD1IMG_PATH, ret);
		return ret;
	}
	buf = vmalloc(XAGA_SCAN_CHUNK + XAGA_SEG_HDR_LEN);
	if (!buf) {
		ret = -ENOMEM;
		goto out_close;
	}
	for (blk = 0; blk < XAGA_SCAN_BLOCKS && !seg_len; blk++) {
		pos = (loff_t)blk * XAGA_SCAN_CHUNK;
		got = kernel_read(f, buf, XAGA_SCAN_CHUNK, &pos);
		if ((int)got <= 0)
			break;
		for (i = 0; i + 16 <= got; i += 8) {
			if (xaga_le32(buf + i) != XAGA_SEG_MAGIC)
				continue;
			if (memcmp(buf + i + 8, XAGA_DRDI_SEG_NAME, 8))
				continue;
			seg_len = xaga_le32(buf + i + 4);
			xaga_drdi_seg_off = (unsigned int)pos - got + i;
			break;
		}
	}
	if (!seg_len) {
		pr_err("XAGA-AMMS: %s segment not found\n", XAGA_DRDI_SEG_NAME);
		ret = -ENOENT;
		goto out_free;
	}
	xaga_drdi_data = vmalloc(seg_len);
	if (!xaga_drdi_data) {
		ret = -ENOMEM;
		goto out_free;
	}
	pos = (loff_t)xaga_drdi_seg_off + XAGA_SEG_HDR_LEN;
	got = kernel_read(f, xaga_drdi_data, seg_len, &pos);
	if ((int)got != (int)seg_len) {
		pr_err("XAGA-AMMS: read drdi seg short %u/%u\n", got, seg_len);
		vfree(xaga_drdi_data);
		xaga_drdi_data = NULL;
		ret = -EIO;
		goto out_free;
	}
	xaga_drdi_len = seg_len;
	pr_info("XAGA-AMMS: md1drdi loaded seg_off=0x%x data=0x%x len=0x%x\n",
		xaga_drdi_seg_off, xaga_drdi_seg_off + XAGA_SEG_HDR_LEN,
		xaga_drdi_len);
out_free:
	vfree(buf);
out_close:
	filp_close(f, NULL);
	return ret;
}

/* ===== XAGA: NVRAM cache 共享内存填充 =====
 * Mobian 没有 Android 的 nvram 服务，SMEM_USER_MD_NVRAM_CACHE（AP 视图
 * 0x8a180000，1.5MB）实测全为 0。MODEM 的 RF 校准数据来自 NVRAM，
 * 读到全零就会在 mml1_rf_error_check 断言，所以这里在 MODEM 读它之前
 * （AMMS init 请求时刻）把 nvram 分区内容灌进该共享区。
 * 源、偏移、长度都可配，便于实验。
 */
#define XAGA_NVRAM_SRC_DEFAULT	"/dev/disk/by-partlabel/nvram"

static char *xaga_nvram_src = XAGA_NVRAM_SRC_DEFAULT;
module_param(xaga_nvram_src, charp, 0444);
MODULE_PARM_DESC(xaga_nvram_src, "XAGA: NVRAM source for the MD cache region");

static unsigned int xaga_nvram_skip;
module_param(xaga_nvram_skip, uint, 0444);
MODULE_PARM_DESC(xaga_nvram_skip, "XAGA: source offset");

static unsigned int xaga_nvram_len;
module_param(xaga_nvram_len, uint, 0444);
MODULE_PARM_DESC(xaga_nvram_len, "XAGA: bytes to copy (0 = whole region)");

static unsigned int xaga_nvram_fill = 1;
module_param(xaga_nvram_fill, uint, 0644);
MODULE_PARM_DESC(xaga_nvram_fill, "XAGA: fill NVRAM cache region at AMMS init");

static void *xaga_nvram_data;
static unsigned int xaga_nvram_data_len;
static unsigned int xaga_nvram_done[XAGA_AMMS_MD_NUM];

/* insmod 时把 NVRAM 源读进内存 */
static int xaga_nvram_load(void)
{
	struct file *f;
	loff_t pos;
	unsigned int want = 0x200000;	/* 先读 2MB，够覆盖 1.5MB 的 cache 区 */
	int got;

	if (xaga_nvram_data)
		return 0;
	f = filp_open(xaga_nvram_src, O_RDONLY | O_LARGEFILE, 0);
	if (IS_ERR(f)) {
		pr_err("XAGA-AMMS: open nvram src %s fail %ld\n",
			xaga_nvram_src, PTR_ERR(f));
		return PTR_ERR(f);
	}
	xaga_nvram_data = vmalloc(want);
	if (!xaga_nvram_data) {
		filp_close(f, NULL);
		return -ENOMEM;
	}
	pos = xaga_nvram_skip;
	got = kernel_read(f, xaga_nvram_data, want, &pos);
	filp_close(f, NULL);
	if (got <= 0) {
		pr_err("XAGA-AMMS: read nvram src fail %d\n", got);
		vfree(xaga_nvram_data);
		xaga_nvram_data = NULL;
		return -EIO;
	}
	xaga_nvram_data_len = got;
	pr_info("XAGA-AMMS: nvram src %s skip=0x%x read=0x%x\n",
		xaga_nvram_src, xaga_nvram_skip, xaga_nvram_data_len);
	return 0;
}

/* MODEM 即将读 NVRAM 之前，把数据写进它的 cache 共享区。
 * mark=false 用于 insmod 时先填一次（此时可能还会被 MD 启动流程清掉）。
 */
static void xaga_nvram_fill_cache(int md_id, int mark)
{
	struct ccci_smem_region *r;
	void __iomem *dst;
	bool own = false;
	unsigned int len, i, nz_before = 0, nz_after = 0;
	u8 *rb;

	if (!xaga_nvram_fill)
		return;
	if (mark && xaga_nvram_done[md_id & 1])
		return;
	if (!xaga_nvram_data && xaga_nvram_load())
		return;
	r = ccci_md_get_smem_by_user_id(md_id, SMEM_USER_MD_NVRAM_CACHE);
	if (!r || !r->size) {
		CCCI_ERROR_LOG(md_id, RPC,
			"XAGA-AMMS: no NVRAM cache region (%p)\n", r);
		return;
	}
	len = xaga_nvram_len ? xaga_nvram_len : r->size;
	if (len > r->size)
		len = r->size;
	if (len > xaga_nvram_data_len)
		len = xaga_nvram_data_len;

	dst = r->base_ap_view_vir;
	if (!dst) {
		dst = ioremap_wc(r->base_ap_view_phy, r->size);
		own = true;
	}
	if (!dst) {
		CCCI_ERROR_LOG(md_id, RPC, "XAGA-AMMS: map nvram cache fail\n");
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
			"XAGA-AMMS: NVRAM cache before fill: len=0x%x nonzero=%u first=%*ph\n",
			len, nz_before, 16, rb);
	}
	memcpy_toio(dst, xaga_nvram_data, len);
	if (rb) {
		memcpy_fromio(rb, dst, len);
		for (i = 0; i < len; i++)
			if (rb[i])
				nz_after++;
		CCCI_ERROR_LOG(md_id, RPC,
			"XAGA-AMMS: NVRAM cache filled%s len=0x%x nonzero=%u first=%*ph\n",
			mark ? "" : "(insmod)", len, nz_after, 16, rb);
		vfree(rb);
	}
	if (own)
		iounmap(dst);
	if (mark)
		xaga_nvram_done[md_id & 1] = 1;
}

static void xaga_amms_dump(int md_id, const unsigned char *p,
	unsigned int len, const char *tag)
{
	unsigned int k;

	CCCI_ERROR_LOG(md_id, RPC, "XAGA-AMMS %s len=%u\n", tag, len);
	if (!xaga_amms_dump_req)
		return;
	for (k = 0; k + 16 <= len; k += 16)
		CCCI_ERROR_LOG(md_id, RPC,
			"XAGA-AMMS %s %03u: %08x %08x %08x %08x\n", tag, k,
			*(u32 *)(p + k), *(u32 *)(p + k + 4),
			*(u32 *)(p + k + 8), *(u32 *)(p + k + 12));
	if (k < len) {
		u32 w[4] = {0, 0, 0, 0};
		unsigned int i;

		for (i = 0; i < 4 && (k + i * 4) < len; i++)
			memcpy(&w[i], p + k + i * 4,
				min_t(unsigned int, 4, len - k - i * 4));
		CCCI_ERROR_LOG(md_id, RPC,
			"XAGA-AMMS %s %03u: %08x %08x %08x %08x\n",
			tag, k, w[0], w[1], w[2], w[3]);
	}
}

/* COPY：按 {src,dst,len} 把 md1drdi 数据搬进 64KiB DRDI smem */
static int xaga_amms_copy_sets(int md_id, int slot, struct xaga_amms_req *req)
{
	struct ccci_smem_region *smem;
	struct xaga_amms_set_copy *cs;
	void __iomem *dst_base;
	unsigned int i, num = xaga_amms_set_total[slot];
	int ret = 0;

	if (!num || num > XAGA_AMMS_MAX_SET) {
		CCCI_ERROR_LOG(md_id, RPC,
			"XAGA-AMMS: no valid set_total_num (%u), use req value %u\n",
			num, req->set_total_num);
		num = req->set_total_num;
	}
	smem = ccci_md_get_smem_by_user_id(md_id, SMEM_USER_MD_DRDI);
	if (!smem || !xaga_drdi_data) {
		CCCI_ERROR_LOG(md_id, RPC,
			"XAGA-AMMS: smem=%p drdi=%p, cannot copy\n",
			smem, xaga_drdi_data);
		return -ENODEV;
	}
	dst_base = memremap(smem->base_ap_view_phy, smem->size, MEMREMAP_WB);
	if (!dst_base) {
		CCCI_ERROR_LOG(md_id, RPC, "XAGA-AMMS: memremap smem fail\n");
		return -ENOMEM;
	}
	/* COPY 表从 req+0x08 开始，stride 12：{src, dst, len}
	 * （ccci_rpcd 反汇编：x9=req+0x10 指向 len，src=[x9-8], dst=[x9-4]）
	 */
	cs = (struct xaga_amms_set_copy *)req->tbl;
	for (i = 0; i < num && i < XAGA_AMMS_MAX_SET; i++) {
		u32 src = cs[i].src, dst = cs[i].dst, len = cs[i].len;

		if (!len)
			continue;
		if (len > XAGA_AMMS_MAX_COPY_LEN || src + len > xaga_drdi_len ||
		    dst + len > smem->size) {
			CCCI_ERROR_LOG(md_id, RPC,
				"XAGA-AMMS copy set(%u) bad: src=0x%x dst=0x%x len=0x%x (img 0x%x smem 0x%x)\n",
				i, src, dst, len, xaga_drdi_len, smem->size);
			ret = -ERANGE;
			continue;
		}
		memcpy_toio(dst_base + dst, xaga_drdi_data + src, len);
		CCCI_ERROR_LOG(md_id, RPC,
			"XAGA-AMMS copy set(%u) from 0x%x to smem+0x%x len=0x%x\n",
			i, src, dst, len);
	}
	memunmap(dst_base);
	return ret;
}

/* 处理一条 AMMS DRDI 请求：写 opkt[]，返回参数个数（应答固定 2 个参数） */
/* ===== XAGA-MDLOG-AMMS-BEGIN ===== */
/* XAGA-MDLOG: 任务要求的「首次 AMMS 触发」——AMMS 说明基带已经起来并在通信，
 * 此时顺手把日志使能消息发一次（只发一次；同样受 xaga_mdlog_auto 开关约束）。 */
extern int xaga_mdlog_send_armed(unsigned int msg, unsigned int resv,
	int blocking, const char *why);
extern void xaga_mdlog_kick(void);

static void xaga_mdlog_amms_kick(void)
{
	/* XAGA-MDLOG-EXC: 这里不能发。
	 *
	 * AMMS 请求正好出现在基带启动握手的 HS1/HS2 窗口里（约 7.7-8.1s），
	 * 而厂商 port_proxy.c 在这个窗口明确禁止一切非 FS/RPC 端口流量；实验 M
	 * 证明此时发 0x0C 会让基带固定在 HS1+5.43s 断言（ccismcore_ccci.c:1326）。
	 * 拉日志改由进入 EXCEPTION 时的 xaga_mdlog_kick() 负责。
	 */
}
/* ===== XAGA-MDLOG-AMMS-END ===== */








static int xaga_amms_handle(struct port_t *port, struct rpc_buffer *rpc_buf,
	struct rpc_pkt *pkt, int pkt_num, struct rpc_pkt *opkt, u32 *tmp_data)
{
	int md_id = port->md_id;
	int slot = md_id & 1;
	struct xaga_amms_req *req;
	struct xaga_amms_rsp *rsp;
	struct xaga_amms_set_init *is;
	u32 *ret_code = &tmp_data[0];
	int i, copy_ret = 0;
	bool fail = false;

	xaga_mdlog_amms_kick();

	if (pkt_num < 1 || pkt[0].len < XAGA_AMMS_REQ_SIZE) {
		CCCI_ERROR_LOG(md_id, RPC,
			"XAGA-AMMS bad request pkt_num=%d len=%u (need %u)\n",
			pkt_num, pkt_num > 0 ? pkt[0].len : 0,
			XAGA_AMMS_REQ_SIZE);
		tmp_data[0] = FS_PARAM_ERROR;
		opkt[0].len = sizeof(u32);
		opkt[0].buf = (void *)&tmp_data[0];
		return 1;
	}
	req = (struct xaga_amms_req *)pkt[0].buf;
	rsp = (struct xaga_amms_rsp *)&tmp_data[1];
	memset(rsp, 0, sizeof(*rsp));
	rsp->seq_id = req->seq_id;
	*ret_code = 0;
	xaga_amms_req_cnt[slot]++;
	/* 与 ccci_rpcd 一致：清掉 data[0] 的"还有分片"位 */
	rpc_buf->header.data[0] &= ~0x80000000U;

	xaga_amms_dump(md_id, (const unsigned char *)req, pkt[0].len,
		(req->cmd == XAGA_AMMS_CMD_INIT) ? "init" : "copy");
	CCCI_ERROR_LOG(md_id, RPC,
		"XAGA-AMMS cmd=%u seq=%u ver=%u set_total=%u cnt=%u\n",
		req->cmd, req->seq_id, req->ver, req->set_total_num,
		xaga_amms_req_cnt[slot]);

	switch (req->cmd) {
	case XAGA_AMMS_CMD_INIT:
		/*
		 * XAGA: CCCI 内建后 port_rpc_init 在 ~2.7s 就返回了，那一刻
		 * /dev/disk/by-partlabel 尚未建立（udev 还没跑），
		 * xaga_drdi_load_image() 打不开 md1img 分区，xaga_drdi_data
		 * 恒为 NULL，AMMS init 只能回 0xFFFFFFFF。这里改成与 NVRAM
		 * 缓存同样的懒加载：真正要用的时候再读一次。
		 */
		if (!xaga_drdi_data)
			xaga_drdi_load_image();
		/* MODEM 马上要读 RF/NVRAM 数据了：先把 NVRAM cache 区灌好 */
		xaga_nvram_fill_cache(md_id, 1);
		xaga_amms_set_total[slot] = req->set_total_num;
		is = (struct xaga_amms_set_init *)req->tbl;
		for (i = 0; i < XAGA_AMMS_MAX_SET; i++)
			if (is[i].len)
				CCCI_ERROR_LOG(md_id, RPC,
					"XAGA-AMMS set[%d] off=0x%x len=0x%x\n",
					i, is[i].off, is[i].len);
		if (req->ver != XAGA_AMMS_INIT_VER) {
			CCCI_ERROR_LOG(md_id, RPC,
				"XAGA-AMMS init version(%u) error\n", req->ver);
			fail = true;
		} else if (req->set_total_num > XAGA_AMMS_MAX_SET) {
			CCCI_ERROR_LOG(md_id, RPC,
				"XAGA-AMMS init set_total_num(%u) error\n",
				req->set_total_num);
			fail = true;
		} else if (!xaga_drdi_data) {
			CCCI_ERROR_LOG(md_id, RPC,
				"XAGA-AMMS init: no drdi image\n");
			fail = true;
		} else {
			for (i = 0; i < req->set_total_num; i++) {
				if (is[i].off + is[i].len > xaga_drdi_len) {
					CCCI_ERROR_LOG(md_id, RPC,
						"XAGA-AMMS init set[%d] out of range off=0x%x len=0x%x (img 0x%x)\n",
						i, is[i].off, is[i].len,
						xaga_drdi_len);
					fail = true;
					break;
				}
			}
		}
		rsp->ver = XAGA_AMMS_INIT_VER;
		if (fail) {
			rsp->stats = 0xFF;
			rsp->drdiinfostat = 0xFF;
			*ret_code = 0xFFFFFFFF;
		} else {
			rsp->stats = 0;
			rsp->copystat =
				(xaga_amms_copy_done[slot] == 1) ? 0xFF : 0;
			rsp->drdiinfostat = 0;
		}
		break;

	case XAGA_AMMS_CMD_COPY:
		xaga_amms_copy_done[slot] = (req->ver == XAGA_AMMS_COPY_VER);
		rsp->ver = (req->ver == XAGA_AMMS_COPY_VER) ? 0 : 0xFF;
		if (xaga_amms_do_copy)
			copy_ret = xaga_amms_copy_sets(md_id, slot, req);
		/* 完全照 rpcd 行为：copy_done 不为 -1 时 stats=0，ret=0 */
		rsp->stats = 0;
		*ret_code = 0;
		if (copy_ret)
			CCCI_ERROR_LOG(md_id, RPC,
				"XAGA-AMMS copy error %d (still reply success like rpcd)\n",
				copy_ret);
		break;

	default:
		CCCI_ERROR_LOG(md_id, RPC, "XAGA-AMMS unknown cmd %u\n",
			req->cmd);
		rsp->stats = 0xFF;
		*ret_code = 0xFFFFFFFF;
		break;
	}

	if (xaga_amms_fail_ok) {
		*ret_code = 0xFFFFFFFF;
		rsp->stats = 0xFF;
	}
	CCCI_ERROR_LOG(md_id, RPC,
		"XAGA-AMMS reply ret=0x%x stats=0x%x seq=%u ver=%u copystat=0x%x drdiinfo=0x%x\n",
		*ret_code, rsp->stats, rsp->seq_id, rsp->ver, rsp->copystat,
		rsp->drdiinfostat);

	opkt[0].len = sizeof(u32);
	opkt[0].buf = (void *)ret_code;
	opkt[1].len = sizeof(struct xaga_amms_rsp);
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
		/* XAGA: 内核侧实现 Android ccci_rpcd 的 AMMS DRDI 应答 */
		{
			struct rpc_pkt opkt[RPC_MAX_ARG_NUM];
			int xa_i;

			pkt_num = xaga_amms_handle(port, p_rpc_buf, pkt, pkt_num,
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
	/* switch to Tx request */
	ret = port_send_skb_to_md(port, skb, 1);
	if (ret)
		goto err_out;
	kfree(tmp_data);
	return;

 err_out:
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
		xaga_drdi_load_image();	/* XAGA: AMMS DRDI copy 的数据源 */
		first_init = 0;
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
			is_userspace_msg = 1;
			break;
		case IPC_RPC_AMMS_DRDI_CONTROL:
			/* XAGA: no ccci_rpcd daemon on Mobian; keep DRDI
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

