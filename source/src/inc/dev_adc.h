/*
 * dev_adc.h
 *
 *  Created on: 10 May 2018
 *      Author: dimtass
 */

#ifndef DEV_ADC_H_
#define DEV_ADC_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f10x.h"
#include "platform_config.h"
#include "list.h"

enum en_adc_mode {
	DEV_ADC_MODE_SINGLE,
	DEV_ADC_MODE_POLLING
};

#define DECLARE_ADC_CH(OWNER, CHANNEL, ENABLE) { OWNER, CHANNEL, ENABLE, 0, 0 }

#define DECLARE_DEV_ADC(OWNER, NAME, CHANNEL, ENABLE) \
	struct dev_adc NAME = { \
		.owner = OWNER, \
		.channel = CHANNEL, \
		.enable = ENABLE, \
		.ready = 0, \
		.value = 0, \
	}

struct dev_adc {
	struct dev_adc_module * owner;
	uint8_t	channel;
	uint8_t enable;
	uint8_t ready;
	volatile uint16_t value;
	struct list_head list;
};


#define DECLARE_MODULE_ADC(NAME, ADC, MODE, TICK_MS) \
	struct dev_adc_module NAME = { \
		.adc = ADC, \
		.mode = MODE, \
		.tick_ms = TICK_MS, \
		.curr_channel = NULL, \
	}

struct dev_adc_module {
	ADC_TypeDef * adc;
	enum en_adc_mode	mode;
	uint16_t	tick_ms;
	struct dev_adc * curr_channel;
	struct list_head adc_ch_list;
};

void dev_adc_module_init(struct dev_adc_module * adc);
void dev_adc_start(struct dev_adc_module * dev);
void* dev_adc_probe(struct dev_adc * adc_dev_arr);
void dev_adc_remove(struct dev_adc * adc_dev_arr);
uint8_t dev_adc_set_channel(struct dev_adc_module * dev, uint8_t ch);
uint8_t dev_adc_enable_channel(struct dev_adc_module * dev, uint8_t ch, uint8_t enable);
uint8_t dev_adc_reset_channels(struct dev_adc_module * dev);
void dev_adc_update(struct dev_adc_module * dev);


#endif /* DEV_ADC_H_ */
