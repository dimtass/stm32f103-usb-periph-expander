/*
 * dev_gpio.h
 *
 *  Created on: 10 May 2018
 *      Author: dimtass
 */

#ifndef DEV_GPIO_H_
#define DEV_GPIO_H_

#include <dev_uart.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f10x.h"
#include "list.h"
#include "platform_config.h"
#include "common_tools.h"

//#define DEV_GPIO_EVALUATE_EVERY_SET

enum en_gpio_direction {
	GPIO_DIR_IN,
	GPIO_DIR_OUT
};

#define DECLARE_GPIO_PIN(OWNER, PORT, PIN, DIR, NEW_VALUE) { OWNER, PORT, PIN, DIR, NEW_VALUE }

/**
 * @brief This is a fast declaration to be used in external code when
 * need to run functions like gpio_pin_add(), gpio_pin_del() e.t.c.
 */
#define DECLARE_GPIO_TMP_PIN(NAME, PORT, PIN) \
	struct dev_gpio_pin NAME = { \
		.port = PORT, \
		.pin = PIN, \
	}

#define DECLARE_DEV_GPIO(PORT, PIN, DIR, VALUE, OWNER) \
	struct dev_gpio NAME = { \
		.owner = OWNER, \
		.port = PORT, \
		.pin = PIN, \
		.dir = DIR, \
		.new_value = 0, \
		.value = 0, \
	}

struct dev_gpio {
	struct dev_gpio_module * owner;
	GPIO_TypeDef * 	port;
	uint16_t		pin;
	enum en_gpio_direction dir;
	uint8_t			new_value;
	uint8_t			value;
	struct list_head list;
};


#define DECLARE_MODULE_GPIO(NAME, TICK_MS, EVAL_ON_EVERY_SET) \
	struct dev_gpio_module NAME = { \
		.tick_ms = TICK_MS, \
		.evaluate_on_every_pin_set = EVAL_ON_EVERY_SET, \
		.num_of_gpios = 0, \
	}

struct dev_gpio_module {
	uint16_t	tick_ms;
	uint8_t 	evaluate_on_every_pin_set;
	uint8_t		num_of_gpios;
	struct list_head gpio_list;	// support multiple GPIOs
};

void dev_gpio_module_init(struct dev_gpio_module * dev);

void* dev_gpio_probe(struct dev_gpio * gpio);

void dev_gpio_remove(struct dev_gpio * gpio);

void dev_gpio_set_dir(struct dev_gpio * gpio, enum en_gpio_direction dir);

void dev_gpio_set_value(struct dev_gpio * gpio, uint8_t value);

int dev_gpio_get_value(struct dev_gpio * gpio);

void dev_gpio_update(struct dev_gpio_module * dev);

#endif // DEV_GPIO_H_
