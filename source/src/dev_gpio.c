/*
 * dev_gpio.c
 *
 *  Created on: 10 May 2018
 *      Author: dimtass
 */

#include "dev_gpio.h"

#define PIN_EXISTS(PIN, ITTERATOR) ( (PIN->port == ITTERATOR->port) && (PIN->pin == ITTERATOR->pin) )

/**
 * @brief This function initializes the GPIO subsystem.
 * This should only be called once in the main().
 * @param[in] dev A pointer to a dev_gpio
 */
void dev_gpio_module_init(struct dev_gpio_module * dev)
{
	/* init dev head list */
	INIT_LIST_HEAD(&dev->gpio_list);
	dev->num_of_gpios = 0;
}


static inline struct dev_gpio * dev_gpio_find_gpio(struct dev_gpio * gpio)
{
	struct dev_gpio_module * dev = gpio->owner;

	if (!dev || !gpio) return NULL;

	if (!list_empty(&dev->gpio_list)) {
		struct dev_gpio * gpio_it = NULL;
		list_for_each_entry(gpio_it, &dev->gpio_list, list) {
			if PIN_EXISTS(gpio, gpio_it) {
				/* found */
				return(gpio_it);
			}
		}
	}
	return NULL;
}


void* dev_gpio_probe(struct dev_gpio * gpio)
{
	/* do not allow duplicates */
	if (dev_gpio_find_gpio(gpio)) {
		TRACEL(TRACE_LEVEL_GPIO,("GPIO-%c.%d already exists\n",
				port_to_char(gpio->port), gpio->pin));
		return NULL;
	}

	struct dev_gpio_module * dev = gpio->owner;

	struct dev_gpio * new_gpio = (struct dev_gpio *) malloc(sizeof(struct dev_gpio));
	memcpy(new_gpio, gpio, sizeof(struct dev_gpio));

	/* init pin head list */
	INIT_LIST_HEAD(&new_gpio->list);
	/* Add to pin to gpio list */
	list_add(&new_gpio->list, &dev->gpio_list);

	dev->num_of_gpios++;

	dev_gpio_set_dir(new_gpio, new_gpio->dir);

	if (new_gpio->port == GPIOA) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	}
	else if (new_gpio->port == GPIOB) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	}
	else if (new_gpio->port == GPIOC) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	}
	else if (new_gpio->port == GPIOD) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	}
	else if (new_gpio->port == GPIOE) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	}
	else if (new_gpio->port == GPIOF) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
	}
	else if (new_gpio->port == GPIOG) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
	}

	if (new_gpio->dir == GPIO_DIR_IN) {
		new_gpio->value = dev_gpio_get_value(new_gpio);
	}
	else {
		new_gpio->value = dev_gpio_get_value(new_gpio);
		dev_gpio_set_value(new_gpio, new_gpio->new_value);
	}
	TRACEL(TRACE_LEVEL_GPIO,("Init GPIO-%c.%d, %d->%d\n", port_to_char(new_gpio->port), new_gpio->pin, new_gpio->value, new_gpio->new_value));

	return new_gpio;
}


void dev_gpio_remove(struct dev_gpio * gpio)
{
	struct dev_gpio * found_gpio = dev_gpio_find_gpio(gpio);
	if (found_gpio) {
		/* remove */
		gpio->owner->num_of_gpios--;
		TRACEL(TRACE_LEVEL_GPIO,("GPIO: remaining %d gpios\n", gpio->owner->num_of_gpios));
		list_del(&found_gpio->list);
		free(found_gpio);
	}
}


void dev_gpio_set_dir(struct dev_gpio * gpio, enum en_gpio_direction dir)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	struct dev_gpio * found_gpio = dev_gpio_find_gpio(gpio);
	if (found_gpio) {
		found_gpio->dir = gpio->dir;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

		GPIO_InitStructure.GPIO_Pin = (1 << found_gpio->pin);
		if (found_gpio->dir == GPIO_DIR_OUT)
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		else
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;

		GPIO_Init(gpio->port, &GPIO_InitStructure);
	}
}


void dev_gpio_set_value(struct dev_gpio * gpio, uint8_t value)
{
	struct dev_gpio * found_gpio = dev_gpio_find_gpio(gpio);
	if (found_gpio) {
		found_gpio->new_value = value;
	}
}

static inline void dev_gpio_set_real_value(struct dev_gpio * gpio, uint8_t value)
{
	struct dev_gpio * found_gpio = dev_gpio_find_gpio(gpio);
	if (found_gpio) {
		if (value)
			found_gpio->port->BSRR = (1 << found_gpio->pin);
		else
			found_gpio->port->BRR = (1 << found_gpio->pin);
		found_gpio->value = found_gpio->new_value;
	}
}


int dev_gpio_get_value(struct dev_gpio * gpio)
{
	struct dev_gpio * found_gpio = dev_gpio_find_gpio(gpio);
	if (found_gpio) {
		return !!(found_gpio->port->ODR & (1 << found_gpio->pin));
	}
	return -1;
}


void dev_gpio_update(struct dev_gpio_module * dev)
{
	if (!list_empty(&dev->gpio_list)) {
		struct dev_gpio * pin_it;
		list_for_each_entry(pin_it, &dev->gpio_list, list) {
			if (pin_it->new_value != pin_it->value) {
				TRACEL(TRACE_LEVEL_GPIO, ("GPIO.%d=>%d,", pin_it->pin, pin_it->new_value));
				dev_gpio_set_real_value(pin_it, pin_it->new_value);
				if (dev->evaluate_on_every_pin_set)
					pin_it->value = dev_gpio_get_value(pin_it);
				else
					pin_it->value = pin_it->new_value;
				TRACEL(TRACE_LEVEL_GPIO, ("curr:%d\n", pin_it->value));
			}
		}
	}
}
