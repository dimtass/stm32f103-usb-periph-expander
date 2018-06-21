/*
 * platform_config.h
 *
 *  Created on: 8 May 2018
 *      Author: Dimitris Tassopoulos
 */

#ifndef __PLATFORM_CONFIG_H
#define __PLATFORM_CONFIG_H

#include <stdint.h>
#include <stdio.h>
#include "stm32f10x.h"

/**
 * Trace levels for this project.
 * Have in mind that these are bit flags!
 */
typedef enum {
	TRACE_LEVEL_DEFAULT = 	(1 << 0),
	TRACE_LEVEL_USB = 		(1 << 1),
	TRACE_LEVEL_GPIO = 		(1 << 2),
	TRACE_LEVEL_ADC = 		(1 << 3),
	TRACE_LEVEL_I2C = 		(1 << 4),
	TRACE_LEVEL_SPI = 		(1 << 5),
	TRACE_LEVEL_USART = 	(1 << 6),
	TRACE_LEVEL_COMM = 		(1 << 7),
	TRACE_LEVEL_PARSER = 	(1 << 8),
} en_trace_level;

#define DEBUG_TRACE

#ifdef DEBUG_TRACE
#define TRACE(X) TRACEL(TRACE_LEVEL_DEFAULT, X)
#define TRACEL(TRACE_LEVEL, X) do { if (glb.trace_levels & TRACE_LEVEL) printf X;} while(0)
#else
#define TRACE(X)
#define TRACEL(X,Y)
#endif

/* LED patterns */
enum {
	LED_PATTERN_IDLE = 0b00001111,
};

struct tp_glb {
	volatile uint16_t tmr_1ms;
	en_trace_level trace_levels;
};

extern struct tp_glb glb;

static inline void set_trace_level(en_trace_level level, uint8_t enable)
{
	if (enable) {
		glb.trace_levels |= (uint32_t) level;
	}
	else {
		glb.trace_levels &= ~((uint32_t) level);
	}
}

#define PIN_STATUS_LED	GPIO_Pin_13
#define PORT_STATUS_LED GPIOC

#endif /* __PLATFORM_CONFIG_H */
