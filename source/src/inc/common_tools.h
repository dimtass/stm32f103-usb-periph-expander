/*
 * common_tools.h
 *
 *  Created on: 13 May 2018
 *      Author: dimtass
 */

#ifndef COMMON_TOOLS_H_
#define COMMON_TOOLS_H_

#include <stdint.h>
#include <stdio.h>
#include "stm32f10x.h"
#include "platform_config.h"
#include "list.h"

//#define offsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)
#define container_of(ptr, type, member) ({ \
	const typeof(((type *)0)->member) * __mptr = (ptr); \
	(type *)((char *)__mptr - offsetof(type, member)); \
	})


uint8_t usart_to_uint(USART_TypeDef * usart);
USART_TypeDef* uint_to_usart(uint8_t port);

char port_to_char(GPIO_TypeDef * port);
GPIO_TypeDef * char_to_port(char cport);

uint8_t pin_to_uint(uint16_t pin);

#endif /* COMMON_TOOLS_H_ */
