/*
 * common_tools.c
 *
 *  Created on: 25 May 2018
 *      Author: Dimitris Tassopoulos
 */

#include "common_tools.h"

uint8_t usart_to_uint(USART_TypeDef * usart)
{
	uint8_t port = 0;
	if (usart == USART1) port = 1;
	else if (usart == USART2) port = 2;
	return port;
}

USART_TypeDef* uint_to_usart(uint8_t port)
{
	USART_TypeDef* usart = NULL;
	if (port == 1) usart = USART1;
	else if (port == 2) usart = USART2;
	return usart;
}

char port_to_char(GPIO_TypeDef * port)
{
	char ch = '?';
	if (port == GPIOA) ch = 'A';
	else if (port == GPIOB) ch = 'B';
	else if (port == GPIOC) ch = 'C';
	else if (port == GPIOD) ch = 'D';
	else if (port == GPIOE) ch = 'E';
	else if (port == GPIOF) ch = 'F';
	else if (port == GPIOG) ch = 'G';
	return ch;
}

GPIO_TypeDef * char_to_port(char cport)
{
	GPIO_TypeDef *port = NULL;

	switch (cport) {
	case 'A':
		port = GPIOA;
		break;
	case 'B':
		port = GPIOB;
		break;
	case 'C':
		port = GPIOC;
		break;
	case 'D':
		port = GPIOD;
		break;
	case 'E':
		port = GPIOE;
		break;
	case 'F':
		port = GPIOF;
		break;
	case 'G':
		port = GPIOG;
		break;
	default:
		port = NULL;
		break;
	}
	return port;
}

uint8_t pin_to_uint(uint16_t pin)
{
	uint8_t upin = 0;

	if (pin) {
		while(pin) {
			upin++;
			pin >>= 1;
		};
	}
	return upin-1;
}
