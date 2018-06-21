/*
 * dev_uart.h
 *
 *  Created on: 8 May 2018
 *      Author: dimtass
 *
 * Add this line to the interrupt file (e.g. stm32f10x_it.c)
 *		extern struct debug_uart_dev uart_dev;
 *
 * Add this line to the IRQ function (e.g. USART1_IRQHandler):
 * 		dev_uart_irq(&uart_dev);
 *
 * Add this line to the __io_putchar(int ch) function (used for printf)
 * 		dev_uart_send(&uart_dev, ch);
 *
 * Initialize the device in the main.c
 * 		DECLARE_UART_BUFFER(uart_buffer, 64, 16);
 * 		DECLARE_DEBUG_UART_DEV(uart_dev, USART1, 115200, TRACE_LEVEL_DEFAULT, NULL, NULL);
 *		uart_dev.uart_buff = &uart_buffer;
 *		dev_uart_init(&uart_dev);
 */

#ifndef DEV_UART_H_
#define DEV_UART_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f10x.h"
#include "list.h"
#include "comm_buffer.h"

#define DECLARE_MODULE_UART(NAME, TICK_MS) \
	struct dev_uart_module NAME = { \
		.tick_ms = TICK_MS, \
	}

struct dev_uart_module {
	uint16_t	tick_ms;
	struct list_head uart_list;
};

#define DECLARE_UART_DEV(NAME, OWNER, PORT, BAUDRATE, BUFFER_SIZE, TIMEOUT_MS, DEBUG, CALLBACK) \
	struct dev_uart NAME = { \
		.owner = OWNER, \
		.port = PORT, \
		.config = { \
			.USART_BaudRate = BAUDRATE, \
			.USART_WordLength = USART_WordLength_8b, \
			.USART_StopBits = USART_StopBits_1, \
			.USART_Parity = USART_Parity_No, \
			.USART_Mode = USART_Mode_Rx | USART_Mode_Tx, \
			.USART_HardwareFlowControl = USART_HardwareFlowControl_None, \
		}, \
		.uart_buff = { \
			.rx_buffer_size = BUFFER_SIZE, \
			.tx_buffer_size = BUFFER_SIZE, \
		}, \
		.timeout_ms = TIMEOUT_MS, \
		.debug = DEBUG, \
		.fp_dev_uart_cb = CALLBACK, \
	}

/**
* @brief Debug UART structure
*/
struct dev_uart {
	struct dev_uart_module * owner;
	USART_TypeDef*		port;
	USART_InitTypeDef	config;
	NVIC_InitTypeDef	nvic;
	uint8_t				debug;
	uint8_t				timeout_ms;
	uint8_t				available;
	volatile struct tp_comm_buffer uart_buff;
	/**
	* @brief Callback function definition for reception
	* @param[in] buffer Pointer to the RX buffer
	* @param[in] bufferlen The length of the received data
	*/
	void (*fp_dev_uart_cb)(struct dev_uart * uart, uint8_t *buffer, size_t bufferlen);
	struct list_head list;
};



/**
 * @brief Initialize the debugging UART interface
 * @param[in] dev_uart A pointer to the UART device
 */
void dev_uart_module_init(struct dev_uart_module * dev);

/**
 * @brief Configure the STM32 uart port (including the port pins)
 * @param[in] dev_uart A pointer to the UART device
 */
void* dev_uart_probe(struct dev_uart * dev);

/**
 * @brief Remove the STM32 uart port (including the port pins)
 * @param[in] dev_uart A pointer to the UART device
 */
void dev_uart_remove(struct dev_uart * dev);

/**
 * @brief IRQ handler for the debug interface
 * @param[in] dev_uart A pointer to the UART device
 */
void dev_uart_irq(struct dev_uart * dev);

/**
 * @brief Send a single byte on the debug uart. This should be used with the
 * 		syscalls.c file to implement the printf() function
 * @param[in] dev_uart A pointer to the UART device
 * @param[in] ch The byte to send
 * @return int The byte sent
 */
int dev_uart_send_ch(struct dev_uart * dev, int ch);

size_t dev_uart_send_buffer(struct dev_uart * dev, uint8_t * buffer, size_t buffer_len);

int dev_uart_receive(struct dev_uart * dev);


void dev_uart_set_baud_rate(struct dev_uart * dev, uint32_t baudrate);


/**
 * @brief Poll the RX buffer for new data. If new data are found then
 * 		the fp_debug_uart_cb will be called.
 * @param[in] dev_uart A pointer to the UART device
 */
void dev_uart_update(struct dev_uart_module * dev);


#endif /* DEV_UART_H_ */
