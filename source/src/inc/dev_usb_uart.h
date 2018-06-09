/*
 * dev_usb_uart.h
 *
 *  Created on: 9 May 2018
 *      Author: dimtass
 */

#ifndef DEV_USB_UART_H_
#define DEV_USB_UART_H_

#include <stdint.h>
#include "stm32f10x.h"
#include "platform_config.h"
#include "usb_regs.h"
#include "usb_mem.h"
#include "list.h"
#include "comm_buffer.h"

//#pragma pack(1)
struct dev_usb_uart {
	uint8_t		irq_rx;
	uint8_t 	EP_in;
	uint16_t	EP_in_address;
	uint8_t 	EP_out;
	uint16_t	EP_out_address;
	uint16_t	usb_endpoint_data_size;	// This is hardware specific (usually 64 bytes)
	uint16_t	usb_rx_polling_ms;
	volatile struct tp_comm_buffer* buffer;
	void (*fp_recv_parser)(uint8_t * recv_buffer, uint16_t recv_len);
	struct list_head list;	// support multiple USB uart devices
};

#define USB_REMAIN_TX_LENGTH(USB_DEV) (USB_DEV->buffer->tx_ptr_in >= USB_DEV->buffer->tx_ptr_out)? \
							USB_DEV->buffer->tx_ptr_in - USB_DEV->buffer->tx_ptr_out : \
							USB_DEV->buffer->tx_buffer_size - (USB_DEV->buffer->tx_ptr_out-USB_DEV->buffer->tx_ptr_in)

/* Static declaration of a UART device */
#define DECLARE_USB_UART_DEV(NAME, EP_IN, EP_IN_ADDRESS, EP_OUT, EP_OUT_ADDRESS, VIRTUAL_PORT_DATA_SIZE, RX_POLLING_MS, PARSER_FUNCTION, COMM_BUFFER) \
	struct dev_usb_uart NAME = { \
		.irq_rx = 0, \
		.EP_in = EP_IN, \
		.EP_in_address = EP_IN_ADDRESS, \
		.EP_out = EP_OUT, \
		.EP_out_address = EP_OUT_ADDRESS, \
		.usb_endpoint_data_size = VIRTUAL_PORT_DATA_SIZE, \
		.usb_rx_polling_ms = RX_POLLING_MS, \
		.fp_recv_parser = PARSER_FUNCTION, \
		.buffer = COMM_BUFFER \
	}


/**
 * @brief This function initializes the dev_usb_uart device. You must
 * always call this function after you create a new dev with either
 * DECLARE_USB_UART_DEV or manually.
 * @param[in] dev A pointer to the dev_usb_uart struct
 */
void USB_dev_init(struct dev_usb_uart * dev);


/**
 * @brief This sends data back to the USB CDC
 * @param[in] dev A pointer to the dev_usb_uart struct
 * @param[in] ptrBuffer A pointer to the Tx buffer
 * @param[in] bufflen The size of the buffer to send
 */
uint32_t USB_dev_send(struct dev_usb_uart *dev, const uint8_t *ptrBuffer, uint16_t bufflen);

/**
 * @brief This is a polling function that reads the received data from
 * the endpoint buffer. You can either use a timer or add this to your
 * main loop.
 */
uint32_t USB_dev_recv(void);

/**
 * @brief This is the Rx interrupt function. Normally you should add
 * this to the proper EPx_OUT_Callback function, but you cal also use
 * the dev.irq_rx as a flag in the real IRQ and handle this to the
 * main loop.
 * @param[in] dev A pointer to the dev_usb_uart struct
 */
void USB_Rx_IRQ(struct dev_usb_uart * dev);

/**
 * @brief This is the interrupt handler for the Tx. Add this to the
 * proper EPx_IN_Callback callback.
 * @param[in] dev A pointer to the dev_usb_uart struct
 */
void USB_Tx_IRQ(struct dev_usb_uart * dev);

/**
 * @brief This function updates the timers that are used for Rx. When
 * these timers are expired (see: dev.usb_rx_polling_ms) then the
 * fp_recv_parser() function is called. Add this to your 1ms IRQ
 */
void USB_update_timers(void);


#endif /* DEV_USB_UART_H_ */
