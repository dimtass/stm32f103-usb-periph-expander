/*
 * dev_usb_uart.c
 *
 *  Created on: 10 May 2018
 *      Author: dimtass
 */
#include "dev_usb_uart.h"

static LIST_HEAD(usb_uart_list);

void USB_dev_init(struct dev_usb_uart * dev)
{
	/* init dev head list */
	INIT_LIST_HEAD(&dev->list);
	/* Add to led_list */
	list_add(&dev->list, &usb_uart_list);

	/* Initialize buffer */
	dev->buffer->rx_length = dev->buffer->rx_ptr_in = dev->buffer->rx_ptr_out = 0;
	dev->buffer->rx_ready = dev->buffer->rx_ready_tmr = 0;
	dev->buffer->tx_length = dev->buffer->tx_ptr_in = dev->buffer->tx_ptr_out = 0;
	dev->buffer->tx_ready = 1;
}

void USB_Rx_IRQ(struct dev_usb_uart * dev)
{
	volatile uint16_t * p_out = &dev->buffer->rx_ptr_out;
	volatile uint16_t * p_in = &dev->buffer->rx_ptr_in;
	uint16_t rx_count = GetEPRxCount(dev->EP_out);

	PMAToUserBufferCopy((unsigned char*) &dev->buffer->rx_buffer[*p_in],
			dev->EP_out_address, rx_count);
	*p_in += rx_count;
	dev->buffer->rx_length = *p_in - *p_out;
	dev->buffer->rx_ready = 1;
	dev->buffer->rx_ready_tmr = 0;
}

/**
*
*/
void USB_Tx_IRQ(struct dev_usb_uart * dev)
{
	volatile uint16_t * p_out = &dev->buffer->tx_ptr_out;
	volatile uint16_t * p_in = &dev->buffer->tx_ptr_in;

	if (*p_out != *p_in) {
		uint16_t remain_len = USB_REMAIN_TX_LENGTH(dev);
		uint8_t send_len = 0;

		if (remain_len < dev->usb_endpoint_data_size)
			send_len = remain_len;
		else
			send_len = dev->usb_endpoint_data_size;

		/* send  packet to PMA*/
		UserToPMABufferCopy((unsigned char*) &dev->buffer->tx_buffer[*p_out],
				dev->EP_in_address,
				send_len);
		SetEPTxCount(dev->EP_in, send_len);
		SetEPTxValid(dev->EP_in);

		*p_out = (*p_out + send_len)%(dev->buffer->tx_buffer_size - 1);
	}
	else {
		dev->buffer->tx_ready = 1;
		*p_out = 0;
		*p_in = 0;
		dev->buffer->tx_length = 0;
	}
}

/*******************************************************************************
 * Function Name  : Send DATA .
 * Description    : send the data received from the STM32 to the PC through USB
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
uint32_t USB_dev_send(struct dev_usb_uart *dev, const uint8_t *ptrBuffer, uint16_t bufflen)
{
	uint16_t remain_len = USB_REMAIN_TX_LENGTH(dev);
	uint16_t remain_buffer_size = dev->buffer->tx_buffer_size - remain_len;
	volatile uint16_t *p_in = &dev->buffer->tx_ptr_in;

	/* Check for buffer overflow */
	if (remain_buffer_size < bufflen) {
		TRACE(("\nOVF\n"));
		return 0;
	}

	int i = 0;
	/* copy buffer */
	for (i=0; i<bufflen; i++) {
		dev->buffer->tx_buffer[*p_in] = ptrBuffer[i];
		*p_in = (*p_in + 1)%(dev->buffer->tx_buffer_size - 1);
	}
	USB_Tx_IRQ(dev);

	return 1;
}

/*******************************************************************************
 * Function Name  : Receive DATA .
 * Description    : receive the data from the PC to STM32 and send it through USB
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
uint32_t USB_dev_recv(void)
{
	if (!list_empty(&usb_uart_list)) {
		struct dev_usb_uart * dev;
		list_for_each_entry(dev, &usb_uart_list, list) {
			SetEPRxValid(dev->EP_out);
			/* check timeout and call the parser function */
			if (dev->buffer->rx_ready_tmr >= dev->usb_rx_polling_ms) {
				dev->buffer->rx_ready = 0;
				dev->buffer->rx_ready_tmr = 0;
				if ((dev->fp_recv_parser != NULL) && dev->buffer->rx_ptr_in)
					dev->fp_recv_parser(dev->buffer->rx_buffer, dev->buffer->rx_ptr_in);
				dev->buffer->rx_ptr_in = 0;
				dev->buffer->rx_ptr_out = 0;
				dev->buffer->rx_length = 0;
			}

			if (dev->irq_rx) {
				USB_Rx_IRQ(dev);
				dev->irq_rx = 0;
			}
		}
	}

	return 1;
}


void USB_update_timers(void)
{
	if (!list_empty(&usb_uart_list)) {
		struct dev_usb_uart * dev;
		list_for_each_entry(dev, &usb_uart_list, list) {
			if (dev->buffer->rx_ready) dev->buffer->rx_ready_tmr++;
		}
	}
}
