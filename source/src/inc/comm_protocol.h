/*
 * comm_protocol.h
 *
 *  Created on: 21 May 2018
 *      Author: dimtass
 */

#ifndef COMM_PROTOCOL_H_
#define COMM_PROTOCOL_H_

#include <stdint.h>
#include <stdlib.h>
#include "platform_config.h"
#include "crc16.h"

enum {
	COMM_PREAMBLE = 0xDA7A,
	COMM_CRC_POLY = 0xFFFF,
	NAME_STR_LENGTH = 20,
};

/* Errors */
enum en_comm_errors {
    COMM_RET_OK = 0,
    COMM_RET_ERR_INV_PREAMBLE,
    COMM_RET_ERR_INV_LENGTH,
    COMM_RET_ERR_INV_CRC16,
	COMM_RET_ERR_VERIF,
	COMM_RET_ERR_END,
};

enum en_device_type {
	DEV_TYPE_LED,
	DEV_TYPE_GPIO,
	DEV_TYPE_ADC,
	DEV_TYPE_UART,
	DEV_TYPE_SPI,
	DEV_TYPE_I2C,
	DEV_TYPE_USB_UART,
	DEV_TYPE_TIMER,
	/* To refer to the module of one if the
	 * above devices then must OR with the
	 * DEV_TYPE_MODULE. E.g. to refer to the
	 * SPI module then you need to:
	 * (DEV_TYPE_MODULE | DEV_TYPE_SPI)
	 */
	DEV_TYPE_MODULE = 100,
};

enum en_comm_cmd {
	COMM_CMD_SET_CONF,
	COMM_CMD_GET_CONF,
	/* supported only for devices
	 * not for modules
	 */
	COMM_CMD_ADD,
	COMM_CMD_REMOVE,
	COMM_CMD_READ,
	COMM_CMD_WRITE,
	/* response flag
	 * This is set when a packet is a response
	 */
	COMM_CMD_RESP = 100
};

//#pragma pack(1)
struct comm_header {
	uint16_t	preamble;
	uint16_t	length;
	uint16_t	crc;
	uint16_t	cmd;		/* en_comm_cmd */
	union un_data {
		uint8_t 	dev_type;	/* en_device_type */
		uint8_t		resp;
	} data;
};

/* LED device and module */
//#pragma pack(1)
struct comm_prot_led_module {
	uint8_t		reset_led_pattern_index;
	uint16_t	tick_ms;
};

//#pragma pack(1)
struct comm_prot_led_dev {
	char		name[NAME_STR_LENGTH];
	char		port;
	uint16_t 	pin;
	uint8_t		pattern;
};

/* GPIO device and module */
//#pragma pack(1)
struct comm_prot_gpio_module {
	uint16_t	tick_ms;
	uint8_t 	evaluate_on_every_pin_set;
};

//#pragma pack(1)
struct comm_prot_gpio_dev {
	char		name[NAME_STR_LENGTH];
	uint8_t 	port;
	uint16_t	pin;
	uint8_t		dir;
	uint8_t		value;
};

/* UART device and module */
//#pragma pack(1)
struct comm_prot_uart_module {
	uint8_t		port;
	uint16_t	tick_ms;
};

//#pragma pack(1)
struct comm_prot_uart_dev {
	char		name[NAME_STR_LENGTH];
	uint8_t		port;
	uint32_t	baudrate;
	uint8_t		wordlength:4;
	uint8_t		stop_bits:2;
	uint8_t		parity:2;
	uint8_t		debug;
	uint8_t		timeout_ms;
};


/* ADC device and module */
//#pragma pack(1)
struct comm_prot_adc_module {
	uint8_t		adc;
	uint8_t		mode;
	uint16_t	tick_ms;
};

//#pragma pack(1)
struct comm_prot_adc_dev {
	char		name[NAME_STR_LENGTH];
	uint8_t		channel;
	uint8_t 	enable;
	uint16_t 	value;
};

//#pragma pack(1)
struct comm_prot_spi_module {
	uint8_t 	mode;
};

//#pragma pack(1)
struct comm_prot_spi_dev {
	char		name[NAME_STR_LENGTH];
	uint8_t 	channel;
};


static inline void comm_create_header(struct comm_header * header, enum en_comm_cmd cmd, enum en_device_type dev_type)
{
	header->preamble = COMM_PREAMBLE;
	header->length = sizeof(struct comm_header);
	header->crc = 0;
	header->cmd = (uint8_t) cmd;
	header->data.dev_type = (uint8_t) dev_type;
}

int comm_validate(const uint8_t * packet, uint16_t len);

#endif /* COMM_PROTOCOL_H_ */
