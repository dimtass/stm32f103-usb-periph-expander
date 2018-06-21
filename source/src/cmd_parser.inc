/*
 * cmd_parser.inc
 *
 *  Created on: 25 May 2018
 *      Author: Dimitris Tassopoulos
 */
/**
 * continues from platform_devices.c
 */

enum en_cmd_parser_resp {
	CMD_PARSER_OK = 0,
	CMD_PARSER_ERR_BEGIN = COMM_RET_ERR_END,
	CMD_PARSER_ERR_NOT_FOUND,
	CMD_PARSER_ERR_EXISTS,
	CMD_PARSER_ERR_NO_AVAILABLE_DATA,
};


void usb_0_rx_parser(uint8_t * recv_buffer, uint16_t recv_len)
{
	TRACE(("USB0_recv: %d\n", recv_len));
	uint8_t * resp_buff_p = NULL;
	int resp_len = cmd_parser(recv_buffer, recv_len, &resp_buff_p);
	if (resp_len && resp_buff_p) {
		USB_dev_send(&usb_comm_0, resp_buff_p, resp_len);
		TRACE(("USB0_send: %d\n", resp_len));
	}
	/* clean up */
	if (resp_buff_p) {
		free(resp_buff_p);
		resp_buff_p = NULL;
	}
}


void usb_1_rx_parser(uint8_t * recv_buffer, uint16_t recv_len)
{
	int resp_len = 0;
	TRACE(("USB1: %s\n", recv_buffer));
	TRACE(("USB1_recv: %d\n", recv_len));
	TRACE(("USB1_send: %d\n", resp_len));
}


void uart1_rx_parser(uint8_t *buffer, size_t bufferlen, uint8_t sender)
{
	uint8_t resp_code = CMD_PARSER_OK;
	uint16_t packet_len = sizeof(struct comm_header) + sizeof(uint16_t) + sizeof(bufferlen);
	uint8_t * out_buffer = NULL;

	struct comm_header * header = (struct comm_header*) out_buffer;
	header->cmd = COMM_CMD_READ;
	packet_len = cmd_create_resp(&out_buffer, header->cmd | COMM_CMD_RESP, DEV_TYPE_UART, resp_code, packet_len);

	uint16_t * out_data_size = (uint16_t*) &out_buffer[sizeof(struct comm_header)];
	*out_data_size = bufferlen;
	uint8_t * out_data = (uint8_t*) &out_buffer[sizeof(struct comm_header) + sizeof(uint16_t)];
	memcpy(out_data, buffer, bufferlen);
	USB_dev_send(&usb_comm_0, out_buffer, bufferlen);
	free(out_buffer);
}


void uart2_rx_parser(uint8_t *buffer, size_t bufferlen, uint8_t sender)
{
	uint8_t resp_code = CMD_PARSER_OK;
	uint16_t packet_len = sizeof(struct comm_header) + sizeof(uint16_t) + sizeof(bufferlen);
	uint8_t * out_buffer = NULL;

	struct comm_header * header = (struct comm_header*) out_buffer;
	header->cmd = COMM_CMD_READ;
	packet_len = cmd_create_resp(&out_buffer, header->cmd | COMM_CMD_RESP, DEV_TYPE_UART, resp_code, packet_len);

	uint16_t * out_data_size = (uint16_t*) &out_buffer[sizeof(struct comm_header)];
	*out_data_size = bufferlen;
	uint8_t * out_data = (uint8_t*) &out_buffer[sizeof(struct comm_header) + sizeof(uint16_t)];
	memcpy(out_data, buffer, bufferlen);
	USB_dev_send(&usb_comm_0, out_buffer, bufferlen);
	free(out_buffer);
}

/**
 * @brief Create a standard response packet. If additional data
 * required (like device specific data) then add their size and
 * skip calculate CRC16. Note: You need to calculate CRC after!
 * @param[out] buffer_p A pointer to alloc bytes and write data. NULL if fail.
 * @param[in] cmd The type of command
 * @param[in] dev_type The device type
 * @param[in] resp_code The response code to add to the packet
 * @param[in] additional_data_size The size of the device specific data
 * @return uint16_t The size of the packet created.
 */
uint16_t cmd_create_resp(uint8_t ** buffer_p, enum en_comm_cmd cmd,
		enum en_device_type dev_type, uint8_t resp_code, uint16_t additional_data_size)
{
	*buffer_p = (uint8_t*)malloc(sizeof(struct comm_header) + additional_data_size);
	if (!*buffer_p) {
		TRACE(("malloc failed\n"));
		return 0;
	}

	struct comm_header * resp = (struct comm_header*) *buffer_p;
	comm_create_header(resp, cmd, dev_type);
	resp->length += additional_data_size;
	resp->data.resp = resp_code;
	/* if there are additional data then don't calculate the crc twice */
	if (!additional_data_size)
		resp->crc = crc16(*buffer_p, sizeof(struct comm_header), COMM_CRC_POLY);
	return resp->length;
}


/**
 * @brief Parse a LED module command
 * @param[in] data The data in buffer
 * @param[out] buffer A pointer to the response that will be created
 * @return int The size of the response buffer
 */
uint16_t cmd_led_module_parser(uint8_t * data, uint8_t ** buffer_p)
{
	uint8_t resp_code = CMD_PARSER_OK;
	uint16_t packet_len = 0;
	struct comm_header * header = (struct comm_header *) data;
	struct comm_prot_led_module * req_mod = (struct comm_prot_led_module*) &data[sizeof(struct comm_header)];

	if (header->cmd == COMM_CMD_SET_CONF) {
		if (req_mod->tick_ms)
			led_module.tick_ms = req_mod->tick_ms;
		if (req_mod->reset_led_pattern_index)
			led_module.led_pattern_index = 0;
		goto exit_resp;
	}
	else if (header->cmd == COMM_CMD_GET_CONF) {
		req_mod->tick_ms = led_module.tick_ms;
		packet_len = cmd_create_resp(buffer_p, header->cmd | COMM_CMD_RESP, DEV_TYPE_LED, resp_code, sizeof(struct comm_prot_led_module));
		if (*buffer_p)
			memcpy(&(*buffer_p)[sizeof(struct comm_header)], req_mod, sizeof(struct comm_prot_led_module));

		struct comm_header * resp_header = (struct comm_header *) *buffer_p;
		resp_header->crc = crc16(*buffer_p, packet_len, COMM_CRC_POLY);
		goto exit_func;
	}

exit_resp:
	/* send resp */
	packet_len = cmd_create_resp(buffer_p, header->cmd | COMM_CMD_RESP, DEV_TYPE_LED, resp_code, 0);

exit_func:
	return packet_len;
}


/**
 * @brief Parse a LED device command
 * @param[in] data The data in buffer
 * @param[out] buffer A pointer to the response that will be created
 * @return int The size of the response buffer
 */
uint16_t cmd_led_dev_parser(uint8_t * data, uint8_t ** buffer_p)
{
	uint8_t resp_code = CMD_PARSER_OK;
	uint16_t packet_len = 0;
	struct comm_header * header = (struct comm_header *) data;
	struct comm_prot_led_dev * req_led = (struct comm_prot_led_dev*) &data[sizeof(struct comm_header)];

	/* Add a new device */
	if (header->cmd == COMM_CMD_ADD) {
		/* Should not exist already */
		if (platform_dev_find(req_led->name, DEV_TYPE_LED)) {
			resp_code = CMD_PARSER_ERR_EXISTS;
			goto exit_resp;
		}
		struct dev_led new_led;
		new_led.port = char_to_port(req_led->port);
		new_led.pin = 1 << req_led->pin;
		new_led.pattern = req_led->pattern;
		platform_dev_probe(req_led->name, DEV_TYPE_LED, &new_led);
	}
	else {
		struct platform_device * dev = platform_dev_find(req_led->name, DEV_TYPE_LED);
		if (!dev) {
			resp_code = CMD_PARSER_ERR_NOT_FOUND;
		}
		else if (header->cmd == COMM_CMD_REMOVE) {
			platform_dev_remove(dev->name, dev->type);
		}
		else if (header->cmd == COMM_CMD_WRITE) {
			struct dev_led * led = (struct dev_led*) dev->data;
			led->pattern = req_led->pattern;
		}
		else if (header->cmd == COMM_CMD_READ) {
			struct dev_led * led = (struct dev_led*) dev->data;
			req_led->pattern = led->pattern;
			req_led->port = port_to_char(led->port);
			req_led->pin = pin_to_uint(led->pin);

			packet_len = cmd_create_resp(buffer_p, header->cmd | COMM_CMD_RESP, DEV_TYPE_LED, resp_code, sizeof(struct comm_prot_led_dev));
			if (*buffer_p)
				memcpy(&(*buffer_p)[sizeof(struct comm_header)], req_led, sizeof(struct comm_prot_led_dev));

			struct comm_header * resp_header = (struct comm_header *) *buffer_p;
			resp_header->crc = crc16(*buffer_p, packet_len, COMM_CRC_POLY);
			goto exit_func;
		}
	}

exit_resp:
	/* send resp */
	packet_len = cmd_create_resp(buffer_p, header->cmd | COMM_CMD_RESP, DEV_TYPE_LED, resp_code, 0);

exit_func:
	return packet_len;
}


/**
 * @brief Parse a GPIO module command
 * @param[in] data The data in buffer
 * @param[out] buffer A pointer to the response that will be created
 * @return int The size of the response buffer
 */
uint16_t cmd_gpio_module_parser(uint8_t * data, uint8_t ** buffer_p)
{
	uint8_t resp_code = CMD_PARSER_OK;
	uint16_t packet_len = 0;
	struct comm_header * header = (struct comm_header *) data;
	struct comm_prot_gpio_module * req_mod = (struct comm_prot_gpio_module*) &data[sizeof(struct comm_header)];

	if (header->cmd == COMM_CMD_SET_CONF) {
		if (req_mod->tick_ms)
			gpio_module.tick_ms = req_mod->tick_ms;
		gpio_module.evaluate_on_every_pin_set = req_mod->evaluate_on_every_pin_set;
		goto exit_resp;
	}
	else if (header->cmd == COMM_CMD_GET_CONF) {
		req_mod->tick_ms = gpio_module.tick_ms;
		packet_len = cmd_create_resp(buffer_p, header->cmd | COMM_CMD_RESP, DEV_TYPE_GPIO, resp_code, sizeof(struct comm_prot_gpio_module));
		if (*buffer_p)
			memcpy(&(*buffer_p)[sizeof(struct comm_header)], req_mod, sizeof(struct comm_prot_gpio_module));

		struct comm_header * resp_header = (struct comm_header *) *buffer_p;
		resp_header->crc = crc16(*buffer_p, packet_len, COMM_CRC_POLY);
		goto exit_func;
	}

exit_resp:
	/* send resp */
	packet_len = cmd_create_resp(buffer_p, header->cmd | COMM_CMD_RESP, DEV_TYPE_GPIO, resp_code, 0);

exit_func:
	return packet_len;
}


/**
 * @brief Parse a GPIO device command
 * @param[in] data The data in buffer
 * @param[out] buffer A pointer to the response that will be created
 * @return int The size of the response buffer
 */
uint16_t cmd_gpio_dev_parser(uint8_t * data, uint8_t ** buffer_p)
{
	uint8_t resp_code = CMD_PARSER_OK;
	uint16_t packet_len = 0;
	struct comm_header * header = (struct comm_header *) data;
	struct comm_prot_gpio_dev * req_gpio = (struct comm_prot_gpio_dev*) &data[sizeof(struct comm_header)];

	/* Add a new device */
	if (header->cmd == COMM_CMD_ADD) {
		/* Should not exist already */
		if (platform_dev_find(req_gpio->name, DEV_TYPE_GPIO)) {
			resp_code = CMD_PARSER_ERR_EXISTS;
			goto exit_resp;
		}
		struct dev_gpio new_gpio;
		new_gpio.port = char_to_port(req_gpio->port);
		new_gpio.pin = 1 << req_gpio->pin;
		new_gpio.dir = req_gpio->dir;
		new_gpio.value = !req_gpio->value;
		new_gpio.new_value = req_gpio->value;
		platform_dev_probe(req_gpio->name, DEV_TYPE_GPIO, &new_gpio);
	}
	else {
		struct platform_device * dev = platform_dev_find(req_gpio->name, DEV_TYPE_GPIO);
		if (!dev) {
			resp_code = CMD_PARSER_ERR_NOT_FOUND;
		}
		else if (header->cmd == COMM_CMD_REMOVE) {
			platform_dev_remove(dev->name, dev->type);
		}
		else if (header->cmd == COMM_CMD_WRITE) {
			struct dev_gpio * gpio = (struct dev_gpio*) dev->data;
			gpio->new_value = req_gpio->value;
		}
		else if (header->cmd == COMM_CMD_READ) {
			struct dev_gpio * gpio = (struct dev_gpio*) dev->data;
			req_gpio->value = gpio->new_value;
			req_gpio->port = port_to_char(gpio->port);
			req_gpio->pin = pin_to_uint(gpio->pin);
			req_gpio->dir = gpio->dir;

			packet_len = cmd_create_resp(buffer_p, header->cmd | COMM_CMD_RESP, DEV_TYPE_GPIO, resp_code, sizeof(struct comm_prot_gpio_dev));
			if (*buffer_p)
				memcpy(&(*buffer_p)[sizeof(struct comm_header)], req_gpio, sizeof(struct comm_prot_gpio_dev));

			struct comm_header * resp_header = (struct comm_header *) *buffer_p;
			resp_header->crc = crc16(*buffer_p, packet_len, COMM_CRC_POLY);
			goto exit_func;
		}
	}

exit_resp:
	/* send resp */
	packet_len = cmd_create_resp(buffer_p, header->cmd | COMM_CMD_RESP, DEV_TYPE_GPIO, resp_code, 0);

exit_func:
	return packet_len;
}


/**
 * @brief Parse a ADC module command
 * @param[in] data The data in buffer
 * @param[out] buffer A pointer to the response that will be created
 * @return int The size of the response buffer
 */
uint16_t cmd_adc_module_parser(uint8_t * data, uint8_t ** buffer_p)
{
	uint8_t resp_code = CMD_PARSER_OK;
	uint16_t packet_len = 0;
	struct comm_header * header = (struct comm_header *) data;
	struct comm_prot_adc_module * req_mod = (struct comm_prot_adc_module*) &data[sizeof(struct comm_header)];

	if (header->cmd == COMM_CMD_SET_CONF) {
		if (req_mod->tick_ms)
			adc_module.tick_ms = req_mod->tick_ms;
		adc_module.mode = req_mod->mode;
		goto exit_resp;
	}
	else if (header->cmd == COMM_CMD_GET_CONF) {
		req_mod->tick_ms = adc_module.tick_ms;
		packet_len = cmd_create_resp(buffer_p, header->cmd | COMM_CMD_RESP, DEV_TYPE_ADC, resp_code, sizeof(struct comm_prot_adc_module));
		if (*buffer_p)
			memcpy(&(*buffer_p)[sizeof(struct comm_header)], req_mod, sizeof(struct comm_prot_adc_module));

		struct comm_header * resp_header = (struct comm_header *) *buffer_p;
		resp_header->crc = crc16(*buffer_p, packet_len, COMM_CRC_POLY);
		goto exit_func;
	}

exit_resp:
	/* send resp */
	packet_len = cmd_create_resp(buffer_p, header->cmd | COMM_CMD_RESP, DEV_TYPE_ADC, resp_code, 0);

exit_func:
	return packet_len;
}


/**
 * @brief Parse a GPIO device command
 * @param[in] data The data in buffer
 * @param[out] buffer A pointer to the response that will be created
 * @return int The size of the response buffer
 */
uint16_t cmd_adc_dev_parser(uint8_t * data, uint8_t ** buffer_p)
{
	uint8_t resp_code = CMD_PARSER_OK;
	uint16_t packet_len = 0;
	struct comm_header * header = (struct comm_header *) data;
	struct comm_prot_adc_dev * req_adc = (struct comm_prot_adc_dev*) &data[sizeof(struct comm_header)];

	/* Add a new device */
	if (header->cmd == COMM_CMD_ADD) {
		/* Should not exist already */
		if (platform_dev_find(req_adc->name, DEV_TYPE_ADC)) {
			resp_code = CMD_PARSER_ERR_EXISTS;
			goto exit_resp;
		}
		struct dev_adc new_adc;
		new_adc.channel = req_adc->channel;
		new_adc.enable = req_adc->enable;
		platform_dev_probe(req_adc->name, DEV_TYPE_ADC, &new_adc);
	}
	else {
		struct platform_device * dev = platform_dev_find(req_adc->name, DEV_TYPE_ADC);
		if (!dev) {
			resp_code = CMD_PARSER_ERR_NOT_FOUND;
		}
		else if (header->cmd == COMM_CMD_REMOVE) {
			platform_dev_remove(dev->name, dev->type);
		}
		else if (header->cmd == COMM_CMD_WRITE) {
			struct dev_adc * adc = (struct dev_adc*) dev->data;
			adc->enable = req_adc->enable;
		}
		else if (header->cmd == COMM_CMD_READ) {
			struct dev_adc * adc = (struct dev_adc*) dev->data;
			req_adc->value = adc->value;
			req_adc->enable = adc->enable;

			packet_len = cmd_create_resp(buffer_p, header->cmd | COMM_CMD_RESP, DEV_TYPE_ADC, resp_code, sizeof(struct comm_prot_adc_dev));
			if (*buffer_p)
				memcpy(&(*buffer_p)[sizeof(struct comm_header)], req_adc, sizeof(struct comm_prot_adc_dev));

			struct comm_header * resp_header = (struct comm_header *) *buffer_p;
			resp_header->crc = crc16(*buffer_p, packet_len, COMM_CRC_POLY);
			goto exit_func;
		}
	}

exit_resp:
	/* send resp */
	packet_len = cmd_create_resp(buffer_p, header->cmd | COMM_CMD_RESP, DEV_TYPE_GPIO, resp_code, 0);

exit_func:
	return packet_len;
}


/**
 * @brief Parse a UART module command
 * @param[in] data The data in buffer
 * @param[out] buffer A pointer to the response that will be created
 * @return int The size of the response buffer
 */
uint16_t cmd_uart_module_parser(uint8_t * data, uint8_t ** buffer_p)
{
	uint8_t resp_code = CMD_PARSER_OK;
	uint16_t packet_len = 0;
	struct comm_header * header = (struct comm_header *) data;
	struct comm_prot_uart_module * req_mod = (struct comm_prot_uart_module*) &data[sizeof(struct comm_header)];

	if (header->cmd == COMM_CMD_SET_CONF) {
		if (req_mod->tick_ms)
			uart_module.tick_ms = req_mod->tick_ms;
		goto exit_resp;
	}
	else if (header->cmd == COMM_CMD_GET_CONF) {
		req_mod->tick_ms = uart_module.tick_ms;
		packet_len = cmd_create_resp(buffer_p, header->cmd | COMM_CMD_RESP, DEV_TYPE_UART, resp_code, sizeof(struct comm_prot_uart_module));
		if (*buffer_p)
			memcpy(&(*buffer_p)[sizeof(struct comm_header)], req_mod, sizeof(struct comm_prot_uart_module));

		struct comm_header * resp_header = (struct comm_header *) *buffer_p;
		resp_header->crc = crc16(*buffer_p, packet_len, COMM_CRC_POLY);
		goto exit_func;
	}

exit_resp:
	/* send resp */
	packet_len = cmd_create_resp(buffer_p, header->cmd | COMM_CMD_RESP, DEV_TYPE_UART, resp_code, 0);

exit_func:
	return packet_len;
}


/**
 * @brief Parse a UART device command
 * @param[in] data The data in buffer
 * @param[out] buffer A pointer to the response that will be created
 * @return int The size of the response buffer
 */
uint16_t cmd_uart_dev_parser(uint8_t * data, uint8_t ** buffer_p)
{
	uint8_t resp_code = CMD_PARSER_OK;
	uint16_t packet_len = 0;
	struct comm_header * header = (struct comm_header *) data;
	struct comm_prot_uart_dev * req_dev = (struct comm_prot_uart_dev*) &data[sizeof(struct comm_header)];

	/* Add a new device */
	if (header->cmd == COMM_CMD_ADD) {
		/* Should not exist already */
		if (platform_dev_find(req_dev->name, DEV_TYPE_UART)) {
			resp_code = CMD_PARSER_ERR_EXISTS;
			goto exit_resp;
		}
		struct dev_uart new_dev;
		new_dev.port = uint_to_usart(req_dev->port);
		new_dev.timeout_ms = req_dev->timeout_ms;
		new_dev.config.USART_BaudRate = req_dev->baudrate;
		new_dev.config.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		new_dev.config.USART_WordLength = req_dev->wordlength;
		new_dev.config.USART_Parity = req_dev->parity;
		new_dev.config.USART_StopBits = req_dev->stop_bits;
		new_dev.config.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		new_dev.debug = req_dev->debug;
		if (new_dev.port == USART1)
			new_dev.fp_dev_uart_cb = uart1_rx_parser;
		if (new_dev.port == USART2)
			new_dev.fp_dev_uart_cb = uart2_rx_parser;
		platform_dev_probe(req_dev->name, DEV_TYPE_UART, &new_dev);
	}
	else {
		struct platform_device * dev = platform_dev_find(req_dev->name, DEV_TYPE_UART);
		if (!dev) {
			resp_code = CMD_PARSER_ERR_NOT_FOUND;
		}
		else if (header->cmd == COMM_CMD_REMOVE) {
			platform_dev_remove(dev->name, dev->type);
		}
		else if (header->cmd == COMM_CMD_WRITE) {
			struct dev_uart * uart = (struct dev_uart*) dev->data;
			/* get data from packet and send them to port */
			uint16_t data_len = *((uint16_t*) &data[sizeof(struct comm_header)+sizeof(struct comm_prot_uart_dev)]);
			uint8_t * data_p = (uint8_t*) &data[sizeof(struct comm_header) + sizeof(struct comm_prot_uart_dev) + sizeof(uint16_t)];
			dev_uart_send_buffer(uart, data_p, data_len);
		}
		else if (header->cmd == COMM_CMD_READ) {
			// TODO:
			/* send data to the USB port */
		}
	}

exit_resp:
	/* send resp */
	packet_len = cmd_create_resp(buffer_p, header->cmd | COMM_CMD_RESP, DEV_TYPE_GPIO, resp_code, 0);

	return packet_len;
}


/**
 * @brief Parse a SPI module command
 * @param[in] data The data in buffer
 * @param[out] buffer A pointer to the response that will be created
 * @return int The size of the response buffer
 */
uint16_t cmd_spi_module_parser(uint8_t * data, uint8_t ** buffer_p)
{
	uint8_t resp_code = CMD_PARSER_OK;
	uint16_t packet_len = 0;
	struct comm_header * header = (struct comm_header *) data;
	struct comm_prot_spi_module * req_mod = (struct comm_prot_spi_module*) &data[sizeof(struct comm_header)];

	if (header->cmd == COMM_CMD_SET_CONF) {
		spi_module.mode = req_mod->mode;
		goto exit_resp;
	}
	else if (header->cmd == COMM_CMD_GET_CONF) {
		req_mod->mode = spi_module.mode;
		packet_len = cmd_create_resp(buffer_p, header->cmd | COMM_CMD_RESP, DEV_TYPE_SPI, resp_code, sizeof(struct comm_prot_spi_module));
		if (*buffer_p)
			memcpy(&(*buffer_p)[sizeof(struct comm_header)], req_mod, sizeof(struct comm_prot_spi_module));

		struct comm_header * resp_header = (struct comm_header *) *buffer_p;
		resp_header->crc = crc16(*buffer_p, packet_len, COMM_CRC_POLY);
		goto exit_func;
	}

exit_resp:
	/* send resp */
	packet_len = cmd_create_resp(buffer_p, header->cmd | COMM_CMD_RESP, DEV_TYPE_SPI, resp_code, 0);

exit_func:
	return packet_len;
}


/**
 * @brief Parse a SPI device command
 * @param[in] data The data in buffer
 * @param[out] buffer A pointer to the response that will be created
 * @return int The size of the response buffer
 */
uint16_t cmd_spi_dev_parser(uint8_t * data, uint8_t ** buffer_p)
{
	uint8_t resp_code = CMD_PARSER_OK;
	uint16_t packet_len = 0;
	struct comm_header * header = (struct comm_header *) data;
	struct comm_prot_spi_dev * req_dev = (struct comm_prot_spi_dev*) &data[sizeof(struct comm_header)];

	/* Add a new device */
	if (header->cmd == COMM_CMD_ADD) {
		/* Should not exist already */
		if (platform_dev_find(req_dev->name, DEV_TYPE_SPI)) {
			resp_code = CMD_PARSER_ERR_EXISTS;
			goto exit_resp;
		}
		struct dev_spi new_dev;
		new_dev.channel = req_dev->channel;
		platform_dev_probe(req_dev->name, DEV_TYPE_SPI, &new_dev);
	}
	else {
		struct platform_device * dev = platform_dev_find(req_dev->name, DEV_TYPE_UART);
		if (!dev) {
			resp_code = CMD_PARSER_ERR_NOT_FOUND;
		}
		else if (header->cmd == COMM_CMD_REMOVE) {
			platform_dev_remove(dev->name, dev->type);
		}
		else if (header->cmd == COMM_CMD_WRITE) {
			/**
			 * Packet format:
			 * [struct comm_header | struct comm_prot_spi_dev | DATALEN TO WRITE:16-bit | N DATA BYTES ]
			 */
			struct dev_spi * spi = (struct dev_spi*) dev->data;
			/* get data from packet and send them to port */
			uint16_t data_len = *((uint16_t*) &data[sizeof(struct comm_header)+sizeof(struct comm_prot_spi_dev)]);
			uint8_t * data_p = (uint8_t*) &data[sizeof(struct comm_header) + sizeof(struct comm_prot_spi_dev) + sizeof(uint16_t)];
			dev_spi_send(spi, data_p, data_len);
		}
		else if (header->cmd == COMM_CMD_READ) {
			/**
			 * Packet format:
			 * [struct comm_header | struct comm_prot_spi_dev | DATALEN TO READ:16-bit]
			 */
			/* send data to the USB port */
			struct dev_spi * spi = (struct dev_spi*) dev->data;
			uint16_t data_len = *((uint16_t*) &data[sizeof(struct comm_header)+sizeof(struct comm_prot_spi_dev)]);

			packet_len = cmd_create_resp(buffer_p, header->cmd | COMM_CMD_RESP, DEV_TYPE_SPI, resp_code, sizeof(uint16_t) + data_len);
			if (*buffer_p) {
				uint16_t * spi_resp_len = (uint16_t*) buffer_p[sizeof(struct comm_header)];
				uint8_t * spi_resp = buffer_p[sizeof(struct comm_header) + sizeof(uint16_t)];

				*spi_resp_len = data_len;

				dev_spi_recv(spi, spi_resp, data_len);

				struct comm_header * resp_header = (struct comm_header *) *buffer_p;
				resp_header->crc = crc16(*buffer_p, packet_len, COMM_CRC_POLY);
				goto exit_func;
			}
		}
	}

exit_resp:
	/* send resp */
	packet_len = cmd_create_resp(buffer_p, header->cmd | COMM_CMD_RESP, DEV_TYPE_GPIO, resp_code, 0);

exit_func:
	return packet_len;
}



int cmd_parser(uint8_t * buffer, uint16_t buffer_len, uint8_t ** resp_p)
{
	int index = comm_validate(buffer, buffer_len);

	if (index < 0) {
//		int tmp = cmd_create_resp(resp_p, COMM_CMD_RESP, 0, -index, 0);
//		return tmp;
	}

	int resp_len = 0;
	struct comm_header * header = (struct comm_header*) &buffer[index];
	uint8_t is_module = (header->data.dev_type >= DEV_TYPE_MODULE);
	if (is_module) header->data.dev_type -= DEV_TYPE_MODULE;
	/* LED */
	if (header->data.dev_type == DEV_TYPE_LED) {
		if (is_module) {
			resp_len = cmd_led_module_parser(&buffer[index], resp_p);
		}
		else {
			resp_len = cmd_led_dev_parser(&buffer[index], resp_p);
		}
	}
	/* GPIO */
	else if (header->data.dev_type == DEV_TYPE_GPIO) {
		if (is_module) {
			cmd_gpio_module_parser(&buffer[index], resp_p);
		}
		else {
			cmd_gpio_dev_parser(&buffer[index], resp_p);
		}
	}
	/* ADC */
	else if (header->data.dev_type == DEV_TYPE_ADC) {
		if (is_module) {
			cmd_adc_module_parser(&buffer[index], resp_p);
		}
		else {
			cmd_adc_dev_parser(&buffer[index], resp_p);
		}
	}
	/* UART */
	else if (header->data.dev_type == DEV_TYPE_UART) {
		if (is_module) {
			cmd_uart_module_parser(&buffer[index], resp_p);
		}
		else {
			cmd_uart_dev_parser(&buffer[index], resp_p);
		}
	}
	/* SPI */
	else if (header->data.dev_type == DEV_TYPE_SPI) {
		if (is_module) {
			cmd_spi_module_parser(&buffer[index], resp_p);
		}
		else {
			cmd_spi_dev_parser(&buffer[index], resp_p);
		}
	}
	return resp_len;
}



/** Tester functions for protocol **/
#define TEST(NAME) [NAME] = "NAME"
enum en_test_names {
	LED_DEV_ADD,
	LED_DEV_REMOVE,
	LED_DEV_READ,
	LED_DEV_WRITE,
	LED_MOD_GET_CONF,
	LED_MOD_SET_CONF,
};
char test_names[][17] = {
		TEST(LED_DEV_ADD),
		TEST(LED_DEV_REMOVE),
		TEST(LED_DEV_READ),
		TEST(LED_DEV_WRITE),
		TEST(LED_MOD_GET_CONF),
		TEST(LED_MOD_SET_CONF),
};





void test_cmd_parser(char * test_name)
{

	if (!(strncmp(test_name, "LED_DEV_ADD", 11))) {
		uint16_t data_size = sizeof(struct comm_header) + sizeof(struct comm_prot_led_dev);
		uint8_t data[data_size];
		struct comm_header * header = (struct comm_header *) data;
		struct comm_prot_led_dev * dev = (struct comm_prot_led_dev *) &data[sizeof(struct comm_header)];

		comm_create_header(header, COMM_CMD_ADD, DEV_TYPE_LED);
		strcpy(dev->name, "LED.C13");
		dev->port = 'C';
		dev->pin = 13;
		dev->pattern = 0b00001111;
		header->length = data_size;
		header->crc = crc16(data, data_size, COMM_CRC_POLY);

		/* decode */
		uint8_t * resp = NULL;
//		int resp_len = cmd_parser(data, data_size, &resp);
		cmd_parser(data, data_size, &resp);

		if (resp) {
			struct comm_header * header = (struct comm_header *) resp;
//			TRACE(("resp_len: %d\n", resp_len));
//			TRACE(("H->preamble: %d\n", header->preamble));
//			TRACE(("H->length: %d\n", header->length));
//			TRACE(("H->cmd: %d\n", header->cmd));
			TRACE(("H->data.resp: %d\n", header->data.resp));
			free(resp);
		}

	}
	else if (!(strncmp(test_name, "LED_DEV_REMOVE", 14))) {
		uint16_t data_size = sizeof(struct comm_header) + sizeof(struct comm_prot_led_dev);
		uint8_t data[data_size];
		struct comm_header * header = (struct comm_header *) data;
		struct comm_prot_led_dev * dev = (struct comm_prot_led_dev *) &data[sizeof(struct comm_header)];

		comm_create_header(header, COMM_CMD_REMOVE, DEV_TYPE_LED);
		strcpy(dev->name, "LED.C13");
		dev->pattern = 0b00001111;
		header->length = data_size;
		header->crc = crc16(data, data_size, COMM_CRC_POLY);

		/* decode */
		uint8_t * resp = NULL;
		cmd_parser(data, data_size, &resp);

		if (resp) {
			struct comm_header * header = (struct comm_header *) resp;
//			TRACE(("resp_len: %d\n", resp_len));
//			TRACE(("H->preamble: %d\n", header->preamble));
//			TRACE(("H->length: %d\n", header->length));
//			TRACE(("H->cmd: %d\n", header->cmd));
			TRACE(("H->data.resp: %d\n", header->data.resp));
			free(resp);
		}

	}
	else if (!(strncmp(test_name, "LED_DEV_READ", 12))) {
		uint16_t data_size = sizeof(struct comm_header) + sizeof(struct comm_prot_led_dev);
		uint8_t data[data_size];
		struct comm_header * header = (struct comm_header *) data;
		struct comm_prot_led_dev * dev = (struct comm_prot_led_dev *) &data[sizeof(struct comm_header)];

		comm_create_header(header, COMM_CMD_READ, DEV_TYPE_LED);
		strcpy(dev->name, "LED.C13");
		header->length = data_size;
		header->crc = crc16(data, data_size, COMM_CRC_POLY);

		/* decode */
		uint8_t * resp = NULL;
		int resp_len = cmd_parser(data, data_size, &resp);

		if (resp) {
			struct comm_header * header = (struct comm_header *) resp;
			TRACE(("resp_len: %d\n", resp_len));
			TRACE(("H->preamble: %d\n", header->preamble));
			TRACE(("H->length: %d\n", header->length));
			TRACE(("H->cmd: %d\n", header->cmd));
			TRACE(("H->data.resp: %d\n", header->data.resp));
			if (resp_len > (int) sizeof(struct comm_header)) {
				struct comm_prot_led_dev * dev_resp = (struct comm_prot_led_dev *) &resp[sizeof(struct comm_header)];
				TRACE(("LED name: %s\n", dev_resp->name));
				TRACE(("LED pattern: %d\n", dev_resp->pattern));
				TRACE(("LED pin: %d\n", dev_resp->pin));
				TRACE(("LED port: %c\n", dev_resp->port));
			}
			free(resp);
		}
	}
	else if (!(strncmp(test_name, "LED_DEV_WRITE", 12))) {
		uint16_t data_size = sizeof(struct comm_header) + sizeof(struct comm_prot_led_dev);
		uint8_t data[data_size];
		struct comm_header * header = (struct comm_header *) data;
		struct comm_prot_led_dev * dev = (struct comm_prot_led_dev *) &data[sizeof(struct comm_header)];

		comm_create_header(header, COMM_CMD_WRITE, DEV_TYPE_LED);
		strcpy(dev->name, "LED.C13");
		dev->pattern = 0b10101010;
		header->length = data_size;
		header->crc = crc16(data, data_size, COMM_CRC_POLY);

		/* decode */
		uint8_t * resp = NULL;
		int resp_len = cmd_parser(data, data_size, &resp);

		if (resp) {
			struct comm_header * header = (struct comm_header *) resp;
			TRACE(("resp_len: %d\n", resp_len));
			TRACE(("H->preamble: %d\n", header->preamble));
			TRACE(("H->length: %d\n", header->length));
			TRACE(("H->cmd: %d\n", header->cmd));
			TRACE(("H->data.resp: %d\n", header->data.resp));
			free(resp);
		}
	}
	else if (!(strncmp(test_name, "LED_MOD_GET_CONF", 16))) {
		uint16_t data_size = sizeof(struct comm_header);
		uint8_t data[data_size];
		struct comm_header * header = (struct comm_header *) data;

		comm_create_header(header, COMM_CMD_GET_CONF, DEV_TYPE_LED | DEV_TYPE_MODULE);
		header->length = data_size;
		header->crc = crc16(data, data_size, COMM_CRC_POLY);

		/* decode */
		uint8_t * resp = NULL;
		int resp_len = cmd_parser(data, data_size, &resp);

		if (resp) {
			struct comm_header * header = (struct comm_header *) resp;
			TRACE(("resp_len: %d\n", resp_len));
			TRACE(("H->preamble: %d\n", header->preamble));
			TRACE(("H->length: %d\n", header->length));
			TRACE(("H->cmd: %d\n", header->cmd));
			TRACE(("H->data.resp: %d\n", header->data.resp));
			if (resp_len > (int) sizeof(struct comm_header)) {
				struct comm_prot_led_module * mod_resp = (struct comm_prot_led_module *) &resp[sizeof(struct comm_header)];
				TRACE(("LED mod->tick_ms:%d\n", mod_resp->tick_ms));
			}
			free(resp);
		}
	}
	else if (!(strncmp(test_name, "LED_MOD_SET_CONF", 16))) {
		uint16_t data_size = sizeof(struct comm_header) + sizeof(struct comm_prot_led_module);
		uint8_t data[data_size];
		struct comm_header * header = (struct comm_header *) data;
		struct comm_prot_led_module * mod = (struct comm_prot_led_module *) &data[sizeof(struct comm_header)];

		comm_create_header(header, COMM_CMD_SET_CONF, DEV_TYPE_LED | DEV_TYPE_MODULE);
		header->length = data_size;
		mod->tick_ms = 500;
		header->crc = crc16(data, data_size, COMM_CRC_POLY);

		/* decode */
		uint8_t * resp = NULL;
		int resp_len = cmd_parser(data, data_size, &resp);

		if (resp) {
			struct comm_header * header = (struct comm_header *) resp;
			TRACE(("resp_len: %d\n", resp_len));
			TRACE(("H->preamble: %d\n", header->preamble));
			TRACE(("H->length: %d\n", header->length));
			TRACE(("H->cmd: %d\n", header->cmd));
			TRACE(("H->data.resp: %d\n", header->data.resp));
			free(resp);

//			dev_timer_del(, &dev_timer_list);
//			dev_timer_add((void*) &led_module, led_module.tick_ms, (void*) &dev_led_update, &dev_timer_list);
		}
	}
}
