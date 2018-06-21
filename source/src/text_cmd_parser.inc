enum {
	ERR_INDEX = 1,
	ERR_LEN,
	ERR_UART_NE,
	ERR_DEV_NE,
};

#define GET_NEXT_PARAM(BUFFER) \
		tmp_idx = find_char_index(&buffer[idx], bufferlen-idx, ','); \
		if (tmp_idx < 0) return(-ERR_INDEX); \
		tmp_idx++; \
		snprintf(BUFFER, tmp_idx, "%s", &buffer[idx]); \
		TRACEL(TRACE_LEVEL_PARSER,("PARAM: %s\n", BUFFER)); \
		idx += tmp_idx


int find_char_index(uint8_t * buffer, size_t bufferlen, char find)
{
	size_t i = 0;
	for(i=0; i<bufferlen; i++) {
		if ((buffer[i] == find) || (buffer[i] == '\n') || (buffer[i] == 0)) return i;
	}
	return -1;
}

int led_parser(uint8_t *buffer, size_t bufferlen, char ** resp)
{
	/* GET name */
	char tmp[bufferlen];
	char name[NAME_STR_LENGTH];
	size_t idx = 0;
	size_t tmp_idx = 0;
	uint8_t exists = 0;
	int err = 0;
	
	GET_NEXT_PARAM(tmp);
	struct platform_device * dev = platform_dev_find(tmp, DEV_TYPE_LED);
	if (dev) exists = 1;
	if (tmp_idx > NAME_STR_LENGTH) {
		err = -ERR_LEN;
		goto exit;
	}
	snprintf(name, NAME_STR_LENGTH, "%s", tmp);
	
	/* get command */
	GET_NEXT_PARAM(tmp);
	TRACEL(TRACE_LEVEL_PARSER,("CMD: %s\n", tmp));
	if (!strncmp(tmp, "SET", 3)) {
		if (!exists) {
			struct dev_led led;
			/* Get port */
			GET_NEXT_PARAM(tmp);
			if (!strncmp(tmp, "GPIOA", 5))
				led.port = GPIOA;
			else if (!strncmp(tmp, "GPIOB", 5))
				led.port = GPIOB;
			else if (!strncmp(tmp, "GPIOC", 5))
				led.port = GPIOC;
			else if (!strncmp(tmp, "GPIOD", 5))
				led.port = GPIOD;
			else {
				err = -ERR_UART_NE;
				goto exit;
			}
			/* Get pin */
			GET_NEXT_PARAM(tmp);
			led.pin = 1 << atoi(tmp);
			/* Get pattern */
			GET_NEXT_PARAM(tmp);
			led.pattern = strbin2i(tmp);
			/* add device */
			platform_dev_probe(name, DEV_TYPE_LED, &led);
			goto exit;
		}
		else {
			/* device exists */
			/* get pattern */
			GET_NEXT_PARAM(tmp);	// port
			GET_NEXT_PARAM(tmp);	// pin
			GET_NEXT_PARAM(tmp);	// pattern
			((struct dev_led*)dev->data)->pattern = strbin2i(tmp);
			goto exit;
		}
	}
	else if (!strncmp(tmp, "GET", 3)) {
		if (!exists) {
			err = ERR_DEV_NE;
			goto exit;
		}
		else {
			struct dev_led * led = (struct dev_led*)dev->data;
			char tmpstr1[100];
			char tmpstr2[NAME_STR_LENGTH];
			/* device */
			strcpy(tmpstr1, "LED:");
			/* name */
			strcat(tmpstr1, dev->name);
			strcat(tmpstr1, ",GPIO");
			/* port */
			tmpstr2[0] = port_to_char(led->port);
			tmpstr2[1] = 0;
			strcat(tmpstr1, tmpstr2);
			strcat(tmpstr1, ",");
			/* pin */
			sprintf(tmpstr2, "%d", pin_to_uint(led->pin));
			strcat(tmpstr1, tmpstr2);
			strcat(tmpstr1, ",");
			/* pattern */
			u82strbin(led->pattern, tmpstr2);
			strcat(tmpstr1, tmpstr2);
			/* newline */
			strcat(tmpstr1, "\n");
			
			TRACEL(TRACE_LEVEL_PARSER,("%s\n",tmpstr1));
		}
	}

exit:
	return err;
}


/**
 * Text protocol format
 * [PERIPHERAL]=[NAME],[TYPE],[DATA] 
*/
void text_cmd_parser(uint8_t *buffer, size_t bufferlen, uint8_t sender, void * dev)
{
	size_t index = find_char_index(buffer, bufferlen, '=');
	char * resp_str = NULL;
	char std_resp[10];
	int resp_len = 0;

	TRACEL(TRACE_LEVEL_PARSER,("text_cmd_parser\n"));

	if (!index) return;
	if (!strncmp(buffer, "LED", index)) {
		index++; // skip '='
		resp_len = led_parser(&buffer[index], bufferlen - index, &resp_str);
	}
	else if (!strncmp(buffer, "GPIO", index)) {

	}
	else if (!strncmp(buffer, "UART", index)) {

	}
	else if (!strncmp(buffer, "ADC", index)) {

	}
	else if (!strncmp(buffer, "SPI", index)) {

	}
	else if (!strncmp(buffer, "I2C", index)) {

	}

	/* OK response */
	if (!resp_len) {
		strcpy(std_resp, "OK\n");
		resp_str = std_resp;
		resp_len = strlen(resp_str);
	}
	/* ERROR resp */
	else if (resp_len < 0) {
		sprintf(std_resp, "ERROR:%d\n", resp_len);
		resp_str = std_resp;
		resp_len = strlen(resp_str);
	}

	if (resp_str) {
		if (sender == SENDER_USB0)
			USB_dev_send((struct dev_usb_uart*)dev, resp_str, resp_len);
		else if (sender == SENDER_USB1)
			USB_dev_send((struct dev_usb_uart*)dev, resp_str, resp_len);
		else if (sender == SENDER_UART1)
			dev_uart_send_buffer((struct dev_uart*)dev, resp_str, resp_len);
		else if (sender == SENDER_UART2)
			dev_uart_send_buffer((struct dev_uart*)dev, resp_str, resp_len);
	}
}