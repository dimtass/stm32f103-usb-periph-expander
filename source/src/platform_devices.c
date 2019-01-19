/*
 * platform_devices.c
 *
 *  Created on: 13 May 2018
 *      Author: Dimitris Tassopoulos
 */

#include "platform_devices.h"

#define DEBUG_STM32F103

const char dev_type_names[][20] = {
	[DEV_TYPE_LED] = "LED",
	[DEV_TYPE_GPIO] = "GPIO",
	[DEV_TYPE_UART] = "UART",
	[DEV_TYPE_ADC] = "ADC",
	[DEV_TYPE_SPI] = "SPI",
	[DEV_TYPE_I2C] = "I2C",
};

struct platform_device {
	char name[NAME_STR_LENGTH];
	enum en_device_type type;
	void * data;
	struct list_head list;
};

static LIST_HEAD(platform_dev_list);

static LIST_HEAD(dev_timer_list);


// int cmd_parser(uint8_t * buffer, uint16_t buffer_len, uint8_t ** resp_p);
void usb_0_rx_parser(uint8_t * recv_buffer, uint16_t recv_len);
void usb_1_rx_parser(uint8_t * recv_buffer, uint16_t recv_len);
void uart1_rx_parser(struct dev_uart * uart, uint8_t *buffer, size_t bufferlen);
void uart2_rx_parser(struct dev_uart * uart, uint8_t *buffer, size_t bufferlen);

#define UART_BUFFER_SIZE	512


/* Declare USB endpoints */
DECLARE_COMM_BUFFER(usb_0_comm_buffer, 1024, 1024);
DECLARE_USB_UART_DEV(usb_comm_0, ENDP1, ENDP1_TXADDR, ENDP3, ENDP3_RXADDR, VIRTUAL_COM_PORT_DATA_SIZE, 10, usb_0_rx_parser, NULL);

DECLARE_COMM_BUFFER(usb_1_comm_buffer, 1024, 1024);
DECLARE_USB_UART_DEV(usb_comm_1, ENDP4, ENDP4_TXADDR, ENDP6, ENDP6_RXADDR, VIRTUAL_COM_PORT_DATA_SIZE, 10, usb_1_rx_parser, NULL);


/* Declare the default GPIO device that will handle all new GPIOs */
DECLARE_MODULE_GPIO(gpio_module, 100, 1);

DECLARE_MODULE_ADC(adc_module, ADC1, DEV_ADC_MODE_SINGLE, 1000);

DECLARE_MODULE_SPI(spi_module, DEV_SPI1_GPIOA);

DECLARE_MODULE_UART(uart_module, 1);


#ifdef DEBUG_STM32F103

/* Declare the led master and by default a status led on pin B.13 */
DECLARE_MODULE_LED(led_module, 250);
DECLARE_DEV_LED(led_status, PORT_STATUS_LED, PIN_STATUS_LED, &led_module);

/* Declare GPIOs */

/* Pin configurations */
#define PIN_GPIO_0		8
#define PORT_GPIO_0		GPIOA
#define PIN_GPIO_1		2
#define PORT_GPIO_1		GPIOB
#define PIN_GPIO_2		8
#define PORT_GPIO_2		GPIOB
#define PIN_GPIO_3		9
#define PORT_GPIO_3		GPIOB
#define PIN_GPIO_4		10
#define PORT_GPIO_4		GPIOB
#define PIN_GPIO_5		11
#define PORT_GPIO_5		GPIOB
#define PIN_GPIO_6		12
#define PORT_GPIO_6		GPIOB
#define PIN_GPIO_7		13
#define PORT_GPIO_7		GPIOB
#define PIN_GPIO_8		14
#define PORT_GPIO_8		GPIOB
#define PIN_GPIO_9		15
#define PORT_GPIO_9		GPIOB
#define PIN_GPIO_10		14
#define PORT_GPIO_10	GPIOC
#define PIN_GPIO_11		15
#define PORT_GPIO_11	GPIOC

#define NUM_OF_GPIOS (sizeof(gpios)/sizeof(gpios[0]))
struct dev_gpio gpios[] = {
		DECLARE_GPIO_PIN(&gpio_module, PORT_GPIO_0, PIN_GPIO_0, GPIO_DIR_OUT, 0),
		DECLARE_GPIO_PIN(&gpio_module, PORT_GPIO_1, PIN_GPIO_1, GPIO_DIR_OUT, 0),
		DECLARE_GPIO_PIN(&gpio_module, PORT_GPIO_2, PIN_GPIO_2, GPIO_DIR_OUT, 0),
//		DECLARE_GPIO_PIN(&gpio_module, PORT_GPIO_3, PIN_GPIO_3, GPIO_DIR_OUT, 0),
//		DECLARE_GPIO_PIN(&gpio_module, PORT_GPIO_4, PIN_GPIO_4, GPIO_DIR_OUT, 0),
//		DECLARE_GPIO_PIN(&gpio_module, PORT_GPIO_5, PIN_GPIO_5, GPIO_DIR_OUT, 0),
//		DECLARE_GPIO_PIN(&gpio_module, PORT_GPIO_6, PIN_GPIO_6, GPIO_DIR_OUT, 0),
//		DECLARE_GPIO_PIN(&gpio_module, PORT_GPIO_7, PIN_GPIO_7, GPIO_DIR_OUT, 1),
//		DECLARE_GPIO_PIN(&gpio_module, PORT_GPIO_8, PIN_GPIO_8, GPIO_DIR_OUT, 1),
//		DECLARE_GPIO_PIN(&gpio_module, PORT_GPIO_9, PIN_GPIO_9, GPIO_DIR_OUT, 0),
//		DECLARE_GPIO_PIN(&gpio_module, PORT_GPIO_10, PIN_GPIO_10, GPIO_DIR_OUT, 0),
//		DECLARE_GPIO_PIN(&gpio_module, PORT_GPIO_11, PIN_GPIO_11, GPIO_DIR_OUT, 0),
};


/* Declare ADCs */
#define NUM_OF_ADCS (sizeof(adc_ch)/sizeof(adc_ch[0]))
struct dev_adc adc_ch[] = {
		DECLARE_ADC_CH(&adc_module, ADC_Channel_0, 1),
		DECLARE_ADC_CH(&adc_module, ADC_Channel_1, 0),
//		DECLARE_ADC_CH(&adc_module, ADC_Channel_4, 0),
//		DECLARE_ADC_CH(&adc_module, ADC_Channel_5, 0),
//		DECLARE_ADC_CH(&adc_module, ADC_Channel_6, 0),
//		DECLARE_ADC_CH(&adc_module, ADC_Channel_7, 0),
//		DECLARE_ADC_CH(&adc_module, ADC_Channel_8, 0),
//		DECLARE_ADC_CH(&adc_module, ADC_Channel_9, 0),
};

DECLARE_SPI_CHANNEL(spi_ch1, NULL, 1);

DECLARE_UART_DEV(dbg_uart, NULL, USART1, 115200, 1024, 10, 1, uart1_rx_parser);
DECLARE_UART_DEV(dbg_uart2, NULL, USART2, 115200, 512, 10, 1, NULL);

#endif

enum {
	PD_ERR_NULL_INPUT = 1,
	PD_ERR_PROBE_ERR = 2,
	PD_ERR_NOT_FOUND = 3,
};


static inline struct platform_device * platform_dev_find(char * name, enum en_device_type type)
{
	if (!list_empty(&platform_dev_list)) {
		struct platform_device * dev = NULL;
		list_for_each_entry(dev, &platform_dev_list, list) {
			if (!memcmp(name, dev->name, strlen(name)) && (dev->type == type)) {
				/* found */
				return(dev);
			}
		} //:~ foreach
	}
	return NULL;
}


int platform_dev_probe(char * name, enum en_device_type type, void * dev_data)
{
	if (!dev_data || !name) return -PD_ERR_NULL_INPUT;

	TRACE(("Adding device \'%s\' with name: %s\n", dev_type_names[type], name));

	struct platform_device * dev = (struct platform_device *) malloc(sizeof(struct platform_device));

	strncpy(dev->name, name, NAME_STR_LENGTH);
	dev->type = type;

	if (dev->type == DEV_TYPE_LED) {
		struct dev_led * led = (struct dev_led*)malloc(sizeof(struct dev_led));
		memcpy(led, dev_data, sizeof(struct dev_led));
		led->owner = &led_module;

		dev->data = dev_led_probe(led);
		if (!dev_data) goto err;
	}
	else if (dev->type == DEV_TYPE_GPIO) {
		struct dev_gpio * gpio = (struct dev_gpio*)malloc(sizeof(struct dev_gpio));
		memcpy(gpio, dev_data, sizeof(struct dev_gpio));
		gpio->owner = &gpio_module;

		dev->data = dev_gpio_probe(gpio);
		if (!dev_data) goto err;
	}
	else if (dev->type == DEV_TYPE_ADC) {
		struct dev_adc * adc = (struct dev_adc*)malloc(sizeof(struct dev_adc));
		memcpy(adc, dev_data, sizeof(struct dev_adc));
		adc->owner = &adc_module;

		dev->data = dev_adc_probe(adc);
		if (!dev_data) goto err;
	}
	else if (dev->type == DEV_TYPE_SPI) {
		struct dev_spi * spi = (struct dev_spi*)malloc(sizeof(struct dev_spi));
		memcpy(spi, dev_data, sizeof(struct dev_spi));
		spi->owner = &spi_module;

		dev->data = dev_spi_probe(spi);
		if (!dev_data) goto err;
	}
	else if (dev->type == DEV_TYPE_UART) {
		struct dev_uart * uart = (struct dev_uart*)malloc(sizeof(struct dev_uart));
		memcpy(uart, dev_data, sizeof(struct dev_uart));
		uart->owner = &uart_module;

		dev->data = dev_uart_probe(uart);
		if (!dev_data) goto err;
	}

	INIT_LIST_HEAD(&dev->list);
	list_add(&dev->list, &platform_dev_list);

	return 0;
err:
	TRACE(("Platform probe error.\n"));
	free(dev);
	return -PD_ERR_PROBE_ERR;
}

int platform_dev_remove(char * name, enum en_device_type type)
{
	if (!name) return -PD_ERR_NULL_INPUT;
	struct platform_device * dev = platform_dev_find(name, type);
	if (!dev) return -PD_ERR_NOT_FOUND;

	list_del_init(&dev->list);
	if (dev->type == DEV_TYPE_LED) {
		if (dev->data) {
			dev_led_remove((struct dev_led *)dev->data);
			free(dev);
		}
	}
	else if (dev->type == DEV_TYPE_GPIO) {
		if (dev->data) {
			dev_gpio_remove((struct dev_gpio *)dev->data);
			free(dev);
		}
	}
	else if (dev->type == DEV_TYPE_ADC) {
		if (dev->data) {
			dev_adc_remove((struct dev_adc *)dev->data);
			free(dev);
		}
	}
	else if (dev->type == DEV_TYPE_SPI) {
		if (dev->data) {
			dev_spi_remove((struct dev_spi *)dev->data);
			free(dev);
		}
	}
	else if (dev->type == DEV_TYPE_UART) {
		if (dev->data) {
			dev_uart_remove((struct dev_uart *)dev->data);
			free(dev);
		}
	}
	return 0;
}


void platform_dev_init(void)
{
	INIT_LIST_HEAD(&platform_dev_list);
	INIT_LIST_HEAD(&dev_timer_list);

	RCC_PCLK1Config(RCC_HCLK_Div2);
	RCC_PCLK2Config(RCC_HCLK_Div1);

	usb_comm_0.buffer = &usb_0_comm_buffer;
	usb_comm_1.buffer = &usb_1_comm_buffer;
	USB_dev_init(&usb_comm_0);
	USB_dev_init(&usb_comm_1);

	/* uarts */
	dev_uart_module_init(&uart_module);

	/* led status */
	dev_led_module_init(&led_module);
	dev_timer_add((void*) &led_module, led_module.tick_ms, (void*) &dev_led_update, &dev_timer_list);

	// platform_dev_probe("LED.C13", DEV_TYPE_LED, (void*) &led_status);
	// dev_led_set_pattern(&led_status, LED_PATTERN_IDLE);


	/* Initialize GPIOs */
	dev_gpio_module_init(&gpio_module);
	dev_timer_add((void*) &gpio_module, gpio_module.tick_ms, (void*) &dev_gpio_update, &dev_timer_list);

	/* Initialize ADCs */
	dev_adc_module_init(&adc_module);
	dev_timer_add((void*) &adc_module, adc_module.tick_ms, (void*) &dev_adc_update, &dev_timer_list);

#ifdef DEBUG_STM32F103
	dbg_uart.owner = &uart_module;
	platform_dev_probe("UART1", DEV_TYPE_UART, &dbg_uart);
	dbg_uart2.owner = &uart_module;
	platform_dev_probe("UART2", DEV_TYPE_UART, &dbg_uart2);

	int i = 0;
	/* Add devices */
	for(i=0; i<NUM_OF_GPIOS; i++) {
		char name[20];
		snprintf(name, 20, "GPIO.%d", i);
		platform_dev_probe(name, DEV_TYPE_GPIO, (void*) &gpios[i]);
	}
	TRACE(("Number of GPIOS: %d\n", NUM_OF_GPIOS));

	for(i=0; i<NUM_OF_ADCS; i++) {
		char name[20];
		snprintf(name, 20, "ADC.%d", i);
		platform_dev_probe(name, DEV_TYPE_ADC, (void*) &adc_ch[i]);
	}
	TRACE(("Number of ADCs: %d\n", NUM_OF_ADCS));

	spi_ch1.owner = &spi_module;
	platform_dev_probe("SPI.1", DEV_TYPE_SPI, &spi_ch1);
#endif

	dev_timer_add((void*) &uart_module, uart_module.tick_ms, (void*) &dev_uart_update, &dev_timer_list);

//	test_cmd_parser("LED_DEV_ADD");
// 	test_cmd_parser("LED_DEV_REMOVE");
//	test_cmd_parser("LED_DEV_ADD");
}


void platform_dev_polling(void)
{
	USB_update_timers();	// update USB timers

	dev_timer_polling(&dev_timer_list);

	IWDG_ReloadCounter();
	GPIOB->ODR ^= GPIO_Pin_11;
}

#ifdef BINARY_COMMS
/**
 * I've created a cmd_parser.inc file for the parsing functions
 * to make easier to read code.
 */
#include "cmd_parser.inc"
#else

enum {
	SENDER_USB0,
	SENDER_USB1,
	SENDER_UART1,
	SENDER_UART2
};


#include "text_cmd_parser.inc"

void usb_0_rx_parser(uint8_t * recv_buffer, uint16_t recv_len)
{
	TRACE(("USB0_recv: %d\n", recv_len));
}


void usb_1_rx_parser(uint8_t * recv_buffer, uint16_t recv_len)
{
	TRACE(("USB1_recv: %d\n", recv_len));
}



void uart1_rx_parser(struct dev_uart * uart, uint8_t *buffer, size_t bufferlen)
{
	text_cmd_parser(buffer, bufferlen, SENDER_UART1, (void*) uart);
}


void uart2_rx_parser(struct dev_uart * uart, uint8_t *buffer, size_t SENDER_UART2)
{

}


#endif