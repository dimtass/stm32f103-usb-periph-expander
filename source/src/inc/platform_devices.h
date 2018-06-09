/*
 * platform_devices.h
 *
 *  Created on: 13 May 2018
 *      Author: Dimitris Tassopoulos
 */

#ifndef PLATFORM_DEVICES_H_
#define PLATFORM_DEVICES_H_

#include "platform_config.h"
#include "usb_lib.h"
#include "usb_pwr.h"
#include "usb_regs.h"
#include "usb_desc.h"
#include "dev_timer.h"
#include "dev_led.h"
#include "dev_gpio.h"
#include "dev_adc.h"
#include "dev_usb_uart.h"
#include "dev_spi.h"
#include "comm_protocol.h"
#include "common_tools.h"

#define DEBUG_STM32F103

void platform_dev_init(void);

void platform_dev_polling(void);


void test_cmd_parser(char * test_name);

#endif /* PLATFORM_DEVICES_H_ */
