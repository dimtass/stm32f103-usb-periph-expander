/*
 * main.c
 *
 *  Created on: 8 May 2018
 *      Author: Dimitris Tassopoulos
 */

#include "dev_uart.h"
#include "platform_config.h"
#include "usb_lib.h"
#include "usb_regs.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "comm_buffer.h"
#include "platform_devices.h"


/* Declare glb struct and initialize buffers */
struct tp_glb glb;

static inline void main_loop(void)
{
	/* 1 ms timer */
	if (glb.tmr_1ms) {
		glb.tmr_1ms = 0;
		platform_dev_polling();
//		IWDG_ReloadCounter();
	}
	/* USB reception is done by polling,
	 * but normally the interrupt reception
	 * is done in EPx_OUT_Callback
	 * */
	if (bDeviceState == CONFIGURED) {
		USB_dev_recv();
	}
}

int main(void)
{
	if (IVECT_TAB_OFFSET == 0x0) {
		if (SysTick_Config(SystemCoreClock / 1000)) {
			/* Capture error */
			while (1);
		}
	}
	/* debug pin */

	IWDG_ReloadCounter();
	IWDG_Configuration();

	BKP_Configuration();
	/* Reset re-tries counter */
	BKP_WriteBackupRegister(BKP_DR2, 0);

	/* System Clocks Configuration */
	RCC_Configuration();
	/* NVIC Configuration */
	NVIC_Configuration();

	set_trace_level(
			0
			| TRACE_LEVEL_DEFAULT
//			| TRACE_LEVEL_USB
			| TRACE_LEVEL_GPIO
			| TRACE_LEVEL_ADC
//			| TRACE_LEVEL_I2C
//			| TRACE_LEVEL_SPI
//			| TRACE_LEVEL_USART
			| TRACE_LEVEL_COMM
			| TRACE_LEVEL_PARSER
			,1);

	/* USB Configuration */
	USB_Configuration();
	USB_Init();

	{
		GPIO_InitTypeDef GPIO_InitStructure;
		/* GPIOD Periph clock enable */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

		/* Configure PD0 and PD2 in output pushpull mode */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	}

	platform_dev_init();

	TRACE(("Application started...\n"));

	while(1) {
		main_loop();
	}
}
