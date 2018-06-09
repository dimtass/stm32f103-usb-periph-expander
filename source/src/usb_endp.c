/**
  ******************************************************************************
  * @file    usb_endp.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Endpoint routines
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "hw_config.h"
#include "usb_istr.h"
#include "usb_pwr.h"
#include "dev_usb_uart.h"

extern struct dev_usb_uart usb_comm_0;
extern struct dev_usb_uart usb_comm_1;

/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/

void EP1_IN_Callback(void)
{
   USB_Tx_IRQ(&usb_comm_0);
}


void EP4_IN_Callback(void)
{
	USB_Tx_IRQ(&usb_comm_1);
}

/*******************************************************************************
* Function Name  : EP3_OUT_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP3_OUT_Callback(void)
{
	/* Instead of handle the RX data inside the interrupt
	 * flag the reception and handle them in the main loop
	 */
	usb_comm_0.irq_rx = 1;
}

void EP6_OUT_Callback(void)
{
	/* Instead of handle the RX data inside the interrupt
	 * flag the reception and handle them in the main loop
	 */
	usb_comm_1.irq_rx = 1;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
