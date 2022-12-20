/**
  ******************************************************************************
  * @file    usb_endp.c
  * @author  MCD Application Team
  * @version V4.1.0
  * @date    26-May-2017
  * @brief   Endpoint routines
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Interval between sending IN packets in frame number (1 frame = 1ms) */
#define VCOMPORT_IN_FRAME_INTERVAL             5

/* Private macro -------------------------------------------------------------*/
extern __IO uint8_t data_flag;
extern __IO uint8_t change_exposure_flag;
extern __IO uint8_t avg_exps;
extern __IO uint8_t exps_left;
extern __IO uint8_t coll_mode;

extern __IO uint32_t SH_period;
extern __IO uint32_t ICG_period;

//extern uint8_t serial_tx_buffer[];
//extern uint8_t serial_tx_buffer_head;
//extern volatile uint8_t serial_tx_buffer_tail;

uint8_t USB_Rx_Buffer[VIRTUAL_COM_PORT_DATA_SIZE];
//extern  uint8_t USART_Rx_Buffer[];
//extern uint32_t USART_Rx_ptr_out;
//extern uint32_t USART_Rx_length;
//extern uint8_t  USB_Tx_State;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback (void)
{
	send_a_transaction();
}

/*******************************************************************************
* Function Name  : EP3_OUT_Callback
* Description    : When the board receives data over USB this callback is envoked.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP3_OUT_Callback(void)
{
  
    uint16_t USB_Rx_Cnt;
  
  /* Get the received data buffer and update the counter */
  USB_Rx_Cnt = USB_SIL_Read(EP3_OUT, USB_Rx_Buffer);

  /* Enable the receive of data on EP3 */
  SetEPRxValid(ENDP3);
  
  /* Do stuff here: */
  /* for example return what we just received */
  //USB_SIL_Write(EP1_IN, USB_Rx_Buffer, USB_Rx_Cnt);

  //SetEPTxValid(ENDP1);
  
	if ((USB_Rx_Buffer[0]==69)&&(USB_Rx_Buffer[1]==82))
	{
		/* set flags for main-loop */
		change_exposure_flag = 1;
		data_flag = 0;

		ICG_period = USB_Rx_Buffer[6]<<24 | USB_Rx_Buffer[7]<<16 | USB_Rx_Buffer[8]<<8 | USB_Rx_Buffer[9];
		SH_period  = USB_Rx_Buffer[2]<<24 | USB_Rx_Buffer[3]<<16 | USB_Rx_Buffer[4]<<8 | USB_Rx_Buffer[5];


		/* disable averaging by default */
		avg_exps = 1;

		/* continous or one-shot mode? */
        coll_mode = USB_Rx_Buffer[10];		

		/* check if user averaging-request is valid */
		if ((USB_Rx_Buffer[11]<16)&&(USB_Rx_Buffer[11]>0))
			avg_exps = USB_Rx_Buffer[11];

			exps_left = avg_exps;

			/* Resend the received data */
			//VCP_DataTx(Buf,2);
	}

}


/*******************************************************************************
* Function Name  : SOF_Callback / INTR_SOFINTR_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SOF_Callback(void)
{
	if(bDeviceState == CONFIGURED)
	{
		//EP1_IN_Callback();
	} 
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

