/*-
 * Copyright (c) 2019 Esben Rossel
 * All rights reserved.
 *
 * Author: Esben Rossel <esbenrossel@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "main.h"


void GPIO_conf(void);
void flush_CCD(void);
void NVIC_conf(void);
void send_a_transaction(void);
void block_transfer( uint8_t * buf, uint16_t len );

uint8_t Rxbuffer[64]; 

uint8_t * USB_Tx_ptr;
uint16_t   USB_Tx_length = 0;

__IO uint32_t SH_period = 25;
__IO uint32_t ICG_period = 65000;

__IO uint8_t pulse_counter = 0;
__IO uint8_t CCD_flushed = 0;
__IO uint8_t change_exposure_flag = 0;

__IO uint16_t aTxBuffer[CCDSize];
__IO uint16_t avgBuffer[CCDSize] = {0};

__IO uint8_t avg_exps = 0;
__IO uint8_t exps_left = 0;
__IO uint8_t data_flag = 0;
__IO uint8_t coll_mode = 0;



/* GPIOs for the TCD1304 are:
    PA1	- OS (ADC-in)
    PA15- fM   TIM2 C1 
    PB4 - SH   TIM3 C1 
    PA10- ICG  TIM1 C3 */

int main(void)
{
  Set_System();
  Set_USBClock();
  USB_Interrupts_Config();
  USB_Init();
  

	/* Enable GPIOA, GPIOB and GPIOC clocks */
	GPIO_conf();
	NVIC_conf();

	TIM_CCD_fM_conf();	
	TIM_ADC_conf();

	ADC1_conf();

	TIM_SH_ICG_conf(SH_period, ICG_period);
	
	int i = 0;
  while (1)
  {

		if (change_exposure_flag == 1)
		{
			/* reset flag */
			change_exposure_flag = 0;

			flush_CCD();

			/* set new integration time - handled in usb_endp.c */
	//		ICG_period = nRxBuffer[6]<<24|nRxBuffer[7]<<16|nRxBuffer[8]<<8|nRxBuffer[9];
	//		SH_period = nRxBuffer[2]<<24|nRxBuffer[3]<<16|nRxBuffer[4]<<8|nRxBuffer[5];

			/*	Disable ICG (TIM1) and SH (TIM3) before reconfiguring*/
			TIM_Cmd(TIM1, DISABLE);
			TIM_Cmd(TIM3, DISABLE);

			/* 	Reconfigure TIM2 and TIM5 */
			TIM_SH_ICG_conf(SH_period, ICG_period);

			//GPIOC->ODR ^= GPIO_Pin_13;
			//block_transfer((uint8_t*)aTxBuffer, sizeof(aTxBuffer));
		}
		switch (data_flag){
		case 1:
			/* reset flags */
			data_flag = 0;

            if (coll_mode == 1)
				pulse_counter=5;

			//GPIOC->ODR ^= GPIO_Pin_13;
			/* Transmit data in aTxBuffer */
			block_transfer((uint8_t*)aTxBuffer, sizeof(aTxBuffer));
			break;		

		case 2:
			/* reset flags */
			data_flag = 0;

			/* This is the first integration of several so overwrite avgBuffer */
			for (i=0; i<CCDSize; i++)
				avgBuffer[i] = aTxBuffer[i];	
			break;

		case 3:
			/* reset flags */
			data_flag = 0;

			/* Add new to previous integrations.
			   This loop takes 3-4ms to complete. */		
			for (i=0; i<CCDSize; i++)
				avgBuffer[i] = avgBuffer[i] + aTxBuffer[i];		
			break;

		case 4:
			/* reset flags */
			data_flag = 0;

			/* Add new to previous integrations.
			   This loop takes 3-4ms to complete. */		
			for (i=0; i<CCDSize; i++)
				avgBuffer[i] = avgBuffer[i] + aTxBuffer[i];		

			/* Store average values in aTxBuffer */
			for (i=0; i<CCDSize; i++)
				aTxBuffer[i] = avgBuffer[i]/avg_exps;

			/* if continuous mode, restart collection */
			if (coll_mode == 1){
				exps_left = avg_exps;			
				pulse_counter = 5;
			}

			/* Transmit data in aTxBuffer */
			block_transfer((uint8_t*)aTxBuffer, sizeof(aTxBuffer));
			break;
		}

  }
}




/* 	To keep noise-level on ADC-in down, the following GPIO's are
	set as output, driven low and physically connected to GND:
		 */
void GPIO_conf(void)
{
	GPIO_InitTypeDef    	GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

	/* 	Clock the GPIOs */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);


	/* Drive these low */
	/* PA0, PA2, PA7, PA12, PB0, PB3, PB10 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_7 | GPIO_Pin_9 | GPIO_Pin_11 |  GPIO_Pin_12;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_3 | GPIO_Pin_10;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	/* LED is on PC13 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	/* turn it off */
    GPIOC->ODR ^= GPIO_Pin_13;


 	/* Configure pins for SH (PB4), ICG (PA10) and MCLK (PA15)
	and TIM4 PB9 */
	/* Remap GPIOs for TIM3 on PB4, TIM2 on PA15 */
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
	GPIO_PinRemapConfig(AFIO_MAPR_TIM3_REMAP_PARTIALREMAP, ENABLE);
	GPIO_PinRemapConfig(AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_15;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_4 | GPIO_Pin_9;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	/*	Configure ADC1 Channel1 pin (PA1) as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}


/* Run this function prior to datacollection */
void flush_CCD()
{
	/* Set exposure very low */
	//ICG_period = 15000;
	//SH_period = 20;

	/*	Disable ICG (TIM1) and SH (TIM3) before reconfiguring*/
	TIM_Cmd(TIM1, DISABLE);
	TIM_Cmd(TIM3, DISABLE);

	/*	Reset flags and counters */
	CCD_flushed = 0;
	pulse_counter = 0;

	/* 	Reconfigure TIM2 and TIM5 */
	TIM_SH_ICG_conf(20, 15000);

	/*	Block until CCD is properly flushed */
	while(CCD_flushed == 0);
}


/* Configure interrupts */
void NVIC_conf(void)
{
	NVIC_InitTypeDef		NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* ICG (TIM1) IRQ */
	/* The TIM1 update interrupts starts TIM4 and ADC */
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* ADC-DMA IRQ */
	/* DMA1 Transfer complete interrupt stops TIM4 and ADC */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


}

void send_a_transaction(void)
{
  uint16_t send_len;

  if ( USB_Tx_length ) {
    send_len = (USB_Tx_length < MAX_PACKET_SIZE_EP1IN ) ? USB_Tx_length : MAX_PACKET_SIZE_EP1IN;

    UserToPMABufferCopy(USB_Tx_ptr, ENDP1_TXADDR, send_len);
    SetEPTxCount(ENDP1, send_len);
    SetEPTxValid(ENDP1);

    USB_Tx_ptr    += send_len;
    USB_Tx_length -= send_len;
  }
}

void block_transfer( uint8_t * buf, uint16_t len )
{
	if ( len == 0 ) return;

	USB_Tx_ptr    = buf;
	USB_Tx_length = len;

	send_a_transaction();
}
