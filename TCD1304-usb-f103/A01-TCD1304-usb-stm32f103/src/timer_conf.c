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

#include "main.h"


/* fM is served by TIM2 C1 on PA15 */  
void TIM_CCD_fM_conf(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* Clock TIM2 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/*	TIM2 Time base configuration 
	Prescaler is 1 so timer clock is 72 MHz / 1 
	Period is 72 MHz / CCD_fm  */
	TIM_TimeBaseStructure.TIM_Prescaler = 1 - 1;	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 72000000/CCD_fm - 1;	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/*	TIM2 PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 72000000/CCD_fm/2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM2, ENABLE);

	/*	TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}


/* ADC is paced by TIM4 */
void TIM_ADC_conf(void)
{
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef		TIM_OCInitStructure;

	/* Clock TIM4 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);


	/* Output on PB9 - optional */
	GPIO_InitTypeDef    	GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOB, &GPIO_InitStructure); 

	/*	TIM4 Time base configuration */
	/*  Prescaler is 1 so timer clock is identical to TIM2's clock */
	TIM_TimeBaseStructure.TIM_Prescaler = 1 - 1;	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 4 * 72000000/CCD_fm - 1;	
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	/*	Pulse is apb1_freq / (2*CCD_fm), so the duty cycle is 12.5% */
	/*	TIM4 PWM1: Channel4 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 2*72000000/CCD_fm - 1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM4, ENABLE);

	/*	TIM4 is enabled/disabled by IRQs */
	//TIM_Cmd(TIM4, ENABLE);
}



/* As the STM32F103 does not have any 32-bit timers, and only 4 16-timers
in total, it's not possible to dedicate two timers to both SH and ICG, as
fM and the ADC-trigger both need each their own timer.

To compensate for the limited amount of timers SH and ICG are served by
TIM1 and TIM3, in different configurations depending on the requirements
for the pulses.

1. Integration time (SH) is longer than 7.4 ms:
      TIM1 and TIM3 are chained to give a single 32-bit timer. TIM1 has
   complementary outputs, and C3 serves ICG, while CN3 serves SH. With
   a sufficiently long dead time, the CCD's timing requirements are met.
   This means the CCD will work in normal mode ie. SH-period = ICG-period.
2. Integration time (SH) is shorter than 7.4 ms:
   	  SH and ICG are served by TIM3 and TIM1 respectively 

   SH is on PB1 (TIM1 CN3, or TIM3 C4) and ICG on PA10 (TIM1 C3) */
void TIM_SH_ICG_conf(uint32_t SH, uint32_t ICG)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	/* Reset timer registers */
	TIM_DeInit(TIM1);
	TIM_DeInit(TIM3);

	/* Clock TIM1 and TIM3 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

   if (SH > 65535) //do we need 32-bit resolution
   {
   		/*################################################################
	       Start of 32-bit timer configuration
		 This is uncompleted, so keep SH and ICG period below 2^16.

	     The initial idea was to chain TIM3+TIM1 and use the complementary
		 outputs of TIM1 to generate both SH and ICG, with a delay defined
		 by dead time inserting. It's not possible to do this AND get 
		 sufficiently short pulses for SH and ICG.
 			Instead here is the configuration of TIM3+TIM1 to give a 32-bit
		 timer. The toggling of pins can be handled in TIM1 update interrupt
		 in stm32_it.c (but I've not done so).
		 
		 The firmware assumes that ICG and SH periods are identical ie. the
		 CCD cannot be run in electronic shutter mode for long integrations. 
         The reason is the limited amount of timers on the STM32F103. If you
         require electronic shutter mode with long integrations, use the
         STM32F401 or STM32F405.		
    	 #################################################################*/

	
		/* TIM3 has a period corresponding to the 16 LSB of the
    	32-bit timer (TIM1+TIM3) */
		TIM_TimeBaseStructure.TIM_Prescaler = 72000000 / CCD_fm - 1;	
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
		/* Watch out for boundary conditions ie when LSB is 0xFFFF or 0x0000 */
		TIM_TimeBaseStructure.TIM_Period = SH - 1;
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

		/* TIM1 has a period of corresponding to the 16 MSB of the
    	32-bit timer (TIM1+TIM3) */
		TIM_TimeBaseStructure.TIM_Period = (SH >> 16) -1;
		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_Pulse = 1;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OC3Init(TIM3, &TIM_OCInitStructure);

		/* Master Mode selection, TIM3 is master for TIM1 */
		TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_OC3Ref);
		TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);
		

		/*	TIM1 PWM1 Mode configuration: Channel1
		Consider if any outputs are needed */
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
		TIM_OCInitStructure.TIM_Pulse = 6;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

		TIM_OC3Init(TIM1, &TIM_OCInitStructure);

		/* Slave Mode selection: TIM1 */
		TIM_SelectInputTrigger(TIM1, TIM_TS_ITR2);
		TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Gated);

		/*	Clear TIM1 update pending flags */
		TIM_ClearFlag(TIM1, TIM_FLAG_Update);
		/*	Enable the TIM1 Interrupt. Interrupt routine is located in stm32f4xx_it.c */
		TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);



		/* TIM3 counter enable */
		TIM_Cmd(TIM3, ENABLE);
		/* TIM1 counter enable */
		TIM_Cmd(TIM1, ENABLE);


		/* Main Output Enable */
		TIM_CtrlPWMOutputs(TIM1, ENABLE);

    }
    else
	{
   		/*################################################################
	       Start of 16-bit timer configuration
    	 #################################################################*/

 		/*	TIM1 Time base configuration 
		Prescaler is calculated so timer clock is CCD_fm */
		TIM_TimeBaseStructure.TIM_Prescaler = 72000000 / CCD_fm - 1;	
		TIM_TimeBaseStructure.TIM_ClockDivision = 0;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

		/* TIM1 serves ICG  */
		TIM_TimeBaseStructure.TIM_Period = ICG - 1;
		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

		/* TIM3 serves SH */
		TIM_TimeBaseStructure.TIM_Period = SH - 1;
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

		/*	TIM1 PWM1 Mode configuration: Channel3
		Pulse is app 5 us, so TIM_Pulse is CCD_fm in MHz times 5. 
    	    The polarity of the following is under the condition that the signals
    	are inverted by a 74HC04D or similar, before reaching the CCD. */
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
		TIM_OCInitStructure.TIM_Pulse = (CCD_fm*5)/1000000;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

		TIM_OC3Init(TIM1, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
		TIM_ARRPreloadConfig(TIM1, ENABLE);

		/*	TIM3 PWM1 Mode configuration: Channel1
		Pulse is app 2 µs, so TIM_pulse is CCD_fm in MHz times 2. 
    	    The polarity of the following is under the condition that the signals
    	are inverted by a 74HC04D or similar, before reaching the CCD. */
		TIM_OCInitStructure.TIM_Pulse = (CCD_fm*2)/1000000;
		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;

		TIM_OC1Init(TIM3, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
		TIM_ARRPreloadConfig(TIM3, ENABLE);


		/*	Clear TIM1 update pending flags */
		TIM_ClearFlag(TIM1, TIM_FLAG_Update);
		/*	Enable the TIM1 Interrupt. Interrupt routine is located in stm32f4xx_it.c */
		TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);



		/* TIM1 counter enable */
		TIM_Cmd(TIM1, ENABLE);
		/* Main Output Enable */
		TIM_CtrlPWMOutputs(TIM1, ENABLE);
		/* TIM1 counter enable */
		TIM_Cmd(TIM3, ENABLE);

		/*	Set counters close to expiration, as the integration times may be very long. 
		(For example: with an ICG-period of 300s we'd have to wait 600s for two ICG-
	 	pulses if we don't cut the first one short.)
		The SH-period is slightly delayed to comply with the CCD's timing requirements. */
		TIM3->CNT = SH - SH_delay;// + (SH_period % 2);
		TIM1->CNT = ICG - ICG_delay;

	}
}


