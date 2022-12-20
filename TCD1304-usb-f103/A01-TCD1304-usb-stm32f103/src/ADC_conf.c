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

extern __IO uint16_t aTxBuffer[CCDSize];


/* ADC1 - Input on Ch10 (PA1)  - Triggered by TIM4 Ch4 - DMA2 Ch0 Stream0 */
void ADC1_conf()
{
	ADC_InitTypeDef       ADC_InitStructure;
	DMA_InitTypeDef       DMA_InitStructure;

	/* ADCCLK = PCLK2/6 */
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);

	/* Clock DMA2 and ADC1 */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* DMA1 Channel1 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel1);
    DMA_ClearITPendingBit(DMA1_IT_TC4);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;//(uint32_t)TIM1_CCR1_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&aTxBuffer;//(uint32_t)ADC1_DR_Address;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = CCDSize;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	/* Enable DMA1 Channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);

	/*	DMA1 interrupt configuration */
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv =  ADC_ExternalTrigConv_T4_CC4;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 RegularChannelConfig Test */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_7Cycles5);

	ADC_ExternalTrigConvCmd(ADC1, ENABLE);
	/*	Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/*	Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

/* Enable ADC1 reset calibration register */
    ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibration */
	ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));

}


/*	TIM2 (fM) and TIM4 (ADC-clock) are not in phase. If you suspect the ADC is 
reading across two pixels, try and increase TIM1 pulse with 1. This will
shift the pixel readings one period in relation to fM (and 1/4 of a period of
the ADC-clock). */

/*	Since the pixel values come out at CCD_fM / 4 the maximum time the ADC has for 
	each conversion is (with CCD_fM = 800 kHz):
		t_pix = (0.20 MHz)⁻¹ = 5.0 µs
	With a systemcoreclock of 72 MHz the ADC completes a conversion in 1.17 µs. 
	The maximum conversion rate is achieved with a systemcoreclock of 56 MHz, but
	USB	requires a 48 MHz clock, and that's unattainable with 56 MHz.

	At 72 MHz the ADC runs with 12 MHz, and so a sampletime of 0.83 µs corresponds
	to this number of ADC-clock cycles:
		4.83 µs · 12 MHz = 57

	The available sampling times are:
	1.5, 7.5, 13.5, 28.5, 41.5, 55.5, 71.5 and 239.5 clock cycles.
	
	So 55.5 is the longest possible sampletime. However, setting it to 7.5 lets
	the user recompile the firmware with a CCD_fm of up to 2.0 MHz, without having
	to think about ADC sampling time. */




