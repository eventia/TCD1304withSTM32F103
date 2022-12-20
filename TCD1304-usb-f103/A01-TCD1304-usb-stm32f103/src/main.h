#ifndef MAINH
#define MAINH


/* Data definitions */
#define CCDSize 3694
#define RxDataSize 12
#define USB_DATA_SIZE 64
#define MAX_PACKET_SIZE_EP1IN  64

#include <stm32f10x.h>

/* CCD master clock in Hz */
/* ##### Yet to be evaluated with the STM32F103 #####
   The values presented here are the prescalable frequencies between
   1-2MHz with the STM32F401RE running at 84 MHz. Other frequencies
   are achievable when prescaling APB1 differently. 1 MHz does not
   seem to work well. */
#define CCD_fm 800000
//#define CCD_fm 1000000
//#define CCD_fm 2000000

/*  Comply with the CCD's timing requirements:
	The delay is dependent on the CCD_fM. These values appear to work:
		For 2.0 MHz use: ICG_delay = 10 and SH_delay = 11
		For 1.0 MHz use: ICG_delay = 11 and SH_delay = 11
		For 800 kHz use: ICG_delay = 11 and SH_delay = 11   */
#define SH_delay 11
#define ICG_delay 11
#define fm_delay 3

#include "timer_conf.h"
#include "ADC_conf.h"




#endif
