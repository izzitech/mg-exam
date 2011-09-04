/*----------------------------------------------------------------------------
 * Name:    Blinky.c
 * Purpose: LED Flasher for MCB1200
 * Note(s):
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2010 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include <stdio.h>
#include "LPC122x.h"                            /* LPC122x definitions        */
#include "ssp.h"
#include "gpio.h"
#include <RICReg.h>
#include <MfRc500.h>
#include <PICCCmdConst.h>
#include <MfErrNo.h>

#define LED_NUM     8                           /* Number of user LEDs        */
const unsigned long led_mask[] = {1UL << 0, 1UL << 1, 1UL << 2, 1UL << 3, 
                                  1UL << 4, 1UL << 5, 1UL << 6, 1UL << 7 };

/* Import external functions from Serial.c file                               */
extern void SER_init (void);

/* Import external variables from IRQ.c file                                  */
extern volatile unsigned short AD_last;
extern volatile unsigned char  clock_1s;
extern volatile unsigned long  ticks;

/* variable to trace in LogicAnalyzer (should not read to often)              */
volatile unsigned short AD_dbg; 

unsigned long SysTick_Get(void)
{
	return ticks;
}         

/*----------------------------------------------------------------------------
  Function that initializes ADC
 *----------------------------------------------------------------------------*/
void ADC_init (void) {

  /* configure PIN GPIO0.30 for AD0 */
  LPC_SYSCON->SYSAHBCLKCTRL |= ((1UL << 31) |   /* enable clock for GPIO0     */
                                (1UL << 16) );  /* enable clock for IOCON     */

  LPC_IOCON->PIO0_30   = ((3UL <<  0) |         /* P0.30 is AD0               */
                          (0UL <<  4) |         /* PullUp disabled            */
                          (0UL <<  7) );        /* Analog mode enablde        */

  LPC_GPIO0->DIR &= ~(1UL << 30);               /* configure GPIO as input    */

  /* configure ADC */
  LPC_SYSCON->PDRUNCFG      &= ~(1UL <<  4);    /* Enable power to ADC block  */
  LPC_SYSCON->SYSAHBCLKCTRL |=  (1UL << 14);    /* Enable clock to ADC block  */

  LPC_ADC->CR          =  ( 1UL <<  0) |        /* select AD0 pin             */
                          (23UL <<  8) |        /* ADC clock is 24MHz/24      */
                          ( 1UL << 21);         /* enable ADC                 */ 

  LPC_ADC->INTEN       =  ( 1UL <<  8);         /* global enable interrupt    */

  NVIC_EnableIRQ(ADC_IRQn);                     /* enable ADC Interrupt       */

}

void init_mfrc500(void)
{
	int i;
	LPC_IOCON->PIO0_4 &= ~0x07;		/* SSP SSEL is a GPIO pin */
	/* port0, bit 15 is set to GPIO output and high */
	GPIOSetDir( PORT0, 4, 1 );
	GPIOSetValue( PORT0, 4, 1 );
	for(i=0; i<500000; i++);
	GPIOSetValue( PORT0, 4, 0 );
}

void reset_mfrc500(void)
{
	GPIOSetValue( PORT0, 4, 1 );
}

void unreset_mfrc500(void)
{
	GPIOSetValue( PORT0, 4, 0 );
}

extern void SingleResponseIsr(void);

void PIOINT0_IRQHandler(void)
{
  uint32_t regVal;

  regVal = GPIOIntStatus( PORT0, 7 );
  if ( regVal )
  {
	SingleResponseIsr();
	GPIOIntClear( PORT0, 7 );
  }		
  return;
}

// interrupter in PIO
void init_eint(void)
{
//P0.7
  GPIO_SetIOCONFunc(PORT0, 7);	
  /* 使用Port0.7作为外部中断输入测试 */
  GPIOSetDir( PORT0, 7, 0 );
  /* P0.7外部中断单端触发、高电平有效 */
  GPIOSetInterrupt( PORT0, 1, 7, 0, 0 );
  GPIOIntEnable( PORT0, 7 );
}

unsigned char disptemp[16];

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) 
{   
	unsigned long status, current_tick, last_tick, i, j;  
	unsigned char data[5];
	unsigned char temp;
	                      
//	SysTick_Config(SystemCoreClock/1000);          /* Generate IRQ each 1 ms    */

	init_mfrc500();
  	
	SER_init();                                   /* UART#1 Initialization      */
	SSP_IOConfig();
	SSP_Init();

//	init_eint();

	printf("Welcome\r\n");

	if(PcdReset() != MI_OK);
	{
		printf("PcdReset Fail\r\n");
		while(1);
	}

	while(1);
}
