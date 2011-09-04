/*----------------------------------------------------------------------------
 * Name:    IRQ.C
 * Purpose: IRQ Handler
 * Note(s):
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2009 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------*/

#include "LPC122x.h"                    /* LPC122x definitions                */


volatile unsigned short AD_last;        /* Last converted value               */
volatile unsigned char  clock_1s;       /* Flag activated each second         */
volatile unsigned long	ticks;
/*----------------------------------------------------------------------------
  SysTick IRQ: Executed periodically (10ms) 
 *----------------------------------------------------------------------------*/
void SysTick_Handler (void) 
{
	ticks++;
}

/*----------------------------------------------------------------------------
  A/D IRQ: Executed when A/D Conversion is done 
 *----------------------------------------------------------------------------*/
void ADC_IRQHandler(void) {
  unsigned int ADC_reg;

  ADC_reg = LPC_ADC->STAT;              /* Read ADC status clears interrupt   */

  ADC_reg = LPC_ADC->GDR;               /* Read conversion result             */
  AD_last = (ADC_reg >> 6) & 0x3FF;     /* Store converted value              */
}
