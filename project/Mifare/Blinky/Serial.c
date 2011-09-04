/*----------------------------------------------------------------------------
 * Name:    Serial.c
 * Purpose: Low level serial routines
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

#include "LPC122x.h"                    /* LPC122x definitions                */


/*----------------------------------------------------------------------------
  Initialize UART0 pins, Baudrate
 *----------------------------------------------------------------------------*/
void SER_init (void) {
   
  /* configure PINs GPIO0.1, GPIO0.1 for UART0 */
  LPC_SYSCON->SYSAHBCLKCTRL |= ((1UL << 31) |   /* enable clock for GPIO0     */
                                (1UL << 16) );  /* enable clock for IOCON     */

  LPC_IOCON->PIO0_1  =  (2UL <<  0);            /* P0.1 is RxD0               */
  LPC_IOCON->PIO0_2  =  (2UL <<  0);            /* P0.2 is TxD0               */

  /* configure UART0 */
  LPC_SYSCON->SYSAHBCLKCTRL |=  (1UL << 12);    /* Enable clock to UART0      */
  LPC_SYSCON->UART0CLKDIV    =  (2UL <<  0);    /* UART0 clock =  CCLK / 2    */

  LPC_UART0->LCR = 0x83;                  /* 8 bits, no Parity, 1 Stop bit    */
  LPC_UART0->DLL = 4;                     /* 115200 Baud Rate @ 12.0 MHZ PCLK */
  LPC_UART0->FDR = 0x85;                  /* FR 1.627, DIVADDVAL 5, MULVAL 8  */
  LPC_UART0->DLM = 0;                     /* High divisor latch = 0           */
  LPC_UART0->LCR = 0x03;                  /* DLAB = 0                         */
}


/*----------------------------------------------------------------------------
  Write character to Serial Port
 *----------------------------------------------------------------------------*/
int sendchar (int c) {

  while (!(LPC_UART0->LSR & 0x20));
  LPC_UART0->THR = c;

  return (c);
}


/*----------------------------------------------------------------------------
  Read character from Serial Port   (blocking read)
 *----------------------------------------------------------------------------*/
int getkey (void) {

  while (!(LPC_UART0->LSR & 0x01));
  return (LPC_UART0->RBR);
}
