/****************************************************************************
 *   $Id:: ssp.c 3737 2010-06-24 02:17:30Z usb00423                         $
 *   Project: NXP LPC122x SSP example
 *
 *   Description:
 *     This file contains SSP code example which include SSP 
 *     initialization, SSP interrupt handler, and APIs for SSP
 *     reading.
 *
 ****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
****************************************************************************/
#include "LPC122x.h"			/* LPC122x Peripheral Registers */
#include "gpio.h"
#include "ssp.h"
#include "stdio.h"

/* statistics of all the interrupts */
volatile uint32_t interruptRxStat = 0;
volatile uint32_t interruptOverRunStat = 0;
volatile uint32_t interruptRxTimeoutStat = 0;

/*****************************************************************************
** Function name:		SSP_IRQHandler
**
** Descriptions:		SSP port is used for SPI communication.
**						SSP interrupt handler
**						The algorithm is, if RXFIFO is at least half full, 
**						start receive until it's empty; if TXFIFO is at least
**						half empty, start transmit until it's full.
**						This will maximize the use of both FIFOs and performance.
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void SSP_IRQHandler(void) 
{
  uint32_t regValue;

  regValue = LPC_SSP->MIS;
  if ( regValue & SSPMIS_RORMIS )	/* Receive overrun interrupt */
  {
	interruptOverRunStat++;
	LPC_SSP->ICR = SSPICR_RORIC;	/* clear interrupt */
  }
  if ( regValue & SSPMIS_RTMIS )	/* Receive timeout interrupt */
  {
	interruptRxTimeoutStat++;
	LPC_SSP->ICR = SSPICR_RTIC;	/* clear interrupt */
  }

  /* please be aware that, in main and ISR, CurrentRxIndex and CurrentTxIndex
  are shared as global variables. It may create some race condition that main
  and ISR manipulate these variables at the same time. SSPSR_BSY checking (polling)
  in both main and ISR could prevent this kind of race condition */
  if ( regValue & SSPMIS_RXMIS )	/* Rx at least half full */
  {
	interruptRxStat++;		/* receive until it's empty */		
  }
  return;
}

/*****************************************************************************
** Function name:		SSP_IOConfig
**
** Descriptions:		SSP port initialization routine
**				
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void SSP_IOConfig( void )
{
  /* SSP clock and reset configuration. */
  LPC_SYSCON->PRESETCTRL |= (0x1<<0);
  LPC_SYSCON->SYSAHBCLKCTRL |= (0x1<<11);
  LPC_SYSCON->SSPCLKDIV = 0x02;			/* Divided by 2 */
  
  /*  SSP I/O config */
  LPC_IOCON->PIO0_16 &= ~0x07;	
  LPC_IOCON->PIO0_16 |= 0x02;		/* SSP MISO */
  LPC_IOCON->PIO0_17  &= ~0x07;	
  LPC_IOCON->PIO0_17  |= 0x02;		/* SSP MOSI */
  LPC_IOCON->PIO0_14 &= ~0x07;
  LPC_IOCON->PIO0_14 |= 0x02;		/* SSP CLK */

  /* Enable AHB clock to the GPIO port0 domain. */
  LPC_SYSCON->SYSAHBCLKCTRL |= 0x80000000;

//  LPC_IOCON->PIO0_15 &= ~0x07;		/* SSP SSEL is a GPIO pin */
//  /* port0, bit 15 is set to GPIO output and high */
//  GPIOSetDir( PORT0, 15, 1 );
//  GPIOSetValue( PORT0, 15, 1 );
  LPC_IOCON->PIO2_0 &= ~0x07;		/* SSP SSEL is a GPIO pin */
  /* port0, bit 15 is set to GPIO output and high */
  GPIOSetDir( PORT2, 0, 1 );
  GPIOSetValue( PORT2, 0, 1 );

  return;		
}

/*****************************************************************************
** Function name:		SSP_Init
**
** Descriptions:		SSP port initialization routine
**				
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void SSP_Init( void )
{
  uint8_t i, Dummy=Dummy;

  /* Set DSS data to 8-bit, Frame format SPI, CPOL = 1, CPHA = 1, and SCR is 15 */
  LPC_SSP->CR0 = 0x0707;

  /* SSPCPSR clock prescale register, master mode, minimum divisor is 0x02 */
  LPC_SSP->CPSR = 0x5;

  for ( i = 0; i < FIFOSIZE; i++ )
  {
	Dummy = LPC_SSP->DR;		/* clear the RxFIFO */
  }

  /* Enable the SSP Interrupt */
  NVIC_EnableIRQ(SSP_IRQn);
	
  /* Master mode */
  LPC_SSP->CR1 = SSPCR1_SSE;

  /* Set SSPINMS registers to enable interrupts */
  /* enable all error related interrupts */
  LPC_SSP->IMSC = SSPIMSC_RORIM | SSPIMSC_RTIM;
  return;
}

/*****************************************************************************
** Function name:		SSP_Send
**
** Descriptions:		Send a block of data to the SSP port, the 
**						first parameter is the buffer pointer, the 2nd 
**						parameter is the block length.
**
** parameters:			buffer pointer, and the block length
** Returned value:		None
** 
*****************************************************************************/
void SSP_Send( uint8_t *buf, uint32_t Length )
{
  uint32_t i;
  uint8_t Dummy = Dummy;
  Dummy = *buf;
  printf("\r\nSSP Send Data 0x%x\r\n", Dummy);  
  for ( i = 0; i < Length; i++ )
  {
	/* Move on only if NOT busy and TX FIFO not full. */
	while ( (LPC_SSP->SR & (SSPSR_TNF|SSPSR_BSY)) != SSPSR_TNF );
	LPC_SSP->DR = *buf;
	buf++;
	/* Wait until the Busy bit is cleared. */
	while ( LPC_SSP->SR & SSPSR_BSY );
  }
  return; 
}

void SSP_Snd_OneByte(uint8_t data)
{ 
	/* Move on only if NOT busy and TX FIFO not full. */
	while ( (LPC_SSP->SR & (SSPSR_TNF|SSPSR_BSY)) != SSPSR_TNF );
	LPC_SSP->DR = data;
	/* Wait until the Busy bit is cleared. */
	while ( LPC_SSP->SR & SSPSR_BSY );	
	return;
}

/*****************************************************************************
** Function name:		SSP_Receive
** Descriptions:		the module will receive a block of data from 
**						the SSP, the 2nd parameter is the block 
**						length.
** parameters:			buffer pointer, and block length
** Returned value:		None
** 
*****************************************************************************/
void SSP_Receive( uint8_t *buf, uint32_t Length )
{
  uint32_t i;
 
  for ( i = 0; i < Length; i++ )
  {
	/* As long as Receive FIFO is not empty, I can always receive. */
	/* If it's a loopback test, clock is shared for both TX and RX,
	no need to write dummy byte to get clock to get the data */
	/* if it's a peer-to-peer communication, SSPDR needs to be written
	before a read can take place. */
	LPC_SSP->DR = 0xFF;
	/* Wait until the Busy bit is cleared */
	while ( (LPC_SSP->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE );
	*buf = LPC_SSP->DR;
	printf("buf 0x%x, ", *buf);
	buf++;
  }
  return; 
}

unsigned char SSP_Rcv_OneByte( void )
{
	unsigned char temp;
	/* As long as Receive FIFO is not empty, I can always receive. */
	/* If it's a loopback test, clock is shared for both TX and RX,
	no need to write dummy byte to get clock to get the data */
	/* if it's a peer-to-peer communication, SSPDR needs to be written
	before a read can take place. */
	LPC_SSP->DR = 0xFF;
	/* Wait until the Busy bit is cleared */
	while ( (LPC_SSP->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE );
	temp = LPC_SSP->DR;
	return temp; 
}

void ssp_rddata(unsigned char addr, unsigned char* data, unsigned long length)
{
	unsigned char temp = 0x00;
	addr = (addr<<1)|0x80;					// keep MSB as 1

	GPIOSetValue( PORT0, 15, 0 );		// CS Low Seleted
	SSP_Send(&addr, 1);
	SSP_Send(data,  length);
	SSP_Send(&temp,  1);
	GPIOSetValue( PORT0, 15, 1 );		// CS High UnSeleted
}

void ssp_wrdata(unsigned char addr, unsigned char* data, unsigned long length)
{
	addr = addr&0x7F;

	GPIOSetValue( PORT0, 15, 0 );		// CS Low Seleted
	SSP_Send(&addr, 1);
	SSP_Send(data,  length);
	GPIOSetValue( PORT0, 15, 1 );		// CS High UnSeleted	
}

/******************************************************************************
**                            End Of File
******************************************************************************/

