/****************************************************************************
 *   $Id:: gpio.c 3740 2010-06-24 22:24:09Z usb00423                        $
 *   Project: NXP LPC122x GPIO example
 *
 *   Description:
 *     This file contains GPIO code example which include GPIO 
 *     initialization, GPIO interrupt handler, and related APIs for 
 *     GPIO access.
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

/* Shadow registers used to prevent chance of read-modify-write errors */
/* Ultra-conservative approach... */
volatile uint32_t GPIOShadowPort0;
volatile uint32_t GPIOShadowPort1;
volatile uint32_t GPIOShadowPort2;

/*********************************************************************//**
 * @brief 		Setup the GPIO port selection function
 * @param[in]	port	       port 0/1/2
 * @param[in]	pinnum	Pin number, from 0 to 31
 * @param[in]	mode      Selects function mode (on-chip pull-up/pull-down resistor control).
 * @param[in] hys   	Hysteresis enable/disable
 *
 * @return 		None
 **********************************************************************/

void GPIO_SetIOCONFunc ( uint32_t port, uint8_t pinnum)
{
	uint32_t reg_idx = 0;
	uint32_t func = 0x00;	
	uint32_t *p = (uint32_t *)&LPC_IOCON->PIO2_28;

	/* LPC_IOCON->PIO2_28 */ 
	static const uint8_t PIO0_Offset[32]={
	  17, 18, 19, 21, 22, 23, 24, 25,
	  26, 27, 36, 37, 38, 39, 40, 41,
	  42, 43, 44,  2,  3,  4,  5,  6,
	   7,  8,  9, 10, 15, 16, 45, 46
	};
	
	/* LPC_IOCON->PIO2_28 */
	static const uint8_t PIO2_Offset[16]={
	  28, 29, 30, 31, 32, 33, 34, 35,
	  56, 57, 58, 59, 11, 12, 13, 14
	};

	switch ( port )
	{
    	case PORT0:	
			if((pinnum == 13)||(pinnum == 18)||(pinnum == 30)||(pinnum == 31)){
               func = 0x01;
			}			
			reg_idx = PIO0_Offset[pinnum];
    	break;			
    	case PORT1:
			if((pinnum == 0)||(pinnum == 1)||(pinnum == 2)){
               func = 0x01;
			}			
			p = (uint32_t *)&LPC_IOCON->PIO1_0;		
			reg_idx += pinnum;
    	break;				
    	case PORT2:		
			reg_idx = PIO2_Offset[pinnum];
    	break;				
        default:
		break;
	}
	
	*(uint32_t *)(p + reg_idx) = 0xC0;	
	//*(uint32_t *)(p + reg_idx) |= (uint32_t)( func );
	*(uint32_t *)(p + reg_idx) |= (uint32_t)( func | 0x10 );	
}

/*****************************************************************************
** Function name:		GPIOInit
**
** Descriptions:		Initialize GPIO, install the
**						GPIO interrupt handler
**
** parameters:			None
** Returned value:		true or false, return false if the VIC table
**						is full and GPIO interrupt handler can be
**						installed.
** 
*****************************************************************************/
void GPIOInit( void )
{
  /* Enable AHB clock to the GPIO domain. Enable all the clocks to all
  GPIO ports. */
  LPC_SYSCON->SYSAHBCLKCTRL |= ((1<<29)|(1<<30)|(0x80000000));

  /* Set up NVIC when I/O pins are configured as external interrupts. */
  NVIC_EnableIRQ(EINT0_IRQn);
  NVIC_EnableIRQ(EINT1_IRQn);
  NVIC_EnableIRQ(EINT2_IRQn);
  return;
}

/*****************************************************************************
** Function name:		GPIOSetDir
**
** Descriptions:		Set the direction in GPIO port
**
** parameters:			port num, bit position, direction (1 out, 0 input)
** Returned value:		None
** 
*****************************************************************************/
void GPIOSetDir( uint32_t portNum, uint32_t bitPosi, uint32_t dir )
{
  /* if DIR is OUT(1), but GPIOx_DIR is not set, set DIR
  to OUT(1); if DIR is IN(0), but GPIOx_DIR is set, clr
  DIR to IN(0). All the other cases are ignored. 
  On port3(bit 0 through 3 only), no error protection if 
  bit value is out of range. */
  switch ( portNum )
  {
	case PORT0:
	  if ( !(LPC_GPIO0->DIR & (0x1<<bitPosi)) && (dir == 1) )
		LPC_GPIO0->DIR |= (0x1<<bitPosi);
	  else if ( (LPC_GPIO0->DIR & (0x1<<bitPosi)) && (dir == 0) )
		LPC_GPIO0->DIR &= ~(0x1<<bitPosi);	  
	break;
 	case PORT1:
	  if ( !(LPC_GPIO1->DIR & (0x1<<bitPosi)) && (dir == 1) )
		LPC_GPIO1->DIR |= (0x1<<bitPosi);
	  else if ( (LPC_GPIO1->DIR & (0x1<<bitPosi)) && (dir == 0) )
		LPC_GPIO1->DIR &= ~(0x1<<bitPosi);	  
	break;
	case PORT2:
	  if ( !(LPC_GPIO2->DIR & (0x1<<bitPosi)) && (dir == 1) )
		LPC_GPIO2->DIR |= (0x1<<bitPosi);
	  else if ( (LPC_GPIO2->DIR & (0x1<<bitPosi)) && (dir == 0) )
		LPC_GPIO2->DIR &= ~(0x1<<bitPosi);	  
	break;
	default:
	  break;
  }
  return;
}

/*****************************************************************************
** Function name:		GPIOSetValue
**
** Descriptions:		Set/clear a bitvalue in a specific bit position
**						in GPIO portX(X is the port number.)
**
** parameters:			port num, bit position, bit value
** Returned value:		None
** 
*****************************************************************************/
void GPIOSetValue( uint32_t portNum, uint32_t bitPosi, uint32_t bitVal )
{
  /* if bitVal is 1, the bitPosi bit is set in the GPIOShadowPortx. Then
   * GPIOShadowPortx is written to the I/O port register. */
  switch ( portNum )
  {
	/* Use of shadow prevents bit operation error if the read value
	 * (external hardware state) of a pin differs from the I/O latch
	 * value. A potential side effect is that other GPIO code in this
	 * project that is not aware of the shadow will have its GPIO
	 * state overwritten.
	*/
	case PORT0:
	  if(bitVal)
		GPIOShadowPort0 |= (1<<bitPosi);
	  else
		GPIOShadowPort0 &= ~(1<<bitPosi);
	 LPC_GPIO0->OUT = GPIOShadowPort0;
	break;
 	case PORT1:
	 if(bitVal)
		 GPIOShadowPort1 |= (1<<bitPosi);
	 else
		 GPIOShadowPort1 &= ~(1<<bitPosi);
	 LPC_GPIO1->OUT = GPIOShadowPort1;
	break;
	case PORT2:
	 if(bitVal)
		 GPIOShadowPort2 |= (1<<bitPosi);
	 else
		 GPIOShadowPort2 &= ~(1<<bitPosi);
	 LPC_GPIO2->OUT = GPIOShadowPort2;
	break;
	default:
	  break;
  }
  return;
}

/*********************************************************************//**
 * @brief		Read Current state on port pin, PIN register value
 * @param[in]	pGPIO		Port Number selection,  
 *                                       should be LPC_GPIO0, LPC_GPIO1, LPC_GPIO2
 * @param[in]	bitPosi	       bit position value, should be from 0 to 31
 *                                       if it is oxff, value is the 32bit value of register 
 * @return		Current value of GPIO port.
 *
 * Note: Return value contain state of each port pin (bit) on that GPIO regardless
 * its direction is input or output.
 **********************************************************************/
uint32_t GPIO_GetPinValue( LPC_GPIO_TypeDef* pGPIO, uint8_t bitPosi)
{
    uint32_t regVal = 0;	

	if( bitPosi < 0x20 ){	
	    if ( pGPIO->PIN & (0x1<<bitPosi) ){
	  		regVal = 1;
	    }
    }
	else if( bitPosi == 0xFF ){
        regVal = pGPIO->PIN;
	}
    return ( regVal );		
}

/*********************************************************************//**
 * @brief		Read Current state on port pin, OUT register value
 * @param[in]	pGPIO		Port Number selection,  
 *                                       should be LPC_GPIO0, LPC_GPIO1, LPC_GPIO2
 * @param[in]	bitPosi	       bit position value, should be from 0 to 31
 *                                       if it is oxff, value is the 32bit value of register
 * @return		Current value of GPIO port.
 *
 * Note: Return value contain state of each port pin (bit) on that GPIO regardless
 * its direction is input or output.
 **********************************************************************/
uint32_t GPIO_GetOutValue( LPC_GPIO_TypeDef* pGPIO, uint8_t bitPosi)
{
    uint32_t regVal = 0;
	
    if( bitPosi < 0x20 ){	
	    if ( pGPIO->OUT & (0x1<<bitPosi) ){
	  		regVal = 1;
	    }
    }
	else if( bitPosi == 0xFF ){
        regVal = pGPIO->OUT;
	}  
    return ( regVal );
}


/*****************************************************************************
** Function name:		GPIOSetInterrupt
**
** Descriptions:		Set interrupt sense, event, etc.
**						edge or level, 0 is edge, 1 is level
**						single or double edge, 0 is single, 1 is double 
**						active high or low, etc.
**
** parameters:			port num, bit position, sense, single/doube, polarity
** Returned value:		None
** 
*****************************************************************************/
void GPIOSetInterrupt( uint32_t portNum, uint32_t bitPosi, uint32_t sense,
			uint32_t single, uint32_t event )
{
  switch ( portNum )
  {
	case PORT0:
	  if ( sense == 0 )
	  {
		LPC_GPIO0->IS &= ~(0x1<<bitPosi);
		/* single or double only applies when sense is 0(edge trigger). */
		if ( single == 0 )
		  LPC_GPIO0->IBE &= ~(0x1<<bitPosi);
		else
		  LPC_GPIO0->IBE |= (0x1<<bitPosi);
	  }
	  else
	  	LPC_GPIO0->IS |= (0x1<<bitPosi);
	  if ( event == 0 )
		LPC_GPIO0->IEV &= ~(0x1<<bitPosi);
	  else
		LPC_GPIO0->IEV |= (0x1<<bitPosi);
	break;
 	case PORT1:
	  if ( sense == 0 )
	  {
		LPC_GPIO1->IS &= ~(0x1<<bitPosi);
		/* single or double only applies when sense is 0(edge trigger). */
		if ( single == 0 )
		  LPC_GPIO1->IBE &= ~(0x1<<bitPosi);
		else
		  LPC_GPIO1->IBE |= (0x1<<bitPosi);
	  }
	  else
	  	LPC_GPIO1->IS |= (0x1<<bitPosi);
	  if ( event == 0 )
		LPC_GPIO1->IEV &= ~(0x1<<bitPosi);
	  else
		LPC_GPIO1->IEV |= (0x1<<bitPosi);  
	break;
	case PORT2:
	  if ( sense == 0 )
	  {
		LPC_GPIO2->IS &= ~(0x1<<bitPosi);
		/* single or double only applies when sense is 0(edge trigger). */
		if ( single == 0 )
		  LPC_GPIO2->IBE &= ~(0x1<<bitPosi);
		else
		  LPC_GPIO2->IBE |= (0x1<<bitPosi);
	  }
	  else
	  	LPC_GPIO2->IS |= (0x1<<bitPosi);
	  if ( event == 0 )
		LPC_GPIO2->IEV &= ~(0x1<<bitPosi);
	  else
		LPC_GPIO2->IEV |= (0x1<<bitPosi);  
	break;
	default:
	  break;
  }
  return;
}

/*****************************************************************************
** Function name:		GPIOIntEnable
**
** Descriptions:		Enable Interrupt Mask for a port pin.
**
** parameters:			port num, bit position
** Returned value:		None
** 
*****************************************************************************/
void GPIOIntEnable( uint32_t portNum, uint32_t bitPosi )
{
  switch ( portNum )
  {
	case PORT0:
	  LPC_GPIO0->IE |= (0x1<<bitPosi); 
	break;
 	case PORT1:
	  LPC_GPIO1->IE |= (0x1<<bitPosi);	
	break;
	case PORT2:
	  LPC_GPIO2->IE |= (0x1<<bitPosi);	    
	break;
	default:
	  break;
  }
  return;
}

/*****************************************************************************
** Function name:		GPIOIntDisable
**
** Descriptions:		Disable Interrupt Mask for a port pin.
**
** parameters:			port num, bit position
** Returned value:		None
** 
*****************************************************************************/
void GPIOIntDisable( uint32_t portNum, uint32_t bitPosi )
{
  switch ( portNum )
  {
	case PORT0:
	  LPC_GPIO0->IE &= ~(0x1<<bitPosi); 
	break;
 	case PORT1:
	  LPC_GPIO1->IE &= ~(0x1<<bitPosi);	
	break;
	case PORT2:
	  LPC_GPIO2->IE &= ~(0x1<<bitPosi);	    
	break;
	default:
	  break;
  }
  return;
}

/*****************************************************************************
** Function name:		GPIOIntStatus
**
** Descriptions:		Get Interrupt status for a port pin.
**
** parameters:			port num, bit position
** Returned value:		None
** 
*****************************************************************************/
uint32_t GPIOIntStatus( uint32_t portNum, uint32_t bitPosi )
{
  uint32_t regVal = 0;

  switch ( portNum )
  {
	case PORT0:
	  if ( LPC_GPIO0->MIS & (0x1<<bitPosi) )
		regVal = 1;
	break;
 	case PORT1:
	  if ( LPC_GPIO1->MIS & (0x1<<bitPosi) )
		regVal = 1;	
	break;
	case PORT2:
	  if ( LPC_GPIO2->MIS & (0x1<<bitPosi) )
		regVal = 1;		    
	break;
	default:
	  break;
  }
  return ( regVal );
}

/*****************************************************************************
** Function name:		GPIOIntClear
**
** Descriptions:		Clear Interrupt for a port pin.
**
** parameters:			port num, bit position
** Returned value:		None
** 
*****************************************************************************/
void GPIOIntClear( uint32_t portNum, uint32_t bitPosi )
{
  switch ( portNum )
  {
	case PORT0:
	  LPC_GPIO0->IC |= (0x1<<bitPosi); 
	break;
 	case PORT1:
	  LPC_GPIO1->IC |= (0x1<<bitPosi);	
	break;
	case PORT2:
	  LPC_GPIO2->IC |= (0x1<<bitPosi);	    
	break;
	default:
	  break;
  }
  return;
}

/******************************************************************************
**                            End Of File
******************************************************************************/
