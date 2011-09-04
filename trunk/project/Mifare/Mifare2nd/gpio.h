/****************************************************************************
 *   $Id:: gpio.h 3740 2010-06-24 22:24:09Z usb00423                        $
 *   Project: NXP LPC122x software example
 *
 *   Description:
 *     This file contains definition and prototype for GPIO.
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
#ifndef __GPIO_H 
#define __GPIO_H

#define PORT0		0
#define PORT1		1
#define PORT2		2

void PIOINT0_IRQHandler(void);
void PIOINT1_IRQHandler(void);
void PIOINT2_IRQHandler(void);
void GPIO_SetIOCONFunc ( uint32_t port, uint8_t pinnum);
void GPIOInit( void );
void GPIOSetDir( uint32_t portNum, uint32_t bitPosi, uint32_t dir );
void GPIOSetValue( uint32_t portNum, uint32_t bitPosi, uint32_t bitVal );
uint32_t GPIO_GetPinValue( LPC_GPIO_TypeDef* pGPIO, uint8_t bitPosi);
uint32_t GPIO_GetOutValue( LPC_GPIO_TypeDef* pGPIO, uint8_t bitPosi);
void GPIOSetInterrupt( uint32_t portNum, uint32_t bitPosi, uint32_t sense,
		uint32_t single, uint32_t event );
void GPIOIntEnable( uint32_t portNum, uint32_t bitPosi );
void GPIOIntDisable( uint32_t portNum, uint32_t bitPosi );
uint32_t GPIOIntStatus( uint32_t portNum, uint32_t bitPosi );
void GPIOIntClear( uint32_t portNum, uint32_t bitPosi );

#endif /* end __GPIO_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/
