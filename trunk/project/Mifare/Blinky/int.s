;/*
; * File      : context_rvds.S
; * This file is part of RT-Thread RTOS
; * COPYRIGHT (C) 2009, RT-Thread Development Team
; *
; * The license and distribution terms for this file may be
; * found in the file LICENSE in this distribution or at
; * http://www.rt-thread.org/license/LICENSE
; *
; * Change Logs:
; * Date           Author       Notes
; * 2010-01-25     Bernard      first version
; */

;/**
; * @addtogroup LPC1100
; */
;/*@{*/

NVIC_INT_CTRL   EQU     0xE000ED04               ; interrupt control state register
NVIC_SYSPRI2    EQU     0xE000ED20               ; system priority register (2)
NVIC_PENDSV_PRI EQU     0x00FF0000               ; PendSV priority value (lowest)
NVIC_PENDSVSET  EQU     0x10000000               ; value to trigger PendSV exception

	AREA |.text|, CODE, READONLY, ALIGN=2
	THUMB
	REQUIRE8
	PRESERVE8

;/*
; * rt_base_t rt_hw_interrupt_disable();
; */
int_disable    PROC
	EXPORT  int_disable
	MRS		r0, PRIMASK
	CPSID   I
	BX		LR
	ENDP

;/*
; * void rt_hw_interrupt_enable(rt_base_t level);
; */
int_enable    PROC
	EXPORT  int_enable
	MSR		PRIMASK, r0
	BX      LR
	ENDP

	END
