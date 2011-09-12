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

#include "MfRc531.h"
#include "iso14443a.h"
#include "main.h"

/*************************************状态变量******************************************/
/***************************************************************************************/
unsigned char PcdAntena = 0x01; 
unsigned char c_RecvBuf[32];
unsigned char cRecvNum;							//接收到上位机的字节数
unsigned char cCommand;                          	//接收到的命令码
unsigned char cRecvOk;							//正确接收到上位机指令标志
unsigned char cAutoReqFlag;						//自动寻卡标识
unsigned char iRecvDelay;							//接收间隔计数
unsigned char cFrameHeadFlag;						//接收头标识
unsigned char cRecvFinished;

/***************************************************************************************/
/***************************************************************************************/
#define	cFrameDelay	4				//定时接收最大间隔
#define PROTOCOL_LEN 6				//通讯协议段最短长度

/*************************************M1卡常用数据**************************************/
/***************************************************************************************/
unsigned char c_CardType[2];										//M1卡类型号
unsigned char c_CardSrlNum[4];									//M1卡序列号
unsigned char sak1[1];											//M1卡内部数据空间大小

/***********************************状态信息数据****************************************/
/***************************************************************************************/
unsigned char hardmodel[12]  = {"Linpo-Rc5xx-"};

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
/****************************************************************************************
** 函数原型:    void COM_Recv(void)
** 函数功能:    接收数据解析
** 入口参数:    -
** 出口参数:    -
** 返 回 值:    -
** 描　  述:    
****************************************************************************************/
void COM_Recv(void)
{
	unsigned int cCount;
	unsigned char verify = 0;

	if ((cFrameHeadFlag == 1) && (iRecvDelay >= cFrameDelay) && (cRecvFinished == 1))
	{
		cFrameHeadFlag = 0;
		cRecvFinished = 0;

		DelayMs(10);
		
		if ((c_RecvBuf[0] == 0xaa) && (c_RecvBuf[1] == 0xbb))
		{
			for (cCount = 0; cCount < cRecvNum; cCount ++)
			{
				verify ^= c_RecvBuf[cCount];
			}
			if (!verify)
			{
				cRecvOk = 1;									//接收数据正确
				cCommand = c_RecvBuf[3];						//命令码			
			}		
		} 
	}
}

void init_mfrc500(void)
{
	int i;
	char status;

	LPC_IOCON->PIO0_4 &= ~0x07;		/* SSP SSEL is a GPIO pin */
	/* port0, bit 15 is set to GPIO output and high */
	GPIOSetDir( PORT0, 4, 1 );
	GPIOSetValue( PORT0, 4, 1 );
	for(i=0; i<500000; i++);
	GPIOSetValue( PORT0, 4, 0 );

	
	DelayMs(100);
	DelayMs(100);

	status = PcdReset();
	if (status != MI_OK)
	{
		DelayMs(10);
#ifdef DEBUG_UART
		printf("mfrc500 1st fail\r\n");
#endif
		status = PcdReset();
	} 
	if (status != MI_OK)
	{
		DelayMs(10);
#ifdef DEBUG_UART
		printf("mfrc500 2nd fail\r\n");
#endif
		status = PcdReset();
	} 
	if (status == MI_OK)
	{
#ifdef DEBUG_UART
		printf("mfrc500 init ok\r\n");
#endif
		DelayMs(5500);
	//	DelayMs(80);
	}  
}

void reset_mfrc500(void)
{
	GPIOSetValue( PORT0, 4, 1 );
}

void unreset_mfrc500(void)
{
	GPIOSetValue( PORT0, 4, 0 );
}

/****************************************************************************************
** 函数原型:    void ComPcdAntenna(void)
** 函数功能:    响应天线命令
** 入口参数:    -
** 出口参数:    -
** 返 回 值:    -
** 描　  述:    
****************************************************************************************/
void compcd_antenna(void)
{
	char status = status;

	if (!PcdAntena)
	{
		printf("Antenna Off\r\n");
		status = PcdAntennaOff();
	}
	else
	{  
		printf("Antenna On\r\n");
		DelayMs(10); 
		status = PcdAntennaOn();
		DelayMs(10);
	}
}

/****************************************************************************************
** 函数原型:    char CheckSum(const void *pData, unsigned char nDataLen) 
** 函数功能:    计算校验码，对各数据异或
** 入口参数:    const void *pData, unsigned char nDataLen
** 出口参数:    -
** 返 回 值:    nCheckSum
** 描　  述:    
****************************************************************************************/   
char CheckSum(const void *pData, unsigned char nDataLen)   
{   
	const unsigned char *p = (const unsigned char *) pData;   
       
	unsigned char nCheckSum = 0;    // 校验码    
	while (nDataLen-- > 0)   
	{   
		nCheckSum ^= *p++;   
	}   
       
	return nCheckSum;   
}

/****************************************************************************************
** 函数原型:    void COM_Send(unsigned char* pInfo, unsigned char nInfoLen, unsigned char status)
** 函数功能:    发送执行命令
** 入口参数:    unsigned char* pInfo, unsigned char nInfoLen, unsigned char status
** 出口参数:    -
** 返 回 值:    -
** 描　  述:    
****************************************************************************************/
void COM_Send(unsigned char *pInfo, unsigned char nInfoLen, unsigned char status)
{
	unsigned char senddat[40];
	memset(senddat, 0x00, 40);
	if ((PROTOCOL_LEN + nInfoLen) > sizeof(senddat))
	{
		return;
	}

	senddat[0] = 0xAA;
	senddat[1] = 0xBB;								//帧头
	senddat[2] = nInfoLen + 3;						//发送字节数
	senddat[3] = cCommand;							//命令码
	senddat[4] = status;							//状态字
	memcpy(&senddat[5],pInfo,nInfoLen);				//信息
	senddat[5+nInfoLen] = CheckSum(senddat,5+nInfoLen);		//校验和

//	UART_Send(senddat,senddat[2] + 3);
//	printf("%s\r\n", senddat);
	printf("0x%x, 0x%x, 0x%x, 0x%x, 0x%x\r\n", senddat[5], senddat[6], senddat[7], senddat[8], senddat[9] );
}

/****************************************************************************************
** 函数原型:    void PcdAuthentication(void)
** 函数功能:    密钥验证
** 入口参数:    -
** 出口参数:    -
** 返 回 值:    -
** 描　  述:    
****************************************************************************************/
char PcdAuthentication(unsigned char keyType, unsigned char blockaddr, unsigned char *pKey, unsigned char *pCardSnr)
{
	char status = MI_COM_ERR;
	unsigned char AuthCmd, c_TempData[12];

	if (keyType == 0x0A)
	{		
		AuthCmd = PICC_AUTHENT1A;	
	}
	if (keyType == 0x0B)
	{
		AuthCmd = PICC_AUTHENT1B;	
	}

	ChangeCodeKey(pKey,c_TempData);							//转换密钥格式											    
	PcdAuthKey(c_TempData);									//写入FIFO										
	status = PcdAuthState(AuthCmd, blockaddr, pCardSnr);	//密钥验证

	return status;		
}

/****************************************************************************************
** 函数原型:    void M1Identify(void)
** 函数功能:    卡片识别，包含寻卡、防冲突和选择卡
** 入口参数:    -
** 出口参数:    -
** 返 回 值:    -
** 描　  述:    
****************************************************************************************/
void M1Identify(void)
{
	char status;
	
	status = PcdRequest(PICC_REQIDL, c_CardType);
	if (status != MI_OK )
	{
		PcdRequest(PICC_REQIDL, c_CardType);
#ifdef DEBUG_UART
		printf("PcdRequest fail 0x%x, CardType 0x%x%x\r\n", status, c_CardType[1], c_CardType[0]);
#endif
	}

	PcdAnticoll(c_CardSrlNum);
	if (MI_OK == PcdSelect(c_CardSrlNum, sak1))
	{
#ifdef DEBUG_UART
		printf("PcdSelect success\r\n");
#endif
		status = 0;
	}
	else
	{
#ifdef DEBUG_UART
		printf("PcdSelect fail\r\n");
#endif
		status = 0x01;										//选卡失败
	}

	COM_Send(c_CardSrlNum,4,status);
}

/****************************************************************************************
** 函数原型:    void M1Read(void)
** 函数功能:    读卡块值
** 入口参数:    -
** 出口参数:    -
** 返 回 值:    -
** 描　  述:    
****************************************************************************************/
void M1Read(void)
{
	char status;

	status = PcdAuthentication(c_RecvBuf[4], c_RecvBuf[5], &c_RecvBuf[6], c_CardSrlNum);
		
	if (status == MI_OK)
	{														//密钥验证成功
		if (MI_OK == PcdRead(c_RecvBuf[5],&c_RecvBuf[14]))
		{
			status = 0;										//读卡成功
		}
		else
		{
			status = 0x03;									//读卡失败
		}	
	}
	else
	{
		status = 0x02;										//密钥验证失败
	}

	COM_Send(&c_RecvBuf[14],0x10,status);
}

/****************************************************************************************
** 函数原型:    void M1Write(void)
** 函数功能:    写卡块值
** 入口参数:    -
** 出口参数:    -
** 返 回 值:    -
** 描　  述:    
****************************************************************************************/
void M1Write(void)
{
	char status;
	
	status = PcdAuthentication(c_RecvBuf[4], c_RecvBuf[5], &c_RecvBuf[6], c_CardSrlNum);
		
	if (MI_OK == status)
	{														//密钥验证成功
		if (MI_OK == PcdWrite(c_RecvBuf[5],&c_RecvBuf[12]))
		{	
			status = 0x0;									//写块数据成功
		}
		else
		{
			status = 0x04;									//写块数据失败
		}
	}
	else
	{
		status = 0x02;										//密钥验证失败
	}
		
	COM_Send(0,0,status);	
}

/****************************************************************************************
** 函数原型:    void M1Initval(void)
** 函数功能:    初始化钱包
** 入口参数:    -
** 出口参数:    -
** 返 回 值:    -
** 描　  述:    
****************************************************************************************/
void M1Initval(void)
{
	char status;

	c_RecvBuf[16] = ~c_RecvBuf[12];		c_RecvBuf[17] = ~c_RecvBuf[13];
	c_RecvBuf[18] = ~c_RecvBuf[14];		c_RecvBuf[19] = ~c_RecvBuf[15];
	c_RecvBuf[20] = c_RecvBuf[12];		c_RecvBuf[21] = c_RecvBuf[13];
	c_RecvBuf[22] = c_RecvBuf[14];		c_RecvBuf[23] = c_RecvBuf[15];
	c_RecvBuf[24] = c_RecvBuf[5];		c_RecvBuf[25] = ~c_RecvBuf[5];
	c_RecvBuf[26] = c_RecvBuf[5];		c_RecvBuf[27] = ~c_RecvBuf[5]; 
		
	status = PcdAuthentication(c_RecvBuf[4], c_RecvBuf[5], &c_RecvBuf[6], c_CardSrlNum);

	if (MI_OK == status)										//验证密钥
	{			
		if (MI_OK == PcdWrite(c_RecvBuf[5],&c_RecvBuf[12]))	//写块数据
		{
			status = 0;
		}
		else
		{
			status = 0x04;
		}
	}
	else
	{
		status = 0x02;
	}
		
	COM_Send(0,0,status);
}

/****************************************************************************************
** 函数原型:    void M1ReadVal(void)
** 函数功能:    读取钱包值
** 入口参数:    -
** 出口参数:    -
** 返 回 值:    -
** 描　  述:    
****************************************************************************************/
void M1ReadVal(void)
{
	char status;

	status = PcdAuthentication(c_RecvBuf[4], c_RecvBuf[5], &c_RecvBuf[6], c_CardSrlNum);

	if (MI_OK == status)										//验证密钥
	{
		if (MI_OK == PcdRead(c_RecvBuf[5],&c_RecvBuf[14]))		//读块数据
		{
			status = 0;
		}
		else
		{
			status = 0x03;
		}
	}
	else
	{
		status = 0x02;
	}
		
	COM_Send(&c_RecvBuf[14],0x04,status);
}

/****************************************************************************************
** 函数原型:    void M1Decrement(void)
** 函数功能:    钱包扣款
** 入口参数:    -
** 出口参数:    -
** 返 回 值:    -
** 描　  述:    
****************************************************************************************/
void M1Decrement(void)
{
	char status;

	status = PcdAuthentication(c_RecvBuf[4], c_RecvBuf[5], &c_RecvBuf[6], c_CardSrlNum);

	if (MI_OK == status)											//验证密钥
	{
		if (MI_OK == PcdValue(PICC_DECREMENT,c_RecvBuf[5],&c_RecvBuf[12]))		//钱包扣款
		{	
			status = 0;
		}
		else
		{
			status = 0x03;
		}
	}
	else
	{
		status = 0x02;
	}
		
	COM_Send(0,0,status);
}

/****************************************************************************************
** 函数原型:    void M1Increment(void)
** 函数功能:    钱包充值
** 入口参数:    -
** 出口参数:    -
** 返 回 值:    -
** 描　  述:    
****************************************************************************************/
void M1Increment(void)
{
	char status;

	status = PcdAuthentication(c_RecvBuf[4], c_RecvBuf[5], &c_RecvBuf[6], c_CardSrlNum);

	if (MI_OK == status)										//验证密钥
	{	
		if (MI_OK == PcdValue(PICC_INCREMENT,c_RecvBuf[5],&c_RecvBuf[12]))	//钱包充值
		{			
			status = 0;
		}
		else
		{
			status = 0x03;
		}
	}
	else
	{
		status = 0x02;
	}
		
	COM_Send(0,0,status);
}

/****************************************************************************************
** 函数原型:    void M1Transfer(void)
** 函数功能:    备份钱包
** 入口参数:    -
** 出口参数:    -
** 返 回 值:    -
** 描　  述:    
****************************************************************************************/
void M1Transfer(void)
{
	char status;

	status = PcdAuthentication(c_RecvBuf[4], c_RecvBuf[5], &c_RecvBuf[7], c_CardSrlNum);

	if (MI_OK == status)										//验证密钥
	{
		PcdRestore(c_RecvBuf[5]);							//块数据写入FIFO
		if (MI_OK == PcdTransfer(c_RecvBuf[6]))				//块数据转移
		{
			status = 0;
		}
		else
		{
			status = 0x03;
		}
	}
	else
	{
		status = 0x02;
	}

	COM_Send(0,0,status);
}

/****************************************************************************************
** 函数原型:    void M1Halt(void)
** 函数功能:    卡片休眠
** 入口参数:    -
** 出口参数:    -
** 返 回 值:    -
** 描　  述:    
****************************************************************************************/
void M1Halt(void)
{
	char status = MI_COM_ERR;

	PcdHalt();
	status = PcdHalt();
	if (status == MI_OK)
	{
		status = 0;
	}
	else
	{
		status = 0xf1;
	}

	COM_Send(0,0,status);
}


/****************************************************************************************
** 函数原型:    void PcdIdentify(void)
** 函数功能:    卡片识别，包含寻卡、防冲突和选择卡
** 入口参数:    -
** 出口参数:    -
** 返 回 值:    -
** 描　  述:    
****************************************************************************************/
void PcdIdentify(void)
{
	char status;

	status = PcdRequest(PICC_REQIDL, c_CardType);
	if (status != MI_OK )
	{
#ifdef DEBUG_UART
		printf("PcdRequest fail 0x%x\r\n", status);
#endif
		status = PcdRequest(PICC_REQIDL, c_CardType);
	}
	if (status == MI_OK)
	{
#ifdef DEBUG_UART
		printf("PcdRequest OK 0x%x\r\n", status);
#endif
		status = PcdAnticoll(c_CardSrlNum);
#ifdef DEBUG_UART
		printf("PcdAnticoll status 0x%x\r\n", status);
#endif
	}
	
	status = PcdSelect(c_CardSrlNum, sak1);

	if (status == MI_OK)
	{
#ifdef DEBUG_UART
		printf("PcdSelect success\r\n");
#endif
	}	
	else
	{
#ifdef DEBUG_UART
		printf("PcdSelect fail 0x%x\r\n", status);
#endif 
	}
}

//extern void SingleResponseIsr(void);
//
//void PIOINT0_IRQHandler(void)
//{
//  uint32_t regVal;
//
//  regVal = GPIOIntStatus( PORT0, 7 );
//  if ( regVal )
//  {
//	SingleResponseIsr();
//	GPIOIntClear( PORT0, 7 );
//  }		
//  return;
//}

// interrupter in PIO
//void init_eint(void)
//{
////P0.7
//  GPIO_SetIOCONFunc(PORT0, 7);	
//  /* 使用Port0.7作为外部中断输入测试 */
//  GPIOSetDir( PORT0, 7, 0 );
//  /* P0.7外部中断单端触发、高电平有效 */
//  GPIOSetInterrupt( PORT0, 1, 7, 0, 0 );
//  GPIOIntEnable( PORT0, 7 );
//}

volatile unsigned char disptemp[16];

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) 
{   
	unsigned char status = MI_NOTAGERR;
	unsigned long current_tick, last_tick, i, j;  
	unsigned char data[5];
	unsigned char temp;
	                      
//	SysTick_Config(SystemCoreClock/1000);          /* Generate IRQ each 1 ms    */

//	init_mfrc500();
  	
	SER_init();                                   /* UART#1 Initialization      */
	SSP_IOConfig();
	SSP_Init();

//	init_eint();
#ifdef DEBUG_UART
	printf("Welcome\r\n");
#endif

	init_mfrc500();
//while(1)
//{
//	for(i=0; i<5; i++)
//	data[i] = ReadE2RC(0x00+i);
//	printf("Addr 0x00 content 0x%x, ", data[0]);
//#ifdef DEBUG_UART
//	for(i=1; i<4; i++)
//	printf("0x%x, ", data[i]);
//#endif
//	printf("0x%x\r\n", data[4]);
//	DelayMs(2000);
//}
	compcd_antenna();

	PcdConfigISOType();

	while(1)
	{
		COM_Recv();		
		//M1Identify();
		if (cRecvOk == 1)
		{			
			cRecvOk = 0;
			switch (cCommand)
			{
				case ReadCdSrl:
					M1Identify();
				break;				
				case ReadCdDat:
					M1Read();
				break;
				case WritCdDat:
					M1Write();
				break;
				case InitCdPrs:
					M1Initval();
				break;
				case ReadCdPrs:
					M1ReadVal();
				break;
				case IncrCdPrs:
					M1Increment();
				break;
				case DecrCdPrs:
					M1Decrement();
				break;
				case TranCdPrs:
					M1Transfer();
				break;
				case HaltCdIns:
					M1Halt();
				break;
				case SetBound:
//					SetBaud();
				break;				
			}
		}
		PcdIdentify();
		printf("-------------------------^.^-------------------------------------\r\n");
	}
}
