#include "LPC122x.h"
#include "gpio.h" 
#include "ssp.h"
#include "mfrc531.h"

extern void int_disable(void);
extern void int_enable(void) ;
extern void init_mfrc500(void);
extern void reset_mfrc500(void);
extern void unreset_mfrc500(void);

#define GetRegPage(addr)        (0x80 | (addr>>3)) 

#define READER_RESET			reset_mfrc500()
#define READER_CLEAR_RESET		unreset_mfrc500()
#define SSP_CS_LOW				GPIOSetValue( PORT2, 0, 0 )
#define SSP_CS_HIGH				GPIOSetValue( PORT2, 0, 1 )

typedef struct 
         {
            unsigned char  cmd;           //!< command code 
            char           status;        //!< communication status
            unsigned short nBytesSent;    //!< how many bytes already sent
            unsigned short nBytesToSend;  //!< how many bytes to send
            unsigned short nBytesReceived;//!< how many bytes received
            unsigned long  nBitsReceived; //!< how many bits received
            unsigned char  irqSource;     //!< which interrupts have occured
            unsigned char  collPos;       /*!< at which position occured a
                                          collision*/
            unsigned char  errFlags;      //!< error flags
            unsigned char  saveErrorState;//!< accumulated error flags for
                                          //!< multiple responses
            unsigned char  RxAlignWA;     //!< workaround for RxAlign = 7
            unsigned char  DisableDF;     //!< disable disturbance filter
         } MfCmdInfo;

// In order to exchange some values between the ISR and the calling function,
// a struct is provided. 
volatile MfCmdInfo     MInfo;  

#define MEMORY_BUFFER_SIZE    300
unsigned char MemPool[MEMORY_BUFFER_SIZE];

unsigned char *MSndBuffer = MemPool; // pointer to the transmit buffer
unsigned char *MRcvBuffer = MemPool; // pointer to the receive buffer                


///////////////////////////////////////////////////////////////////////////////
//          				 Time Delay
///////////////////////////////////////////////////////////////////////////////

extern volatile unsigned long	ticks;

void DelayMs(unsigned int ms)
{
	SysTick->LOAD  = (SystemCoreClock/1000 & SysTick_LOAD_RELOAD_Msk)*ms - 1;  //往重载计数器里写值        
	SysTick->VAL   = 0;                //计数器清零        
	SysTick->CTRL |= ((1<<1)|(1<<0));     	   //开启计数器,开启计数器中断
	while(!ticks);
	ticks = 0;
	SysTick->CTRL =0; 
}

///////////////////////////////////////////////////////////////////////////////
//          				 RC531_Init
///////////////////////////////////////////////////////////////////////////////
void RC531Init(void)
{	
//	RC531_RST = 0;	
	READER_CLEAR_RESET;
	DelayMs(500);
	
//	RC531_RST = 1;
	READER_RESET;
	DelayMs(1500);
	
//	RC531_RST = 0;
	READER_CLEAR_RESET;
	DelayMs(1000);
}

//////////////////////////////////////////////////////////////////////////////
//								WriteRawIO
/////////////////////////////////////////////////////////////////////////////
void WriteRawIO(unsigned char addr, unsigned char dat)
{  	
    unsigned char temp; 
    temp = (unsigned char)((addr << 1) & 0x7e); 

	SSP_CS_LOW;		// CS Low Seleted
	SSP_Snd_OneByte(temp);
//	SSP_CS_HIGH;
//	SSP_CS_LOW;
	SSP_Snd_OneByte(dat);
	SSP_CS_HIGH;	// CS Low UnSeleted
}

//////////////////////////////////////////////////////////////////////////////
//								ReadRawIO
/////////////////////////////////////////////////////////////////////////////
unsigned char ReadRawIO(unsigned char addr)
{
	unsigned char dat;
	unsigned char temp;

	temp = (unsigned char)(((addr << 1) | 0x80) & 0xfe); 

	SSP_CS_LOW;						// CS Low Seleted
	SSP_Snd_OneByte(temp);			// Send address info
//	SSP_CS_HIGH;
//	SSP_CS_LOW;
	dat = SSP_Rcv_OneByte();	// Rcv data
	SSP_Snd_OneByte(0x00);
	SSP_CS_HIGH;

	return dat;
}

//////////////////////////////////////////////////////////////////////////////
//									WriteRawRC
/////////////////////////////////////////////////////////////////////////////    
void WriteRawRC(unsigned char addr, unsigned char dat)//ok    
{   
    WriteRawIO(0x00,GetRegPage(addr));   // select appropriate page    
    WriteRawIO(addr,dat);                // write value at the specified    
}   

//////////////////////////////////////////////////////////////////////////////
//									ReadRawRC
/////////////////////////////////////////////////////////////////////////////   
unsigned char ReadRawRC(unsigned char addr)//ok    
{   
    WriteRawIO(0x00,GetRegPage(addr));   // select appropriate page    
    return ReadRawIO(addr);              // read value at the specified    
}

/////////////////////////////////////////////////////////////////////
//复位并初始化RC531
//注意:RC531上电后应延时500ms才能可靠初始化
/////////////////////////////////////////////////////////////////////
char PcdReset(void)
{
	char status = MI_OK;
	
	RC531Init();

	if ((ReadRawRC(RegCommand))==0x00) 
		status = MI_OK;
	else  
		status = MI_RESETERR;

	WriteRawRC(RegPage,0x00);
	DelayMs(2);
	
	return status;
}


//////////////////////////////////////////////////////////////////////
//设置RC531的工作方式 
//////////////////////////////////////////////////////////////////////
void PcdConfigISOType(void)
{
       ClearBitMask(RegControl,0x08);

       WriteRawRC(RegClockQControl,0x0);
       WriteRawRC(RegClockQControl,0x40);
       DelayMs(1);                   // wait approximately 100 us - calibration in progress
       ClearBitMask(RegClockQControl,0x40);
       
       WriteRawRC(RegTxControl,0x5b);
       WriteRawRC(RegCwConductance,0x0F);
       WriteRawRC(RegModConductance,0x3F);       
       WriteRawRC(RegCoderControl,0x19);
       WriteRawRC(RegModWidth,0x13);             
       WriteRawRC(RegModWidthSOF,0x00);          
       WriteRawRC(RegTypeBFraming,0x00);
       
       WriteRawRC(RegRxControl1,0x73);
       WriteRawRC(RegDecoderControl,0x08);
       WriteRawRC(RegBitPhase,0xAD);	
       WriteRawRC(RegRxThreshold,0xAA);
       WriteRawRC(RegBPSKDemControl,0);
       WriteRawRC(RegRxControl2,0x41);

       WriteRawRC(RegRxWait,0x06);
       WriteRawRC(RegChannelRedundancy,0x0F);    
       WriteRawRC(RegCRCPresetLSB,0x63);
       WriteRawRC(RegCRCPresetMSB,0x63);
       WriteRawRC(RegTimeSlotPeriod,0x00);
       WriteRawRC(RegMfOutSelect,0x00);
       WriteRawRC(RFU27,0x00);   	      

       WriteRawRC(RegFIFOLevel,0x3F);
       WriteRawRC(RegTimerClock,0x07);
       WriteRawRC(RegTimerReload,0x0A);
       WriteRawRC(RegTimerControl,0x06);   
       WriteRawRC(RegIRqPinConfig,0x02);      
       WriteRawRC(RFU2E,0x00);
       WriteRawRC(RFU2F,0x00);	  

       PcdSetTmo(106);
       DelayMs(1);
       PcdAntennaOn();
}


/////////////////////////////////////////////////////////////////////
//置RC531寄存器位
//input:reg=寄存器地址
//      mask=置位值
/////////////////////////////////////////////////////////////////////
void SetBitMask(unsigned char reg,unsigned char mask)  
{
   char tmp = 0x0;
   tmp = ReadRawRC(reg);
   WriteRawRC(reg,tmp | mask);  // set bit mask
}

/////////////////////////////////////////////////////////////////////
//清RC531寄存器位
//input:reg=寄存器地址
//      mask=清位值
/////////////////////////////////////////////////////////////////////
void ClearBitMask(unsigned char reg,unsigned char mask)  
{
   char tmp = 0x0;
   tmp = ReadRawRC(reg);
   WriteRawRC(reg, tmp & ~mask);  // clear bit mask
} 

/////////////////////////////////////////////////////////////////////
//设置RC531定时器
//input:tmolength=设置值
/////////////////////////////////////////////////////////////////////
void PcdSetTmo(unsigned char tmoLength)
{
   switch (tmoLength)
   {  
      case 0:                         // (0.302 ms) FWI=0
         WriteRawRC(RegTimerClock,0x07); // TAutoRestart=0,TPrescale=128
         WriteRawRC(RegTimerReload,0x21);// TReloadVal = 'h21 =33(dec) 
         break;
      case 1:                         // (0.604 ms) FWI=1
         WriteRawRC(RegTimerClock,0x07); // TAutoRestart=0,TPrescale=128
         WriteRawRC(RegTimerReload,0x41);// TReloadVal = 'h41 =65(dec) 
         break;
      case 2:                         // (1.208 ms) FWI=2
         WriteRawRC(RegTimerClock,0x07); // TAutoRestart=0,TPrescale=128
         WriteRawRC(RegTimerReload,0x81);// TReloadVal = 'h81 =129(dec) 
         break;
      case 3:                         // (2.416 ms) FWI=3
         WriteRawRC(RegTimerClock,0x09); // TAutoRestart=0,TPrescale=4*128
         WriteRawRC(RegTimerReload,0x41);// TReloadVal = 'h41 =65(dec) 
         break;
      case 4:                         // (4.833 ms) FWI=4
         WriteRawRC(RegTimerClock,0x09); // TAutoRestart=0,TPrescale=4*128
         WriteRawRC(RegTimerReload,0x81);// TReloadVal = 'h81 =129(dec) 
         break;
      case 5:                         // (9.666 ms) FWI=5
         WriteRawRC(RegTimerClock,0x0B); // TAutoRestart=0,TPrescale=16*128
         WriteRawRC(RegTimerReload,0x41);// TReloadVal = 'h41 =65(dec) 
         break;
      case 6:                         // (19.33 ms) FWI=6
         WriteRawRC(RegTimerClock,0x0B); // TAutoRestart=0,TPrescale=16*128
         WriteRawRC(RegTimerReload,0x81);// TReloadVal = 'h81 =129(dec) 
         break;
      case 7:                         // (38.66 ms) FWI=7
         WriteRawRC(RegTimerClock,0x0D); // TAutoRestart=0,TPrescale=64*128
         WriteRawRC(RegTimerReload,0x41);// TReloadVal = 'h41 =65(dec) 
         break;
      case 8:                         // (77.32 ms) FWI=8
         WriteRawRC(RegTimerClock,0x0D); // TAutoRestart=0,TPrescale=64*128
         WriteRawRC(RegTimerReload,0x81);// TReloadVal = 'h81 =129(dec) 
         break;
      case 9:                         // (154.6 ms) FWI=9
         WriteRawRC(RegTimerClock,0x0F); // TAutoRestart=0,TPrescale=256*128
         WriteRawRC(RegTimerReload,0x41);// TReloadVal = 'h41 =65(dec) 
         break;
      case 10:                        // (309.3 ms) FWI=10
         WriteRawRC(RegTimerClock,0x0F); // TAutoRestart=0,TPrescale=256*128
         WriteRawRC(RegTimerReload,0x81);// TReloadVal = 'h81 =129(dec) 
         break;
      case 11:                        // (618.6 ms) FWI=11
         WriteRawRC(RegTimerClock,0x13); // TAutoRestart=0,TPrescale=4096*128
         WriteRawRC(RegTimerReload,0x11);// TReloadVal = 'h21 =17(dec) 
         break;
      case 12:                        // (1.2371 s) FWI=12
         WriteRawRC(RegTimerClock,0x13); // TAutoRestart=0,TPrescale=4096*128
         WriteRawRC(RegTimerReload,0x21);// TReloadVal = 'h41 =33(dec) 
         break;
      case 13:                        // (2.4742 s) FWI=13
         WriteRawRC(RegTimerClock,0x13); // TAutoRestart=0,TPrescale=4096*128
         WriteRawRC(RegTimerReload,0x41);// TReloadVal = 'h81 =65(dec) 
         break;
      case 14:                        // (4.9485 s) FWI=14
         WriteRawRC(RegTimerClock,0x13); // TAutoRestart=0,TPrescale=4096*128
         WriteRawRC(RegTimerReload,0x81);// TReloadVal = 'h81 =129(dec) 
         break;
      case 15:                        // (4.9485 s) FWI=14
         WriteRawRC(RegTimerClock,0x9); // TAutoRestart=0,TPrescale=4096*128
         WriteRawRC(RegTimerReload,0x0ff);// TReloadVal = 'h81 =129(dec) 
         break;
      default:                       // 
         WriteRawRC(RegTimerClock,0x19); // TAutoRestart=0,TPrescale=128
         WriteRawRC(RegTimerReload,tmoLength);// TReloadVal = 'h6a =tmoLength(dec) 
         break;
   }     
   WriteRawRC(RegTimerControl,0x06);
}

/////////////////////////////////////////////////////////////////////
//通过RC531和ISO14443卡通讯
//input: pi->MfCommand = RC632命令字
//       pi->MfLength  = 发送的数据长度
//       pi->MfData[]  = 发送数据
//output:status        = 错误字
//       pi->MfLength  = 接收的数据BIT长度
//       pi->MfData[]  = 接收数据
/////////////////////////////////////////////////////////////////////
char PcdComTransceive(struct TranSciveBuffer *pi)
{
   unsigned char recebyte = 0;
   char status;
   unsigned char irqEn   = 0x00;
   unsigned char waitFor = 0x00;
   unsigned char lastBits;
   unsigned char n;
   unsigned int i;

   switch (pi->MfCommand)
   {
      case PCD_IDLE:
         irqEn   = 0x00;
         waitFor = 0x00;
         break;
      case PCD_WRITEE2:
         irqEn   = 0x11;
         waitFor = 0x10;
         break;
      case PCD_READE2:
         irqEn   = 0x07;
         waitFor = 0x04;
         recebyte=1;
         break;
      case PCD_LOADCONFIG:
      case PCD_LOADKEYE2:
      case PCD_AUTHENT1:
         irqEn   = 0x05;
         waitFor = 0x04;
         break;
      case PCD_CALCCRC:
         irqEn   = 0x11;
         waitFor = 0x10;
         break;
      case PCD_AUTHENT2:
         irqEn   = 0x04;
         waitFor = 0x04;
         break;
      case PCD_RECEIVE:
         irqEn   = 0x06;
         waitFor = 0x04;
         recebyte=1;
         break;
      case PCD_LOADKEY:
         irqEn   = 0x05;
         waitFor = 0x04;
         break;
      case PCD_TRANSMIT:
         irqEn   = 0x05;
         waitFor = 0x04;
         break;
      case PCD_TRANSCEIVE:
         irqEn   = 0x3D;
         waitFor = 0x04;
         recebyte=1;
         break;
      default:
         pi->MfCommand = MI_UNKNOWN_COMMAND;
         break;
   }
   
   if (pi->MfCommand != MI_UNKNOWN_COMMAND)
   {
      WriteRawRC(RegPage,0x00);
      WriteRawRC(RegInterruptEn,0x7F);
      WriteRawRC(RegInterruptRq,0x7F);
      WriteRawRC(RegCommand,PCD_IDLE);
      SetBitMask(RegControl,0x01);
      WriteRawRC(RegInterruptEn,irqEn|0x80);
      for (i=0; i<pi->MfLength; i++)
      {
         WriteRawRC(RegFIFOData, pi->MfData[i]);
      }
      WriteRawRC(RegCommand, pi->MfCommand);
      i = 0x3500;
      do
      {
         n = ReadRawRC(RegInterruptRq);
         i--;
      }
      while ((i!=0) && !(n&irqEn&0x20) && !(n&waitFor));
      status = MI_COM_ERR;
      if ((i!=0) && !(n&irqEn&0x20))
      {
         if (!(ReadRawRC(RegErrorFlag)&0x17))
         {
            status = MI_OK;
            if (recebyte)
            {
              	n = ReadRawRC(RegFIFOLength);
              	lastBits = ReadRawRC(RegSecondaryStatus) & 0x07;
                if (lastBits)
                {
                   pi->MfLength = (n-1)*8 + lastBits;
                }
                else
                {
                   pi->MfLength = n*8;
                }
                if (n == 0)
                {
                   n = 1;
                }
                for (i=0; i<n; i++)
                {
                    pi->MfData[i] = ReadRawRC(RegFIFOData);
                }
            }
         }
		 else if (ReadRawRC(RegErrorFlag)&0x01)
         {
		    status = MI_COLLERR;
            if (recebyte)
            {
              	n = ReadRawRC(RegFIFOLength);
              	lastBits = ReadRawRC(RegSecondaryStatus) & 0x07;
                if (lastBits)
                {
                   pi->MfLength = (n-1)*8 + lastBits;
                }
                else
                {
                   pi->MfLength = n*8;
                }
                if (n == 0)
                {
                   n = 1;
                }
                for (i=0; i<n; i++)
                {
                    pi->MfData[i+1] = ReadRawRC(RegFIFOData);
                }
            }
			pi->MfData[0]=ReadRawRC(0x0B);
         }

      }
      else if (n & irqEn & 0x20)
      {   status = MI_NOTAGERR;   }
      else if (!(ReadRawRC(RegErrorFlag)&0x17))
      {   status = MI_ACCESSTIMEOUT;   }
      else
      {   status = MI_COM_ERR;    }
      
      WriteRawRC(RegInterruptEn,0x7F);
      WriteRawRC(RegInterruptRq,0x7F);
      SetBitMask(RegControl,0x04);           // stop timer now
      WriteRawRC(RegCommand,PCD_IDLE); 
   }
   return status;
}

/////////////////////////////////////////////////////////////////////
//开启天线  
//每次启动或关闭天险发射之间应至少有1ms的间隔
/////////////////////////////////////////////////////////////////////
char PcdAntennaOn(void)
{
    unsigned char i;
	while(1)
	{
    i = ReadRawRC(RegTxControl);
	printf("RegTxControl 0x%x\r\n", i);
	if(i == 0x3B)
	{
		ClearBitMask(RegTxControl, 0x03);
	}
    DelayMs(1000);
	}
	if (i & 0x03)
    {   
		printf("i==0x03\r\n");
		return MI_OK;	
	}
    else
    {
		printf("Set mask 0x03\r\n");
        SetBitMask(RegTxControl, 0x03);
        return MI_OK;
    }
}

/////////////////////////////////////////////////////////////////////
//关闭天线
/////////////////////////////////////////////////////////////////////
char PcdAntennaOff(void)
{
    ClearBitMask(RegTxControl, 0x03);
    return MI_OK;
}


