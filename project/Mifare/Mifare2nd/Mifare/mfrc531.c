
#include <intrins.h>
#include <reg931.h>
#include "mfrc531.h"
#include "main.h"


#define GetRegPage(addr)        (0x80 | (addr>>3)) 


///////////////////////////////////////////////////////////////////////////////
//          				 Time Delay
///////////////////////////////////////////////////////////////////////////////

void DelayMs(unsigned char dat)
{
	unsigned char i,k;
	unsigned int j;
	
	k = dat;
	for (i = 0; i < k; i ++)
	{
		for (j = 0; j < 675; j ++)
			{;}
	}
}

///////////////////////////////////////////////////////////////////////////////
//          				 RC531_Init
///////////////////////////////////////////////////////////////////////////////
void RC531Init(void)
{	

   SleepMs(500);        // wait after POR    
   READER_RESET;        // reset reader IC
   SleepMs(1500);        // wait
   READER_CLEAR_RESET; // clear reset pin
   SleepUs(1000);

	RC531_RST = 0;	
 	DelayMs(20);
 
	RC531_RST = 1;
	DelayMs(20);

	RC531_RST = 0;
	DelayMs(20);
}

//////////////////////////////////////////////////////////////////////////////
//								WriteRawIO
/////////////////////////////////////////////////////////////////////////////
void WriteRawIO(unsigned char addr, unsigned char dat)
{  	
	P0 = addr;
	_nop_();
	RC531_ALE = 1;
	_nop_();
	RC531_ALE = 0;
	_nop_();	

	RC531_CS = 0;
	P0 = dat;
	RC531_WR = 0;
	_nop_();     
	RC531_WR = 1;
	RC531_CS = 1;
}

//////////////////////////////////////////////////////////////////////////////
//								ReadRawIO
/////////////////////////////////////////////////////////////////////////////
unsigned char ReadRawIO(unsigned char addr)
{
	unsigned char dat;
	
	P0 = addr;
	_nop_();
	RC531_ALE = 1;
	_nop_();
	RC531_ALE = 0;
	_nop_();	
	
	P0 = 0xff;
	RC531_CS = 0;
	RC531_RD = 0;
	dat = P0;
	_nop_();
	RC531_RD = 1;
	RC531_CS = 1;
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
//��λ����ʼ��RC531
//ע��:RC531�ϵ��Ӧ��ʱ500ms���ܿɿ���ʼ��
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
//����RC531�Ĺ�����ʽ 
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
//��RC531�Ĵ���λ
//input:reg=�Ĵ�����ַ
//      mask=��λֵ
/////////////////////////////////////////////////////////////////////
void SetBitMask(unsigned char reg,unsigned char mask)  
{
   char tmp = 0x0;
   tmp = ReadRawRC(reg);
   WriteRawRC(reg,tmp | mask);  // set bit mask
}

/////////////////////////////////////////////////////////////////////
//��RC531�Ĵ���λ
//input:reg=�Ĵ�����ַ
//      mask=��λֵ
/////////////////////////////////////////////////////////////////////
void ClearBitMask(unsigned char reg,unsigned char mask)  
{
   char tmp = 0x0;
   tmp = ReadRawRC(reg);
   WriteRawRC(reg, tmp & ~mask);  // clear bit mask
} 

/////////////////////////////////////////////////////////////////////
//����RC531��ʱ��
//input:tmolength=����ֵ
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
//ͨ��RC531��ISO14443��ͨѶ
//input: pi->MfCommand = RC632������
//       pi->MfLength  = ���͵����ݳ���
//       pi->MfData[]  = ��������
//output:status        = ������
//       pi->MfLength  = ���յ�����BIT����
//       pi->MfData[]  = ��������
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
//��������  
//ÿ��������ر����շ���֮��Ӧ������1ms�ļ��
/////////////////////////////////////////////////////////////////////
char PcdAntennaOn(void)
{
    unsigned char i;
    i = ReadRawRC(RegTxControl);
    if (i & 0x03)
    {   return MI_OK;	}
    else
    {
        SetBitMask(RegTxControl, 0x03);
        return MI_OK;
    }
}

/////////////////////////////////////////////////////////////////////
//�ر�����
/////////////////////////////////////////////////////////////////////
char PcdAntennaOff(void)
{
    ClearBitMask(RegTxControl, 0x03);
    return MI_OK;
}


