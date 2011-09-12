
#include <string.h>
#include "mfrc531.h" 
#include "iso14443a.h"

#define FSD 64              //RC500 FIFO BUFFER SIZE
                              
/////////////////////////////////////////////////////////////////////
//功    能：寻卡
//参数说明: req_code[IN]:寻卡方式
//                0x52 = 寻感应区内所有符合14443A标准的卡
//                0x26 = 寻未进入休眠状态的卡
//         pTagType[OUT]：卡片类型代码
//                0x4400 = Mifare_UltraLight
//                0x0400 = Mifare_One(S50)
//                0x0200 = Mifare_One(S70)
//                0x0800 = Mifare_Pro
//                0x0403 = Mifare_ProX
//                0x4403 = Mifare_DESFire
//返    回: 成功返回MI_OK
/////////////////////////////////////////////////////////////////////
char PcdRequest(unsigned char req_code,unsigned char *pTagType)
{
   char status;   
   struct TranSciveBuffer MfComData;
   struct TranSciveBuffer *pi;
   pi = &MfComData;

   WriteRawRC(RegChannelRedundancy,0x03);
   ClearBitMask(RegControl,0x08);
   WriteRawRC(RegBitFraming,0x07);
   SetBitMask(RegTxControl,0x03);
   PcdSetTmo(4);
   MfComData.MfCommand = PCD_TRANSCEIVE;
   MfComData.MfLength  = 1;
   MfComData.MfData[0] = req_code;

   status = PcdComTransceive(pi);
//#ifdef DEBUG_UART
	printf("PcdComTransceive 0x%x\r\n", status);
//#endif	
   
   if (!status)
   {    
        if (MfComData.MfLength != 0x10)
        {   status = MI_BITCOUNTERR;   }
   }
//   *pTagType     = MfComData.MfData[0];
//   *(pTagType+1) = MfComData.MfData[1];
   memcpy(&MfComData.MfData[0], pTagType, 2); 
   
   return status;
}

/////////////////////////////////////////////////////////////////////
//防冲撞
//input: g_cSNR=存放序列号(4byte)的内存单元首地址
//output:status=MI_OK:成功
//       得到的序列号放入指定单元
/////////////////////////////////////////////////////////////////////
char PcdAnticoll(unsigned char *pSnr)
{
    char status ;
    unsigned char i;
    unsigned char ucBits;
    unsigned char ucBytes;
    unsigned char snr_check = 0;
    unsigned char ucCollPosition = 0;
    unsigned char ucTemp;
    unsigned char ucSNR[5] = {0, 0, 0, 0 ,0};
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;

    WriteRawRC(RegDecoderControl,0x28);
    ClearBitMask(RegControl,0x08);
    WriteRawRC(RegChannelRedundancy,0x03);
    PcdSetTmo(4);
    

    do
    {
        ucBits = (ucCollPosition) % 8;
        if (ucBits != 0)
        {
             ucBytes = ucCollPosition / 8 + 1;
             WriteRawRC(RegBitFraming, (ucBits << 4) + ucBits);
        }
        else
        {
             ucBytes = ucCollPosition / 8;
        }
	
        MfComData.MfCommand = PCD_TRANSCEIVE;
        MfComData.MfData[0] = PICC_ANTICOLL1;
        MfComData.MfData[1] = 0x20 + ((ucCollPosition / 8) << 4) + (ucBits & 0x0F);
        for (i=0; i<ucBytes; i++)
	    {
	        MfComData.MfData[i + 2] = ucSNR[i];
	    }
	    MfComData.MfLength = ucBytes + 2;
	
	    status = PcdComTransceive(pi);
	
	    ucTemp = ucSNR[(ucCollPosition / 8)];
	    if (status == MI_COLLERR)
	    {
	        for (i=0; i < 5 - (ucCollPosition / 8); i++)
	        {
		         ucSNR[i + (ucCollPosition / 8)] = MfComData.MfData[i+1];
	        }
	        ucSNR[(ucCollPosition / 8)] |= ucTemp;
	        ucCollPosition = MfComData.MfData[0];
        }
        else if (status == MI_OK)
        {
            for (i=0; i < (MfComData.MfLength / 8); i++)
            {
                 ucSNR[4 - i] = MfComData.MfData[MfComData.MfLength/8 - i - 1];
            }
            ucSNR[(ucCollPosition / 8)] |= ucTemp;
        }
    } while (status == MI_COLLERR);
			
			
    if (status == MI_OK)
    {
    	 for (i=0; i<4; i++)
         {   
             *(pSnr+i)  = ucSNR[i];
             snr_check ^= ucSNR[i];
         }
         if (snr_check != ucSNR[i])
         {   status = MI_COM_ERR;    }
    }
    
    ClearBitMask(RegDecoderControl,0x20);
    return status;
}

/////////////////////////////////////////////////////////////////////
//选定一张卡
//input:g_cSNR=序列号
/////////////////////////////////////////////////////////////////////
char PcdSelect(unsigned char *pSnr, unsigned char *pSize)
{
    unsigned char i;
    char status;
    unsigned char snr_check = 0;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;

	pSize = pSize;

    WriteRawRC(RegChannelRedundancy,0x0F);
    ClearBitMask(RegControl,0x08);
    PcdSetTmo(4);
    
    MfComData.MfCommand = PCD_TRANSCEIVE;
    MfComData.MfLength  = 7;
    MfComData.MfData[0] = PICC_ANTICOLL1;
    MfComData.MfData[1] = 0x70;
    for (i=0; i<4; i++)
    {
    	snr_check ^= *(pSnr+i);
    	MfComData.MfData[i+2] = *(pSnr+i);
    }
    MfComData.MfData[6] = snr_check;

    status = PcdComTransceive(pi);
	 
    if (status == MI_OK)
    {
        if (MfComData.MfLength != 0x08)
        {   status = MI_BITCOUNTERR;   }
        else
        {  pSize = &MfComData.MfData[0];  }
    }

    return status;
}

/////////////////////////////////////////////////////////////////////
//将Mifare_One卡密钥转换为RC500接收格式
//input: uncoded=6字节未转换的密钥
//output:coded=12字节转换后的密钥
/////////////////////////////////////////////////////////////////////
char ChangeCodeKey(unsigned char *pUncoded,unsigned char *pCoded)
{
   unsigned char cnt=0;
   unsigned char ln=0;
   unsigned char hn=0;

   for (cnt=0; cnt<6; cnt++)
   {
      ln = pUncoded[cnt] & 0x0F;
      hn = pUncoded[cnt] >> 4;
      pCoded[cnt*2+1] = (~ln<<4) | ln;
      pCoded[cnt*2]   = (~hn<<4) | hn;
   }
   return MI_OK;
}

/////////////////////////////////////////////////////////////////////
//功能：将已转换格式后的密钥送到RC500的FIFO中
//input:keys=密钥
/////////////////////////////////////////////////////////////////////
char PcdAuthKey(unsigned char *pKeys)
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;

    PcdSetTmo(4);
    MfComData.MfCommand = PCD_LOADKEY;
    MfComData.MfLength  = 12;
    memcpy(&MfComData.MfData[0], pKeys, 12);    

    status = PcdComTransceive(pi);

    return status;
}

/////////////////////////////////////////////////////////////////////
//功能：用存放RC500的FIFO中的密钥和卡上的密钥进行验证
//input:auth_mode=验证方式,0x60:验证A密钥,0x61:验证B密钥
//      block=要验证的绝对块号
//      g_cSNR=序列号首地址
/////////////////////////////////////////////////////////////////////
char PcdAuthState(unsigned char auth_mode,unsigned char block,unsigned char *pSnr)
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;

    WriteRawRC(RegChannelRedundancy,0x0F);
    PcdSetTmo(4);
    MfComData.MfCommand = PCD_AUTHENT1;
    MfComData.MfLength  = 6;
    MfComData.MfData[0] = auth_mode;
    MfComData.MfData[1] = block;
    memcpy(&MfComData.MfData[2], pSnr, 4);    
      
    status = PcdComTransceive(pi);
    if (status == MI_OK)
    {
        if (ReadRawRC(RegSecondaryStatus) & 0x07) 
        {   status = MI_BITCOUNTERR;    }
        else
        {
             MfComData.MfCommand = PCD_AUTHENT2;
             MfComData.MfLength  = 0;
             status = PcdComTransceive(pi);
             if (status == MI_OK)
             {
                 if (ReadRawRC(RegControl) & 0x08)
                 {   status = MI_OK;   }
                 else
                 {   status = MI_AUTHERR;   }
                
             }
         }
    }
    return status;
}

/////////////////////////////////////////////////////////////////////
//读mifare_one卡上一块(block)数据(16字节)
//input: addr = 要读的绝对块号
//output:readdata = 读出的数据
/////////////////////////////////////////////////////////////////////
char PcdRead(unsigned char addr,unsigned char *pReaddata)
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;

    PcdSetTmo(4);
    WriteRawRC(RegChannelRedundancy,0x0F);
    MfComData.MfCommand = PCD_TRANSCEIVE;
    MfComData.MfLength  = 2;
    MfComData.MfData[0] = PICC_READ;
    MfComData.MfData[1] = addr;

    status = PcdComTransceive(pi);
    if (status == MI_OK)
    {
        if (MfComData.MfLength != 0x80)
        {   status = MI_BITCOUNTERR;   }
        else
        {   memcpy(pReaddata, &MfComData.MfData[0], 16);  }
    }
    return status;
}

/////////////////////////////////////////////////////////////////////
//写数据到卡上的一块
//input:adde=要写的绝对块号
//      writedata=写入数据
/////////////////////////////////////////////////////////////////////
char PcdWrite(unsigned char addr,unsigned char *pWritedata)
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;
    
    PcdSetTmo(5);
    WriteRawRC(RegChannelRedundancy,0x07); 
    MfComData.MfCommand = PCD_TRANSCEIVE;
    MfComData.MfLength  = 2;
    MfComData.MfData[0] = PICC_WRITE;
    MfComData.MfData[1] = addr;

    status = PcdComTransceive(pi);
    if (status != MI_NOTAGERR)
    {
        if (MfComData.MfLength != 4)
        {   status=MI_BITCOUNTERR;   }
        else
        {
           MfComData.MfData[0] &= 0x0F;
           switch (MfComData.MfData[0])
           {
              case 0x00:
                 status = MI_NOTAUTHERR;
                 break;
              case 0x0A:
                 status = MI_OK;
                 break;
              default:
                 status = MI_CODEERR;
                 break;
           }
        }
    }
    if (status == MI_OK)
    {
        PcdSetTmo(5);
        MfComData.MfCommand = PCD_TRANSCEIVE;
        MfComData.MfLength  = 16;
        memcpy(&MfComData.MfData[0], pWritedata, 16);
        
        status = PcdComTransceive(pi);
        if (status != MI_NOTAGERR)
        {
            MfComData.MfData[0] &= 0x0F;
            switch(MfComData.MfData[0])
            {
               case 0x00:
                  status = MI_WRITEERR;
                  break;
               case 0x0A:
                  status = MI_OK;
                  break;
               default:
                  status = MI_CODEERR;
                  break;
           }
        }
        PcdSetTmo(4);
    }
    return status;
}

/////////////////////////////////////////////////////////////////////
//扣款和充值
//input:dd_mode=命令字,0xC0:扣款,0xC1:充值
//      addr=钱包的绝对块号
//      value=4字节增(减)值首地址,16进制数,低位在前
/////////////////////////////////////////////////////////////////////
char PcdValue(unsigned char dd_mode,unsigned char addr,unsigned char *pValue)
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;

    PcdSetTmo(5);
//	PcdSetTmo(2);
    WriteRawRC(RegChannelRedundancy,0x0F);
    MfComData.MfCommand = PCD_TRANSCEIVE;
    MfComData.MfLength  = 2;
    MfComData.MfData[0] = dd_mode;
    MfComData.MfData[1] = addr;

    status = PcdComTransceive(pi);
    if (status != MI_NOTAGERR)
    {
        if (MfComData.MfLength != 4)
        {   status = MI_BITCOUNTERR;   }
        else
        {
           MfComData.MfData[0] &= 0x0F;
           switch (MfComData.MfData[0])
           {
              case 0x00:
                 status = MI_NOTAUTHERR;
                 break;
              case 0x0A:
                 status = MI_OK;
                 break;
              case 0x01:
                 status = MI_VALERR;
                 break;
              default:
                 status = MI_CODEERR;
                 break;
           }
        }
     }
     if (status == MI_OK)
     {
        PcdSetTmo(5);
        MfComData.MfCommand = PCD_TRANSCEIVE;
        MfComData.MfLength  = 4;
        pi = &MfComData;
        memcpy(&MfComData.MfData[0], pValue, 4);

        status = PcdComTransceive(pi);
        if (status==MI_OK)
        {
           if (MfComData.MfLength != 4)
           {   status = MI_BITCOUNTERR;   }
           else
           {   status = MI_OK;            }
        }
        else if(status == MI_NOTAGERR)
        {   status = MI_OK;    }
        else
        {   status=MI_COM_ERR;     }
     }
     
     if (status == MI_OK)
     {
        MfComData.MfCommand = PCD_TRANSCEIVE;
        MfComData.MfLength  = 2;
        MfComData.MfData[0] = PICC_TRANSFER;
        MfComData.MfData[1] = addr;
        
        status = PcdComTransceive(pi);
        if (status != MI_NOTAGERR)
        {
            if (MfComData.MfLength != 4)
            {   status = MI_BITCOUNTERR;    }
            else
            {
               MfComData.MfData[0] &= 0x0F;
               switch(MfComData.MfData[0])
               {
                  case 0x00:
                     status = MI_NOTAUTHERR;
                     break;
                  case 0x0a:
                     status = MI_OK;
                     break;
                  case 0x01:
                     status = MI_VALERR;
                     break;
                  default:
                     status = MI_CODEERR;
                     break;
               }
            }
        }
     }
     return status;
}


/////////////////////////////////////////////////////////////////////
//将mifare_one卡一块数据调入卡缓冲区
//input:addr=绝对块号
/////////////////////////////////////////////////////////////////////
char PcdRestore(unsigned char addr)
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;

    PcdSetTmo(4);
    MfComData.MfCommand = PCD_TRANSCEIVE;
    MfComData.MfLength  = 2;
    MfComData.MfData[0] = PICC_RESTORE;
    MfComData.MfData[1] = addr;

    status = PcdComTransceive(pi);
    if (status != MI_NOTAGERR)
    {
        if (MfComData.MfLength != 4)
        {   status = MI_BITCOUNTERR;   }
        else
        {
           MfComData.MfData[0] &= 0x0F;
           switch(MfComData.MfData[0])
           {
              case 0x00:
                 status = MI_NOTAUTHERR;
                 break;
              case 0x0A:
                 status = MI_OK;
                 break;
              case 0x01:
                 status = MI_VALERR;
                 break;
              default:
                 status = MI_CODEERR;
                 break;
           }
        }
     }
     if (status == MI_OK)
     {
        PcdSetTmo(4);
        MfComData.MfCommand = PCD_TRANSCEIVE;
        MfComData.MfLength  = 4;
        MfComData.MfData[0] = 0;
        MfComData.MfData[1] = 0;
        MfComData.MfData[2] = 0;
        MfComData.MfData[3] = 0;
        
        status = PcdComTransceive(pi);
        if (status == MI_NOTAGERR)
        {   status = MI_OK;    }
     }
     return status;
}

/////////////////////////////////////////////////////////////////////
//将卡缓冲区中数据保存到块
//input:addr=绝对块号
/////////////////////////////////////////////////////////////////////
char PcdTransfer(unsigned char addr)
{
    char status;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;
    pi = &MfComData;

    PcdSetTmo(5);
    MfComData.MfCommand = PCD_TRANSCEIVE;
    MfComData.MfLength  = 2;
    MfComData.MfData[0] = PICC_TRANSFER;
    MfComData.MfData[1] = addr;

    status = PcdComTransceive(pi);
    if (status != MI_NOTAGERR)
    {
        if (MfComData.MfLength != 4)
        {  status = MI_BITCOUNTERR;    }
        else
        {
           MfComData.MfData[0] &= 0x0F;
           switch (MfComData.MfData[0])
           {
              case 0x00:
                 status = MI_NOTAUTHERR;
                 break;
              case 0x0A:
                 status = MI_OK;
                 break;
              case 0x01:
                 status = MI_VALERR;
                 break;
              default:
                 status = MI_CODEERR;
                 break;
           }
        }
     }
     return status;
}

/////////////////////////////////////////////////////////////////////
//命令卡进入休眠状态
/////////////////////////////////////////////////////////////////////
char PcdHalt(void)
{
    char status = MI_OK;
    struct TranSciveBuffer MfComData;
    struct TranSciveBuffer *pi;

    pi = &MfComData;

    MfComData.MfCommand = PCD_TRANSCEIVE;
    MfComData.MfLength  = 2;
    MfComData.MfData[0] = PICC_HALT;
    MfComData.MfData[1] = 0;

    status = PcdComTransceive(pi);
    if (status)
    {
        if (status==MI_NOTAGERR || status==MI_ACCESSTIMEOUT)
        status = MI_OK;
    }
    WriteRawRC(RegCommand,PCD_IDLE);
    return status;
}




