
#include <string.h>
#include "mfrc531.h" 
#include "iso14443a.h"

#define FSD 64              //RC500 FIFO BUFFER SIZE
                              
/////////////////////////////////////////////////////////////////////
//��    �ܣ�Ѱ��
//����˵��: req_code[IN]:Ѱ����ʽ
//                0x52 = Ѱ��Ӧ�������з���14443A��׼�Ŀ�
//                0x26 = Ѱδ��������״̬�Ŀ�
//         pTagType[OUT]����Ƭ���ʹ���
//                0x4400 = Mifare_UltraLight
//                0x0400 = Mifare_One(S50)
//                0x0200 = Mifare_One(S70)
//                0x0800 = Mifare_Pro
//                0x0403 = Mifare_ProX
//                0x4403 = Mifare_DESFire
//��    ��: �ɹ�����MI_OK
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
//����ײ
//input: g_cSNR=������к�(4byte)���ڴ浥Ԫ�׵�ַ
//output:status=MI_OK:�ɹ�
//       �õ������кŷ���ָ����Ԫ
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
//ѡ��һ�ſ�
//input:g_cSNR=���к�
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
//��Mifare_One����Կת��ΪRC500���ո�ʽ
//input: uncoded=6�ֽ�δת������Կ
//output:coded=12�ֽ�ת�������Կ
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
//���ܣ�����ת����ʽ�����Կ�͵�RC500��FIFO��
//input:keys=��Կ
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
//���ܣ��ô��RC500��FIFO�е���Կ�Ϳ��ϵ���Կ������֤
//input:auth_mode=��֤��ʽ,0x60:��֤A��Կ,0x61:��֤B��Կ
//      block=Ҫ��֤�ľ��Կ��
//      g_cSNR=���к��׵�ַ
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
//��mifare_one����һ��(block)����(16�ֽ�)
//input: addr = Ҫ���ľ��Կ��
//output:readdata = ����������
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
//д���ݵ����ϵ�һ��
//input:adde=Ҫд�ľ��Կ��
//      writedata=д������
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
//�ۿ�ͳ�ֵ
//input:dd_mode=������,0xC0:�ۿ�,0xC1:��ֵ
//      addr=Ǯ���ľ��Կ��
//      value=4�ֽ���(��)ֵ�׵�ַ,16������,��λ��ǰ
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
//��mifare_one��һ�����ݵ��뿨������
//input:addr=���Կ��
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
//���������������ݱ��浽��
//input:addr=���Կ��
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
//�����������״̬
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




