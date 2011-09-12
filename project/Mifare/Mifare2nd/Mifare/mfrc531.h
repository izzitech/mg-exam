
#ifndef __MFRC531_H__
#define __MFRC531_H__

/////////////////////////////////////////////////////////////////////
//����ԭ��
/////////////////////////////////////////////////////////////////////
struct TranSciveBuffer{unsigned char MfCommand;
                              unsigned int  MfLength;
                              unsigned char MfData[64];
                             };

//��ʱ����
extern void DelayMs(unsigned int ms);

//��λRC632
extern char PcdReset(void);      

//����RC632�Ĺ�����ʽ    
extern void PcdConfigISOType(void);                                    

//��RC632-EEPROM����
extern char PcdReadE2(unsigned int startaddr,                     
               unsigned char length,
               unsigned char *readdata);    

//д���ݵ�RC632-EEPROM               
extern char PcdWriteE2(unsigned int startaddr,
                unsigned char length,
                unsigned char *writedata); 

//����RC632���߷��� 
extern char PcdAntennaOn(void);   

//�ر�RC632���߷���                                         
extern char PcdAntennaOff(void); 

//дRC632�Ĵ���
extern void WriteRawRC(unsigned char Address,unsigned char value);

//��RC632�Ĵ���                     
extern unsigned char ReadRawRC(unsigned char Address); 

extern unsigned char ReadE2RC(unsigned char addr);

//��RC632�Ĵ���λ                                
extern void SetBitMask(unsigned char reg,unsigned char mask); 

//��RC632�Ĵ���λ                         
extern void ClearBitMask(unsigned char reg,unsigned char mask); 

//����RC632��ʱ��                       
extern void PcdSetTmo(unsigned char tmoLength);                                

//ISO14443ͨѶ����
extern char PcdComTransceive(struct TranSciveBuffer *pi);

/////////////////////////////////////////////////////////////////////
//RC632������
/////////////////////////////////////////////////////////////////////
#define PCD_IDLE              0x00               //ȡ����ǰ����
#define PCD_WRITEE2           0x01               //дEEPROM
#define PCD_READE2            0x03               //��EEPROM
#define PCD_LOADCONFIG        0x07               //��EEPROM�б���ļĴ�������
#define PCD_LOADKEYE2         0x0B               //��EEPROM�б������Կ���뻺��
#define PCD_AUTHENT1          0x0C               //��֤��Կ��һ��
#define PCD_AUTHENT2          0x14               //��֤��Կ�ڶ���
#define PCD_RECEIVE           0x16               //��������
#define PCD_LOADKEY           0x19               //������Կ
#define PCD_TRANSMIT          0x1A               //��������
#define PCD_TRANSCEIVE        0x1E               //���Ͳ���������
#define PCD_RESETPHASE        0x3F               //��λ
#define PCD_CALCCRC           0x12               //CRC����

/////////////////////////////////////////////////////////////////////
//RC632 FIFO���ȶ���
/////////////////////////////////////////////////////////////////////
#define DEF_FIFO_LENGTH       64                 //FIFO size=64byte

/////////////////////////////////////////////////////////////////////
//RC632�Ĵ�������
/////////////////////////////////////////////////////////////////////
// PAGE 0
#define     RegPage               0x00    
#define     RegCommand            0x01    
#define     RegFIFOData           0x02    
#define     RegPrimaryStatus      0x03    
#define     RegFIFOLength         0x04    
#define     RegSecondaryStatus    0x05
#define     RegInterruptEn        0x06    
#define     RegInterruptRq        0x07    
// PAGE 1      Control and Status
#define     RegControl            0x09    
#define     RegErrorFlag          0x0A
#define     RegCollpos            0x0B
#define     RegTimerValue         0x0C
#define     RegCRCResultLSB       0x0D
#define     RegCRCResultMSB       0x0E
#define     RegBitFraming         0x0F
// PAGE 2      Transmitter and Coder Control
#define     RegTxControl          0x11
#define     RegCwConductance      0x12
#define     RegModConductance     0x13
#define     RegCoderControl       0x14
#define     RegModWidth           0x15
#define     RegModWidthSOF        0x16
#define     RegTypeBFraming       0x17
// PAGE 3      Receiver and Decoder Control
#define     RegRxControl1         0x19
#define     RegDecoderControl     0x1A
#define     RegBitPhase           0x1B
#define     RegRxThreshold        0x1C
#define     RegBPSKDemControl     0x1D
#define     RegRxControl2         0x1E
#define     RegClockQControl      0x1F
// PAGE 4      RF-Timing and Channel Redundancy
#define     RegRxWait             0x21
#define     RegChannelRedundancy  0x22
#define     RegCRCPresetLSB       0x23
#define     RegCRCPresetMSB       0x24
#define     RegTimeSlotPeriod     0x25
#define     RegMfOutSelect        0x26
#define     RFU27                 0x27
// PAGE 5      FIFO, Timer and IRQ-Pin Configuration
#define     RegFIFOLevel          0x29
#define     RegTimerClock         0x2A
#define     RegTimerControl       0x2B
#define     RegTimerReload        0x2C
#define     RegIRqPinConfig       0x2D
#define     RFU2E                 0x2E
#define     RFU2F                 0x2F
// PAGE 6      RFU
#define     RFU31                 0x31
#define     RFU32                 0x32
#define     RFU33                 0x33
#define     RFU34                 0x34
#define     RFU35                 0x35
#define     RFU36                 0x36
#define     RFU37                 0x37
// PAGE 7      Test Control
#define     RFU39                 0x39  
#define     RegTestAnaSelect      0x3A   
#define     RFU3B                 0x3B   
#define     RFU3C                 0x3C   
#define     RegTestDigiSelect     0x3D   
#define     RFU3E                 0x3E   
#define     RFU3F		  0x3F

/////////////////////////////////////////////////////////////////////
//��RC632ͨѶʱ���صĴ������
/////////////////////////////////////////////////////////////////////

#define MI_OK                          0
#define MI_CHK_OK                      0

#define MI_NOTAGERR                    1
#define MI_AUTHERR                     4
#define MI_CODEERR                     6
#define MI_NOTAUTHERR                  10
#define MI_BITCOUNTERR                 11
#define MI_WRITEERR                    15
#define MI_UNKNOWN_COMMAND             23
#define MI_COLLERR                     24
#define MI_RESETERR                    25
#define MI_ACCESSTIMEOUT               27
#define MI_VALERR                      124
#define MI_COM_ERR                     125

#endif

