
#ifndef __ISO14443A_H__
#define __ISO14443A_H__

/////////////////////////////////////////////////////////////////////
//ISO14443A COMMAND
/////////////////////////////////////////////////////////////////////
#define PICC_REQIDL           0x26               //Ѱ��������δ��������״̬
#define PICC_REQALL           0x52               //Ѱ��������ȫ����
#define PICC_ANTICOLL1        0x93               //����ײ
#define PICC_ANTICOLL2        0x95               //����ײ
#define PICC_AUTHENT1A        0x60               //��֤A��Կ
#define PICC_AUTHENT1B        0x61               //��֤B��Կ
#define PICC_READ             0x30               //����
#define PICC_WRITE            0xA0               //д��
#define PICC_DECREMENT        0xC0               //�ۿ�
#define PICC_INCREMENT        0xC1               //��ֵ
#define PICC_RESTORE          0xC2               //�������ݵ�������
#define PICC_TRANSFER         0xB0               //���滺����������
#define PICC_HALT             0x50               //����
#define PICC_RESET            0xE0               //��λ

/////////////////////////////////////////////////////////////////////
//����ԭ��
/////////////////////////////////////////////////////////////////////
extern char PcdRequest(unsigned char req_code,unsigned char *pTagType);                     
extern char PcdAnticoll(unsigned char *pSnr);                                   
extern char PcdSelect(unsigned char *pSnr,unsigned char *pSize);                        
extern char PcdHalt(void);                                            

extern char ChangeCodeKey(unsigned char *pUncoded,unsigned char *pCoded);                
extern char PcdAuthKey(unsigned char *pCoded);                                  
extern char PcdLoadKeyE2(unsigned int startaddr);
extern char PcdAuthState(unsigned char auth_mode,unsigned char block,unsigned char *pSnr);      
extern char PcdRead(unsigned char addr,unsigned char *pReaddata);                       
extern char PcdWrite(unsigned char addr,unsigned char *pWritedata);                     
extern char PcdValue(unsigned char dd_mode,unsigned char addr,unsigned char *pValue);           
extern char PcdRestore(unsigned char addr);                                    
extern char PcdTransfer(unsigned char addr);

#endif                                  


	  

