/*
*         Copyright (c), Philips Semiconductors Gratkorn / Austria
*
*                     (C)PHILIPS Electronics N.V.2000
*       All rights are reserved. Reproduction in whole or in part is 
*      prohibited without the written consent of the copyright owner.
*  Philips reserves the right to make changes without notice at any time.
* Philips makes no warranty, expressed, implied or statutory, including but
* not limited to any implied warranty of merchantability or fitness for any
*particular purpose, or that the use will not infringe any third party patent,
* copyright or trademark. Philips must not be liable for any loss or damage
*                          arising from its use.
*/

#ifndef MFRC500_H
#define MFRC500_H

// PCD Configuration
extern char Mf500PcdConfig(void);

// Active Antenna Slave Configuration of the MF RC500.
char Mf500ActiveAntennaSlaveConfig(void);

// Active Antenna Master Configuration of the MF RC 500
char Mf500ActiveAntennaMasterConfig(void);

// Set default attributes for the baudrate divider
char Mf500PcdSetDefaultAttrib(void);

// Set attributes for the baudrate divider 
char Mf500PcdSetAttrib(unsigned char DSI,
                           unsigned char DRI);

// Get transmission properties of the PCD
char Mf500PcdGetAttrib(unsigned char *FSCImax,
                          unsigned char *FSDImax,
                          unsigned char *DSsupp,
                          unsigned char *DRsupp,
                          unsigned char *DREQDS);

// PICC Request command
char Mf500PiccRequest(unsigned char req_code, 
                       unsigned char *atq);
                       
// PICC Request command for ISO 14443 A-4 Command set
char Mf500PiccCommonRequest(unsigned char req_code, 
                             unsigned char *atq);  

// PICC Anticollision Command
char Mf500PiccAnticoll (unsigned char bcnt,
                         unsigned char *snr);

// PICC Cascaded Anticollision Command
char Mf500PiccCascAnticoll (unsigned char select_code,
                             unsigned char bcnt,
                             unsigned char *snr);                     

// PICC Select Command
char Mf500PiccSelect(unsigned char *snr, 
                      unsigned char *sak);

// PICC Select Command
char Mf500PiccCascSelect(unsigned char select_code, 
                             unsigned char *snr, 
                             unsigned char *sak); 

// Activation of a PICC in IDLE mode
char Mf500PiccActivateIdle(unsigned char br,
                           unsigned char *atq, 
                           unsigned char *sak, 
                           unsigned char *uid, 
                           unsigned char *uid_len);

// Activation of all PICC's in the RF field
char Mf500PiccActivateWakeup(unsigned char br,
                             unsigned char *atq, 
                             unsigned char *sak,
                             unsigned char *uid, 
                             unsigned char uid_len);

// MIFARE®  Authentication
char Mf500PiccAuth(unsigned char auth_mode, 
                      unsigned char key_sector, 
                      unsigned char block);   
  
// MIFARE ® Authentication with  keys stored  in the MF RC 500's EEPROM.
char Mf500PiccAuthE2( unsigned char auth_mode, 
                         unsigned char *snr,      
                         unsigned char key_sector,
                         unsigned char block); 

// Authentication Key Coding
char Mf500HostCodeKey(unsigned char *uncoded, 
                         unsigned char *coded); 

// Key Loading into the MF RC500's EEPROM.
char Mf500PcdLoadKeyE2(unsigned char key_type,
                          unsigned char sector,
                          unsigned char *uncoded_keys); 
                     
// Authentication with direct key loading form the microcontroller
char Mf500PiccAuthKey(unsigned char auth_mode,
                         unsigned char *snr,   
                         unsigned char *keys,  
                         unsigned char sector);   
                     
// PICC Read Block
char Mf500PiccRead(unsigned char addr,  
                       unsigned char* data);

// PICC Read Block of variable length
char Mf500PiccCommonRead(unsigned char cmd,
                             unsigned char addr,  
                             unsigned char datalen,
                             unsigned char *data);
                  
// PICC Write Block
char Mf500PiccWrite(unsigned char addr,
                        unsigned char *data);

// PICC Write 4 Byte Block
char Mf500PiccWrite4(unsigned char addr,
                         unsigned char *data);
                       
// PICC Write Block of variable length
char Mf500PiccCommonWrite(unsigned char cmd,
                              unsigned char addr,
                              unsigned char datalen,
                              unsigned char *data);

// PICC Value Block Operation
char Mf500PiccValue(unsigned char dd_mode, 
                       unsigned char addr, 
                       unsigned char *value,
                       unsigned char trans_addr);

// PICC Value Block Operation for Cards with automatic transfer
char Mf500PiccValueDebit(unsigned char dd_mode, 
                             unsigned char addr, 
                             unsigned char *value);

// Exchange Data Blocks PCD --> PICC --> PCD
char Mf500PiccExchangeBlock(unsigned char *send_data,
                               unsigned short send_bytelen,
                               unsigned char *rec_data,  
                               unsigned short *rec_bytelen,
                               unsigned char append_crc, 
                               unsigned long timeout );                  

// PICC Halt
char Mf500PiccHalt(void);

// Reset the reader ic 
char PcdReset(void);

// Exchange Data Stream PCD --> PICC --> PCD
char ExchangeByteStream(unsigned char Cmd,
                            unsigned char *send_data,
                            unsigned short send_bytelen,
                            unsigned char *rec_data,  
                            unsigned short *rec_bytelen);

// Set RF communication timeout 
char PcdSetTmo(unsigned long numberOfEtus);

// Read Serial Number from Reader IC
char PcdGetSnr(unsigned char *snr);

// Read EEPROM Memory Block
char PcdReadE2(unsigned short startaddr,  
                   unsigned char length,
                   unsigned char* data);

// Writes data to the reader IC's EEPROM blocks.
char PcdWriteE2(  unsigned short startaddr,
                      unsigned char length,
                      unsigned char* data);

// Turns ON/OFF RF field
char PcdRfReset(unsigned short ms);    

#endif
