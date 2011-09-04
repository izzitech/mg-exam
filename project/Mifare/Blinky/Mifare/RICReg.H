//////////////////////////////////////////////////////////////////////////////
//    Copyright (c), Philips Semiconductors Gratkorn
//
//                     (C)PHILIPS Electronics N.V. 2000
//                          All rights are reserved. 
//  Philips reserves the right to make changes without notice at any time.
// Philips makes no warranty, expressed, implied or statutory, including but
// not limited to any implied warranty of merchantibility or fitness for any
//particular purpose, or that the use will not infringe any third party patent,
// copyright or trademark. Philips must not be liable for any loss or damage
//                          arising from its use.
//////////////////////////////////////////////////////////////////////////////
/*! \file RICReg.h
*
* Register Setting of the reader IC
*/
#ifndef RICREG_H
#define RICREG_H

#ifdef __cplusplus
extern "C"
{
#endif

// PAGE 0      Command and Status
#define     RegPage               0x00   //!< Page Select Register
#define     RegCommand            0x01   //!< Command Register
#define     RegFIFOData           0x02   //!< FiFo Register
#define     RegPrimaryStatus      0x03   //!< Modem State/IRQ/ERR/LoHiAlert Reg
#define     RegFIFOLength         0x04   //!< Buffer length Register
#define     RegSecondaryStatus    0x05   //!< diverse status flags
#define     RegInterruptEn        0x06   //!< IRQ enable Register
#define     RegInterruptRq        0x07   //!< IRQ bits Register
// PAGE 1      Control and Status
#define     RegControl            0x09   //!< processor control
#define     RegErrorFlag          0x0A   /*!< error flags showing the error 
                                         status of the last command executed */
#define     RegCollPos            0x0B   /*!< bit position of the first bit 
                                         collision detected on the 
                                         RF-interface */
#define     RegTimerValue         0x0C   //!< preload value of the timer
#define     RegCRCResultLSB       0x0D   //!< LSB of the CRC Coprocessor register
#define     RegCRCResultMSB       0x0E   //!< MSB of the CRC Coprocessor register
#define     RegBitFraming         0x0F   //!< Adjustments for bit oriented frames
// PAGE 2      Transmitter and Coder Control
#define     RegTxControl          0x11   //!< controls the logical behaviour of 
                                         //!< the antenna driver pins TX1 and TX2
#define     RegCwConductance      0x12   /*!< selects the conductance of the 
                                         antenna driver pins TX1 and TX2 */
#define     RFU13                 0x13   //!< RFU
#define     RegModConductance     0x13
#define     RegCoderControl       0x14   //!< selects coder rate
#define     RegModWidth           0x15   /*!< selects the width of the 
                                         modulation pulse */
#define     RFU16                 0x16   //!< RFU
#define     RegModWidthSOF        0x16
#define     RFU17                 0x17   //!< RFU
#define     RegTypeBFraming       0x17
// PAGE 3      Receiver and Decoder Control
#define     RegRxControl1         0x19   //!< controls receiver behaviour
#define     RegDecoderControl     0x1A   //!< controls decoder behaviour
#define     RegBitPhase           0x1B   /*!< selets the bit phase between 
                                         transmitter and receiver clock */
#define     RegRxThreshold        0x1C   /*!< selects thresholds for the bit 
                                         decoder */
#define     RFU1D                 0x1D   //!< RFU
#define     RegBPSKDemControl     0x1D
#define     RegRxControl2         0x1E   /*!< controls decoder behaviour and 
                                         defines the input source for the 
                                         receiver */
#define     RegClockQControl      0x1F   /*!< controls clock generation for the 
                                         90° phase shifted Q-channel clock */
// PAGE 4      RF-Timing and Channel Redundancy
#define     RegRxWait             0x21   /*!< selects the time interval after 
                                         transmission, before receiver starts */
#define     RegChannelRedundancy  0x22   /*!< selects the kind and mode of 
                                         checking the data integrity on the 
                                         RF-channel */
#define     RegCRCPresetLSB       0x23   /*!< LSB of the pre-set value for the 
                                         CRC register */
#define     RegCRCPresetMSB       0x24   /*!< MSB of the pre-set value for the 
                                         CRC register */
#define     RFU25                 0x25   //!< RFU
#define     RegTimeSlotPeriod     0x25
#define     RegMfOutSelect        0x26   /*!< selects internal signal applied to 
                                         pin MfOut */
#define     RFU27                 0x27   //!< RFU
// PAGE 5      FIFO, Timer and IRQ-Pin Configuration
#define     RegFIFOLevel          0x29   /*!< Defines level for FIFO over- and 
                                         underflow warning */
#define     RegTimerClock         0x2A   //!< selects the divider for the timer clock
#define     RegTimerControl       0x2B   /*!< selects start and stop conditions 
                                         for the timer */
#define     RegTimerReload        0x2C   /*!< defines the pre-set value for the 
                                         timer */
#define     RegIRqPinConfig       0x2D   /*!< configures the output stage of 
                                         pin IRq */
#define     RFU2E                 0x2E   //!< RFU
#define     RFU2F                 0x2F   //!< RFU
// PAGE 6      RFU
#define     RFU31                 0x31   //!< RFU
#define     RFU32                 0x32   //!< RFU
#define     RFU33                 0x33   //!< RFU
#define     RFU34                 0x34   //!< RFU
#define     RFU35                 0x35   //!< RFU
#define     RFU36                 0x36   //!< RFU
#define     RFU37                 0x37   //!< RFU
// PAGE 7      Test Control
#define     RFU39                 0x39   //!< RFU
#define     RegTestAnaSelect      0x3A   //!< selects analog test mode
#define     RFU3B                 0x3B   //!< RFU
#define     RFU3C                 0x3C   //!< RFU
#define     RegTestConfiguration  0x3C
#define     RegTestDigiSelect     0x3D   //!< selects digital test mode
#define     RFU3E                 0x3E   //!< RFU
#define     RegTestEE             0x3E
#define     RegTestDigiAccess     0x3F


#define  DEF_FIFO_LENGTH    64  //!< default FIFO size

//                   P C D - C O M M A N D S
#define PCD_IDLE           0x00 /*!< No action: cancel current command 
                                or home state */
#define PCD_WRITEE2        0x01 //!< Get data from FIFO and write it to the E2PROM
#define PCD_READE2         0x03 /*!< Read data from E2PROM and put it into the 
                                FIFO */
#define PCD_LOADCONFIG     0x07 /*!< Read data from E2PROM and initialise the 
                                registers */
#define PCD_LOADKEYE2      0x0B /*!< Read a master key from the E2PROM and put 
                                it into the master key buffer */
#define PCD_AUTHENT1       0x0C /*!< Perform the first part of the card 
                                authentication using the Crypto1 algorithm.
                                
                                Remark: The master key is automatically taken 
                                from the master key buffer. this implies, 
                                that the command LoadKeyE2 has to be executed
                                before using a certain key for card 
                                authentication */
#define PCD_CALCCRC        0x12 /*!< Activate the CRC-Coprocessor

                                Remark: The result of the CRC calculation can
                                be read from the register CRCResultXXX */
#define PCD_AUTHENT2       0x14 /*!< Perform the second part of the card 
                                authentication using the Crypto1 algorithm. */
#define PCD_RECEIVE        0x16 /*!< Activate Receiver Circuitry. Before the 
                                receiver actually starts, the state machine 
                                waits until the time configured in the 
                                register RxWait has passed.
                                
                                Remark: It is possible to read any received 
                                data from the FIFO while the Receive command
                                is active. Thus it is possible to receive an 
                                unlimited number of bytes by reading them 
                                from the FIFO in timer. */
#define PCD_LOADKEY        0x19 /*!< Read a master key from the FIFO and put it 
                                into the master key buffer
                                
                                Remark: The master key has to be prepared in
                                a certain format. Thus, 12 byte have to be 
                                passed to load a 6 byte master key */
#define PCD_TRANSMIT       0x1A /*!< Transmit data from FIFO to the card

                                Remark: If data is already in the FIFO when 
                                the command is activated, this data is 
                                transmitted immediately. It is possible to 
                                write data to the FIFO while the Transmit 
                                command is active. Thus it is possible to 
                                transmit an unlimited number of bytes in one
                                stream by writting them to the FIFO in time.*/
#define PCD_TRANSCEIVE     0x1E /*!< Transmits data from FIFO to the card and 
                                after that automatically activates the 
                                receiver. Before the receiver actually 
                                starts,the state machine waits until the 
                                time configured in the register RxWait has 
                                passed. 
                                
                                Remark: This command is the combination of 
                                Transmit and Receive.*/
#define PCD_RESETPHASE     0x3F /*!< Runs the Reset- and Initialisation Phase
                                Remark: This command can not be activated by 
                                software, but only by a Power-On or 
                                Hard Reset */

#ifdef __cplusplus
}
#endif

#endif //RICREG_H
//////////////////////////////////////////////////////////////////////////////
// End of File
//////////////////////////////////////////////////////////////////////////////
