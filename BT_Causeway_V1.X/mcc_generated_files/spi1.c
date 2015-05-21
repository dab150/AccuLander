
/**
  SPI1 Generated Driver API Source File

  Company:
    Microchip Technology Inc.

  File Name:
    spi1.c

  @Summary
    This is the generated source file for the SPI1 driver using MPLAB® Code Configurator

  @Description
    This source file provides APIs for driver for SPI1.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.10.3
        Device            :  PIC24FJ128GB204
        Driver Version    :  0.5
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.24
        MPLAB             :  MPLAB X 2.26
*/

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*/

/**
  Section: Included Files
*/

#include <xc.h>
#include "spi1.h"

/**
 Section: File specific functions
*/

/**
  SPI1 Transfer Mode Enumeration

  @Summary
    Defines the Transfer Mode enumeration for SPI1.

  @Description
    This defines the Transfer Mode enumeration for SPI1.
 */
typedef enum {
    SPI1_TRANSFER_MODE_32BIT  = 2,
    SPI1_TRANSFER_MODE_16BIT = 1,
    SPI1_TRANSFER_MODE_8BIT = 0
}SPI1_TRANSFER_MODE;

inline __attribute__((__always_inline__)) SPI1_TRANSFER_MODE SPI1_TransferModeGet(void);
void SPI1_Exchange( uint8_t *pTransmitData, uint8_t *pReceiveData );
uint16_t SPI1_ExchangeBuffer(uint8_t *pTransmitData, uint16_t byteCount, uint8_t *pReceiveData);

/**
 Section: Driver Interface Function Definitions
*/


void SPI1_Initialize (void)
{
    // TXELM 0; RXELM 0; 
    SPI1STATH = 0x0000;
    // TXELM 0; RXELM 0; 
    SPI1STATH = 0x0000;
    // WLENGTH 0; 
    SPI1CON2L = 0x0000;
    // AUDEN disabled; FRMPOL disabled; FRMCNT 0x0; AUDMOD I2S; IGNROV disabled; SPISGNEXT not sign-extended; MSSEN disabled; AUDMONO stereo; FRMSYPW One clock wide; URDTEN disabled; FRMSYNC disabled; IGNTUR disabled; FRMEN disabled; 
    SPI1CON1H = 0x0000;
    // SSEN disabled; DISSDI disabled; DISSCK disabled; MODE16 disabled; CKE Idle to Active; MODE32 disabled; SPISIDL disabled; SPIFE Frame Sync pulse precedes; SMP Sample at Middle; MSTEN enabled; SPIEN enabled; ENHBUF disabled; DISSDO disabled; MCLKEN FOSC/2; CKP Idle:Low, Active:High; 
    SPI1CON1L = 0x8020;
    // SPI1BRGL 1; 
    SPI1BRGL = 0x0001;

}


void SPI1_Exchange( uint8_t *pTransmitData, uint8_t *pReceiveData )
{

    while( SPI1STATLbits.SPITBF == true )
    {

    }

    if (SPI1_TransferModeGet() == SPI1_TRANSFER_MODE_32BIT)
    {
        SPI1BUFL = *((uint16_t*)pTransmitData);
        SPI1BUFH = *((uint16_t*)(pTransmitData+2));
    }
    else if (SPI1_TransferModeGet() == SPI1_TRANSFER_MODE_16BIT)
    {
       SPI1BUFL = *((uint16_t*)pTransmitData);
    }
    else
    {
        SPI1BUFL = *((uint8_t*)pTransmitData);
    }

    while ( SPI1STATLbits.SPIRBE == true)
    {
    
    }


    if (SPI1_TransferModeGet() == SPI1_TRANSFER_MODE_32BIT)
    {
        *((uint16_t*)pReceiveData) = SPI1BUFL;
        *((uint16_t*)(pReceiveData+2)) = SPI1BUFH;
    }
    else if (SPI1_TransferModeGet() == SPI1_TRANSFER_MODE_16BIT)
    {
        *((uint16_t*)pReceiveData) = SPI1BUFL;
    }
    else
    {
        *((uint8_t*)pReceiveData) = SPI1BUFL;
    }

}

uint16_t SPI1_ExchangeBuffer(uint8_t *pTransmitData, uint16_t byteCount, uint8_t *pReceiveData)
{

    uint16_t dataSentCount = 0;
    uint16_t count = 0;
    uint16_t dummyDataReceived = 0;
    uint16_t dummyDataTransmit = SPI1_DUMMY_DATA;
    
    uint8_t fifoMultiplier = 1;

    uint8_t  *pSend, *pReceived;
    uint16_t addressIncrement;
    uint16_t receiveAddressIncrement, sendAddressIncrement;

    SPI1_TRANSFER_MODE spiModeStatus;

    spiModeStatus = SPI1_TransferModeGet();
    // set up the address increment variable
    if (spiModeStatus == SPI1_TRANSFER_MODE_32BIT)
    {
        addressIncrement = 4;
        byteCount >>= 2;
        fifoMultiplier = 1;
    }  
    else if (spiModeStatus == SPI1_TRANSFER_MODE_16BIT)
    {
        addressIncrement = 2;
        byteCount >>= 1;
        fifoMultiplier = 2;
    }        
    else
    {
        addressIncrement = 1;
        fifoMultiplier = 4;
    }

    // set the pointers and increment delta 
    // for transmit and receive operations
    if (pTransmitData == NULL)
    {
        sendAddressIncrement = 0;
        pSend = (uint8_t*)&dummyDataTransmit;
    }
    else
    {
        sendAddressIncrement = addressIncrement;
        pSend = (uint8_t*)pTransmitData;
    }
        
    if (pReceiveData == NULL)
    {
       receiveAddressIncrement = 0;
       pReceived = (uint8_t*)&dummyDataReceived;
    }
    else
    {
       receiveAddressIncrement = addressIncrement;        
       pReceived = (uint8_t*)pReceiveData;
    }


    while( SPI1STATLbits.SPITBF == true )
    {

    }

    while (dataSentCount < byteCount)
    {
        if ((count < ((SPI1_FIFO_FILL_LIMIT)*fifoMultiplier)))
        {
            if (spiModeStatus == SPI1_TRANSFER_MODE_32BIT)
            {
                SPI1BUFL = *((uint16_t*)pSend);
                SPI1BUFH = *((uint16_t*)(pSend+2));
            }
            else if (spiModeStatus == SPI1_TRANSFER_MODE_16BIT)
            {
                SPI1BUFL = *((uint16_t*)pSend);
            }
            else
            {
                SPI1BUFL = *pSend;
            }
            pSend += sendAddressIncrement;
            dataSentCount++;
            count++;
        }

        if (SPI1STATLbits.SPIRBE == false)
        {
            if (spiModeStatus == SPI1_TRANSFER_MODE_32BIT)
            {
                *((uint16_t*)pReceived) = SPI1BUFL;
                *((uint16_t*)(pReceived+2)) = SPI1BUFH;
            }
            else if (spiModeStatus == SPI1_TRANSFER_MODE_16BIT)
            {
                *((uint16_t*)pReceived) = SPI1BUFL;
            }
            else
            {
                *pReceived = SPI1BUFL;
            }
            pReceived += receiveAddressIncrement;
            count--;
        }

    }
    while (count)
    {
        if (SPI1STATLbits.SPIRBE == false)
        {
            if (spiModeStatus == SPI1_TRANSFER_MODE_32BIT)
            {
                *((uint16_t*)pReceived) = SPI1BUFL;
                *((uint16_t*)(pReceived+2)) = SPI1BUFH;
            }
            else if (spiModeStatus == SPI1_TRANSFER_MODE_16BIT)
            {
                *((uint16_t*)pReceived) = SPI1BUFL;
            }
            else
            {
                *pReceived = SPI1BUFL;
            }
            pReceived += receiveAddressIncrement;
            count--;
        }
    }

    return dataSentCount;
}



uint8_t SPI1_Exchange8bit( uint8_t data )
{
    uint8_t receiveData;
    
    SPI1_Exchange(&data, &receiveData);

    return (receiveData);
}


uint16_t SPI1_Exchange8bitBuffer(uint8_t *dataTransmitted, uint16_t byteCount, uint8_t *dataReceived)
{
    return (SPI1_ExchangeBuffer(dataTransmitted, byteCount, dataReceived));
}

inline __attribute__((__always_inline__)) SPI1_TRANSFER_MODE SPI1_TransferModeGet(void)
{
    if (SPI1CON1Lbits.MODE32 == 1)
        return SPI1_TRANSFER_MODE_32BIT;
    else if (SPI1CON1Lbits.MODE16 == 1)
        return SPI1_TRANSFER_MODE_16BIT;
    else
        return SPI1_TRANSFER_MODE_8BIT;
}

SPI1_STATUS SPI1_StatusGet()
{
    return(SPI1STATL);
}

/**
 End of File
*/
