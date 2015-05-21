/**
  UART4 Generated Driver File 

  @Company
    Microchip Technology Inc.

  @File Name
    uart4.c

  @Summary
    This is the generated source file for the UART4 driver using MPLAB® Code Configurator

  @Description
    This source file provides APIs for driver for UART4. 
    Generation Information : 
        Product Revision  :  MPLAB® Code Configurator - v2.10.3
        Device            :  PIC24FJ128GB204
        Driver Version    :  0.5
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.24
        MPLAB 	          :  MPLAB X 2.26
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

#include "uart4.h"

/**
  Section: Data Type Definitions
*/

/** UART Driver Queue Status

  @Summary
    Defines the object required for the status of the queue.
*/

typedef union
{
    struct
    {
            uint8_t full:1;
            uint8_t empty:1;
            uint8_t reserved:6;
    }s;
    uint8_t status;
}

UART_BYTEQ_STATUS;

/** UART Driver Hardware Instance Object

  @Summary
    Defines the object required for the maintenance of the hardware instance.

*/
typedef struct
{
    /* RX Byte Q */
    uint8_t                                      *rxTail ;

    uint8_t                                      *rxHead ;

    /* TX Byte Q */
    uint8_t                                      *txTail ;

    uint8_t                                      *txHead ;

    UART_BYTEQ_STATUS                        rxStatus ;

    UART_BYTEQ_STATUS                        txStatus ;

} UART_OBJECT ;

static UART_OBJECT uart4_obj ;

/** UART Driver Queue Length

  @Summary
    Defines the length of the Transmit and Receive Buffers

*/

#define UART4_CONFIG_TX_BYTEQ_LENGTH 64
#define UART4_CONFIG_RX_BYTEQ_LENGTH 64


/** UART Driver Queue

  @Summary
    Defines the Transmit and Receive Buffers

*/

static uint8_t uart4_txByteQ[UART4_CONFIG_TX_BYTEQ_LENGTH] ;
static uint8_t uart4_rxByteQ[UART4_CONFIG_RX_BYTEQ_LENGTH] ;

/** UART Hardware FIFO Buffer Length

  @Summary
    Defines the length of the Transmit and Receive FIFOs
 
*/

#define UART4_TX_FIFO_LENGTH 1
#define UART4_RX_FIFO_LENGTH 1 

/**
  Section: Driver Interface
*/


void UART4_Initialize (void)
{
   // RTSMD enabled; URXINV disabled; BRGH enabled; STSEL 1; UARTEN enabled; PDSEL 8N; LPBACK disabled; WAKE disabled; USIDL disabled; ABAUD disabled; IREN disabled; UEN TX_RX; 
   U4MODE = 0x8808;
   // UTXEN disabled; UTXINV disabled; URXISEL RX_ONE_CHAR; ADDEN disabled; UTXISEL0 TX_ONE_CHAR; UTXBRK COMPLETED; OERR NO_ERROR_cleared; 
   U4STA = 0x0000;
   // LAST disabled; U4TXREG disabled; 
   U4TXREG = 0x0000;
   // U4RXREG disabled; 
   U4RXREG = 0x0000;
   // U4BRG 103; 
   U4BRG = 0x0067;
   // ADMMASK disabled; ADMADDR disabled; 
   U4ADMD = 0x0000;

   IEC5bits.U4RXIE = 1;

   U4STAbits.UTXEN = 1;

   uart4_obj.txHead = uart4_txByteQ;
   uart4_obj.txTail = uart4_txByteQ;
   uart4_obj.rxHead = uart4_rxByteQ;
   uart4_obj.rxTail = uart4_rxByteQ;
   uart4_obj.rxStatus.s.empty = true;
   uart4_obj.txStatus.s.empty = true;
   uart4_obj.txStatus.s.full = false;
   uart4_obj.rxStatus.s.full = false;
}
/**
void DRV_UART4_Initialize (void)
*/
void DRV_UART4_Initialize (void)
{
   UART4_Initialize();
}



/**
    Maintains the driver's transmitter state machine and implements its ISR
*/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U4TXInterrupt ( void )
{ 
    if(uart4_obj.txStatus.s.empty)
    {
        IEC5bits.U4TXIE = false;
        return;
    }

    IFS5bits.U4TXIF = false;

    int count = 0;
    while((count < UART4_TX_FIFO_LENGTH)&& !(U4STAbits.UTXBF == 1))
    {
        count++;

        U4TXREG = *uart4_obj.txHead;

        uart4_obj.txHead++;

        if(uart4_obj.txHead == (uart4_txByteQ + UART4_CONFIG_TX_BYTEQ_LENGTH))
        {
            uart4_obj.txHead = uart4_txByteQ;
        }

        uart4_obj.txStatus.s.full = false;

        if(uart4_obj.txHead == uart4_obj.txTail)
        {
            uart4_obj.txStatus.s.empty = true;
            break;
        }
    }
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U4RXInterrupt( void )
{
    int count = 0;

    while((count < UART4_RX_FIFO_LENGTH) && (U4STAbits.URXDA == 1))
    {
        count++;

        *uart4_obj.rxTail = U4RXREG;

        uart4_obj.rxTail++;

        if(uart4_obj.rxTail == (uart4_rxByteQ + UART4_CONFIG_RX_BYTEQ_LENGTH))
        {
            uart4_obj.rxTail = uart4_rxByteQ;
        }

        uart4_obj.rxStatus.s.empty = false;
        
        if(uart4_obj.rxTail == uart4_obj.rxHead)
        {
            //Sets the flag RX full
            uart4_obj.rxStatus.s.full = true;
            break;
        }
        
    }

    IFS5bits.U4RXIF = false;
   
}


void UART4_TasksError ( void )
{
    if ((U4STAbits.OERR == 1))
    {
        U4STAbits.OERR = 0;
    }

    IFS5bits.U4ERIF = false;
}
/**
   void DRV_UART4_TasksError ( void )
*/
void DRV_UART4_TasksError (void)
{
   UART4_TasksError();
}

/**
  Section: UART Driver Client Routines
*/

uint8_t UART4_Read( void)
{
    uint8_t data = 0;

    data = *uart4_obj.rxHead;

    uart4_obj.rxHead++;

    if (uart4_obj.rxHead == (uart4_rxByteQ + UART4_CONFIG_RX_BYTEQ_LENGTH))
    {
        uart4_obj.rxHead = uart4_rxByteQ;
    }

    if (uart4_obj.rxHead == uart4_obj.rxTail)
    {
        uart4_obj.rxStatus.s.empty = true;
    }

    uart4_obj.rxStatus.s.full = false;

    return data;
}
/**
   uint8_t DRV_UART4_Read ( void )
*/
uint8_t DRV_UART4_Read (void)
{
   return(UART4_Read());
}

unsigned int UART4_ReadBuffer( uint8_t *buffer, const unsigned int bufLen)
{
    unsigned int numBytesRead = 0 ;
    while ( numBytesRead < ( bufLen ))
    {
        if( uart4_obj.rxStatus.s.empty)
        {
            break;
        }
        else
        {
            buffer[numBytesRead++] = UART4_Read () ;
        }
    }

    return numBytesRead ;
}

/**
   unsigned int DRV_UART4_ReadBuffer( uint8_t *buffer, const unsigned int bufLen)
*/
unsigned int DRV_UART4_ReadBuffer( uint8_t *buffer, const unsigned int bufLen)
{
   return(UART4_ReadBuffer(buffer,bufLen));
}

void UART4_Write( const uint8_t byte)
{
    *uart4_obj.txTail = byte;

    uart4_obj.txTail++;
    
    if (uart4_obj.txTail == (uart4_txByteQ + UART4_CONFIG_TX_BYTEQ_LENGTH))
    {
        uart4_obj.txTail = uart4_txByteQ;
    }

    uart4_obj.txStatus.s.empty = false;

    if (uart4_obj.txHead == uart4_obj.txTail)
    {
        uart4_obj.txStatus.s.full = true;
    }

    if (IEC5bits.U4TXIE  == false)
    {
        IEC5bits.U4TXIE = true ;
    }
	
}
/**
   void DRV_UART4_Write( const uint8_t byte)
*/
void DRV_UART4_Write( const uint8_t byte)
{
    UART4_Write(  byte);
}

unsigned int UART4_WriteBuffer( const uint8_t *buffer , const unsigned int bufLen )
{
    unsigned int numBytesWritten = 0 ;

    while ( numBytesWritten < ( bufLen ))
    {
        if((uart4_obj.txStatus.s.full))
        {
            break;
        }
        else
        {
            UART4_Write (buffer[numBytesWritten++] ) ;
        }
    }

    return numBytesWritten ;

}
/**
   unsigned int DRV_UART4_WriteBuffer( const uint8_t *buffer , const unsigned int bufLen )
*/
unsigned int DRV_UART4_WriteBuffer( const uint8_t *buffer , const unsigned int bufLen )
{
    return(UART4_WriteBuffer( buffer ,bufLen));
} 

UART4_TRANSFER_STATUS UART4_TransferStatusGet (void )
{
    UART4_TRANSFER_STATUS status = 0;

    if(uart4_obj.txStatus.s.full)
    {
        status |= UART4_TRANSFER_STATUS_TX_FULL;
    }

    if(uart4_obj.txStatus.s.empty)
    {
        status |= UART4_TRANSFER_STATUS_TX_EMPTY;
    }

    if(uart4_obj.rxStatus.s.full)
    {
        status |= UART4_TRANSFER_STATUS_RX_FULL;
    }

    if(uart4_obj.rxStatus.s.empty)
    {
        status |= UART4_TRANSFER_STATUS_RX_EMPTY;
    }
    else
    {
        status |= UART4_TRANSFER_STATUS_RX_DATA_PRESENT;
    }
    return status;
}
/**
   DRV_UART4_TRANSFER_STATUS UART4_TransferStatusGet (void )
*/
DRV_UART4_TRANSFER_STATUS DRV_UART4_TransferStatusGet (void )
{
    return(UART4_TransferStatusGet());
}

uint8_t UART4_Peek(uint16_t offset)
{
    if( (uart4_obj.rxHead + offset) > (uart4_rxByteQ + UART4_CONFIG_RX_BYTEQ_LENGTH))
    {
      return uart4_rxByteQ[offset - (uart4_rxByteQ + UART4_CONFIG_RX_BYTEQ_LENGTH - uart4_obj.rxHead)];
    }
    else
    {
      return *(uart4_obj.rxHead + offset);
    }
}
/**
  uint8_t DRV_UART4_Peek(uint16_t offset)
*/
uint8_t DRV_UART4_Peek(uint16_t offset)
{
    return(UART4_Peek(offset));
}

unsigned int UART4_ReceiveBufferSizeGet(void)
{
    if(!uart4_obj.rxStatus.s.full)
    {
        if(uart4_obj.rxHead > uart4_obj.rxTail)
        {
            return(uart4_obj.rxHead - uart4_obj.rxTail);
        }
        else
        {
            return(UART4_CONFIG_RX_BYTEQ_LENGTH - (uart4_obj.rxTail - uart4_obj.rxHead));
        } 
    }
    return 0;
}
/**
  unsigned int DRV_UART4_ReceiveBufferSizeGet(void)
*/
unsigned int DRV_UART4_ReceiveBufferSizeGet(void)
{
    return(UART4_ReceiveBufferSizeGet());
}

unsigned int UART4_TransmitBufferSizeGet(void)
{
    if(!uart4_obj.txStatus.s.full)
    { 
        if(uart4_obj.txHead > uart4_obj.txTail)
        {
            return(uart4_obj.txHead - uart4_obj.txTail);
        }
        else
        {
            return(UART4_CONFIG_TX_BYTEQ_LENGTH - (uart4_obj.txTail - uart4_obj.txHead));
        }
    }
    return 0;
}
/**
  unsigned int DRV_UART4_TransmitBufferSizeGet(void)
*/
unsigned int DRV_UART4_TransmitBufferSizeGet(void)
{
    return(UART4_TransmitBufferSizeGet());
}

bool UART4_ReceiveBufferIsEmpty (void)
{
    return(uart4_obj.rxStatus.s.empty);
}
/**
  bool DRV_UART4_ReceiveBufferIsEmpty(void)
*/
bool DRV_UART4_ReceiveBufferIsEmpty(void)
{
    return(UART4_ReceiveBufferIsEmpty());
}

bool UART4_TransmitBufferIsFull(void)
{
    return(uart4_obj.txStatus.s.full);
}
/**
  bool DRV_UART4_TransmitBufferIsFull(void)
*/
bool DRV_UART4_TransmitBufferIsFull(void)
{
    return(UART4_TransmitBufferIsFull());
}

UART4_STATUS UART4_StatusGet (void)
{
    return U4STA;
}
/**
  DRV_UART4_STATUS DRV_UART4_StatusGet (void)
*/
DRV_UART4_STATUS DRV_UART4_StatusGet (void)
{
    return(UART4_StatusGet());
}


/**
  End of File
*/
