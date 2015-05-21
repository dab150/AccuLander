/**
  UART4 Generated Driver API Header File 

  @Company
    Microchip Technology Inc.

  @File Name
    uart4.h

  @Summary
    This is the generated header file for the UART4 driver using MPLAB® Code Configurator

  @Description
    This header file provides APIs for driver for UART4. 
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

#ifndef _UART4_H
#define _UART4_H

/**
 Section: Included Files
*/

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
        
/**
  Section: Data Types
*/

/** UART4 Driver Hardware Flags

  @Summary
    Specifies the status of the hardware receive or transmit

  @Description
    This type specifies the status of the hardware receive or transmit.
    More than one of these values may be OR'd together to create a complete
    status value.  To test a value of this type, the bit of interest must be
    AND'ed with value and checked to see if the result is non-zero.
*/
typedef enum
{
    /* Indicates that Receive buffer has data, at least one more character can be read */
    UART4_RX_DATA_AVAILABLE
        /*DOM-IGNORE-BEGIN*/  = (1 << 0) /*DOM-IGNORE-END*/,
    
    /* Indicates that Receive buffer has overflowed */
    UART4_RX_OVERRUN_ERROR
        /*DOM-IGNORE-BEGIN*/  = (1 << 1) /*DOM-IGNORE-END*/,

    /* Indicates that Framing error has been detected for the current character */
    UART4_FRAMING_ERROR
        /*DOM-IGNORE-BEGIN*/  = (1 << 2) /*DOM-IGNORE-END*/,

    /* Indicates that Parity error has been detected for the current character */
    UART4_PARITY_ERROR
        /*DOM-IGNORE-BEGIN*/  = (1 << 3) /*DOM-IGNORE-END*/,

    /* Indicates that Receiver is Idle */
    UART4_RECEIVER_IDLE
        /*DOM-IGNORE-BEGIN*/  = (1 << 4) /*DOM-IGNORE-END*/,

    /* Indicates that the last transmission has completed */
    UART4_TX_COMPLETE
        /*DOM-IGNORE-BEGIN*/  = (1 << 8) /*DOM-IGNORE-END*/,

    /* Indicates that Transmit buffer is full */
    UART4_TX_FULL
        /*DOM-IGNORE-BEGIN*/  = (1 << 9) /*DOM-IGNORE-END*/

}UART4_STATUS;

/**
   DRV_UART4 Driver Hardware Flags
*/
typedef enum
{
    /* Indicates that Receive buffer has data, at least one more character can be read */
    DRV_UART4_RX_DATA_AVAILABLE
        /*DOM-IGNORE-BEGIN*/  = (1 << 0) /*DOM-IGNORE-END*/,
    
    /* Indicates that Receive buffer has overflowed */
    DRV_UART4_RX_OVERRUN_ERROR
        /*DOM-IGNORE-BEGIN*/  = (1 << 1) /*DOM-IGNORE-END*/,

    /* Indicates that Framing error has been detected for the current character */
    DRV_UART4_FRAMING_ERROR
        /*DOM-IGNORE-BEGIN*/  = (1 << 2) /*DOM-IGNORE-END*/,

    /* Indicates that Parity error has been detected for the current character */
    DRV_UART4_PARITY_ERROR
        /*DOM-IGNORE-BEGIN*/  = (1 << 3) /*DOM-IGNORE-END*/,

    /* Indicates that Receiver is Idle */
    DRV_UART4_RECEIVER_IDLE
        /*DOM-IGNORE-BEGIN*/  = (1 << 4) /*DOM-IGNORE-END*/,

    /* Indicates that the last transmission has completed */
    DRV_UART4_TX_COMPLETE
        /*DOM-IGNORE-BEGIN*/  = (1 << 8) /*DOM-IGNORE-END*/,

    /* Indicates that Transmit buffer is full */
    DRV_UART4_TX_FULL
        /*DOM-IGNORE-BEGIN*/  = (1 << 9) /*DOM-IGNORE-END*/

}DRV_UART4_STATUS;

/** UART4 Driver Transfer Flags

  @Summary
    Specifies the status of the receive or transmit

  @Description
    This type specifies the status of the receive or transmit operation.
    More than one of these values may be OR'd together to create a complete
    status value.  To test a value of this type, the bit of interest must be
    AND'ed with value and checked to see if the result is non-zero.
*/

typedef enum
{
    /* Indicates that the core driver buffer is full */
    UART4_TRANSFER_STATUS_RX_FULL
        /*DOM-IGNORE-BEGIN*/  = (1 << 0) /*DOM-IGNORE-END*/,

    /* Indicates that at least one byte of Data has been received */
    UART4_TRANSFER_STATUS_RX_DATA_PRESENT
        /*DOM-IGNORE-BEGIN*/  = (1 << 1) /*DOM-IGNORE-END*/,

    /* Indicates that the core driver receiver buffer is empty */
    UART4_TRANSFER_STATUS_RX_EMPTY
        /*DOM-IGNORE-BEGIN*/  = (1 << 2) /*DOM-IGNORE-END*/,

    /* Indicates that the core driver transmitter buffer is full */
    UART4_TRANSFER_STATUS_TX_FULL
        /*DOM-IGNORE-BEGIN*/  = (1 << 3) /*DOM-IGNORE-END*/,

    /* Indicates that the core driver transmitter buffer is empty */
    UART4_TRANSFER_STATUS_TX_EMPTY
        /*DOM-IGNORE-BEGIN*/  = (1 << 4) /*DOM-IGNORE-END*/

} UART4_TRANSFER_STATUS;

/** 
    DRV_UART4 Driver Transfer Flags
*/
typedef enum
{
    /* Indicates that the core driver buffer is full */
    DRV_UART4_TRANSFER_STATUS_RX_FULL
        /*DOM-IGNORE-BEGIN*/  = (1 << 0) /*DOM-IGNORE-END*/,

    /* Indicates that at least one byte of Data has been received */
    DRV_UART4_TRANSFER_STATUS_RX_DATA_PRESENT
        /*DOM-IGNORE-BEGIN*/  = (1 << 1) /*DOM-IGNORE-END*/,

    /* Indicates that the core driver receiver buffer is empty */
    DRV_UART4_TRANSFER_STATUS_RX_EMPTY
        /*DOM-IGNORE-BEGIN*/  = (1 << 2) /*DOM-IGNORE-END*/,

    /* Indicates that the core driver transmitter buffer is full */
    DRV_UART4_TRANSFER_STATUS_TX_FULL
        /*DOM-IGNORE-BEGIN*/  = (1 << 3) /*DOM-IGNORE-END*/,

    /* Indicates that the core driver transmitter buffer is empty */
    DRV_UART4_TRANSFER_STATUS_TX_EMPTY
        /*DOM-IGNORE-BEGIN*/  = (1 << 4) /*DOM-IGNORE-END*/

} DRV_UART4_TRANSFER_STATUS;

/**
  Section: UART4 Driver Routines
*/


/**
  @Summary
    Initializes the UART instance : 4

  @Description
    This routine initializes the UART driver instance for : 4
    index.
    This routine must be called before any other UART routine is called.
    
  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Comment
    
 
  @Example
    <code>
        const uint8_t writeBuffer[35] = "1234567890ABCDEFGHIJKLMNOP\n" ;
        unsigned int numBytes = 0;
        int  writebufferLen = strlen((char *)writeBuffer);
        UART4_Initialize();
        while(numBytes < writebufferLen)
        {    
            int bytesToWrite = UART4_TransmitBufferSizeGet();
            numBytes = UART4_WriteBuffer ( writeBuffer+numBytes, bytesToWrite)  ;
            UART4_TasksTransmit ( );
            if (!UART4_TransmitBufferisFull())
            {
                //continue other operation
            }
        }
    </code>

*/

void UART4_Initialize(void);
/**
    void DRV_UART4_Initialize(void)
*/
void DRV_UART4_Initialize(void) __attribute__((deprecated ("\nThis will be removed in future MCC releases. \nUse UART4_Initialize instead. ")));




/**
  @Summary
    Maintains the driver's error-handling state machine in a polled manner.

  @Description
    This routine is used to maintain the driver's internal error-handling state
    machine.This routine is called when the state of the errors needs to be
    maintained in a polled manner.

  @Preconditions
    UART4_Initializer function should have been called 
    before calling this function in a polled loop.

  @Param
    None.

  @Returns
    None.

  @Example
    <code>
    while (true)
    {
        UART4_TasksError ();

        // Do other tasks
    }
    </code>
*/
void UART4_TasksError ( void );
/**
    void DRV_UART4_TasksError ( void )
*/
void DRV_UART4_TasksError ( void ) __attribute__((deprecated ("\nThis will be removed in future MCC releases. \nUse UART4_TasksError instead. ")));


/**
  @Summary
    Read a byte of data from the UART4

  @Description
    This routine reads a byte of data from the UART4.

  @Preconditions
    UART4_Initializer function should have been called 
    before calling this function. The transfer status should be checked to see 
    if the receiver is not empty before calling this function.

  @Param
    None.

  @Returns
    A data byte received by the driver.

  @Example
    <code>
    char            myBuffer[MY_BUFFER_SIZE];
    unsigned int    numBytes;

    numBytes = 0;
    do
    {
        if( UART4_TRANSFER_STATUS_RX_DATA_PRESENT & UART4_TransferStatusGet() )
        {
            myBuffer[numBytes++] = UART4_Read();
        }

        // Do something else...

    } while( numBytes < MY_BUFFER_SIZE);
    </code>
*/

uint8_t UART4_Read( void);
/**
    uint8_t DRV_UART4_Read( void)
*/
uint8_t DRV_UART4_Read( void) __attribute__((deprecated ("\nThis will be removed in future MCC releases. \nUse UART4_Read instead. ")));

/**
  @Summary
    Returns the number of bytes read by the UART4 peripheral

  @Description
    This routine returns the number of bytes read by the Peripheral and fills the
    application read buffer with the read data.

  @Preconditions
    UART4_Initializer function should have been called 
    before calling this function

  @Param
    buffer       - Buffer into which the data read from the UART4

  @Param
    numbytes     - Total number of bytes that need to be read from the UART4
                   (must be equal to or less than the size of the buffer)

  @Returns
    Number of bytes actually copied into the caller's buffer or -1 if there
    is an error.

  @Example
    <code>
    char                     myBuffer[MY_BUFFER_SIZE];
    unsigned int             numBytes;
    UART4_TRANSFER_STATUS status ;

    // Pre-initialize myBuffer with MY_BUFFER_SIZE bytes of valid data.

    numBytes = 0;
    while( numBytes < MY_BUFFER_SIZE);
    {
        status = UART4_TransferStatusGet ( ) ;
        if (status & UART4_TRANSFER_STATUS_RX_FULL)
        {
            numBytes += UART4_ReadBuffer( myBuffer + numBytes, MY_BUFFER_SIZE - numBytes )  ;
            if(numBytes < readbufferLen)
            {
                continue;
            }
            else
            {
                break;
            }
        }
        else
        {
            continue;
        }

        // Do something else...
    }
    </code>
*/

unsigned int UART4_ReadBuffer( uint8_t *buffer ,  const unsigned int numbytes);
/**
    unsigned int DRV_UART4_ReadBuffer( uint8_t *buffer , const unsigned int numbytes)
*/
unsigned int DRV_UART4_ReadBuffer( uint8_t *buffer ,const unsigned int numbytes) __attribute__((deprecated ("\nThis will be removed in future MCC releases. \nUse UART4_ReadBuffer instead. ")));

/**
  @Summary
    Writes a byte of data to the UART4

  @Description
    This routine writes a byte of data to the UART4.

  @Preconditions
    UART4_Initializer function should have been called 
    before calling this function. The transfer status should be checked to see if
    transmitter is not full before calling this function.

  @Param
    byte         - Data byte to write to the UART4

  @Returns
    None.

  @Example
    <code>
    char            myBuffer[MY_BUFFER_SIZE];
    unsigned int    numBytes;

    // Pre-initialize myBuffer with MY_BUFFER_SIZE bytes of valid data.

    numBytes = 0;
    while( numBytes < MY_BUFFER_SIZE);
    {
        if( !(UART4_TRANSFER_STATUS_TX_FULL & UART4_TransferStatusGet()) )
        {
            UART4_Write(handle, myBuffer[numBytes++]);
        }

        // Do something else...
    }
    </code>
*/

void UART4_Write( const uint8_t byte);
/**
    void DRV_UART4_Write( const uint8_t byte)
*/
void DRV_UART4_Write( const uint8_t byte) __attribute__((deprecated ("\nThis will be removed in future MCC releases. \nUse UART4_Write instead. ")));


/**
  @Summary
    Returns the number of bytes written into the internal buffer

  @Description
    This API transfers the data from application buffer to internal buffer and 
    returns the number of bytes added in that queue

  @Preconditions
    UART4_Initializer function should have been called 
    before calling this function

  @Example
    <code>
    char                     myBuffer[MY_BUFFER_SIZE];
    unsigned int             numBytes;
    UART4_TRANSFER_STATUS status ;

    // Pre-initialize myBuffer with MY_BUFFER_SIZE bytes of valid data.

    numBytes = 0;
    while( numBytes < MY_BUFFER_SIZE);
    {
        status = UART4_TransferStatusGet ( ) ;
        if (status & UART4_TRANSFER_STATUS_TX_EMPTY)
        {
            numBytes += UART4_WriteBuffer ( myBuffer + numBytes, MY_BUFFER_SIZE - numBytes )  ;
            if(numBytes < writebufferLen)
            {
                continue;
            }
            else
            {
                break;
            }
        }
        else
        {
            continue;
        }

        // Do something else...
    }
    </code>
*/

unsigned int UART4_WriteBuffer( const uint8_t *buffer , const unsigned int numbytes );
/**
    unsigned int DRV_UART4_WriteBuffer( const uint8_t *buffer , const unsigned int numbytes )
*/
unsigned int DRV_UART4_WriteBuffer( const uint8_t *buffer , const unsigned int numbytes ) __attribute__((deprecated ("\nThis will be removed in future MCC releases. \nUse UART4_WriteBuffer instead. ")));


/**
  @Summary
    Returns the transmitter and receiver transfer status

  @Description
    This returns the transmitter and receiver transfer status.The returned status 
    may contain a value with more than one of the bits
    specified in the UART4_TRANSFER_STATUS enumeration set.  
    The caller should perform an "AND" with the bit of interest and verify if the
    result is non-zero (as shown in the example) to verify the desired status
    bit.

  @Preconditions
    UART4_Initializer function should have been called 
    before calling this function

  @Param
    None.

  @Returns
    A UART4_TRANSFER_STATUS value describing the current status 
    of the transfer.

  @Example
    Refer to UART4_ReadBuffer and UART4_WriteBuffer for example

*/

UART4_TRANSFER_STATUS UART4_TransferStatusGet (void );
/**
    DRV_UART4_TRANSFER_STATUS DRV_UART4_TransferStatusGet (void )
*/
DRV_UART4_TRANSFER_STATUS DRV_UART4_TransferStatusGet (void ) __attribute__((deprecated ("\nThis will be removed in future MCC releases. \nUse UART4_TransferStatusGet instead. ")));



/**
  @Summary
    Returns the character in the read sequence at the offset provided, without
    extracting it

  @Description
    This routine returns the character in the read sequence at the offset provided,
    without extracting it
 
  @Param
    None.
    
  @Example 
    <code>
    const uint8_t readBuffer[5];
    unsigned int data, numBytes = 0;
    unsigned int readbufferLen = sizeof(readBuffer);
    UART4_Initializer();
    
    while(numBytes < readbufferLen)        
    {   
        UART4_TasksReceive ( );
        //Check for data at a particular place in the buffer
        data = UART4_Peek(3);
        if(data == 5)
        {
            //discard all other data if byte that is wanted is received.    
            //continue other operation
            numBytes += UART4_ReadBuffer ( readBuffer + numBytes , readbufferLen ) ;
        }
        else
        {
            break;
        }
    }
    </code>
 
*/
uint8_t UART4_Peek(uint16_t offset);
/**
    uint8_t DRV_UART4_Peek(uint16_t offset)
*/
uint8_t DRV_UART4_Peek(uint16_t offset) __attribute__((deprecated ("\nThis will be removed in future MCC releases. \nUse UART4_Peek instead. ")));


/**
  @Summary
    Returns the size of the receive buffer

  @Description
    This routine returns the size of the receive buffer.

  @Param
    None.

  @Returns
    Size of receive buffer.
    
  @Example 
    <code>
    const uint8_t readBuffer[5];
    unsigned int size, numBytes = 0;
    unsigned int readbufferLen = sizeof(readBuffer);
    UART4__Initializer();
    
    while(size < readbufferLen)
	{
	    UART4_TasksReceive ( );
	    size = UART4_ReceiveBufferSizeGet();
	}
    numBytes = UART4_ReadBuffer ( readBuffer , readbufferLen ) ;
    </code>
 
*/
unsigned int UART4_ReceiveBufferSizeGet(void);
/**
    unsigned int DRV_UART4_ReceiveBufferSizeGet(void)
*/
unsigned int DRV_UART4_ReceiveBufferSizeGet(void) __attribute__((deprecated ("\nThis will be removed in future MCC releases. \nUse UART4_ReceiveBufferSizeGet instead. ")));

/**
  @Summary
    Returns the size of the transmit buffer

  @Description
    This routine returns the size of the transmit buffer.

 @Param
    None.
 
 @Returns
    Size of transmit buffer.

 @Example
    Refer to UART4_Initializer(); for example.
*/
unsigned int UART4_TransmitBufferSizeGet(void);
/**
    unsigned int DRV_UART4_TransmitBufferSizeGet(void)
*/
unsigned int DRV_UART4_TransmitBufferSizeGet(void) __attribute__((deprecated ("\nThis will be removed in future MCC releases. \nUse UART4_TransmitBufferSizeGet instead. ")));


/**
  @Summary
    Returns the status of the receive buffer

  @Description
    This routine returns if the receive buffer is empty or not.

  @Param
    None.
 
  @Returns
    True if the receive buffer is empty
    False if the receive buffer is not empty
    
  @Example
    <code>
    char                     myBuffer[MY_BUFFER_SIZE];
    unsigned int             numBytes;
    UART4_TRANSFER_STATUS status ;

    // Pre-initialize myBuffer with MY_BUFFER_SIZE bytes of valid data.

    numBytes = 0;
    while( numBytes < MY_BUFFER_SIZE);
    {
        status = UART4_TransferStatusGet ( ) ;
        if (!UART4_ReceiveBufferIsEmpty())
        {
            numBytes += UART4_ReadBuffer( myBuffer + numBytes, MY_BUFFER_SIZE - numBytes )  ;
            if(numBytes < readbufferLen)
            {
                continue;
            }
            else
            {
                break;
            }
        }
        else
        {
            continue;
        }

        // Do something else...
    }
    </code>
 
*/
bool UART4_ReceiveBufferIsEmpty (void);
/**
    bool DRV_UART4_ReceiveBufferIsEmpty (void)
*/
bool DRV_UART4_ReceiveBufferIsEmpty (void) __attribute__((deprecated ("\nThis will be removed in future MCC releases. \nUse UART4_ReceiveBufferIsEmpty instead. ")));


/**
  @Summary
    Returns the status of the transmit buffer

  @Description
    This routine returns if the transmit buffer is full or not.

 @Param
    None.
 
 @Returns
    True if the transmit buffer is full
    False if the transmit buffer is not full

 @Example
    Refer to UART4_Initializer() for example.
 
*/
bool UART4_TransmitBufferIsFull (void);
/**
    bool DRV_UART4_TransmitBufferIsFull (void)
*/
bool DRV_UART4_TransmitBufferIsFull (void) __attribute__((deprecated ("\nThis will be removed in future MCC releases. \nUse UART4_TransmitBufferIsFull instead. ")));


/**
  @Summary
    Returns the transmitter and receiver status

  @Description
    This returns the transmitter and receiver status. The returned status may 
    contain a value with more than one of the bits
    specified in the UART4_STATUS enumeration set.  
    The caller should perform an "AND" with the bit of interest and verify if the
    result is non-zero (as shown in the example) to verify the desired status
    bit.

  @Preconditions
    UART4_Initializer function should have been called 
    before calling this function

  @Param
    None.

  @Returns
    A UART4_STATUS value describing the current status 
    of the transfer.

  @Example
    <code>
        while(!(UART4_StatusGet & UART4_TX_COMPLETE ))
        {
           // Wait for the tranmission to complete
        }
    </code>
*/

UART4_STATUS UART4_StatusGet (void );
/**
    DRV_UART4_STATUS UART4_StatusGet (void )
*/
DRV_UART4_STATUS DRV_UART4_StatusGet (void ) __attribute__((deprecated ("\nThis will be removed in future MCC releases. \nUse UART4_StatusGet instead. ")));


#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif
    
#endif  // _UART4_H

/*
  End of File
*/

