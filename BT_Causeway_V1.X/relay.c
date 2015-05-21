/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#endif

#include <stdint.h>        /* For uint8_t definition */
#include <stdbool.h>       /* For true/false definition */

#include "relay.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/uart1.h"
#include "mcc_generated_files/uart3.h"
#include "mcc_generated_files/uart2.h"
#include "inject.h"
//#include "Parsing.c"
#include "gps.h"
#include "parsing.h"
/******************************************************************************/
/* Variable                                                                   */
/******************************************************************************/

relay_data_t relayData;
int y = 0;
int LandInjected = 0; //false

/******************************************************************************/
/* Functions                                                                  */
/******************************************************************************/

/**
 * Init State machine
 */
void RelayInit() 
{
    //Init state
    relayData.state = 0;

}

/**
 * Main code for the Relay Machine
 */
void RelayLoop()
{
    bool usb = relayUSBConneted();

    if(relayData.state > RELAY_STATE_MAX)
    {
        RelayInit();
        return;
    }

    switch (relayData.state)
    {
        case RELAY_STATE_INIT:
            if(usb)
            {
                relayResetInjector();
                relayData.state = RELAY_STATE_USB;
            }
            else
            {
                relayResetInjector();
                relayData.state = RELAY_STATE_BLUETOOTH;
            }
            break;

        case RELAY_STATE_USB:
            relayFromUSB();
            relayFromRadio();
            if(!usb)
            {
                relayResetInjector();
                relayData.state = RELAY_STATE_BLUETOOTH;
            }
            break;

        case RELAY_STATE_BLUETOOTH:
            relayFromRadio();
            relayFromBluetooth();
            if(usb)
            {
                relayResetInjector();
                relayData.state = RELAY_STATE_USB;
            }
            break;

        case RELAY_STATE_MAX:
        default:
            //Error, unknown state
            RelayInit();
            break;            
    }
}


/**
 * Does board connected to an USB port?
 * @return true if connected, false if not
 */
bool relayUSBConneted()
{
    return !(USB_NOT_DETECTED_GetValue());
}


/**
 * Relays data from Radio to USB/Bluetooth
 */
void relayFromRadio()
{
    /* UART1 - FTDI USB
     * UART2 - Bluetooth
     * UART3 - Radio        */

    if(!UART3_ReceiveBufferIsEmpty())       //New data on UART3 (Radio)
    {
        uint8_t rxByte = UART3_Read(); //Read byte from U3

        if(relayUSBConneted())
        {
           // if(!UART1_TransmitBufferIsFull())   //There is free space in U1 tx buffer //commented this out to avoid random loss of connection -D Cironi 2015-05-11
           // {              
                UART1_Write(rxByte);           //Send to U1
           // }
        }
        else
        {
            if(!UART2_TransmitBufferIsFull())   //There is free space in U2 tx buffer
            {
                UART2_Write(rxByte);      //Read byte from U3 and send to U2
            }
        }

        
        //check for land command from RC
        int LandChannel = CheckRCLoop(rxByte); //check to see if data from radio is RC_Channel info

        if(LandChannel != 0 && LandChannel < 1500 && LandInjected == 0)    //Valid reading and the mode we want for landing
        {
            InjectTryInject('c'); //WP_Count

            InjectTryInject('1'); //Landing WP

            InjectTryInject('2'); //Final Approach WP

            InjectTryInject('3'); //First Approach WP
            //delay_ms(300);
            InjectTryInject('a'); //WP_Ack

            LandInjected = 1; //true
        }
        else if (LandChannel != 0 && LandChannel > 1500) //valid reading but not the mode we want
        {
            LandInjected = 0; //set false
            Nop();
        }
    }
}


/**
 * Relays data from USB to Radio
 */
void relayFromUSB()
{
    /* UART1 - FTDI USB
     * UART2 - Bluetooth
     * UART3 - Radio        */

    if(!UART1_ReceiveBufferIsEmpty())
    {
            InjectLoop(UART1_Read()); //Reads data from UART1 and sends to injector
    }

    //Lets see if there is some data available to send to radio
    if(!CircularBufferIsEmpty(&(inject.outBuff)))
        UART3_Write(CircularBufferDeque(&(inject.outBuff)));

}


/**
 * Relays data from Bluetooth to Radio
 */
void relayFromBluetooth()
{
    /* UART1 - FTDI USB
     * UART2 - Bluetooth
     * UART3 - Radio        */

    if(!UART2_ReceiveBufferIsEmpty())       //New data on UART2
        InjectLoop(UART2_Read());           //Reads data from UART2 and sends to injector


    //Lets see if there is some data available to send to radio
    if(!CircularBufferIsEmpty(&(inject.outBuff)))
        UART3_Write(CircularBufferDeque(&(inject.outBuff)));
}


void relayResetInjector()
{
    //This routine looks to not be necesary.
    //Remove in next review
}


