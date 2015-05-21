/**
  System Interrupts Generated Driver File 

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.c

  @Summary:
    This is the generated manager file for the MPLAB® Code Configurator device.  This manager
    configures the pins direction, initial state, analog setting.
    The peripheral pin select, PPS, configuration is also handled by this manager.

  @Description:
    This source file provides implementations for MPLAB® Code Configurator interrupts.
    Generation Information : 
        Product Revision  :  MPLAB® Code Configurator - v2.10.3
        Device            :  PIC24FJ128GB204
        Version           :  1.02
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
    Section: Includes
*/
#include <xc.h>
#include "pin_manager.h"

/**
    void PIN_MANAGER_Initialize(void)
*/
void PIN_MANAGER_Initialize(void)
{
    /****************************************************************************
     * Setting the GPIO of PORTA
     ***************************************************************************/
    LATA = 0x00;
    TRISA = 0x0407;
    /****************************************************************************
     * Setting the GPIO of PORTB
     ***************************************************************************/
    LATB = 0x01C0;
    TRISB = 0x642F;
    /****************************************************************************
     * Setting the GPIO of PORTC
     ***************************************************************************/
    LATC = 0x0208;
    TRISC = 0x0171;

    /****************************************************************************
     * Setting the Analog/Digital Configuration SFR
     ***************************************************************************/
    ANSA = 0x07;
    ANSB = 0x2003;
    ANSC = 0x00;

    /****************************************************************************
     * Set the PPS
     ***************************************************************************/
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    RPOR4bits.RP9R = 0x03;    // RB9->UART1:U1TX
    RPINR17bits.U3RXR = 0x0A;    // RB10->UART3:U3RX
    RPOR5bits.RP11R = 0x13;    // RB11->UART3:U3TX
    RPINR27bits.U4RXR = 0x0E;    // RB14->UART4:U4RX
    RPOR7bits.RP15R = 0x15;    // RB15->UART4:U4TX
    RPINR20bits.SDI1R = 0x10;    // RC0->SPI1:SDI1
    RPOR8bits.RP17R = 0x07;    // RC1->SPI1:SDO1
    RPOR9bits.RP18R = 0x08;    // RC2->SPI1:SCK1OUT
    RPINR18bits.U1RXR = 0x16;    // RC6->UART1:U1RX
    RPOR11bits.RP23R = 0x05;    // RC7->UART2:U2TX
    RPINR19bits.U2RXR = 0x18;    // RC8->UART2:U2RX
    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
}
