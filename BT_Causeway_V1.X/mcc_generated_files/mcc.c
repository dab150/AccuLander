/**
  @Generated MPLAB® Code Configurator Source File

  @Company:
    Microchip Technology Inc.

  @File Name:
    mcc.c

  @Summary:
    This is the mcc.c file generated using MPLAB® Code Configurator

  @Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB® Code Configurator - v2.10.3
        Device            :  PIC24FJ128GB204
        Driver Version    :  1.02
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

// Configuration bits: selected in the GUI

// CONFIG4
#pragma config IOL1WAY = ON    // PPS IOLOCK Set Only Once Enable bit->Once set, the IOLOCK bit cannot be cleared
#pragma config PLLDIV = DISABLED    // USB 96 MHz PLL Prescaler Select bits->PLL Disabled
#pragma config I2C1SEL = ENABLE    // Alternate I2C1 enable bit->I2C1 uses ASCL1 and ASDA1 pins
#pragma config DSWDTPS = DSWDTPS1F    // Deep Sleep Watchdog Timer Postscale Select bits->1:68719476736 (25.7 Days)
#pragma config DSSWEN = ON    // DSEN Bit Enable->Deep Sleep is controlled by the register bit DSEN
#pragma config DSWDTOSC = LPRC    // DSWDT Reference Clock Select->DSWDT uses LPRC as reference clock
#pragma config DSBOREN = ON    // Deep Sleep BOR Enable bit->DSBOR Enabled
#pragma config DSWDTEN = ON    // Deep Sleep Watchdog Timer Enable->DSWDT Enabled

// CONFIG3
#pragma config WPDIS = WPDIS    // Segment Write Protection Disable->Disabled
#pragma config WPFP = WPFP0    // Write Protection Flash Page Segment Boundary->Page 0 (0x00)
#pragma config WPEND = WPENDMEM    // Segment Write Protection End Page Select->Write Protect from WPFP to the last page of memory
#pragma config BOREN = ON    // Brown-out Reset Enable->Brown-out Reset Enable
#pragma config WPCFG = WPCFGDIS    // Write Protect Configuration Page Select->Disabled
#pragma config SOSCSEL = OFF    // SOSC Selection bits->Digital (SCLKI) mode
#pragma config WDTWIN = PS25_0    // Window Mode Watchdog Timer Window Width Select->Watch Dog Timer Window Width is 25 percent
#pragma config PLLSS = PLL_PRI    // PLL Secondary Selection Configuration bit->PLL is fed by the Primary oscillator

// CONFIG2
#pragma config POSCMD = XT    // Primary Oscillator Select->XT Oscillator Enabled
#pragma config IESO = OFF    // Internal External Switchover->Disabled
#pragma config FNOSC = PRI    // Initial Oscillator Select->Primary Oscillator (XT, HS, EC)
#pragma config WDTCLK = LPRC    // WDT Clock Source Select bits->WDT uses LPRC
#pragma config WDTCMX = WDTCLK    // WDT Clock Source Select bits->WDT clock source is determined by the WDTCLK Configuration bits
#pragma config ALTCMPI = CxINC_RB    // Alternate Comparator Input bit->C1INC is on RB13, C2INC is on RB9 and C3INC is on RA0
#pragma config FCKSM = CSDCMD    // Clock Switching and Fail-Safe Clock Monitor Configuration bits->Clock switching and Fail-Safe Clock Monitor are disabled
#pragma config ALTRB6 = APPEND    // Alternate RB6 pin function enable bit->Append the RP6/ASCL1/PMPD6 functions of RB6 to RA1 pin functions
#pragma config OSCIOFCN = ON    // OSCO Pin Configuration->OSCO/CLKO/RA3 functions as port I/O (RA3)

// CONFIG1
#pragma config WDTPS = PS2    // Watchdog Timer Postscaler Select->1:2
#pragma config LPCFG = ON    // Low power regulator control->Low voltage regulator controlled by RETEN bit
#pragma config ICS = PGx1    // Emulator Pin Placement Select bits->Emulator functions are shared with PGEC1/PGED1
#pragma config FWPSA = PR128    // WDT Prescaler Ratio Select->1:128
#pragma config WINDIS = OFF    // Windowed WDT Disable->Standard Watchdog Timer
#pragma config GWRP = OFF    // General Segment Write Protect->Write to program memory allowed
#pragma config GCP = OFF    // General Segment Code Protect->Code protection is disabled
#pragma config FWDTEN = OFF    // Watchdog Timer Enable->WDT disabled in hardware; SWDTEN bit disabled
#pragma config JTAGEN = OFF    // JTAG Port Enable->Disabled

#include "mcc.h"

void SYSTEM_Initialize(void)
{
    OSCILLATOR_Initialize();
    PIN_MANAGER_Initialize();
    INTERRUPT_Initialize();
    ADC1_Initialize();
    UART1_Initialize();
    UART2_Initialize();
    UART3_Initialize();
    SPI1_Initialize();
    TMR1_Initialize();
    TMR2_Initialize();
    UART4_Initialize();
    I2C2_Initialize();
}

void OSCILLATOR_Initialize(void)
{
    // DOZEN disabled; DOZE 1:8; CPDIV 1:1; RCDIV FRC/2; PLLEN disabled; ROI disabled; 
    CLKDIV = 0x3100;
    // STSRC SOSC; STEN disabled; STOR disabled; STORPOL Interrupt when STOR is 1; STLOCK disabled; STLPOL Interrupt when STLOCK is 1; STSIDL disabled; TUN Center frequency; 
    OSCTUN = 0x0000;
    // Set the secondary oscillator
    
}

/**
 End of File
*/