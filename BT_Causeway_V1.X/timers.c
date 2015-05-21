/*
 * File:   timers.c
 * Author: ruslanOffice
 *
 * Created on February 4, 2015, 3:22 PM
 */


#include "xc.h"
#include "timers.h"
#include "mcc_generated_files/tmr2.h"
#include "inject.h"
#include "mcc_generated_files/pin_manager.h"




/******************************************************************************/
/* Variables                                                                  */
/******************************************************************************/

uint8_t __delay_ms_timer;






/******************************************************************************/
/* Functions                                                                  */
/******************************************************************************/

/**
 * Delays a given ammount of miliseconds. It's a bloking function
 * @param delay - ms to delay. Max allowed value: 0xFF
 */
void delay_ms(uint8_t delay) 
{
    __delay_ms_timer = delay;       //Setup ms to delay
    while(__delay_ms_timer);        //Wait timer to expire
}


/**
 * Delays a given ammount of microseconds. It's a bloking function
 * @param delay - uS to delay. Max allowed value: 0x3FFF (16383)
 */
void delay_us(uint16_t delay)
{
    TMR2_Counter16BitSet(0);            //Clear timer
    TMR2_Period16BitSet(delay << 2);    //Setup delay period
    IFS0bits.T2IF = 0;                  //Clear flag

    while(!IFS0bits.T2IF);
}


/******************************************************************************/
/* Callback Functions                                                         */
/******************************************************************************/

extern uint16_t AppTimer, GPSpowerTimer;
extern bool BorDetected;

/**
 * Called every 1ms
 */
void one_ms_callback()
{
    static uint16_t ms = 1000;

    if(__delay_ms_timer)    __delay_ms_timer--;

    //Is it time to process 1sec timers?
    if(--ms)return;
    ms = 1000;

    one_sec_callback();
}


/**
 * Called every 1 sec
 */
void one_sec_callback()
{
    //seconds timers here
    if(inject.timer)
        inject.timer--;

    if(GPSpowerTimer)
        GPSpowerTimer--;
}