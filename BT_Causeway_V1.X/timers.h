/* 
 * File:   timers.h
 * Author: ruslanOffice
 *
 * Created on February 4, 2015, 3:55 PM
 */

#ifndef TIMERS_H
#define	TIMERS_H

#include <stdint.h>

/******************************************************************************
 *                          EXTERN VARIABLES                                    *
 ******************************************************************************/

extern uint8_t __delay_ms_timer;



/******************************************************************************
 *                        FUNCTIONS PROTOTYPES                                *
 ******************************************************************************/
void delay_ms(uint8_t delay);
void delay_us(uint16_t delay);
void one_ms_callback();
void one_sec_callback();


#endif	/* TIMERS_H */

