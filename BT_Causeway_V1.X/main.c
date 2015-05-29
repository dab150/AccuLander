/*
 * File:   main.c
 * Author: ruslanOffice
 *
 * Created on January 13, 2015, 3:53 PM
 */

/*
 * UART1 - FTDI USB
 * UART2 - Bluetooth
 * UART3 - Radio
 * 
 */    
 
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stdlib.h"
#include <xc.h>
#include "mcc_generated_files/mcc.h"
#include "barometer.h"
#include "timers.h"
#include "debug.h"
#include "compass.h"
#include "power.h"
#include "gps.h"
#include "relay.h"
#include "inject.h"
#include "./utests/unit.h"
#include "parsing.h"

uint16_t AppTimer, GPSpowerTimer;

union
{
    double floatwise;
    unsigned int bytewise[15];

} LatUnion;

int main(void)
{

    //System Init
    SYSTEM_Initialize();
    run_all_tests();


    //Components Init
    RelayInit();
    InjectInit();
    gps_Init();
    Bar_Init();
    
    //gps_PowerOn(2500);
    GPS_ON_SetLow(); //turn on gps

    //Init timer
    AppTimer = 0;

    while(1)
    {
        gps_Routine();            //update GPS
        RelayLoop();              //primary relay function
        InjectPressure();         //injects pressure packet every 10 seconds
    }
}
