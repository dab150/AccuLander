/* 
 * File:   barometer.h
 * Author: ruslanOffice
 *
 * Created on January 15, 2015, 5:48 PM
 */

#ifndef BAROMETER_H
#define	BAROMETER_H

#include "mcc_generated_files/pin_manager.h"
#include <stdbool.h>

/*
 * Delete or comment next line when running on real hardware
 */
//#define __BAR_SIM_CODE

/*
 * Delete or comment next line when debug propose output is not requiered
 */
//#define __BAR_DEBUG_CODE

/******************************************************************************
 *                              CONSTANTS                                     *
 ******************************************************************************/

#define BAR_CMD_RESET       0x1E // ADC reset command
#define BAR_CMD_ADC_READ    0x00 // ADC read command
#define BAR_CMD_ADC_CONV    0x40 // ADC conversion command
#define BAR_CMD_ADC_D1      0x00 // ADC D1 conversion
#define BAR_CMD_ADC_D2      0x10 // ADC D2 conversion
#define BAR_CMD_ADC_256     0x00 // ADC OSR=256
#define BAR_CMD_ADC_512     0x02 // ADC OSR=512
#define BAR_CMD_ADC_1024    0x04 // ADC OSR=1024
#define BAR_CMD_ADC_2048    0x06 // ADC OSR=2056
#define BAR_CMD_ADC_4096    0x08 // ADC OSR=4096
#define BAR_CMD_PROM_RD     0xA0 // Prom read command



/******************************************************************************
 *                              TYPEDEFS                                      *
 ******************************************************************************/

typedef struct
{
    uint16_t c[7];      //Calibration factory data
    int32_t temp, pres; //Calculate temperature and pressure

    
}bar_data_t;

extern bar_data_t bar;


/******************************************************************************
 *                              MACROS                                        *
 ******************************************************************************/

#define Bar_EnableChip()    SS_BAROMETER_SetLow()
#define Bar_DisableChip()   SS_BAROMETER_SetHigh()

#define _2POW_16(exp)          (1U   << exp)
#define _2POW_32(exp)          (1UL  << exp)
#define _2POW_64(exp)          (1ULL << exp)




/******************************************************************************
 *                          FUNCTION PROTOTYPES                               *
 ******************************************************************************/

void Bar_Init();
void Bar_reset();
uint16_t Bar_Read_Prom_Cal(uint8_t index);
uint32_t Bar_Read_ADC(uint8_t cmd);
void Bar_Calculate();
void __Bar_Test();
void __Bar_TestPrintTemp(bool newLine);
void __Bar_TestPrintPressure(bool newLine);


#ifdef __BAR_DEBUG_CODE

#warning "You are compiling your code with barometer DEBUG routines enabled!!!!"

void __Bar_debug_print_Coef();
void __Bar_debug_print_calc(uint32_t d1,uint32_t d2,int32_t dT,int64_t off,
        int64_t sens);

#else

#define __Bar_debug_print_Coef()                    ;
#define __Bar_debug_print_calc(d1,d2,dT,off,sens)   ;

#endif

#ifdef __BAR_SIM_CODE

#warning "You are compiling your code with barometer SIMULATOR routines enabled!!!!"
#define __Bar_Read_ADC          Bar_Read_ADC_Sim
#define __Bar_Init_Simulate()   Bar_Init_Simulate(); return;

void Bar_Init_Simulate();
uint32_t Bar_Read_ADC_Sim(uint8_t cmd);

#else
#define __Bar_Read_ADC           Bar_Read_ADC
#define __Bar_Init_Simulate()    ;
#endif

#endif	/* BAROMETER_H */

