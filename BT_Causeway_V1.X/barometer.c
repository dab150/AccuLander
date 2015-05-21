/*
 * File:   barometer.c
 * Author: ruslanOffice
 *
 * Created on January 15, 2015, 5:48 PM
 */


#include "xc.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "barometer.h"
#include "timers.h"
#include "mcc_generated_files/pin_manager.h"
#include "mcc_generated_files/spi1.h"
#include "debug.h"



bar_data_t bar;


/**
 * Initialice the barometer library
 */
void Bar_Init()
{
    uint8_t i;

    //Code for barometer simulation
    __Bar_Init_Simulate();

    //reset sequence
    Bar_reset();

    //Read calibration data
    for(i=0; i<7; i++)
        bar.c[i] = Bar_Read_Prom_Cal(i);

    //Print cal data if debug mode is on
    __Bar_debug_print_Coef();
}

/**
 * Resets the barometer chip
 */
void Bar_reset()
{
    Bar_EnableChip();                   // pull CS low to start the command
    SPI1_Exchange8bit(BAR_CMD_RESET);   // send reset sequence
    delay_ms(3);                        // wait for the reset sequence timing
    Bar_DisableChip();                  // pull CSB high to finish the command
}

/**
 * Reads calibration constant from Barometer PROM memory
 * @param index - Base 0 memory position
 * @return 
 */
uint16_t Bar_Read_Prom_Cal(uint8_t index)
{
    uint16_t msb, lsb;
    
    Bar_EnableChip();   // pull CSB low
    
    // send PROM READ command and get MSB and LSB bytes
    SPI1_Exchange8bit(BAR_CMD_PROM_RD + (index << 1)); 
    msb = SPI1_Exchange8bit(0x00);
    lsb = SPI1_Exchange8bit(0x00);
    
    Bar_DisableChip(); // pull CSB high

    return (msb << 8) + lsb;
}

/**
 * Read ADC data from temp or hum
 * @param cmd Indicates what to read and OSR
 * @return result value
 */
uint32_t Bar_Read_ADC(uint8_t cmd)
{
    uint32_t data, res=0;

    Bar_EnableChip();   // pull CSB low

    //Send convert command and delay the requiered time
    SPI1_Exchange8bit(BAR_CMD_ADC_CONV + cmd);
    switch (cmd & 0x0f) // wait necessary conversion time
    {
        case BAR_CMD_ADC_256 : delay_us(900);   break;
        case BAR_CMD_ADC_512 : delay_ms(3);     break;
        case BAR_CMD_ADC_1024: delay_ms(4);     break;
        case BAR_CMD_ADC_2048: delay_ms(6);     break;
        case BAR_CMD_ADC_4096: delay_ms(10);    break;
    }

    Bar_DisableChip(); // pull CSB high to finish the conversion
    Bar_EnableChip();   // pull CSB low to start new command

    //Read 3 bytes
    SPI1_Exchange8bit(BAR_CMD_ADC_READ);// send ADC read command
    data = SPI1_Exchange8bit(0x00);      // send 0 to read 1st byte (MSB)
    res += data << 16;

    data = SPI1_Exchange8bit(0x00);      // send 0 to read 2nd byte
    res += (data << 8);

    data = SPI1_Exchange8bit(0x00);      // send 0 to read 3rd byte (LSB)
    res += data;

    Bar_DisableChip(); // pull CSB high to finish the read command

    return res;
}


/**
 * Reads temp and press, calculate compensated values and store resulsts
 * to bar structure
 */
void Bar_Calculate()
{
    uint32_t d1, d2;
    int64_t dT;
    int64_t temp;
    int64_t off, sens, pres;

    //Read digital pressure and temp
    d1 = __Bar_Read_ADC(BAR_CMD_ADC_D1 + BAR_CMD_ADC_4096);
    d2 = __Bar_Read_ADC(BAR_CMD_ADC_D2 + BAR_CMD_ADC_4096);

    //Calculate temperature
    dT = d2 - (bar.c[5] * _2POW_32(8));            // dT = d2 - c5 * 2^8
    temp = 2000 + (dT * bar.c[6]) / _2POW_32(23);  // temp = 2000 + dT*C6 / 2^23

    //Calculate compensed pressure
    off = bar.c[2] * _2POW_64(16) + (bar.c[4] * dT) / _2POW_64(7);  //C2 * 2^16 + (C4 * dT) / 2^7
    sens= bar.c[1] * _2POW_64(15) + (bar.c[3] * dT) / _2POW_64(8);  //C1 * 2^15 + (C3 * dT) / 2^8
    pres = (d1 * sens / _2POW_64(21) - off) / _2POW_64(15);         //(D1 * SENS / 2^21 - OFF) / 2^15

    //Save results
    bar.temp = (int32_t)(temp);
    bar.pres = (int32_t)(pres);

    //Print calc if debug mode is on
    __Bar_debug_print_Coef();
    __Bar_debug_print_calc(d1, d2, dT, off, sens);
}

/*
 * Performs a reading, calculates and prints on debug output Temp and Press values
 */
void __Bar_Test()
{
    char buf[128];
    Bar_Calculate();

    //Print values
    sprintf(buf, "Temp: %d.%.2u C;   Pres: %u.%.2u mBar \r\n" ,
            (int16_t)(bar.temp / 100),
            (uint16_t)(labs(bar.temp) % 100),
            (uint16_t)(bar.pres / 100),
            (uint16_t)(bar.pres % 100));
    __debug_print(buf);
}


void __Bar_TestPrintTemp(bool newLine)
{
    char buf[128];

    //Print values
    sprintf(buf, "%d.%.2u%s" ,
            (int16_t)(bar.temp / 100),
            (uint16_t)(labs(bar.temp) % 100),
            newLine ? "\r\n" : "");
    __debug_print(buf);
}

void __Bar_TestPrintPressure(bool newLine)
{
    char buf[128];

    //Print values
    sprintf(buf, "%d.%.2u%s" ,
            (uint16_t)(bar.pres / 100),
            (uint16_t)(bar.pres % 100),
            newLine ? "\r\n" : "");
    __debug_print(buf);
}

#ifdef __BAR_DEBUG_CODE

void __Bar_debug_print_Coef()
{
    char buf[128];

    __debug_print("C1,C2,C3,C4,C5,C6\n\r");

    sprintf(buf, "%u,%u,%u,%u,%u,%u\n\r",
            bar.c[1],
            bar.c[2],
            bar.c[3],
            bar.c[4],
            bar.c[5],
            bar.c[6]);

    __debug_print(buf);
}

void __Bar_debug_print_calc(
        uint32_t d1,
        uint32_t d2,
        int32_t dT,
        int64_t off,
        int64_t sens)
{
    char buf[128];

    sprintf(buf, "D1, %lu\n\r", d1);
    __debug_print(buf);

    sprintf(buf, "D2, %lu\n\r", d2);
    __debug_print(buf);

    sprintf(buf, "dT, %ld\n\r", dT);
    __debug_print(buf);

    sprintf(buf, "Temp, %ld\n\r", bar.temp);
    __debug_print(buf);
    
    sprintf(buf, "OFF, %s\n\r", __i64toa(off));
    __debug_print(buf);

    sprintf(buf, "SENS, %s\n\r", __i64toa(sens));
    __debug_print(buf);

    sprintf(buf, "Press, %ld\n\r\n\r", bar.pres);
    __debug_print(buf);
}

#endif



#ifdef __BAR_SIM_CODE

void Bar_Init_Simulate()
{
    bar.c[1] = 51380;
    bar.c[2] = 52592;
    bar.c[3] = 31026;
    bar.c[4] = 29585;
    bar.c[5] = 31260;
    bar.c[6] = 27464;

    //Bar_Calculate();
}


uint32_t Bar_Read_ADC_Sim(uint8_t cmd)
{
    //return test value
    if((cmd & 0xF0) == BAR_CMD_ADC_D1)
        return 8225208;
    else
        return 8200816;
}

#endif
