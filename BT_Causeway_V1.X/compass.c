/*
 * File:   compass.c
 * Author: ruslanOffice
 *
 * Created on January 23, 2015, 10:18 AM
 */


#include "xc.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "debug.h"
#include "compass.h"
#include "mcc_generated_files/i2c2.h"
#include "debug.h"

compass_data_t CompassData;


/*
 * Initialice the compass library
 */
void Compass_Init(void)
{
    uint8_t config[2] =
        {
            0x02,   // Mode register
            0x00    // Continuous measurement mode
        };

    //Init variables
    memset(&CompassData, 0, sizeof(compass_data_t));

    //Init device
    I2C2_MasterWrite(config, 2, COMPASS_I2CADDR, &(CompassData.status));

    //TODO: Check if data was sucessfully sent?
    while(CompassData.status != I2C2_MESSAGE_COMPLETE);
}

/*
 * Read compass data and store to x/y/z variables
 */
void Compass_Read(void)
{
    uint8_t read[6];
    uint8_t a = 0x03;

    I2C2_MasterWrite(&a, 1, COMPASS_I2CADDR, &(CompassData.status));

    //Read 6 bytes of data
    I2C2_MasterRead(read, 6, COMPASS_I2CADDR, &(CompassData.status));

    CompassData.x = read[0] << 8;
    CompassData.x|= read[1];

    CompassData.y = read[4] << 8;
    CompassData.y|= read[5];

    CompassData.z = read[2] << 8;
    CompassData.z|= read[3];
}

/**
 * Performs a compas reading and print results over debug console. 
 */
void __Compass_Test(void)
{
    char buf[128];

    Compass_Read();

    sprintf(buf, "X = %d; Y = %d; Z = %d;\n\r",
            CompassData.x,
            CompassData.y,
            CompassData.z);

    __debug_print(buf);
}

