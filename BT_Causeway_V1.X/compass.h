/* 
 * File:   compass.h
 * Author: ruslanOffice
 *
 * Created on January 23, 2015, 10:20 AM
 */

#ifndef COMPASS_H
#define	COMPASS_H

#include "mcc_generated_files/i2c2.h"

/******************************************************************************
 *                              CONSTANTS                                     *
 ******************************************************************************/

#define COMPASS_I2CADDR     0x1E


/******************************************************************************
 *                              TYPEDEFS                                      *
 ******************************************************************************/

typedef struct
{
    int16_t x, y, z;
    I2C2_MESSAGE_STATUS status; //I2C bus status
    
}compass_data_t;

extern compass_data_t CompassData;


/******************************************************************************
 *                              MACROS                                        *
 ******************************************************************************/



/******************************************************************************
 *                          FUNCTION PROTOTYPES                               *
 ******************************************************************************/
void Compass_Init(void);
void Compass_Read(void);
void __Compass_Test(void);


#endif	/* COMPASS_H */

