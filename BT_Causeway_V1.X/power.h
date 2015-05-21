/* 
 * File:   power.h
 * Author: ruslanOffice
 *
 * Power Manager Macros
 *
 * Created on January 23, 2015, 12:13 PM
 */

#ifndef POWER_H
#define	POWER_H

#include "mcc_generated_files/pin_manager.h"


/******************************************************************************
 *                              MACROS                                        *
 ******************************************************************************/

#define POWER_GPS_COMPASS_TURN_ON()     GPS_ON_SetLow()
#define POWER_GPS_COMPASS_TURN_OFF()    GPS_ON_SetHigh()



#endif	/* POWER_H */

