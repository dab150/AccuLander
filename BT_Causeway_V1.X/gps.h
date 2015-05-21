/******************************************************************************
 *
 *                             PIC16 LIBRARY
 *
 * File: StateMachine.c/h
 * Version: 1.01
 * Author: ruslan
 *
 * Change list:
 * ------------------------------------------------------
 *  - v1.00 - Original version
 *  - v1.01 - Variables are inside a struct
			  States are defined as enum
 ******************************************************************************/

#ifndef GPS_H
#define	GPS_H

#define GPS_LOC_LEN     15
#define GPS_BUF_MAX     250
#define ASCII_LF        0x0A
#define ASCII_CR        0x0D

/******************************************************************************/
/* TypeDefs                                                                   */
/******************************************************************************/

/**
 * Main machine states
 */
typedef enum
{
    GPS_STATE_WAIT_START = 0,       // <-- Init state is 0
    GPS_STATE_WAIT_COMMA,
    GPS_STATE_WAIT_END,
    GPS_STATE_MAX
}gps_states_t;


/**
 * Actual location info state
 */
typedef enum
{
    GPS_LOC_UNKNOWN,
    GPS_LOC_INVALID,
    GPS_LOC_VALID
}gps_loc_status_t;


/**
 * Location definition as integer values
 */
typedef struct
{
    uint8_t deg;    //Degrees
    uint8_t min;    //Minutes
    uint32_t minf;  //Minutes fraction
    char side;      //N/S or E/W
}gps_pos_t;


/**
 * Private data structure
 */
typedef struct
{
    gps_states_t state;         //Actual state of the machine
    uint8_t buf[GPS_BUF_MAX];   //Buffer for incomming string
    uint8_t len;                //How many data are inside the buffer
    bool updated;               //There are new data to read    
    bool onFlag;                //Indicate if turnon procedure started
}gps_private_data_t;


/**
 * Public data structure
 */
typedef struct
{
    gps_loc_status_t locState;  //Actual location information status
    char latitude [GPS_LOC_LEN];//Lat  coordinate
    char longitude[GPS_LOC_LEN];//Long coordinate
    uint16_t onTimer;           //Delay to turn on the GPS module
}gps_public_data_t;


/*
 * Latitude and Longitude Structures
 */
typedef union
{
    double doublewise;
    unsigned char bytewise[4];

} Latitude;

typedef union
{
    double doublewise;
    unsigned char bytewise[4];

} Longitude;

typedef union
{
    double doublewise;
    unsigned char bytewise[4];
} Altitude;

/******************************************************************************/
/* Variable                                                                   */
/******************************************************************************/

extern gps_public_data_t gps;
extern char LatDegree[4], LatMin[3], LatSec[8];
extern char LonDegree[5], LonMin[3], LonSec[8];
extern Latitude LatAsDecimal;
extern Longitude LonAsDecimal;
extern Altitude AltAsDecimal;

/******************************************************************************/
/* Functions                                                                  */
/******************************************************************************/

void gps_Init();
void gps_PowerOn(uint16_t delay);
void gps_Routine();
bool gps_IsLocationUpdated();
void __gps_TestPrintResults(bool newLine);
void __gps_UnitTest_Fixture( void );

#endif	

