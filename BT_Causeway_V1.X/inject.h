/******************************************************************************
 *
 *                             PIC16 LIBRARY
 *
 * File: StateMachine.c/h
 * Version: 1.02
 * Author: ruslan
 *
 * Change list:
 * ------------------------------------------------------
 *  - v1.00 - Original version
 *  - v1.01 - Variables are inside a struct
              States are defined as enum
 *  - v1.02 - More states added
 ******************************************************************************/

#ifndef INJECT_H
#define	INJECT_H

#include "CircularBuffer.h"

#define INJECT_PERIOD                   (60 * 2) //seconds

#define RADIO_OUTPUT_BUFFER             256

#define PARAM_UPDATE_MSG_LEN            23
#define PARAM_UPDATE_FULL_LEN           (PARAM_UPDATE_MSG_LEN + 6 + 2)

#define WP_COUNT_MSG_LEN                4
#define WP_COUNT_FULL_LEN               (WP_COUNT_MSG_LEN + 6 + 2)

#define WP_MSG_LEN                      37
#define WP_FULL_LEN                     (WP_MSG_LEN + 6 + 2)

#define WP_ACK_MSG_LEN                  3
#define WP_ACK_FULL_LEN                 (WP_ACK_MSG_LEN + 6 + 2)

#define WP_CLEAR_MSG_LEN                2
#define WP_CLEAR_FULL_LEN               (WP_CLEAR_MSG_LEN + 6 + 2)
#define PRESS_SYSTEM_ID                 0xE3

#define PRESS_COMP_ID                   191

/******************************************************************************/
/* TypeDefs                                                                   */
/******************************************************************************/

typedef enum
{
    INJECT_STATE_MSG_END = 0,    	// <-- Init state is 0
    INJECT_STATE_WAIT_LEN,
    INJECT_STATE_WAIT_END,
    MACHINE_NAME_STATE_MAX
}inject_states_t;


typedef union
{
    uint8_t buf[PARAM_UPDATE_FULL_LEN];
    struct
    {
        uint8_t     start;              //0 - 1
        uint8_t     len;                //1 - 2
        uint8_t     count;              //2 - 3
        uint8_t     sys_id;             //3 - 4
        uint8_t     comp_id;            //4 - 5
        uint8_t     msgType;            //5 - 6
                                        //6 - 29
        uint8_t     data[PARAM_UPDATE_MSG_LEN];
        uint8_t     checksum[2];        //29- 31
    };
}pressure_packet_t;

typedef union
{
    uint8_t buf[WP_COUNT_FULL_LEN];
    struct
    {
        uint8_t     start;              //0
        uint8_t     len;                //1
        uint8_t     count;              //2
        uint8_t     sys_id;             //3
        uint8_t     comp_id;            //4
        uint8_t     msgType;            //5                                       
        uint8_t     numOfPoints;        //6
        uint8_t     data;               //7 //some random piece of unused data
        uint8_t     target_sys;        //8
        uint8_t     target_comp;         //9
        uint8_t     checksum[2];        //10 - 11
    };
}WP_Count_packet_t;

typedef union
{
    uint8_t buf[WP_FULL_LEN];
    struct
    {
        uint8_t     start;              //0
        uint8_t     len;                //1
        uint8_t     count;              //2
        uint8_t     sys_id;             //3
        uint8_t     comp_id;            //4
        uint8_t     msgType;            //5
        uint8_t     params[16];         //6 - 21  //random params?
        uint8_t     Latitude[4];        //22 - 25
        uint8_t     Longitude[4];       //26 - 29
        uint8_t     Altitude[4];        //30 - 33
        uint8_t     sequence[2];        //34 - 35
        uint8_t     command[2];         //36 - 37
        uint8_t     target_sys;         //38
        uint8_t     target_comp;        //39
        uint8_t     frame;              //40
        uint8_t     data2;              //41 //Unused
        uint8_t     autocontinue;       //42
        uint8_t     checksum[2];        //43 - 44
    };
}WP_packet_t;

typedef union
{
    uint8_t buf[WP_ACK_FULL_LEN];
    struct
    {
        uint8_t     start;              //0
        uint8_t     len;                //1
        uint8_t     count;              //2
        uint8_t     sys_id;             //3
        uint8_t     comp_id;            //4
        uint8_t     msgType;            //5
        uint8_t     target_sys;         //6
        uint8_t     target_comp;        //7
        uint8_t     type;               //8
        uint8_t     checksum[2];        //9 - 10
    };
}WP_Ack_t;

typedef union
{
    uint8_t buf[WP_ACK_FULL_LEN];
    struct
    {
        uint8_t     start;              //0
        uint8_t     len;                //1
        uint8_t     count;              //2
        uint8_t     sys_id;             //3
        uint8_t     comp_id;            //4
        uint8_t     msgType;            //5
        uint8_t     target_sys;         //6
        uint8_t     target_comp;        //7
        uint8_t     checksum[2];        //8-9
    };
}WP_Clear_t;

typedef struct
{    
    uint8_t timer;   
    CircularBuffer_t outBuff;               //Buffer to store TX data software
                                            //will send over Radio UART
}inject_t;


typedef struct
{
    inject_states_t state;
    uint8_t bufArray[RADIO_OUTPUT_BUFFER];  //Array for a circular buffer
    uint16_t fullLen;             //How long is the actual packet
    uint16_t cont;                //How many bytes of the actual buffer I already recieved
    pressure_packet_t press;      //packet for pressure value
    WP_Count_packet_t WPCount;    //packet for WP_Count
    WP_packet_t WPitem;           //packet for a WP
    WP_Ack_t WPAck;               //Mission acknowledgement packet
    WP_Clear_t WPClear;           //clear entire mission
    
}inject_private_t;

/******************************************************************************/
/* Variable                                                                   */
/******************************************************************************/

extern inject_t inject;


/******************************************************************************/
/* Functions                                                                  */
/******************************************************************************/

void InjectInit();
void InjectLoop(uint8_t rx);
void InjectTryInject(char x);


#endif	

