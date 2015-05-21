/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#if defined(__XC)
    #include <xc.h>         /* XC8 General Include File */
#elif defined(HI_TECH_C)
    #include <htc.h>        /* HiTech General Include File */
#endif

#include <stdint.h>        /* For uint8_t definition */
#include <stdbool.h>       /* For true/false definition */
#include "stdio.h"
#include "string.h"
#include "inject.h"
#include "mcc_generated_files/uart1.h"
#include "barometer.h"
#include "parsing.h"


#define X25_INIT_CRC        0xffff
#define X25_VALIDATE_CRC    0xf0b8

/******************************************************************************/
/* Local Variables                                                            */
/******************************************************************************/
/**
 * Local variable. Stores all the local data inject module uses
 */

static inject_private_t CheckRC;

bool RC_PACKET_DETECTED = false;


/******************************************************************************/
/* Public Variables                                                           */
/******************************************************************************/

RC_Channel_PWM RC_Channel_6;

/******************************************************************************/
/* Local Functions                                                            */
/******************************************************************************/

void CheckRCLoopInit()
{
    //Init state
    CheckRC.state = 0;
}


/******************************************************************************/
/* Public Functions                                                           */
/******************************************************************************/

int CheckRCLoop(unsigned char rx)
{
    if(CheckRC.state > MACHINE_NAME_STATE_MAX)
    {
        CheckRCLoopInit();
        return;
    }

    //Analyze recieved byte
    switch (CheckRC.state)
    {
        case INJECT_STATE_MSG_END:
            if(rx == 0xFE)
            {
                //First byte of the packet detected
                CheckRC.state = INJECT_STATE_WAIT_LEN;
            }
            break;

        case INJECT_STATE_WAIT_LEN:
            //Len byte recieved
            CheckRC.cont = 2;                    //Already has 2 bytes
            CheckRC.fullLen = rx + 6 + 2;        //Calc the packet len
            CheckRC.state = INJECT_STATE_WAIT_END;
            break;

        case INJECT_STATE_WAIT_END:
            
            if(++CheckRC.cont >= CheckRC.fullLen)
            {
                //Full packet recieved
                CheckRC.state = INJECT_STATE_MSG_END;
            }

            if(CheckRC.cont == 6 && rx == 35) //if the MessageID is 0x23 (35) then it is a raw_channel_message
            {
                RC_PACKET_DETECTED = true;
            }
            else if(CheckRC.fullLen != 30)
            {
                RC_PACKET_DETECTED = false;
            }

            if(RC_PACKET_DETECTED)
            {
                if(CheckRC.cont == 21) //21-22 = Channel 6
                {
                    RC_Channel_6.bytewise[0] = rx;
                }
                if(CheckRC.cont == 22)
                {
                    RC_Channel_6.bytewise[1] = rx;
                }
                if(CheckRC.cont == 22)
                {
                    int x = RC_Channel_6.intwise; //RC Channel Value PWM
                    return x;
                }

            }
            break;

        case MACHINE_NAME_STATE_MAX:
        default:
            //Error, unknown state
            InjectInit();
            break;
    }
    return 0; //default
}