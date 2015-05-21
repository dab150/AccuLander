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

#ifndef RELAY_MACHINE_H
#define	RELAY_MACHINE_H


/******************************************************************************/
/* TypeDefs                                                                   */
/******************************************************************************/
typedef enum
{
	RELAY_STATE_INIT = 0,    	// <-- Init state is 0
        RELAY_STATE_USB,
        RELAY_STATE_BLUETOOTH,
	RELAY_STATE_MAX
}relay_states_t;

typedef struct
{
    relay_states_t state;
    
}relay_data_t;

/******************************************************************************/
/* Variable                                                                   */
/******************************************************************************/

extern relay_data_t relayData;


/******************************************************************************/
/* Functions                                                                  */
/******************************************************************************/

void RelayInit();
void RelayLoop();
bool relayUSBConneted();
void relayFromRadio();
void relayFromUSB();
void relayFromBluetooth();
void relayResetInjector();


#endif	

