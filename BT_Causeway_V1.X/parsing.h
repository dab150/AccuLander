/* 
 * File:   parsing.h
 * Author: cironid12
 *
 * Created on May 19, 2015, 3:44 PM
 */

#ifndef PARSING_H
#define	PARSING_H

/******************************************************************************
 *                              TYPEDEFS                                      *
 ******************************************************************************/

typedef union
{
    int intwise;
    unsigned char bytewise[2];

} RC_Channel_PWM;

extern RC_Channel_PWM RC_Channel_6;



/******************************************************************************
 *                          FUNCTION PROTOTYPES                               *
 ******************************************************************************/
void CheckRCLoopInit(void);
int CheckRCLoop(unsigned char x);

#endif	/* PARSING_H */

