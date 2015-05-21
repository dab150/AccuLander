/*
 * File:   debug.c
 * Author: ruslanOffice
 *
 * Created on January 20, 2015, 4:41 PM
 */


#include "xc.h"
#include "debug.h"
#include <string.h>
#include <stdio.h>
#include "mcc_generated_files/uart1.h"
#include "mcc_generated_files/uart3.h"

/**
 * Print string over UART1 (USB). This is a blocking routine.
 * This should be the only routine writing to UART. The rest of debug_print
 * should use a temp buf, sprintf and call this routine
 * @param s - string to print
 * @param len if len is unknown, use -1
 */
void __debug_print(char *s)
{
    int i, sent;
    char *ss;
    uint16_t len = strlen(s);

    for(i=len, ss=s; i > 0; )
        if(!(UART1_TransferStatusGet() & UART1_TRANSFER_STATUS_TX_FULL))
        {
            //Have data to send, the buffer has space
            //Lets write next data to send
            sent = UART1_WriteBuffer((uint8_t*)s, len);
            i -= sent;
            ss += sent;
        }
    

    for(i=len, ss=s; i > 0; )
        if(!(UART3_TransferStatusGet() & UART3_TRANSFER_STATUS_TX_FULL))
        {
            //Have data to send, the buffer has space
            //Lets write next data to send
            sent = UART3_WriteBuffer((uint8_t*)s, len);
            i -= sent;
            ss += sent;
        }
}

/**
 * Prints unsigned integer using a _debug_print function
 * @param i Integer to print
 */
void __debug_print_uint(uint64_t i)
{
    __debug_print(__ui64toa(i));
}


/**
 * Converts a signed integer to string. A result is stored in a static buffer
 * Every call to this function overwrites the prevoious result.
 * @param i integer to convert
 * @return poiter to string buffer
 */

#define INT_DIGITS 25           /* enough for 64 bit integer */

char *__i64toa(int64_t i)
{
    /* Room for INT_DIGITS digits, - and '\0' */
    static char buf[INT_DIGITS + 2];
    char *p = buf + INT_DIGITS + 1;       /* points to terminating '\0' */

    if (i > 0)
    {
        do
        {
            *--p = '0' + (i % 10);
            i /= 10;
        } while (i != 0);

        return p;
    }

    if(i < 0)
    {                        /* i < 0 */
        do
        {
            *--p = '0' - (i % 10);
            i /= 10;
        } while (i != 0);
        
        *--p = '-';
        return p;
    }

    *--p = '0';             /* i = 0 */
    return p;
}


/**
 * Converts an unsigned integer to string. A result is stored in a static buffer
 * Every call to this function overwrites the prevoious result.
 * @param i integer to convert
 * @return poiter to string buffer
 */
char *__ui64toa(uint64_t i)
{
    /* Room for INT_DIGITS digits, - and '\0' */
    static char buf[INT_DIGITS + 2];
    char *p = buf + INT_DIGITS + 1;       /* points to terminating '\0' */

    if (i > 0)
    {
        do
        {
            *--p = '0' + (i % 10);
            i /= 10;
        } while (i != 0);

        return p;
    }

    *--p = '0';             /* i = 0 */
    return p;
}

