/* 
 * File:   debug.h
 * Author: ruslanOffice
 *
 * Created on January 20, 2015, 4:44 PM
 */

#ifndef DEBUG_H
#define	DEBUG_H

#include <stdint.h>

/******************************************************************************
 *                          FUNCTION PROTOTYPES                               *
 ******************************************************************************/
void __debug_print(char *s);
void __debug_print_uint(uint64_t i);
char *__i64toa(int64_t i);
char *__ui64toa(uint64_t i);

#endif	/* DEBUG_H */

