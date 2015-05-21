/* 
 * File:   unit.h
 * Author: ruslan
 *
 * Created on 14 de febrero de 2015, 3:53 p.m.
 */

#ifndef UNIT_H
#define	UNIT_H


/**
 * Comment next line to disable unit tests!
 */
//#define __RUN_UNIT_TESTS__






void run_all_tests();

#ifdef __RUN_UNIT_TESTS__
#warning "Unit tests enabled! Disable Unit Testing for production version"
#endif

#endif	/* UNIT_H */

