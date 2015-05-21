/* 
 * File:   circBuff.c
 * Author: ruslanOffice
 *
 * Created on February 3, 2015, 4:40 PM
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "CircularBuffer.h"



/**
 * Init circular buffer
 * @param buf Pointer to a buffer structure
 * @param arr Pointer to an array to store the data
 * @param size Size of the array
 */
void CircularBufferInit(CircularBuffer_t *buf, CircularBufferArray_t *arr, uint16_t size)
{        
    buf->size=size;
    buf->writePointer = 0;
    buf->readPointer  = 0;
    buf->arr = arr;
}


/**
 * Buffer is full?
 * @param buf Pointer to a buffer structure
 * @return true if full
 */
bool CircularBufferIsFull(CircularBuffer_t* buf)
{
    return ((buf->writePointer + 1) % buf->size == buf->readPointer);
}


/**
 * Buffer is empty?
 * @param buf Pointer to a buffer structure
 * @return true if empty
 */
bool CircularBufferIsEmpty(CircularBuffer_t* buf)
{
    return (buf->readPointer == buf->writePointer);
}


/**
 * Add new element to the buffer. This function doesn't check if there is space
 * @param buf Pointer to a buffer structure
 * @param v New value
 */
void CircularBufferEnque(CircularBuffer_t* buf, CircularBufferArray_t v)
{
    buf->arr[buf->writePointer] = v;
    buf->writePointer++;
    buf->writePointer %= buf->size;
}


/**
 * Remove next element from the buffer. This function doesn't check if buffer
 * is empty or not
 * @param buf Pointer to a buffer structure
 * @return Next element from the buffer
 */
CircularBufferArray_t CircularBufferDeque(CircularBuffer_t* buf)
{
    CircularBufferArray_t res = buf->arr[buf->readPointer];
    buf->readPointer++;
    buf->readPointer %= buf->size;
    return(res);
}

/**
 * Calculate the free space available in the buffer
 * @param buf Pointer to a buffer structure
 * @return free space
 */
uint16_t CircularBufferFreeSpace(CircularBuffer_t* buf)
{
    uint16_t r,w;
    r = buf->readPointer;
    w = buf->writePointer;

    if(r < w)   return buf->size - 1 - (w - r);
    if(r > w)   return r - w - 1;

    //r == w, Buffer is empty
    return buf->size - 1;
}

/**
 * Deletes all the information stored in the buffer
 * @param buf Pointer to a buffer structure
 */
void CircularBufferClear(CircularBuffer_t* buf)
{
    buf->readPointer = 0;
    buf->writePointer = 0;
}
