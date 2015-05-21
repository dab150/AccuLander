/* 
 * File:   CircularBuffer.h
 * Author: ruslanOffice
 *
 * Created on February 3, 2015, 5:33 PM
 */

#ifndef CIRCULARBUFFER_H
#define	CIRCULARBUFFER_H


/******************************************************************************/
/* TypeDefs                                                                   */
/******************************************************************************/

/* Elements to store in a buffer*/
typedef uint8_t CircularBufferArray_t;


typedef struct
{
   uint16_t writePointer;
   uint16_t readPointer;
   uint16_t size;               //Size fo the buffer
   CircularBufferArray_t *arr;  //Array where data are stored
} CircularBuffer_t;



/******************************************************************************/
/* Functions                                                                  */
/******************************************************************************/
void CircularBufferInit(CircularBuffer_t *buf, CircularBufferArray_t *arr, uint16_t size);
bool CircularBufferIsFull(CircularBuffer_t* buf);
bool CircularBufferIsEmpty(CircularBuffer_t* buf);
void CircularBufferEnque(CircularBuffer_t* buf, CircularBufferArray_t v);
CircularBufferArray_t CircularBufferDeque(CircularBuffer_t* buf);
uint16_t CircularBufferFreeSpace(CircularBuffer_t* buf);


#endif	/* CIRCULARBUFFER_H */

