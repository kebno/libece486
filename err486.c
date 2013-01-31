/*!
 * @file
 * @brief Implementation of error handling routines for ECE 486
 * @author Don Hummels
 * @date Jan, 2013
 */

/*
	Sets up a global error (circular) buffer. When flagerror() is called, a value
	corresponding to that error is pushed into the buffer. The header file contains
	the list of #define's that match error numbers with their errors. When an error
	occurs, the red LED on port D, pin 14 will light up, and remain lit.
*/

#include "stm32f4_discovery.h"
#include "err486.h"

int errorbuf[ERRORBUFLEN];
int erroridx = 0;

/*
	initerror() is called once to set up the global error buffer
*/
void initerror(void)
{
	int i;
	
	for (i=0; i<ERRORBUFLEN; i++)
		errorbuf[i] = 0;
	
	//turn off our error indicating LED
	GPIO_ResetBits(GPIOD, GPIO_Pin_14);
}

/*
	flagerror() is called when an error occurs.
	errorcode is written to the error buffer.
*/
void flagerror(int errorcode)
{
	//turn on our error indicating LED
	GPIO_SetBits(GPIOD, GPIO_Pin_14);
	
	errorbuf[erroridx] = errorcode;
	
	erroridx++;
	
	if (erroridx == ERRORBUFLEN)
		erroridx = 0;
}