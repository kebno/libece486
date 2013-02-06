/*!
 * @file
 * 
 * @brief Error Handling for ECE 486 STM32F4-Discovery Interface
 * @author Don Hummels
 * @date Jan, 2013
 * 
 * Defines error flags and error indication mechanisms for the ECE 486 interface
 * to the STM32F4-Discovery development boards
 * 
 */

/*!
 * @page ErrorHandling Error Handling Interface
 * 
 * When an error is detected, an error code is recorded in a circularly accessed
 * global error buffer (#errorbuf), and the red LED (Port D, Pin 14) is lit to indicate the 
 * error condition to the user.  Users may then use a debugger to investigate 
 * error buffer to learn the cause of the most recent sequence of errors.
 * 
 *  - When debugging, examine #errorbuf to learn the most recent sequence of errors.
 *  - #errorbuf[] is of length #ERRORBUFLEN, and is written to circularly, so that when
 *    the buffer is filled, errors continue to be written by restarting at the start of the array.
 *  - The global #erroridx gives the index into #errorbuf at which the next error
 *    will be stored.  The most recent error is stored at index #erroridx - 1.
 *  - An error is "thrown" by calling @code
 *     #include "err486.h"
 *       ...
 *     flagerror(error_code);
 *    @endcode where @a error_code is an integer value to be stored in the #errorbuf array
 *  - Predefined error codes are provided in err486.h for #SAMPLE_OVERRUN, 
 *    #MEMORY_ALLOCATION_ERROR, #DAC_CONFIG_ERROR, #ADC_CONFIG_ERROR, #SETBLOCKSIZE_ERROR,
 *    and #DEBUG_ERROR.
 */

/*!
 * @defgroup ECE486_Error ECE 486 Discovery Interface Error Handling
 * @{
 */


#ifndef __ERR486__
#define __ERR486__

#define ERRORBUFLEN 100		//!< Number of errors to record in a circular buffer

/*!
 * @defgroup ECE486_Error_Codes ECE 486/Discovery Error Codes
 * @{
 */
#define SAMPLE_OVERRUN 2		//!< ADC buffer filled before the user serviced the buffer
#define MEMORY_ALLOCATION_ERROR 3	//!< malloc() or calloc() returned NULL
#define DAC_CONFIG_ERROR 4
#define ADC_CONFIG_ERROR 5
#define SETBLOCKSIZE_ERROR 6		//!< setblocksize() must be called BEFORE initialize()
#define DEBUG_ERROR 13			//!< Generic code that users may use
/*! @} End of ECE486_Error_Codes Group */


/* --------- Globals --------------------------- */

/*!
 * @brief Circular buffer to store the most recent error codes
 */
extern int errorbuf[];

/*!
 * @brief Index of the NEXT error to be written into the errorbuf array
 */
extern int erroridx;

/*!
 * @defgroup ErrorFunctions Error Handling Functions
 * @{
 */

/*!
 * @brief Initialization routine to set up error buffers
 * 
 * Called once, at program start-up: Sets all error codes in errorbuf[i]
 * to zero, and clears the red LED.
 */
void initerror(void);

/*!
 * @brief Records and indicates an error condition
 * 
 *  - Writes the input value @a errorcode into the #errorbuf array
 *  - Circularly increments the error index #erroridx
 *  - Indicates an error by lighting the red LED
 */
void flagerror(
  int errorcode		//!< [in] Error code to be stored in #errorbuf[#erroridx]
);

/*! @} End of ErrorFunctions group */
/*! @} End of ECE486_Error group */
#endif