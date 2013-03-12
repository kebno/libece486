/*!
 * @file
 * 
 * @brief MP45DT02 Mems Mic Configuration for STM32F4-Discovery Boards
 * 
 * @author Don Hummels
 * 
 * @date Mar 5, 20133
 * 
 * 
 * @defgroup ECE486_Microphone MP45DT02 Mems Mic Configuration
 * @{
 */

#ifndef __CONFIG_MP45DT02__
#define __CONFIG_MP45DT02__

/*
 * Includes..........................................................................
 */
#include <stdint.h>

/*!
 * @brief Wrapper function to perform initialization of MP45DT02 Microphone
 * 
 * The MP45DT02 Microphone on the STM32F407-Discovery boards is initialized 
 * for use at a desired audio sampling rate.  The SPI2 peripheral is initialized 
 * to provide the required I2S clock to the device, and to capture the 1-bit
 * audio data stream from the device.  The SPI2 generates an interrupt after
 * every 16-bits are captured.  Audio decimation filters are initialized to 
 * process the captured data and provide output PCM data samples.
 * 
 * The input parameter @a fs determines (approximately) the output PCM 
 * sampling frequency.  The value is provided as in integer divisor of the 
 * system clock frequency of 84 MHz...  So the resulting sampling frequency
 * will be close to (84 MHz/fs).  In implementation, an 86 MHz reference clock
 * is divided to provide the I2S Mic clock at approximately 64*(84MHz/fs).  The
 * digital data stream is filtered and decimated by a factor of 64 to get close
 * to the desired sample rate.  Users may call #getsamplingfrequency() to learn
 * the resulting sample rate that is implemented.
 */
void init_mp45dt02(
  uint16_t fs 		//!< Integer determining the PCM output sampling frequency
) ;

#endif

/*!
 * @} End of ECE486_Microphone group
 */