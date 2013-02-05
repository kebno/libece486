/*!
 * @file
 * 
 * @brief ECE 486 Interface fuctions to manipulate blocks of sampled data
 * @author Don Hummels
 * @date Jan 2013
 * 
 * Interface routines that users call to handle data samples which are being 
 * streamed from the ADC and back to the DAC. 
 *
 * @defgroup ECE486_Sample ECE 486 user interface functions to handle streamed ADC/DAC data
 * @{
 */

#ifndef __SAMPLE_486__
#define __SAMPLE_486__

/*! 
 * @defgroup Sample_Buffers Sample buffers for data transfers to/from ADC/DAC
 * @{
 */
// Total Number of Samples.  DMA interrupt comes when half full

#define DEFAULT_BLOCKSIZE 100	//!< Default # samples per block of streamed ADC/DAC data

extern uint32_t ADC_Block_Size;	 //!< Number of samples user accesses per data block
extern uint32_t ADC_Buffer_Size; //!< Total buffer size being filled by DMA for ADC/DAC


/*!
 * @brief Buffer to store samples transfered from the ADC by a DMA.
 * 
 * The DMA is configured to continually transfer samples from the ADC(s) into the 
 * ADC_Input_Buffer.  When the buffer is half-full or full, an interrupt is generated
 * to allow the valid samples to be translated into float values and handed to the user.
 * 
 * In Mono input mode, samples are 12-bits, left-aligned in the least significant 16 bits of each
 * word of the buffer.  In Stereo input mode, the least significant 16 bits holds channel 1,
 * and the most significant 16 bits holds the channel 2 data.
 */
extern volatile uint32_t *ADC_Input_Buffer;

/*!
 * @brief Buffer to hold samples to be transfered to the DAC(s) by a DMA.
 * 
 * A DMA is configured to continually transfer samples from DAC_Output_Buffer to the 
 * DAC(s).
 * 
 * In Mono input mode, samples are 12-bits, left-aligned in the least significant 16 bits of each
 * word of the buffer.  In Stereo input mode, the least significant 16 bits holds channel 1,
 * and the most significant 16 bits holds the channel 2 data.
 */
extern volatile uint32_t *DAC_Output_Buffer;

/*!
 * @}  End Sample_Buffers group
 */

/*!
 * @brief keep track of whether a user is working on a buffer, or waiting for the next buffer
 */
enum Processor_Task {
  PROCESS_BUFFER,	//!< User is working on a buffer of data
  WAIT_FOR_NEXT_BUFFER	//!< User is done working... waiting for the next buffer
};

/*!
 * @defgroup ECE486_Sample_Functions User interface functions for sampled data
 * @{
 */

/*!
 * @brief Return the number of samples that the user should expect to process per block
 * 
 * Users should call getblocksize() once to determine the required number of samples which 
 * must be handled on each call to getblock() or getblockstereo().  Typically, the result is 
 * used to allocate required memory to handle the data
 * 
 * @return The number of ADC samples supplied per data request
 */
int getblocksize(void);

/*!
 * @brief Set the number of samples that the user should expect to process per block
 * 
 * setblocksize() may be called before calling initialize() to specify
 * the number of samples which should be delivered to the user on each
 * call to getblock() or getblockstereo().
 * 
 * If setblocksize() is not called, the default block size of #DEFAULT_BLOCKSIZE
 * is used for data transfers.  Using a smaller block size (down to a limit of 
 * one sample) can reduced the latency between the input and output stream, at 
 * the cost of some efficiency in handling the data.
 * 
 * @attention setblocksize() must be called before calling initialize().
 * 
 */
void setblocksize(
  uint32_t blksiz	//!< Number of samples per block of ADC/DAC data processed
);


/*!
 * @brief Request a block of data from the ADC sample stream
 * 
 * For one input channel (MONO_IN), getblock() is called by the user to request the
 * next input data block.  getblock() will wait until the ADC/DMA completes filling
 * the requested block of samples.  The binary data is then converted to float values
 * and returned to the user.
 * 
 * Output samples will range from -1.0 to +1.0 to cover
 * the range of the ADC input voltages.  The converter is
 * a 12-bit device, so the resolution of the output samples
 * is roughly 2/4096 = 0.000488.
 * 
 * Assuming the original voltage range of the ADC is 0 to 3.0 volts,
 * the conversion between the returned sample value and the input voltage is:
 * Vin = 1.5 + (sample*1.5).
 * (A zero sample value indicates the ADC is mid-scale, at 1.5 volts)
 * For a 0-3 V ADC range, the ADC resolution is 3/4096 = 732 uV.
 * 
 * @return On return, the memory block pointed to by @a working will be filled with the 
 * requested data block.  
 * 
 * @attention The user is responsible for allocating enough memory to hold the requested array
 * of float values.  The number of samples required may be obtained by calling getblocksize()
 * 
 * @sa getblocksize(), getblockstereo()
 */
void getblock(
  float * working	//!< [in,out] pointer to an array of floats (large enough to hold one buffer)
);


/*!
 * @brief Request a block of data from the two ADC sample streams (Stereo input case)
 * 
 * For two input channel (STEREO_IN), getblockstero() is called by the user to request the
 * next input data blocks.  getblockstereo() will wait until the ADCs/DMA completes filling
 * the requested block of samples.  The binary data is then converted to float values
 * and returned to the user.
 *  
 * Output samples will range from -1.0 to +1.0 to cover
 * the range of the ADC input voltages.  The converter is
 * a 12-bit device, so the resolution of the output samples
 * is roughly 2/4096 = 0.000488.
 * 
 * Assuming the original voltage range of the ADC is 0 to 3.0 volts,
 * the conversion between the returned sample value and the input voltage is:
 * Vin = 1.5 + (sample*1.5).
 * (A zero sample value indicates the ADC is mid-scale, at 1.5 volts)
 * For a 0-3 V ADC range, the ADC resolution is 3/4096 = 732 uV.
 * 
 * @return On return, the memory blocks pointed to by @a chan1 and @a chan2 will be filled with the 
 * requested data blocks.  Samples are normalized to range from -1.0 to 1.0 to cover the 
 * entire range of the ADC.
 * 
 * @attention The user is responsible for allocating enough memory to hold the requested arrays
 * of float values.  The number of samples required for each array may be obtained by 
 * calling getblocksize()
 * 
 * @sa getblocksize(), getblockstereo()
 */
void getblockstereo(
  float * chan1, 	//!< [in,out] pointer to an array of floats for the main ADC channel
  float * chan2 	//!< [in,out] pointer to an array of floats for the stereo ADC channel
);

/*!
 * @brief Send a buffer of data to the DAC output stream
 * 
 * For one output stream (MONO_OUT), putblock() is called by the user when they've finished
 * processing a block of data and are ready to stream the data to the DAC output.  Samples passed
 * to putblock() are assumed to range from -1.0 to 1.0 to cover the entire output range of the 
 * DAC.
 * 
 * @attention The number of output samples required can be determined by calling getblocksize()
 */
void putblock(
  float * working	//!< [in] Array of output samples
);

/*!
 * @brief Send two buffers of data to the DACs  for stereo output
 * 
 * For two output streams (STEREO_OUT), putblockstereo() is called by the user when they've finished
 * processing a block of data and are ready to stream the data to the DACs.  Samples passed
 * to putblock() are assumed to range from -1.0 to 1.0 to cover the entire output range of the 
 * DAC.
 * 
 * @attention The number of output samples required can be determined by calling getblocksize()
 */
void putblockstereo(
  float * chan1,	//!< [in] Array of output samples for the main output channel 
  float * chan2		//!< [in] Array of output samples for the stereo output channel
);

/*! @} End of ECE486_Sample_Functions 
 *  @} End of ECE486_Sample group
 */

#endif