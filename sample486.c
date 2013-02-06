/*!
 * @file
 * 
 * @brief Implementation of user sampler interface functions
 * @author Don Hummels
 * @data Jan 2013
 */

#include "stm32f4_discovery.h"
#include <stdlib.h>
#include "sample486.h"
#include "err486.h"
#include "arm_math.h"

/*
 * Global data buffers which are being filled/emptied by the DMAs
 */
volatile uint32_t *ADC_Input_Buffer=NULL;	
volatile uint32_t *DAC_Output_Buffer=NULL;

/*
 * Data Block sizes for streamed ADC/DAC data
 */
uint32_t ADC_Block_Size = DEFAULT_BLOCKSIZE;	//!< Number of samples user accesses per data block
uint32_t ADC_Buffer_Size = 2*DEFAULT_BLOCKSIZE; //!< Total buffer size being filled by DMA for ADC/DAC

/*
 * Sampler_Status indicates whether the user is working on a buffer of data,
 * or has finished working on the buffer and is waiting for the next buffer to 
 * arrive. The variable is used in the interrupt service routine to detect 
 * whether a "buffer overrun" has occurred.
 */
static enum Processor_Task volatile Sampler_Status;

/*
 * The ISR also sets the lowerrdy flag to indicate whether the processor 
 * should be working  on the upper half or the lower half of the DMA 
 * data buffers.  (The user gets to work on
 * one half of the data, while the DMAs are streaming the other half.)
 */
static volatile int lowerrdy = 0;  // Set by the ISR to indicate which 
				   // buffer is available

/*
 * Pointers to the half-buffer which should be worked upon
 */
static volatile uint32_t * inbuf;	
static volatile uint32_t * outbuf;	



/*
	Simple function to return block size.
	Needed for creating a working buffer.
*/
int getblocksize()
{
	return ADC_Block_Size;
}

/*
 * Set the number of samples that the user should expect to process per block
 * 
 */
void setblocksize( uint32_t blksiz )
{
  /*
   * setblocksize() should only be called before calling initialize().
   * If the ADC & DAC buffers have already been allocated, then initialize()
   * must have already been called, and we're too late to change the buffer
   * sizes.  Flag an error and return without changing anything.
   */
  if (ADC_Input_Buffer != NULL) {
    flagerror(SETBLOCKSIZE_ERROR);
    return;
  }
  
  ADC_Block_Size = blksiz;
  ADC_Buffer_Size = 2*blksiz;
}

/*
	getblock() is called by the user when the user is
	ready to process a new block of samples. The user
	provides a pointer to an (already allocated) buffer
	that will be filled with float samples.
	
	Output samples will range from -1.0 to +1.0 to cover
	the range of the ADC input voltages.  The converter is
	a 12-bit device, so the resolution of the output samples
	is roughly 2/4096 = 0.000488.
	
	Assuming the original voltage range of the ADC is 0 to 3.0 volts,
	the conversion between the returned sample value and the input 
	voltage is:
	  Vin = 1.5 + (sample*1.5)
	(A zero sample value indicates the ADC is mid-scale, at 1.5 volts)
	For a 0-3 V ADC range, the ADC resolution is 3/4096 = 732 uV.
*/
void getblock(float * working)
{
  uint32_t i;

  // Wait for the DMA to finish filling a block of data
  Sampler_Status = WAIT_FOR_NEXT_BUFFER;
  while (Sampler_Status == WAIT_FOR_NEXT_BUFFER) __WFI();

  // The DMA ISR sets the lowerrdy flag to indicate whether we should
  // be processing the upper or lower half of the DMA transfer block.
  if (lowerrdy) {
    inbuf = ADC_Input_Buffer;
    outbuf = DAC_Output_Buffer;
  } else {
    inbuf = &(ADC_Input_Buffer[ADC_Block_Size]);
    outbuf = &(DAC_Output_Buffer[ADC_Block_Size]);
  }

  // Now convert the valid ADC data into the caller's array of floats.
  // Samples are normalized to range from -1.0 to 1.0
  for (i=0; i< ADC_Block_Size; i++) {
     // 1/32768 = 3.0517578e-05  (Multiplication is much faster than dividing)
     working[i] = ((float)((int)inbuf[i]-32767))*3.0517578e-05f; 
  }
}

/*
	putblock() is called by the user when they've finished
	processing a block and are ready to send samples to 
	the DAC for output.

	Samples are converted from type float to 16-bit samples
	for the DAC.  The result are placed in the DAC DMA buffer.
	  
	Input sample values should range from -1.0 to 1.0 to cover
	the full-scale range of the DAC
	
	(The DAC is actually a 12-bit converter, with the data 
	aligned in the upper portion of the 16-bit output value)
*/
void putblock(float * working)
{
  uint32_t i;

  // the "outbuf" pointer is set by getblock() to indicate the 
  // appropriate destination of any output samples.
  //
  // floating point values between -1 and +1 are mapped 
  // into the range of the DAC
  for (i=0; i<ADC_Block_Size; i++) {
    outbuf[i] = ((int)((working[i]+1.0)*32768.0f)) & 0x0000ffff;
  }
}


void putblockstereo(float * chan1, float * chan2)
{
	uint32_t i;

  // the "outbuf" pointer is set by getblock() to indicate the 
  // appropriate destination of any output samples.
  //
  // floating point values between -1 and +1 are mapped 
  // into the range of the DAC
  //
  // chan1 goes into the most-significant 16 bits (DAC1), 
  // chan2 in the least significant (DAC1)
  for (i=0; i<ADC_Block_Size; i++) {
    outbuf[i] = ( ((int)((chan2[i]+1.0)*32768.0f)) & 0x0000ffff ) |
                ((((int)((chan1[i]+1.0)*32768.0f)) & 0x0000ffff)<<16);
  }
}


void getblockstereo(float *chan1, float *chan2)
{
  uint32_t i;

  // Wait for the DMA to finish filling a block of data
  Sampler_Status = WAIT_FOR_NEXT_BUFFER;
  while (Sampler_Status == WAIT_FOR_NEXT_BUFFER) __WFI();

  // The DMA ISR sets the lowerrdy flag to indicate whether we should
  // be processing the upper or lower half of the DMA transfer block.
  if (lowerrdy) {
    inbuf = ADC_Input_Buffer;
    outbuf = DAC_Output_Buffer;
  } else {
    inbuf = &(ADC_Input_Buffer[ADC_Block_Size]);
    outbuf = &(DAC_Output_Buffer[ADC_Block_Size]);
  }

  // Now convert the valid ADC data into the caller's arrays of floats.
  // Samples are normalized to range from -1.0 to 1.0
  // Channel 1 is in the least significant 16 bits of the DMA transfer data,
  // Channel 2 is in the most significant 16 bits.
  for (i=0; i< ADC_Block_Size; i++) {
     // 1/32768 = 3.0517578e-05  (Multiplication is much faster than dividing)
     chan1[i]=((float)( (int)(inbuf[i]&0x0000ffff)-32767))*3.0517578e-05f; 
     chan2[i]=((float)( (int)((inbuf[i]&0xffff0000)>>16)-32767))*3.0517578e-05f; 
  }
}




/*
	This handles the interrupt generated when the
	ADC DMA buffer gets half full or full.
*/
void DMA2_Stream4_IRQHandler(void)
{
  
  if (DMA_GetITStatus(DMA2_Stream4, DMA_IT_HTIF4)) {
    DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_HTIF4);
    lowerrdy = 1; 			//lower half ready for processing
    // GPIO_SetBits(GPIOA, GPIO_Pin_1 );
    
  } else if (DMA_GetITStatus(DMA2_Stream4, DMA_IT_TCIF4)) {
    DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_TCIF4);
    lowerrdy = 0;			//upper half ready for processing
    // GPIO_ResetBits(GPIOA, GPIO_Pin_1 );
    
  } else {
    flagerror(DEBUG_ERROR);
  }
	
  if (Sampler_Status == WAIT_FOR_NEXT_BUFFER) {
    Sampler_Status = PROCESS_BUFFER;	// Turn the supervisor loose on the 
					// next buffer
  } else {
    flagerror(SAMPLE_OVERRUN);	// If the supervisor was not waiting for the next
				// buffer, flag the error to let him/her know that 
				// they're missing blocks of data.
  }
}


