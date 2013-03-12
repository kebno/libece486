/*!
  @file
  @brief STM32F4-Discovery ECE 486 Interface
  @author Don Hummels
  @date Jan, 2013

    @mainpage STM32F4-Discovery support library for ECE 486 Digital Signal Processing

    This library defines an interface that allows a user to stream input sampled data
    in real time from one or two ADCs or the Microphone, process the data, 
    and stream the resulting output samples to one or two output DACs.
	
    The user interface is as follows:
	
      -# Required include: @code #include "ece486.h" @endcode
      -# Optionally, the user calls @code
                    setblocksize(nsamp);
        @endcode
        This call is used to change the data block size that will be delivered
        to the user in later function calls. (If setblocksize() is not called,
        a default value of #DEFAULT_BLOCKSIZE is used.)  Using a larger block
        size may result in more efficient code, while using a smaller block 
        size will reduce the latency of the system.
      -# The user calls @code
		    initialize(fs, input_mode, output_mode);
	@endcode 
	where:
	  - @a fs determines the sampling frequency.  @a fs should be one of: 
		  #FS_500K, #FS_400K, #FS_200K, #FS_100K, #FS_96K, #FS_50K, 
		  #FS_48K, #FS_24K, #FS_12K, #FS_10K, #FS_8K, #FS_6K, #FS_5K
	  - @a input_mode should be #MONO_IN or #STEREO_IN or #MONO_MIC_IN
	  - @a output_mode should be #MONO_OUT or #STEREO_OUT
	      
	The same sample rate is used for the input (ADCs or MIC) 
	and output (DAC(s)) data streams.  
	The initialize() call enables clocks and configures the 
	appropriate Microphone, ADC(s), DAC(s), Timers, buttons, DMAs, and 
	interrupts.  The function pauses at the beginning of the 
	initialization (with orange and blue LEDs flashing) to wait 
	for a user push-button indicating to continue (allowing the 
	device to be re-programmed without errors).  
	    
	For @a input_mode = #MONO_IN or #STEREO_IN, 
	Analog input waveforms are sampled using ADC1 (MONO) or 
	ADC1 & ADC2 (STEREO_IN).  ADC1 is accessed as an input on 
	PA3 (MONO_IN or STEREO_IN).  ADC2 is accessed on PA2 (STEREO_IN only).
	
	For @a input_mode = #MONO_MIC_IN, the one-bit on-board microphone is 
	initialized.  In this case the I2S clock controls the sampling frequency,
	and the timers and DMAs are left unused.  PA3 and PA2 are left uninitialized.
	    
	Analog output is generated using DAC2 (MONO_OUT) or DAC2 & DAC1
	(STEREO_OUT).  DAC2 is accessed as an output on PA5 (MONO_OUT or
	STEREO_OUT).  DAC1 is accessed on PA4 (STEREO_OUT only).
	   
	LED GPIO pins (PD12, PD13, PD14, PD15) are enabled, and digital 
	outputs are configured for PA1, PC4, and PC5. 
      -# The user calls @code
		nsamp = getblocksize();
	@endcode 
	@a nsamp gives the data block size which will be delivered to
	the user in later function calls.  Input samples (either from the 
	microphone or from the ADC(s) via the DMA)
	will be accessed in blocks of @a nsamp samples at a time. 
	Likewise, the user programs should generate "nsamp" output 
	samples for delivery to the DAC.  Typically, users call
	getblocksize() to learn the required number of samples so 
	that the correct amount of memory for processing the blocks 
	of data may be allocated.
      -# Optionally, the user calls @code
		fs = getsamplingfrequency();
	@endcode
	This function returns the best guess (assuming the HFE oscillator is 
	exactly at 10 MHz) at the actual sampling rate being implemented (in
	samples/second).  The actual sample rate may be slightly different from
	the requested  rate, particularly if the Microphone is used as the input
	device.
      -# Users then repeatedly call getblock() or getblockstereo() to obtain
	samples from the MIC or ADC(s), and (after processing the samples) 
	putblock() or putblockstereo() to write the results back 
	to the DAC.  Using one of:
	@code
	  getblock(input1);  			// mono input
	  getblockstereo(input1, input2);	// stereo input
	@endcode
	will fill in the user's input array(s) (type float) with 
	normalized input (ADC or MIC) data.  Callers are responsible to make sure 
	that the arrays @a input1 and @a input2 are allocated with enough 
	memory to hold @a nsamp float values.  After processing the 
	input waveforms to create results, the output samples are 
	placed in the DAC output buffers using one of
	@code
	  putblock(result1); 			// Mono Output 
	  putblockstereo(result1, result2);	// Stereo Output
	@endcode
	Typically, after calling the putblock() routine, the user 
	calls getblock() to wait for the next available block of 
	input data, and the process repeats.  If the DMA 
	completes filling a new block of data before the user calls 
	getblock(), a #SAMPLE_OVERRUN error is indicated, and 
	the RED LED is lit.  In this case, the data is not being 
	processed fast enough to keep up with the input data stream, 
	and input samples are being lost.
      .	    
      Sample values in the "input" or "result" arrays are of 
      type float, and are normalized to range from -1.0 to +1.0 
      over the range of the ADCs & DACs. (A sample of -1.0 is near 0V,
      +1.0 is near 3V, and 0.0 indicates the mid-scale voltage of 1.5V)
	   
      Digital outputs are also configured on pins PA1, PC4, and PC5.  
      Users can set the values of these outputs using statements such as:
      @code
         GPIO_ResetBits(GPIOA, GPIO_Pin_1 );	// Reset PA1
         GPIO_SetBits(GPIOC, GPIO_Pin_4 );	// Set PC4
         GPIO_ToggleBits(GPIOC, GPIO_Pin_5 );	// Toggle PC5
      @endcode
      The pins may be set/cleared to enable benchmarking of segments of
      code using a scope.
      
      The LEDs on the discovery boards are accessed in the same way 
      through GPIO pins:
         PD12 (Green), PD13 (Orange), PD14 (Red), PD15 (Blue).  
      Normally the Green LED is lit indicating normal operation, and 
      the Red LED is used to flag an error condition.
      The Orange and Blue LEDs are unused.
      @code
         GPIO_SetBits(GPIOD, GPIO_Pin_15 );	// Light the Blue LED
         GPIO_ResetBits(GPIOD, GPIO_Pin_13 );	// Turn off the Orange LED
      @endcode
      
      Users can check the global variable #UserButtonPressed to 
      check whether the (blue) user button on the Discovery board 
      has been pressed.   #UserButtonPressed==#Button_Pressed indicates that 
      a button press has occurred.  
      The user should reset the value using #UserButtonPressed=#Button_Ready;
*/

/*!
  @page Peripherals Overview of STM32F4xx Peripheral/Pin Configuration
  
  If the Microphone is used as the input device, SPI2 is used to provide
  the I2S clock to the Mic via PB10.  Data is read from the Mic on pin
  PC3.  In this case: ADC1, ADC2, DMA2_Stream4, DMA1_Stream6, TIMER 6, and 
  TIMER 3 are left uninitialized.  All data transfers from the Mic and to the 
  DAC(s) are handled by the interrupt service routine driven by the SPI2 interface.
  
  If the ADC(s) are used to capture the input stream, the following peripheral 
  assignments are used:
 
|Pin    | Peripheral    | Use
|------ | ------------- | -------------------------------------------------
|       | TIMER 6       | Used to trigger the DAC2 and (if STEREO_OUT) DAC1
|PA5    | DAC2		| Main output channel  
|PA4    | DAC1		| Stereo output channel 
|       | DMA1_Stream6  | Used to move samples from a buffer to the DAC(s).
|	| TIMER 3	| Used to trigger ADC1.
|PA3    | ADC1		| Main input channel.
|PA2    | ADC2		| Stereo input channel.  
|	| DMA2_Stream4  | Used to move samples from the ADC(s) to a buffer.

    - DMA1_Stream6 is triggered by DAC2 using channel 7
    - DMA2_Stream4 is triggered by ADC1 using channel 0
    - ADC2 is a slave to ADC1 in dual sampling mode (Stereo input case)
    
  DMA2_Stream4 data is moved into an internal buffer of size #ADC_Buffer_Size.  
  Interrupts from DMA2_Stream4 are generated when the buffer is 
  half-full and full, indicating that the user may process one-half 
  of the samples, while the other half is being filled.  
  getblocksize() returns a value of #ADC_Block_Size, which is actually half 
  of the size of the actual DMA buffer (indicating the amount of data free to be 
  processed in each sample block).
	
*STM32F4-Discovery Port/Pin Assignments:*

|Pin   	| Use             | Description 				|
| ---- 	| --------------- | --------------------------------------------|
|PA0	|User Push-button |	Configured as External Interrupt 0   	| 
|PA1	|GPIO Output	  |Available: Configured as digital output 	|
|PA2	|ADC2		  |Stereo analog input Channel			|
|PA3	|ADC1		  |Mono/Stereo analog input channel		|
|PA4	|DAC1		  |Stereo analog output channel			|
|PA5	|DAC2		  |Mono/Stereo analog output channel		|
|PC3	|SPI2_MOSI	  |Data capture from MP45DT02 Microphone	|
|PC4	|GPIO Output	  |Available: Configured as digital output	|
|PC5	|GPIO Output	  |Available: Configured as digital output	|
|	|		  |						|
|PD12	|GPIO Output	  |Green LED:	Used to indicate normal running	|
|PD13	|GPIO Output	  |Orange LED:	Available 			|
|PD14	|GPIO Output	  |Red LED:	Used to indicate Errors		|
|PD15	|GPIO Output	  |Blue LED:	Available			|
|	|		  |						|
|PB10	|SPI2_SCK	  |I2S Clock used to drive MP45DT02 Microphone	|

   - EXTI0_ISR() sets the global variable #UserButtonPressed to a value
     of #Button_Pressed on each button press
*/

#ifndef __ECE486__
#define __ECE486__

#include "blinkandwait.h"
#include "init486.h"
#include "err486.h"
#include "sample486.h"

#endif