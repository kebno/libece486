/*!
 * @file
 * 
 * @brief MP45DT02 Mems Mic Configuration for STM32F4-Discovery Boards
 * 
 * @author Don Hummels
 * 
 * @date Mar 5, 2013
 */


#include <stdint.h>
#include <stdlib.h>
#include "stm32f4_discovery.h"
#include "sample486.h"
#include "err486.h"
#include "init486.h"

/*
 * Local Functions Prototypes:
 */
static void init_mp45dt02_NVIC(void);
static void init_mp45dt02_spi(uint16_t fs);
static void init_mp45dt02_gpio(void);

/*
 * Wrapper function called by users
 */
void init_mp45dt02(uint16_t fs) 
{
  init_mp45dt02_gpio();
  init_mp45dt02_NVIC();
  init_mp45dt02_spi(fs);
}

/*!
 * @brief GPIO Setup for MP45DT02 Microphone
 * 
 * PB10 (SPI2 SCK) and PC3 (SPI2 MOSI) are configured as "Alternate function"
 * GPIO pins to communicate with the microphone.
 */
static void init_mp45dt02_gpio(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clocks */
  /*
   * Clock Enable:
   * 
   * Two GPIO Pins needed:
   * PB10: SCK clock used to drive the mic clock
   * PC3: Data from the mic
   */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);

  /*
   * PC3: Alternate Function: SPI2_MOSI used to capture data stream from the MP45DT02
   * PB10: Alternate Function: SPI2_SCK used to provide the clock to the MP45DT02
   */
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  /* SPI2 SCK pin configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2);
  
  /* SPI2 MOSI pin configuration */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2);
}

/*!
 * @brief MP45DT02 Microphone SPI2 Initialization
 * 
 * SPI2 is used to capture the 1-bit microphone data stream and to trigger
 * the processor to filter and decimate the data stream in 16-bit blocks.
 */
static void init_mp45dt02_spi(uint16_t Freq)
{
  I2S_InitTypeDef I2S_InitStructure;

  /* Enable the SPI clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
  
  /* SPI configuration */
  SPI_I2S_DeInit(SPI2);
  
  /*
   * OK...  Here's some bogus clock generation calculations:
   * 
   * We're using the I2S clock to drive the clock terminal of the MIC.  The MIC
   * has a 1-bit output, which we'll be filtering and decimating by a factor of 
   * 64 to create the output PCM sample stream.  So we need to generate a clock
   * of 64*(desired sample rate).
   * 
   * Now, the I2S clock is generated by dividing down the PLLI2SCLK clock, which 
   * is set up at 86 MHz in the Discovery board start-up routines.  The I2S_Init()
   * routine will determine and configure the best divisor to match up with
   * a requested I2S_AudioFreq value.  BUT: The I2S clock will be at 32*(I2S_Audio_Freq),
   * (NOT 64) so we need to set I2S_Audio_Freq=2(desired sample rate) in order to get 
   * the right output clock rate.
   * 
   * It gets a little worse:  The "Freq" parameter that comes into this routine 
   * is the integer divisor which is needed to generate the desired sample rate 
   * from the Timers... which are driven by an 84MHz clock (NOT the I2S 86 MHz 
   * value!).  So:
   *    desired sample rate = 84000000/Freq          (Sample rate the caller is requesting)
   *    I2S_AudioFreq = 2(desired sample rate)       (Sample rate we request from I2S)
   *
   * The actual clock rate used to drive the MIC will be close (but not
   * exactly equal) to 32(I2S_Audio_Freq) = 64(desired_sample_rate).  The actual
   * value must be generated by an integer division of 86 MHz.  So the user WON'T
   * quite get the requested sample rate.  The 86 MHz reference was apparently chosen
   * so that the error is within the I2S specs for common audio sample rates.
   * Usually, the result is within a few Hz.  Here's a table of specific values:
   * 
   *    desired sample rate     Actual Sample Rate     Percent Error
   *         50.0000                49.7685                0.463 
   *         48.0000                47.9911                0.019 
   *         32.0000                31.9940                0.019 
   *         25.0000                24.8843                0.463 
   *         24.0000                23.9955                0.019 
   *         16.0000                15.9970                0.019 
   *         12.0000                11.9978                0.019 
   *           (All values are in kHz)
   * 
   * One other note: Requesting sample rates above 50.78 kHz or below 15.625 kHz
   * will run the MP45DT02 out of spec, since it restricts the input clock range 
   * to fall between 1 MHz and 3.25 MHz.
   * 
   */
  
  I2S_InitStructure.I2S_AudioFreq = 2*84000000/Freq;
  
  I2S_InitStructure.I2S_Standard = I2S_Standard_LSB;
  I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_16b;
  I2S_InitStructure.I2S_CPOL = I2S_CPOL_High;
  I2S_InitStructure.I2S_Mode = I2S_Mode_MasterRx;
  I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
  /* Initialize the I2S peripheral with the structure above */
  I2S_Init(SPI2, &I2S_InitStructure);

  /* Enable the Rx buffer not empty interrupt */
  SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
  
  /* Enable the SPI peripheral */
  I2S_Cmd(SPI2, ENABLE);
}

/*!
 *  @brief MP45DT02 Microphone initialization of the NVIC.
 * 
 * Every 16-bits of data from the MP45DT02 triggers an interrupt which must be 
 * processed by an ISR
 */
static void init_mp45dt02_NVIC(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3); 
  /* Configure the SPI interrupt priority */
  NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*!
 *  @brief MP45DT02 SPI2 Interrupt Handler
 * 
 * This ISR processed data from the MP45DT02 microphone.  The one-bit data stream
 * is passed through a 4-stage comb filter (which decimates by a factor of 16).  
 * The output of the comb filter is passed through a 12-coefficient FIR filter
 * (which decimates by an additional factor of 4).  The output PCM data is then
 * passed to the user input sample blocks.
 * 
 * User output samples are also written to the DAC, so that data transfers to
 * the DAC are synchronous to data transfers from the Microphone.
 */
void SPI2_IRQHandler(void)
{  
  u16 rcv;
  static u16 comb1, comb2, comb3, comb4;	// 4-stage comb filter accumulator registers
  static u16 diff1, diff2, diff3, diff4;	// 4-stage comb filter difference registers
  static u16 latch1, latch2, latch3, latch4;	// 4-stage comb filter difference registers

#define FIR_NCOEF 12
/* FIR Filter Coefficients, written as signed Q15 values */
  static int fir_coefs[FIR_NCOEF] = {
	-1229,	-862,	768,	3482,	6386,	8268,	8268,	6386,
	3482,	768,	-862,	-1229
	};
  int y;   // FIR filter input.
  
  static int next_y=0, on_deck_y=0, last_y=0;
  static int phase=0;
  
  
  static unsigned int ADC_buffer_index=0;
  static int Release_ADC_Block=0;	// Flag... set to 1 when data is available for supervisor

  /* Check if data are available in SPI Data register ("Receive Bufer not Empty" state)*/
  if (SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_RXNE) != RESET)
  {
    rcv = SPI_I2S_ReceiveData(SPI2);
	
    /*
      * 4-stage, Decimate by 16 comb filter.
      *   The 16 input bits are integrated (4-times) to get a 16-bit PCM output word
      */
    asm volatile (
      "lsl  %[in], %[in], #16   "   "\n\t" /* Shift the 16-bit word to the most-sig part */ 
      ".rept 16" "\n\t"  /* 4 accumulator stages: execute for each of the 16 bits */
      "  lsls %[in], %[in], #1	"	"\n\t" /* load the next bit into the carry flag */
      "  adc  %[c1], %[c1], #0	"	"\n\t"
      "  add  %[c2], %[c2], %[c1]"	"\n\t"
      "  add  %[c3], %[c3], %[c2]"	"\n\t"
      "  add  %[c4], %[c4], %[c3]"	"\n\t"
      ".endr"	"\n\t"
      : [c1] "+r" (comb1), 
	[c2] "+r" (comb2), 
	[c3] "+r" (comb3), 
	[c4] "+r" (comb4)
      :
	[in] "r" (rcv)
      :
	"cc"
    );
    /* Now ... 4 stages of differences to get the output sample */
    /* (Counting on everything being done modulo 2^16 here) */
    diff1 = comb4 - latch1;
    latch1 = comb4;
    diff2 = diff1 - latch2;
    latch2 = diff1;
    diff3 = diff2 - latch3;
    latch3 = diff2;
    diff4 = diff3 - latch4;
    latch4 = diff3;
    
    /*
      * The comb filter output is filtered by a 12-coef FIR filter
      * which decimates by a factor of 4 
      * 
      * The filter gives a +/- 0.3 dB gain flatness from DC to 0.25*(output sample rate).
      * Signals above 0.75*(output sample rate) are attenuated by at least 
      * 25 dB---but that's about all you get for anti-aliasing of the input
      * audio signal.  
      * 
      * The combination of the above comb filter with this FIR filter attenuates
      * the quantization noise by over 100 dB. 
      */

    /*
     * Polyphase implementation to spread computations over all interrupts 
     * (instead of every 4th).  This is a hard-coded 12-coef fir filter which
     * decimates by a factor of 4
     */
    y = diff4;
    switch(phase) {
      case 0:
	next_y += fir_coefs[0]*y;
	on_deck_y += fir_coefs[4]*y;
	last_y += fir_coefs[8] * y;
	
	/* 
	 * On the zero phase, we push out the "next_y" sample to the user
	 * Also, an output sample is sent to the DAC
	 */
	*DAC_Data_Destination = DAC_Output_Buffer[ADC_buffer_index];
	ADC_Input_Buffer[ADC_buffer_index++] = next_y>>15;



	if (ADC_buffer_index == ADC_Buffer_Size) {
	      ADC_buffer_index = 0;
	      Lower_Ready = 0;
	      Release_ADC_Block = 1;
	}
	
	if(ADC_buffer_index == ADC_Block_Size) { // Half way to completing the input block
	  Lower_Ready = 1;
	  Release_ADC_Block = 1;
	}
	
	if (Release_ADC_Block == 1) {
	  Release_ADC_Block = 0;
	
	  if (Sampler_Status == WAIT_FOR_NEXT_BUFFER) {
	    Sampler_Status = PROCESS_BUFFER;
	  } else {
	    flagerror(SAMPLE_OVERRUN);	// If the supervisor was not waiting for the next
			      // buffer, flag the error to let him/her know that 
			      // they're missing blocks of data.
	  }
        }
	/*
	 * Rotate the filter totals after we've pushed out an output sample
	 */
	next_y = on_deck_y;
	on_deck_y = last_y;
	last_y = 0;
	phase = 1;	// Next Phase
	break;
      case 1:
	next_y += fir_coefs[3]*y;
	on_deck_y += fir_coefs[7]*y;
	last_y += fir_coefs[11] * y;
	phase = 2;	// Next Phase
	break;
      case 2:
	next_y += fir_coefs[2]*y;
	on_deck_y += fir_coefs[6]*y;
	last_y += fir_coefs[10] * y;
	phase = 3;	// Next Phase
	break;
      case 3:
	next_y += fir_coefs[1]*y;
	on_deck_y += fir_coefs[5]*y;
	last_y += fir_coefs[9] * y;
	phase=0;	// Next Phase
	break;
      default:
	phase=0;
    }
  }
}

