/*!
 * @file
 * 
 * @brief Implementation of STM32F-Discovery processor configuration routines to support ECE 486
 * 
 * @author Don Hummels
 * 
 * @date Jan 27, 2013
 */

/*
 * Contains all of the subroutines to configure various peripherals of the stm32f4,
 * such as RCC, GPIO PORT{A,B,C,...}, DMAs, ADCs, DACs, etc.
 * 
 */


#include <stdint.h>
#include <stdlib.h>
#include "stm32f4_discovery.h"
#include "sample486.h"
#include "init486.h"
#include "err486.h"
#include "blinkandwait.h"
#include "config_mp45dt02.h"

/*
 * Destination for the delivery of output DAC samples.  Declared as a global
 * here so that the ISR for the Microphone sample stream can write the output
 * samples to the DAC at the appropriate time
 */
uint32_t *DAC_Data_Destination;

/*
 * Basic wrapper function to initialize peripherals for ECE 486 labs
 */
void initialize(uint16_t fs, 
		enum Num_Channels_In chanin, 
		enum Num_Channels_Out chanout)
{
  uint32_t dac_trigger;
  /*
   * Error Flags, Clocks, I/O pins...  And then rest until we see a "User" button press
   */
  initerror();
  initrcc();
  initgpio();
  blinkandwait();	// Rest here until the user button is pressed to allow 
			// st-flash to reprogram without errors. 
			// (The user may later press RESET, returning to this blocking
                        // function---allowing them to reprogram)
			// In particular, the __WFI seems to cause st-flash grief.
	
  /*
   * request memory for the ADC and DAC DMA transfer buffers
   */
  ADC_Input_Buffer = (uint32_t *)malloc(sizeof(uint32_t)*ADC_Buffer_Size);
  DAC_Output_Buffer = (uint32_t *)malloc(sizeof(uint32_t)*ADC_Buffer_Size);
  
  if (ADC_Input_Buffer==NULL || DAC_Output_Buffer==NULL ) {
    flagerror(MEMORY_ALLOCATION_ERROR);
    while(1);
  }
  
  
  /*
   * DAC Configuration
   */
  if (chanin==MONO_MIC_IN) {  // For the Mic, the I2S interface sets the clock rate
                              // DAC Samples are written as input sample arrive
    dac_trigger = DAC_Trigger_None;
  } else {                    // For the ADCs, the clock is determined using the timers
                              // DMAs are used to manage arrival and delivery of samples
    dac_trigger = DAC_Trigger_T6_TRGO;
  }
      
      
  if (chanout == MONO_OUT) {
    initdac(dac_trigger);
    DAC_Data_Destination = (uint32_t *)DAC_DHR12L2_ADDRESS;
  } else if (chanout == STEREO_OUT) {
    initdac(dac_trigger);
    initdac2(dac_trigger);
    DAC_Data_Destination = (uint32_t *)DAC_DHR12LD_ADDRESS;
  } else {
    flagerror(DAC_CONFIG_ERROR);
  }
  
  if (chanin != MONO_MIC_IN){  // The ADCs take advantage of the timers & DMAs
    initdacdma(chanout);
    inittim6(fs);
  }
	
  /*
   * Input Configuration
   */
  if (chanin == MONO_MIC_IN) {
    init_mp45dt02(fs);
  } else if (chanin == MONO_IN) {
    initadc();
    initadcdma();
    inittim3(fs);	// Triggers the ADC to determine the sample rate
    initnvic();		// Interrupts for detecting ADC Buffer fill status
  } else if (chanin == STEREO_IN) {
    initadcstereo();
    initadcdmastereo();
    inittim3(fs);	// Triggers the ADC to determine the sample rate
    initnvic();		// Interrupts for detecting ADC Buffer fill status
  }

  
  // Enable to see system clock outputs on PA8 and PC9.
  // (Be sure to enable GPIOA and GPIOC in the initrcc call)
  cfgmco();
}


/*
	initrcc() configures the Reset and Clock Control (RCC)
	module to enable the clocks to each peripheral.

	By only clocking the necessary peripherals, less power
	is used.
*/	
void initrcc(void)
{
  /*
   * apb2 max clock 84mhz
   * apb1 max clock 42mhz
   * ahb max clock 168mhz
   */
  
  // Enable port D for LEDs:  Red LED, PD14 (used for error flag)
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  
  // GPIOA clock enable 
  // (Pins 4 & 5 to be used with DACs)
  // (Pins 2 & 3 to be used with ADC1 and ADC2)
  // (Pins 0 & 1 configured as digital output for users)
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  // GPIOC clock enable (Only used for mco2 output)
  // (Needed for mco2 output on PC9)
  // (Pins PC4 and PC5 configured as digital outputs for users)
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    

  // DAC Periph clock enable 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

  // Timer 6 used to trigger the DAC
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  
  // DMA1, Stream 5, Used to transfer data to the DAC
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  
  // DMA2, Stream 0, Used to transfer data from the ADCs
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  
  // ADC1 and ADC2 Periph clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

  // Timer 3 used to trigger the ADC
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  
}

/*
	initgpio() configures the gpio ports, controlling the
	electrical properties of each pin (input, output, 
	push pull, open drain, etc. See STM32 Reference Manual)

	Usually pins being used by peripherals (such as USB,
	SPI, etc.) need to be configured in a particular way
	in the gpio registers (described in the STM32
	Reference Manual section for each peripheral).
*/
void initgpio(void)
{
  GPIO_InitTypeDef GPIO_Init_LEDs, 
                   GPIO_Init_DACs, 
		   GPIO_Init_ADCs, 
		   GPIO_Init_Flags;
  
  /* Configure PD12, PD13, PD14 and PD15 in output pushpull mode for LED access*/
  GPIO_Init_LEDs.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
  GPIO_Init_LEDs.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init_LEDs.GPIO_OType = GPIO_OType_PP;
  GPIO_Init_LEDs.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init_LEDs.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_Init_LEDs);
  
  /* Start with the Green LED lit, all others cleared 
   *  PD12: Green,
   *  PD13: Orange,
   *  PD14: Red,
   *  PD15: Blue
   */
  GPIO_SetBits(GPIOD, GPIO_Pin_12 );
  GPIO_ResetBits(GPIOD, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 );

  /* DAC channel 1 & 2 (DAC_OUT1 = PA.4)(DAC_OUT2 = PA.5) configuration */
  GPIO_Init_DACs.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_Init_DACs.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init_DACs.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_Init_DACs);

  /* ADC1-in2 (PA2) and ADC2-in3 (PA3) configuration */
  GPIO_Init_ADCs.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_Init_ADCs.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init_ADCs.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_Init_ADCs);

  /* Configure PA0, PA1, PC4, PC5 in output pushpull mode for debugging/benchmark flags */
  GPIO_Init_Flags.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_Init_Flags.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_Init_Flags.GPIO_OType = GPIO_OType_PP;
  GPIO_Init_Flags.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init_Flags.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_Init_Flags);
  GPIO_ResetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1);
  
  GPIO_Init_Flags.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_Init(GPIOC, &GPIO_Init_Flags);
  GPIO_ResetBits(GPIOC, GPIO_Pin_4 | GPIO_Pin_5);

}


/*
	initdac() configures the digital-to-analog converter
	(DAC1). Timer T6 triggers the DAC, a DMA request is made
	for new data, and then the conversion is performed.
	  
	  Because of the pinout is easier to deal with, we use 
	  DAC_Channel_2 for both MONO and STEREO channels, and to 
	  trigger the DMA transfers.  DAC_Channel_1 is enabled 
	  when STEREO output is requested.
*/
void initdac(uint32_t dac_trigger)
{
  DAC_InitTypeDef DAC_InitStructure;

  /* DAC channel2 Configuration */
  DAC_InitStructure.DAC_Trigger = dac_trigger;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
  DAC_Init(DAC_Channel_2, &DAC_InitStructure);
  
  /* Enable DAC Channel2 */
  DAC_Cmd(DAC_Channel_2, ENABLE);
  
  /* Tell the DAC to send requests to the DMA for data */
  DAC_DMACmd(DAC_Channel_2, ENABLE);
}

void initdac2(uint32_t dac_trigger)
{
  DAC_InitTypeDef DAC_InitStructure;

  /* DAC channel1 Configuration */
  DAC_InitStructure.DAC_Trigger = dac_trigger;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);
  
  /* Enable DAC Channel1 */
  DAC_Cmd(DAC_Channel_1, ENABLE);
}

/*
	initdacdma() configures the DMA that is used with
	the DAC. This DMA is triggered by a DAC request.
	The DMA grabs a sample from a buffer, and delivers
	it to the DAC.

	The DAC has multiple input registers, each for a
	different bit alignment. It's important to make sure
	the correct DMA_PeripheralBaseAddr is chosen to
	use the desired bit alignment.

	The STM32 Reference Manual and StdPeriph documentation
	have more information about the actual DMA settings.
*/
void initdacdma(enum Num_Channels_Out chanout)
{
  DMA_InitTypeDef dmainfo;
  
  /* DMA1_Stream6 channel7 (which supports requests from the DAC2) */
  DMA_DeInit(DMA1_Stream6);
  
  if (chanout == MONO_OUT) {
    dmainfo.DMA_PeripheralBaseAddr = DAC_DHR12L2_ADDRESS; // MONO: Use Dac2
  } else if (chanout == STEREO_OUT){
    dmainfo.DMA_PeripheralBaseAddr = DAC_DHR12LD_ADDRESS; // Channel 1 & 2 
    // (Channel 1 in least significant 16-bits, Channel 2 in most significant)
  }

  dmainfo.DMA_Channel = DMA_Channel_7;  
  dmainfo.DMA_Memory0BaseAddr = (uint32_t)DAC_Output_Buffer;
  dmainfo.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  dmainfo.DMA_BufferSize =  ADC_Buffer_Size;
  dmainfo.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  dmainfo.DMA_MemoryInc = DMA_MemoryInc_Enable;
  dmainfo.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  dmainfo.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  dmainfo.DMA_Mode = DMA_Mode_Circular;
  dmainfo.DMA_Priority = DMA_Priority_High;
  dmainfo.DMA_FIFOMode = DMA_FIFOMode_Disable;
  dmainfo.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  dmainfo.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  dmainfo.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream6, &dmainfo);

  //these interrupts aren't actually being used.
  //DMA1_Stream5 interrupts are disabled in the NVIC.
  DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);
  DMA_ITConfig(DMA1_Stream6, DMA_IT_HT, ENABLE);

  /* Enable DMA1_Stream5 */
  DMA_Cmd(DMA1_Stream6, ENABLE);

}




/*
	inittim6() configures timer 6 to roll over at a
	the desired sample frequency, and to trigger the DAC every
	time it rolls over.
	
	Timer 6 is associated with the APB1 bus.  The timer 6 clock frequency
	twice the APB1 bus frequency: 2*42MHz = 84 MHz.  The input parameter 
	daccount should be the desired number of timer counts before each dac output
	should be generated.  For example, to generate DAC outputs at a 48k samples/sec,
	set daccount= (84 MHz)/(48 ksps) = 1750.
*/
void inittim6(uint16_t daccount)
{	
  TIM_TimeBaseInitTypeDef timinfo;
  TIM_TimeBaseStructInit(&timinfo);
  timinfo.TIM_Period = daccount;
  timinfo.TIM_Prescaler = 0;
  timinfo.TIM_ClockDivision = TIM_CKD_DIV1;	// no division, 42 MHz timer clock
  timinfo.TIM_CounterMode = TIM_CounterMode_Up;
  timinfo.TIM_RepetitionCounter = 0;		// Not used for TIM6
	
  TIM_TimeBaseInit(TIM6, &timinfo);
	
  /* TIM6 TRGO selection */
  TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);
	
  /* TIM6 enable counter */
  TIM_Cmd(TIM6, ENABLE);
}

/*
	inittim3() configures timer 3 to roll over at a
	the desired sample frequency, and to trigger the ADC every
	time it rolls over.
	
	Timer 3 is associated with the APB1 bus.  The timer 3 clock frequency
	twice the APB1 bus frequency: 2*42MHz = 84 MHz.  The input parameter 
	daccount should be the desired number of timer counts before each adc output
	should be generated.  For example, to generate ADC outputs at a 48k samples/sec,
	set adccount= (84 MHz)/(48 ksps) = 1750.
*/
void inittim3(uint16_t adccount)
{	
  TIM_TimeBaseInitTypeDef timinfo;
  
  TIM_TimeBaseStructInit(&timinfo);
  timinfo.TIM_Period = adccount;
  timinfo.TIM_Prescaler = 0;
  timinfo.TIM_ClockDivision = TIM_CKD_DIV1;	// no division, 42 MHz timer clock
  timinfo.TIM_CounterMode = TIM_CounterMode_Up;
  timinfo.TIM_RepetitionCounter = 0;		// Not used for TIM6
	
  TIM_TimeBaseInit(TIM3, &timinfo);
	
  /* TIM3 TRGO selection */
  TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
	
  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
}


/*
	initadc() configures the analog-to-digital converter ADC1. 
	
	The ADC clock is set to the APB2 rate divided by 4: (84 MHz)/4 = 21 MHz
	This is needed, since the max ADC clock frequency is 36 MHz.
	Each conversion requires 3 cycles setup + 12 cycles conversion = 15 clock cycles
	==> Maximum sample rate = 21 MHz/15 = 1.4 Msamples/second
	(The actual sample frequency will be determined by timer 3, 
	 which triggers each conversion)
	
	When timer T3 triggers the ADC, a conversion is
	performed. The ADC hands off the new sample to the DMA,
	which delivers the sample to a buffer.

*/
void initadc(void)
{
  ADC_InitTypeDef adcinfo;
  ADC_CommonInitTypeDef common_adcinfo;
   
  // Attributes shared by all three ADCs
  common_adcinfo.ADC_Mode = ADC_Mode_Independent;  // ADC1 only
  common_adcinfo.ADC_Prescaler = ADC_Prescaler_Div4;
  common_adcinfo.ADC_DMAAccessMode = ADC_DMAAccessMode_1; 
  common_adcinfo.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;  // Ignored... since not interleaved?

  ADC_CommonInit(&common_adcinfo);
   
  // Attributes specific to ADC1 
  adcinfo.ADC_Resolution = ADC_Resolution_12b;
  adcinfo.ADC_ScanConvMode = DISABLE;
  adcinfo.ADC_ContinuousConvMode = DISABLE;
  adcinfo.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  adcinfo.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
  adcinfo.ADC_DataAlign = ADC_DataAlign_Left;
  adcinfo.ADC_NbrOfConversion = 1;
	
  //apply settings to adc 1, channel 3 (Tied to the PA3 pin)
  ADC_Init(ADC1, &adcinfo);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_3Cycles);
  ADC_DiscModeChannelCountConfig(ADC1,1);
  ADC_DiscModeCmd(ADC1,ENABLE);
  /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	
  ADC_Cmd(ADC1, ENABLE);

}


/*
	initadcdma() configures the DMA that is used with
	the ADC. This DMA is triggered by an ADC request.
	The DMA takes a sample from the ADC, and brings it
	to a buffer.

	This routine also enables the half transfer and
	transfer complete interrupts for the DMA.

	The STM32 Reference Manual and StdPeriph documentation
	have more information about the actual DMA settings.
	
	Using DMA2, Stream 4, Channel 0 (ADC1 trigger)
*/
void initadcdma(void)
{
  DMA_InitTypeDef dmainfo;
  
  DMA_DeInit(DMA2_Stream4);
  DMA_StructInit(&dmainfo);
  
  dmainfo.DMA_Channel = DMA_Channel_0;
  dmainfo.DMA_PeripheralBaseAddr = (uint32_t) (ADC1_DR_ADDRESS);
  dmainfo.DMA_Memory0BaseAddr = (uint32_t) ADC_Input_Buffer;
  dmainfo.DMA_DIR = DMA_DIR_PeripheralToMemory;
  dmainfo.DMA_BufferSize = ADC_Buffer_Size;
  dmainfo.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  dmainfo.DMA_MemoryInc = DMA_MemoryInc_Enable;
  dmainfo.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  dmainfo.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  dmainfo.DMA_Mode = DMA_Mode_Circular;
  dmainfo.DMA_Priority = DMA_Priority_High;
  dmainfo.DMA_FIFOMode = DMA_FIFOMode_Disable;
  dmainfo.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  dmainfo.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  dmainfo.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream4, &dmainfo);
  	
  DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);
  DMA_ITConfig(DMA2_Stream4, DMA_IT_HT, ENABLE);
	
  DMA_Cmd(DMA2_Stream4, ENABLE);
  
  ADC_DMACmd(ADC1, ENABLE);
  ADC_SoftwareStartConv(ADC1);
}


/*
	initadcstereo() configures the analog-to-digital converter ADC1 as the 
	master, with ADC2 as a slave in dual ADC mode. 
	
	The ADC clock is set to the APB2 rate divided by 4: (84 MHz)/4 = 21 MHz
	This is needed, since the max ADC clock frequency is 36 MHz.
	Each conversion requires 3 cycles setup + 12 cycles conversion = 15 clock cycles
	==> Maximum sample rate = 21 MHz/15 = 1.4 Msamples/second
	(The actual sample frequency will be determined by timer 3, 
	 which triggers each conversion)
	
	When timer T3 triggers ADC1, a conversion is
	performed. The ADC hands off the new sample to the DMA,
	which delivers the sample to a buffer.

*/
void initadcstereo(void)
{
  ADC_InitTypeDef adcinfo;
  ADC_CommonInitTypeDef common_adcinfo;
   
  // Attributes shared by all three ADCs
  common_adcinfo.ADC_Mode = ADC_DualMode_RegSimult; // Dual mode
  common_adcinfo.ADC_Prescaler = ADC_Prescaler_Div4;
  common_adcinfo.ADC_DMAAccessMode = ADC_DMAAccessMode_2; // dual ADCs, singel DMA transfer
  common_adcinfo.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;  // Ignored... since not interleaved?
   
  ADC_CommonInit(&common_adcinfo);
   
  // Attributes specific to ADC1 
  adcinfo.ADC_Resolution = ADC_Resolution_12b;
  adcinfo.ADC_ScanConvMode = DISABLE;
  adcinfo.ADC_ContinuousConvMode = DISABLE;
  adcinfo.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  adcinfo.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
  adcinfo.ADC_DataAlign = ADC_DataAlign_Left;
  adcinfo.ADC_NbrOfConversion = 1;
	
  //apply settings to adc 1, channel 3
  ADC_Init(ADC1, &adcinfo);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_3Cycles);
  ADC_DiscModeChannelCountConfig(ADC1,1);
  ADC_DiscModeCmd(ADC1,ENABLE);
  
  // ADC2, channel 2 gets the same configuration, but with no trigger, 
  // since it's a slave to ADC1
  adcinfo.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_Init(ADC2, &adcinfo);
  ADC_RegularChannelConfig(ADC2, ADC_Channel_2, 1, ADC_SampleTime_3Cycles);
  ADC_DiscModeChannelCountConfig(ADC2,1);
  ADC_DiscModeCmd(ADC2,ENABLE);
 
  /* Enable DMA request after last transfer  */
  ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);	
  
  ADC_Cmd(ADC1, ENABLE);
  ADC_Cmd(ADC2, ENABLE);

}



/*
	initadcdmastereo() configures the DMA that is used with
	the ADC. This DMA is triggered by an ADC request.
	The DMA takes a sample from the ADC, and brings it
	to a buffer.

	This routine also enables the half transfer and
	transfer complete interrupts for the DMA.

	The STM32 Reference Manual and StdPeriph documentation
	have more information about the actual DMA settings.
	
	Using DMA2, Stream 0, Channel 0 (ADC1 trigger)
*/
void initadcdmastereo(void)
{
  DMA_InitTypeDef dmainfo;
  
  DMA_DeInit(DMA2_Stream4);
  DMA_StructInit(&dmainfo);
  
  dmainfo.DMA_Channel = DMA_Channel_0;
  dmainfo.DMA_PeripheralBaseAddr = (uint32_t) (CDR_ADDRESS);
  dmainfo.DMA_Memory0BaseAddr = (uint32_t) &ADC_Input_Buffer;
  dmainfo.DMA_DIR = DMA_DIR_PeripheralToMemory;
  dmainfo.DMA_BufferSize = ADC_Buffer_Size;
  dmainfo.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  dmainfo.DMA_MemoryInc = DMA_MemoryInc_Enable;
  dmainfo.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  dmainfo.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  dmainfo.DMA_Mode = DMA_Mode_Circular;
  dmainfo.DMA_Priority = DMA_Priority_High;
  dmainfo.DMA_FIFOMode = DMA_FIFOMode_Disable;
  dmainfo.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  dmainfo.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  dmainfo.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream4, &dmainfo);
  	
  DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);
  DMA_ITConfig(DMA2_Stream4, DMA_IT_HT, ENABLE);
	
  DMA_Cmd(DMA2_Stream4, ENABLE);
  
  ADC_DMACmd(ADC1, ENABLE);
	
  ADC_SoftwareStartConv(ADC1);

}


/*
	initnvic() configures the Nested Vector Interrupt Controller
	(NVIC). All interrupts that are going to be used must be 
	enabled here.
*/
void initnvic(void)
{
	NVIC_InitTypeDef nvic;
	
	nvic.NVIC_IRQChannel = DMA2_Stream4_IRQn;	// ADC 1 buffer half full and complete
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&nvic);
	
	nvic.NVIC_IRQChannel = DMA1_Stream5_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = DISABLE;
	
	NVIC_Init(&nvic);
}


/*
	cfgmco() configures the microcontroller clock output pins
	This is useful for making sure the external oscillator 
	is behaving correctly.
	
	MCO1 is connected to port A, pin 8.
	MCO2 is connected to port C, pin 9.
*/
void cfgmco(void)
{
	GPIO_InitTypeDef porta8;
	GPIO_InitTypeDef portc9;
	
	//mco1 pin AF push-pull (PA8)
	porta8.GPIO_Pin = GPIO_Pin_8;
	porta8.GPIO_Speed = GPIO_Speed_100MHz;
	porta8.GPIO_Mode = GPIO_Mode_AF;
	porta8.GPIO_OType = GPIO_OType_PP;
	porta8.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOA, &porta8);
	
	// Possible Sources for MCO1:
	//    RCC_MCO1Source_HSI
	//    RCC_MCO1Source_LSE
	//    RCC_MCO1Source_HSE
	//    RCC_MCO1Source_PLLCLK
	//
	// Clock Division by 1, 2, 3, 4, or 5 needed to satisfy the GPIO Speed
	RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_4);

	
	//mco2 pin AF push-pull (PC9)
	portc9.GPIO_Pin = GPIO_Pin_9;
	portc9.GPIO_Speed = GPIO_Speed_100MHz;
	portc9.GPIO_Mode = GPIO_Mode_AF;
	portc9.GPIO_OType = GPIO_OType_PP;
	portc9.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOC, &portc9);
	
	// Possible Sources for MCO2:
	//    RCC_MCO2Source_SYSCLK
	//    RCC_MCO2Source_PLLI2SCLK
	//    RCC_MCO2Source_HSE
	//    RCC_MCO2Source_PLLCLK
	//
	// Clock Division by 1, 2, 3, 4, or 5 needed to satisfy the GPIO Speed
	RCC_MCO2Config(RCC_MCO2Source_PLLI2SCLK, RCC_MCO2Div_4);

  
}


