/**!
 * @file
 * 
 * @brief STM32F-Discovery configuration routines to support ECE 486
 * 
 * @author Don Hummels
 * 
 * @date Jan 27, 2013
 * 
 * Contains all of the subroutines to configure various peripherals of the stm32f4,
 * such as RCC, GPIO PORT{A,B,C,...}, DMAs, ADCs, DACs, etc.
 * 
 * @defgroup ECE486_Init ECE 486 Processor Initialization 
 * @{
 */

#ifndef __INIT486__
#define __INIT486__

/*
 * Includes..........................................................................
 */
#include <stdint.h>


/*
 * Defines..........................................................................
 */
/*!
 * @defgroup STM32F4xx_Addresses STM32F4xx Peripheral Config Register addresses
 * @{
 */
// DAC Output Data Registers
#define DAC_DHR12R1_ADDRESS    0x40007408  //!< DAC Ch 1, 12 bit right aligned register
#define DAC_DHR12L1_ADDRESS    0x4000740C  //!< DAC Ch 1, 12 bit left aligned register
#define DAC_DHR12R2_ADDRESS    0x40007414  //!< DAC Ch 2, 12 bit right aligned register
#define DAC_DHR12L2_ADDRESS    0x40007418  //!< DAC Ch 2, 12 bit left aligned register
#define DAC_DHR12RD_ADDRESS    0x40007420  //!< DAC dual transfer, 12 bit right aligned register
#define DAC_DHR12LD_ADDRESS    0x40007424  //!< DAC dual transfer, 12 bit right aligned register

// ADC Contents Data Register
#define ADC1_DR_ADDRESS        0x4001204C  //!< ADC1 Data Register
#define CDR_ADDRESS            0x40012308  //!< ADC Common data register for dual transfers

/*! @} End of STM32F4xx_Addresses group */

// Valid Clock divisors to sample rate constants for initialize(), inittim3(), inittim6()
// (Integers used to divide an 84 MHz clock to obtain a desired sample rate)
/*! 
 * 
 * @defgroup FS_Count_Values Timer Count constants for determining ADC/DAC sample rates
 * @{
 * 
 * Timers 3 and 6 are each driven by 84 MHz clock signals.  These integer constants are 
 * used to divide this clock rate down to standard audio sampling frequencies for the 
 * ADCs (Timer 3) and DACs (Timer 6).
 */
#define FS_500K 168	//!<    500 ksamples/sec
#define FS_400K 210	//!<    400 ksamples/sec
#define FS_200K 420	//!<    200 ksamples/sec
#define FS_100K 840	//!<    100 ksamples/sec
#define FS_96K	875	//!<    96 ksamples/sec
#define FS_50K	1680	//!<    50 ksamples/sec
#define FS_48K	1750	//!<    48 ksamples/sec
#define FS_32K	2625	//!<    32 ksamples/sec
#define FS_25K  3360    //!<    25 ksamples/sec
#define FS_24K	3500	//!<    24 ksamples/sec
#define FS_16K  5250    //!<    16 ksamples/sec
#define FS_12K	7000	//!<    12 ksamples/sec
#define FS_10K	8400	//!<    10 ksamples/sec
#define FS_8K	10500	//!<    8 ksamples/sec
#define FS_6K	14000	//!<    6 ksamples/sec
#define FS_5K	16800	//!<    5 ksamples/sec

/*! @} End of FS_Count_Values group */

/*
 * structs, typedefs, enums .............................................................
 */

/*!
 * @brief Number of input audio channels
 */
enum Num_Channels_In {
  MONO_IN,		//!< Mono Input: Only configure ADC1, single DMA Transfer
  MONO_MIC_IN,		//!< Mic input (MP45DT02): Mono input, no input DMA
  STEREO_IN		//!< Stereo Input: Configure ADC1 and ADC2, dual DMA Transfer
};


/*!
 * @brief Number of output audio channels
 */
enum Num_Channels_Out {
  MONO_OUT,		//!< Mono Output: Only configure DAC1, single DMA Transfer
  STEREO_OUT		//!< Stereo Output: Configure DAC1 and DAC2, dual DMA Transfer
};


/*
 * Global variables.....................................................................
 */

/*!
 * @brief Destination for the delivery of output DAC samples.  
 * 
 * Declared as a global
 * here so that the ISR for the Microphone sample stream can write the output
 * samples to the DAC at the appropriate time
 */
extern uint32_t *DAC_Data_Destination;

/*
 * Function Prototypes.....................................................................
 */

/*!
 * @defgroup Functions Configuration Functions
 * @{
 */

/*!
 * @brief Interrupt Vector Controller Initialization for double-buffer sampling
 * 
 * Enables interrupts generated by DMA2, Stream 4 so that samples being collected by the 
 * ADC(s) may be handed to the user as the input buffer is filled.
 */
void initnvic(void);

/*!
 * @brief Reset and Clock Control (RCC) configuration
 * 
 * Enables clocks to the required peripherals (GPIO ports used, ADCs, DACs, Timers, 
 * DMAs).
 */
void initrcc(
  enum Num_Channels_In chanin   //!< Input channel configuration
);

/*!
 * @brief Configure GPIO ports
 * 
 * Configures and enables digital I/O pins, LEDs, ADC and DAC pins.
 */
void initgpio(void);

/*!
 * @brief Configure DAC Channel 2 for the Mono or Stereo Output cases
 * 
 * DAC Channel 2 is configured to be triggered by Timer 6.  Upon each
 * trigger, a DMA request is generated to obtain the next sample in the 
 * output sequence.  When the data is transfered, the conversion is made
 * to generate the new output voltage.
 */
void initdac(
  uint32_t dac_trigger	//!< Trigger source for the DAC
);


/*!
 * @brief Configure DAC Channel 1 if required to support the Stero Output case
 * 
 * If required for Stereo output, DAC Channel 1 is configured to be 
 * triggered by Timer 6 so that it runs synchronously with DAC channel 2.  
 * No new DMA requests are made by DAC Channel 1, since a single 32-bit DMA 
 * transfer (requested by DAC channel 2) serves both of the DACs.  This function
 * is called after calling initdac().
 * 
 * @see initdac()
 */
void initdac2(
  uint32_t dac_trigger	//!< Trigger source for the DAC
);

/*!
 * @brief Initialize ADC1 for the Mono input case only.
 * 
 * For Mono input mode, ADC1 is configured as the input.  Timer
 * T3 is used to trigger the ADC to perform a conversion.  The converted
 * value is handed to a DMA for transfer into an input buffer.
 * 
 * The ADC clock (Different from the sample rate!) is set to the APB2 rate 
 * divided by 4: (84 MHz)/4 = 21 MHz.  This is needed, since the max ADC clock 
 * frequency is 36 MHz.  Each ADC conversion requires 3 cycles setup + 12 cycles conversion
 * for a total of 15 clock cycles.  
 * ==> Theoretical Maximum sample rate = 21 MHz/15 = 1.4 Msamples/second
 * (The actual sample frequency will be determined by timer 3, which triggers the start
 * of each conversion)
 * 
 * @see initadcstereo()
 */
void initadc(void);

/*!
 * @brief Initialize ADC1 (Master) and ADC2 (Slave) in Dual ADC mode for the Stereo input case.
 * 
 * For Stereo input mode, ADC1 and ADC2 are configured as Master/Slave in Dual ADC mode.  Timer
 * T3 is used to trigger the ADCs to perform a conversion.  The converted
 * values are handed to a DMA for transfer into an input buffer.
 * 
 * The ADC clock (Different from the sample rate!) is set to the APB2 rate 
 * divided by 4: (84 MHz)/4 = 21 MHz.  This is needed, since the max ADC clock 
 * frequency is 36 MHz.  Each ADC conversion requires 3 cycles setup + 12 cycles conversion
 * for a total of 15 clock cycles.  
 * ==> Theoretical Maximum sample rate = 21 MHz/15 = 1.4 Msamples/second
 * (The actual sample frequency will be determined by timer 3, which triggers the start
 * of each conversion)
 * 
 * @see initadc()
 */
void initadcstereo(void);

/*!
 * @brief Configure DMA2, Stream 4 to transfer data from the ADC to a buffer (Mono case only)
 * 
 * DMA2, Stream 4, Channel 0 (which is triggered by ADC1) is configured to transfer a data sample
 * from the ADC data register to an input buffer upon the completion of each conversion.  The input
 * buffer is filled circularly, so that when the buffer becomes full, transfers continue at the
 * beginning of the buffer.
 * 
 * Interrupts are generated when the input buffer become half-full and full, allowing the 
 * data to be processed by user routines.  Normally, the user routines process one half of the 
 * buffer while the DMA continues filling the other half.
 */
void initadcdma(void);

/*!
 * @brief Configure DMA2, Stream 4 to transfer data from the ADCs to a buffer (Stereo in case only)
 * 
 * DMA2, Stream 4, Channel 0 (which is triggered by the master converter: ADC1) is configured 
 * to transfer a data samples from the ADC1/ADC2 common data register
 * to an input buffer upon the completion of each conversion.  The input
 * buffer is filled circularly, so that when the buffer becomes full, transfers continue at the
 * beginning of the buffer.
 * 
 * Interrupts are generated when the input buffer become half-full and full, allowing the 
 * data to be processed by user routines.  Normally, the user routines process one half of the 
 * buffer while the DMA continues filling the other half.
 */
void initadcdmastereo(void);

/*!
 * @brief Initialize DMA1, Stream 6 to transfer data from an output sample buffer to the DAC(s)
 * 
 * DMA1, Stream 6, Channel7 (which supports requests from DAC2) is configured to transfer a 
 * sample from a an output memory buffer to the appropriate DAC data register (depending upon whether
 * the output is mono or stereo). The DAC output buffer is accessed circularly, so that when the 
 * buffer is exhausted, the tranfers continue back at the start of the buffer.
 */
void initdacdma(
  enum Num_Channels_Out chanout	//!< Number of output channels: MONO_OUT or STEREO_OUT
);

/*!
 * @brief Configure Timer 6 to trigger the DAC at the desired sample rate.
 * 
 * Timer 6 is configured to generate a trigger for a DAC conversion at a desired rate.
 * The timer is configured as a counter which generates the DAC trigger every time it 
 * rolls over.
 * 
 * The Timer 6 clock frequency
 * twice the APB1 bus frequency: 2*42MHz = 84 MHz.  
 * The input argument @a daccount should be the desired number of timer clock counts before 
 * each dac output should be generated.  For example, to generate DAC outputs at 48k samples/sec,
 * set @a daccount = (84 MHz)/(48 ksps) = 1750. 
 * 
 * (Appropriate values of @a daccount are provided as #define values: for example, FS_48K is defined
 * as 1750.)
 */
void inittim6(
  uint16_t daccount		//!<  [in] Number of Timer-6 counts required per DAC output sample. 
);


/*!
 * @brief Configure Timer 3 to trigger ADC1 at the desired sample rate.
 * 
 * Timer 3 is configured to generate a trigger for a ADC conversion at a desired rate.
 * The timer is configured as a counter which generates the ADC trigger every time is 
 * rolls over.
 * 
 * The Timer 3 clock is associated with the APB1 bus: the clock frequency is 
 * twice the APB1 bus frequency: 2*42MHz = 84 MHz.  
 * The input argument @a adccount should be the desired number of timer clock counts before 
 * each ADC sample should be collected.  For example, to generate ADC samples at 48k samples/sec,
 * set @a adccount = (84 MHz)/(48 ksps) = 1750. 
 * 
 * (Appropriate values of @a adccount are provided as #define values: for example, FS_48K is defined
 * as 1750.)
 */
void inittim3(
  uint16_t adccount		//!< [in] Number of Timer-3 counts required per ADC sample.
);


/*!
 * @brief Configure microcontroller clock output pins
 * 
 * The MCO ouput pins allow access to the internal clock signals of the microcontroller.
 * These pins are useful to allow some verification that clock frequencies within the 
 * microcontroller are as expected.
 * 
 * MCO1 is connected to port A, pin 8.
 * MCO2 is connected to port C, pin 9.
 */
void cfgmco(void);

/*!
 * @brief Wrapper function to perform all processor initialization for ECE 486
 * 
 * This wrapper function is called by users at the start of ECE 486 routines to
 * perform intialization of I/O pins, ADCs, DACs, Timers, DMAs, and Interrupts.
 * 
 * Input parameters determine the ADC/DAC sample rate, and the number of input
 * and output analog channels.  Sample rates are determined by specifying a 
 * @a timer_count_value, which determines the ultimate sample frequency.  The actual
 * sample frequency is Fs=(84 MHz)/(@a timer_count_value).  
 * Various \#define constants are provided to obtain common sample rates: 
 * FS_100K, FS_96K, FS_50K, FS_48K, FS_24K, FS_12K, FS_8K, FS_6K, FS_5K
 * 
 * For example:
 * @code
 *   initialize( FS_50K, MONO_IN, STEREO_OUT );
 * @endcode
 * is used to configure the STM32-Discovery board for sampling at a 50 ksamples/second
 * input and output rate, with one analog input channel, and two analog output channels.
 * 
 * It is not possible to have different ADC and DAC sample rates using this routine.
 * 
 * A call to initialize() also pauses program execution until the "USER" button is pressed
 * on the STM32F4-Discovery board.  This pause alows the board to be re-programmed without 
 * error.  (Some of the DMA activity and WFI instructions seem to cause st-flash some grief.)
 * 
 */
void initialize(
  uint16_t timer_count_value, 	//!< [in] Number of timer counts required per analog sample processed.
  enum Num_Channels_In chanin,	//!< [in] Number of input channels: MONO_IN or STEREO_IN
  enum Num_Channels_Out chanout	//!< [in] Number of output channels: MONO_OUT or STEREO_OUT
);

/*!
 * @brief  Simple function to return the best guess at the actual sampling frequency.
 *
 *  If the Microphone is used, the I2S clock generation scheme may make the 
 *  actual rate slightly different from the requested rate.  This function 
 *  may be called to obtain the best guess at the actual sample rate being used.
 *  (Assumes perfect accuracy of the external 10 MHz xtal oscillator)
 * 
 * @returns Estimated sampling frequency in samples/second
 */ 
float getsamplingfrequency(void);

/*! @} End of Functions group */

#endif

/*!
 * @} End of ECE486_Init group
 */