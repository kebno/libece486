/*!
 * @file
 * 
 * @brief Flash some LEDs, and wait for the USER button to be pressed
 * @author Don Hummels
 * @date Jan, 2013
 * 
 * These routines allow program execution to be paused until the USER button is 
 * pressed on the STM32F4-Discovery board.  While paused, the Blue and Orange LEDs
 * are flashed.  When the USER button is pressed, the Blue and Orange LEDs are turned 
 * off, the green LED is lit, and the routines return to allow the caller to continue.
 * 
 * As a bonus, the USER button on the STM32F-Discovery board is configured.  Users may
 * periodically monitor the #UserButtonPressed global variable to detect whether a button
 * press has occurred.  Normally, the variable has a value of #Button_Ready.  An interrupt
 * service routine changes the value to #Button_Pressed each time the button is pressed.
 * 
 */

/*!
 * @addtogroup ECE486_Init
 * @{
 *    @defgroup ECE486_User_Button Support for User Pushbutton
 *      @{
 */

#ifndef __BLINKANDWAIT__
#define __BLINKANDWAIT__

/*!
 * @brief User button status
 */
enum Button_Status {
  Button_Ready = SET,			//!< Normal state... no action detected
  Button_Pressed = RESET		//!< Button press has been detected
};

/*!
 * @brief Global variable to indicate status change on every button press
 */
extern enum Button_Status volatile UserButtonPressed;

/*!
 * @brief Blink blue/orange LEDs and wait for the USER button to be pressed
 * 
 * Call blinkandwait() to pause program execution.  While paused, the blue and orange 
 * LEDs on the Discovery board are flashed.  When the USER button is pressed, the blue
 * and orange LEDs are turned off, the green LED is lit, and the function returns.
 */
void blinkandwait(void);

#endif
/*! @} End of ECE486_User_Button group 
 * 
 * @} End of ECE486_Init group
 */