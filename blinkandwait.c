/*!
 * @file
 * 
 * @brief User button configuration and program pause utility
 * 
 * @author Don Hummels
 * @date Jan, 2013
 */

/*
   Flashes the LEDs on the discovery board, and waits until the user
   pushbutton is pressed to return.
*/

#include "stm32f4_discovery.h"
#include "blinkandwait.h"

void Delay(__IO uint32_t nCount);

enum Button_Status volatile UserButtonPressed = Button_Ready;

void blinkandwait(void)
{
  /* Configures User Button */
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
  
  // Blink Blue/Orange until USER button is pushed
  GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 );
  GPIO_SetBits(GPIOD, GPIO_Pin_15);
  while(UserButtonPressed == Button_Ready) {
    GPIO_ToggleBits(GPIOD, GPIO_Pin_13 | GPIO_Pin_15);
    Delay(0xC00000);
  }
  
  // Now Run with only the green LED on
  GPIO_ResetBits(GPIOD, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 );
  GPIO_SetBits(GPIOD, GPIO_Pin_12 );
  UserButtonPressed=Button_Ready;

}

/*!
 * @brief Simple delay function
 */
void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

/*! @addtogroup ECE486_User_Button
 * @{
 */

/*!
 * @brief External Line 0 Interrupt Service Routine (for USER button)
 * 
 * The User pushbutton on the STM32F4-Discovery board generates an external line0 interrupt.
 * This ISR modifies a global variable #UserButtonPressed to allow other routines to easily
 * take action based on button presses.
 * 
 */
void EXTI0_IRQHandler(void)
{
  if(EXTI_GetITStatus(USER_BUTTON_EXTI_LINE)) { 
    UserButtonPressed = Button_Pressed;
    /* Clear the Right Button EXTI line pending bit */
    EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
  }
}

/*! @} End of ECE486_User_Button */