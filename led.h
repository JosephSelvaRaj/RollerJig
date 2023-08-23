/** @file gpio.c
*   @version  DCU.GEN2.XYYY.00.001
*   @date   July 2022
*   @author Renshi Li
*   @brief  Handles Handles LED for the DCU project.
*           This file contains the LED interfaces
*   @note:  Address     : 24 Ang Mo Kio Street 65, Singapore 569061
*           Copyright   : ST Engineering Eletronics Pte. Ltd.
*
*/

#ifndef  LED_H_
#define  LED_H_


/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/
#include "app_main.h"
#include <HL_het.h>

/*
***************************************************************************
*                  TypeDef / Structures / Enumerations
***************************************************************************
*/
/** @enum tenModeKeyStatus
*   @brief status for Mode Select Key
*   This enumeration defines the status of the mode select key or segregate key.
*
*/
typedef enum tenLED_RGBColor
{
   RGB_IDLE   			= 0,
   RGB_LED_OFF    	= 1,     /* All LEDs are off   */
   RGB_COLOR_RED    	= 2,     /* All LED are in Red color          */
   RGB_COLOR_GREEN  	= 3,     /* All LED are in Red color     */
   RGB_COLOR_BLUE   	= 4,     /* All LED are in blue color   */
   RGB_COLOR_AMBER  	= 5,     /* All LED are in Amber color   */
   RGB_COLOR_PURPLE 	= 6,     /* All LED are in purple color */
   RGB_COLOR_WHITE  	= 7,     /* All LED are in white color */
   RGB_COLOR_TOT_NUM         	/* Number of LED Colours */
}
tenLED_RGBColor;

/** @enum tenModeKeyStatus
*   @brief status for Mode Select Key
*   This enumeration defines the status of the mode select key or segregate key.
*
*/
typedef enum tenLED_Chn
{
   RGB_CHN_NONE     = 0,
   RGB_CHN_DOI      = 1,   /* DOI Channel   */
   RGB_CHN_DOS      = 2,   /* DOS Channel   */
   RGB_CHN_TOTNUM
}
tenLED_Chn;

/** @enum tenModeKeyStatus
*   @brief status for Mode Select Key
*   This enumeration defines the status of the mode select key or segregate key.
*
*/
typedef enum tenLEDStatus
{
	LED_INIT   				= 0,
	LED_OFF    				= 1,
	LED_ALWAYS_ON   		= 2,
	LED_BLINK_NORMAL   		= 3,
	LED_BLINK_EXCEPTION   	= 4,
	LED_BLINK_ERROR   		= 5,
	LED_BLINK_BOOT   		= 6,
	LED_BLINK_CALI   		= 7,
	LED_TOTNUM_STATE
}
tenLEDStatus;
/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/
#define LED74HC595   // define the model of 7-segment LED

#define LED_TUB_LEN			8U

#ifdef LED74HC595

#define CLOCK_PIN    		8U//8U   //HET1_8 DISPLAY_SR_CLK_MCU, Gen2 MCU Pinout, E18, SR for serial
#define LATCH_PIN    		26U //HET1_26 DISPLAY_R_CLK_MCU,  Gen2 MCU Pinout, A14, for register
#define DATA_PIN	 		10U // DISPLAY_DIO_MCU,    Gen2 MCU Pinout,D19
#define LED_PORT	 		hetPORT1
#define DIGIT_NUM	 		8U
#define DIG_OFF_IND     	16U
#define DASH_SIGN_IND     	17U
#define E_SIGN_IND     		14U

#endif
/*
*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

/**
 *  @defgroup LED LED
 *  @brief LED Module.
 *
 *  The MCU controls the LED On and Off
 *
 *  Related Files
 *   - gpio.h
 *   - led.h
 *   - led.c
 *   - het.h
 *  @addtogroup GPIO
 *  @{
 */
void vLED_On(void);
void vLED_Off(void);
void vLEDSegmentDisplay(tenLedChar enChar);
void vLedTube_Disp1Digit(uint8_t Digit);
void vLedTube_Update4Digits(uint32_t u32Num);

/**@}*/
/*
*********************************************************************************************************
*                                          CONFIGURATION ERRORS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                              MODULE END
*********************************************************************************************************
*/

#endif/* End of module include. */
