/** @file led.c
*   @version  DCU.GEN2.XYYY.00.001
*   @date   July 2022
*   @author Renshi Li
*   @brief  Handles Handles LED for the DCU project.
*           This file contains the LED interfaces
*   @note:  Address     : 24 Ang Mo Kio Street 65, Singapore 569061
*           Copyright   : ST Engineering Eletronics Pte. Ltd.
*
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include "app_main.h"
#include "gpio.h"
#include "utility.h"
#include "HL_reg_rtp.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "led.h"

/*-----------------------------------------------------------------------------+
|                            Constants / Macros                                |
+-----------------------------------------------------------------------------*/



/*-----------------------------------------------------------------------------+
|                      TypeDef / Structures / Enumerations                     |
+-----------------------------------------------------------------------------*/


uint8_t digits_CommAnode[18]=
{
	0xC0, 0xF9, 0xA4, 0xB0, // '0','1','2,'3',
	0x99, 0x92, 0x82, 0xF8, // '4','5','6,'7',
	0x80, 0x90, 0X88, 0x83, // '8','9','A,'B',
	0xC6, 0xA1, 0x86, 0x8E,  // 'C','D','E,'F',
	0xFF,0b10111111         // ' ', '-'
};

const uint8_t OFF_char = 0xFF;			//no need
const uint8_t DASH_SIGN = 0b01000000;	//no need
#define DIG_OFF_IND     16U             //array 16 in digits_CommAnode[18]
#define DASH_SIGN_IND     17U           //array 17 in digits_CommAnode[18]
#define E_SIGN_IND     14U              //array 14 in digits_CommAnode[18]
uint8_t Segment_CommAnode[4]=
{
    0x08, 0x04, 0x02, 0x01    //Example code uses this way 0x01, 0x02, 0x04, 0x08
};


/*-----------------------------------------------------------------------------+
|                            Extern Declarations                               |
+-----------------------------------------------------------------------------*/



/*-----------------------------------------------------------------------------+
|                             Global Variables                                 |
+-----------------------------------------------------------------------------*/
// all the digit combination in hex format

#if 0
const uint8_t digits_CommAnode[10]=
{
	//0xC0, 0xF9, 0xA4, 0xB0, // '0','1','2,'3',
	//0x99, 0x92, 0x82, 0xF8, // '4','5','6,'7',
	//0x80, 0x90, 0x88, 0x83, // '8','9','A,'B',
	//0xC6, 0xA1, 0x86, 0x8E,  // 'C','D','E,'F',
	//0xFF, 0b10111111  // all off, dash sign only
   0x11,  // 0
   0x9F,  // 1
   0x25,  // 2
   0x02,  // 3
   0x66,  // 4
   0x6D,  // 5
   0x7D,  // 6
   0x07,  // 7
   0x7F,  // 8
   0x6F   // 9
};
const uint8_t OFF_char = 0xFF;
const uint8_t DASH_SIGN = 0b01000000;

// LED tube segment tube position index table
uint8_t Segment_CommAnode[LED_TUB_LEN]=
{
 //0x80,0x40,0x20,0x10,
 0x08,0x04,0x02,0x01
 //0x01,0x02,0x04,0x08
};

#endif

/*-----------------------------------------------------------------------------+
|                        Static Function Prototypes                            |
+-----------------------------------------------------------------------------*/



/*-----------------------------------------------------------------------------+
|                        Static Function Declaration                           |
+-----------------------------------------------------------------------------*/



/*-----------------------------------------------------------------------------+
|                        Global Function Declaration                           |
+-----------------------------------------------------------------------------*/





/** @fn vLedTube_Disp1Digit(void)
*
*   @brief   This function is used to display a single digit in the
*    	     7 segment LED tube
*
*
*   @return  The function has no return value
*
*   This function can be called anywhere to display the single digit from 0 -9
*
*/
void vLedTube_Disp1Digit(uint8_t Digit)
{
    uint8_t i = 0;
    // single digit segment display
    for (i = 0; i < 8; i++)        //Example code uses this, i = 8; i >= 1; i--, on HDK no difference
    {
    	 
    	if((0x80 & Digit))
		{
			// set the data pin for bit 1
			gioSetBit(LED_PORT, DATA_PIN, PIN_HIGH);
		}
		else
		{
			// set the data pin for bit - 0
			gioSetBit(LED_PORT, DATA_PIN, PIN_LOW);
		}

		vDelay_ticks(5U);

        Digit <<= 1U;
        
        // set the clock
    	gioSetBit(LED_PORT, CLOCK_PIN, PIN_LOW);
		// pull clock pin high to save the digit
		gioSetBit(LED_PORT, CLOCK_PIN, PIN_HIGH);

		

		// put a small delay to let the registry save the digit
		vDelay_ticks(60U);
    }
}

void vLedTube_Update4Digits(uint32_t u32Num)
{
	uint8_t au8String[4];
	uint8_t au8Digit;
	uint8_t index = 0;
	bool firstzero = true;
	if(u32Num > 9999UL)
	{
		u32Num = 9999UL; // max number can be displayed
	}
	// print out the display string in decimal format

	sprintf((char *)au8String,"%04u",u32Num);

	for(index = 0; index < 4U; index++)
	{

		au8Digit = (uint8_t)(au8String[index] - '0');
		// make sure the front '0' does not show
		if(firstzero)
		{
			if(0U == au8Digit)
			{
				au8Digit = DIG_OFF_IND;
			}
			else
			{
				firstzero = false;
			}
		}

		// prepare the latch signal
		gioSetBit(LED_PORT, LATCH_PIN, PIN_LOW);
		// send out the digit
		vLedTube_Disp1Digit(digits_CommAnode[au8Digit]);
		//vLedTube_Disp1Digit(Dig_ComCathode[au8Digit]);
		// send out the segment position
		vLedTube_Disp1Digit(Segment_CommAnode[index]);
		//vLedTube_Disp1Digit(Tube_Index[index]);
		// pull high the latch to save the digit information
		gioSetBit(LED_PORT, LATCH_PIN, PIN_HIGH);
        
		// set a little delay
		vDelay_ticks(100);
	}
	firstzero = true;
}

/** @fn vLedTube_Update4Digits(void)
*
*   @brief   This function is used to display a 4-digit in the
*    	     7 segment LED tube array
*
*
*   @return  The function has no return value
*
*   This function can be called anywhere to display the 4 digits number
*   ranging from 0000 - 9999
*
*/



/*** Archive and disabled the single Tube LED function codes since Gen2 DCU will use 4-tube LED ***/

#if 0
/** @fn vLED_Off(void)
*
*   @brief   This function is used to turn off the LED2 and LED3.
*
*
*   @return  The function has no return value
*
*   This function can be called anywhere to turn off the 2 LEDs
*
*/
void vLED_Off(void)
{
    /* Turn off the LED by setting pin as output Low */
    vGpioSetBit(LED2_A,enLOW);
    vGpioSetBit(LED3_A,enLOW);
}


/** @fn vLED_On(void)
*
*   @brief   This function is used to turn on the LED2 and LED3.
*
*
*   @return  The function has no return value
*
*   This function can be called anywhere to turn on the 2 LEDs
*
*/
void vLED_On(void)
{
    /* Turn off the LED by setting pin as output High */
    vGpioSetBit(LED2_A,enHIGH);
    vGpioSetBit(LED3_A,enHIGH);
}

void vLED_Toggle()
{
    //gioSetBit(LED2_GPIOPORT,LED2_GPIOPIN,gioGetBit(LED2_GPIOPORT,LED2_GPIOPIN) ^ 0x00000001);
}

/** @fn vLEDSegmentDisplay(tenLedChar enChar)
*
*   @brief      This function is used to Show Specific Char or Digits on the Segment LED
*   @param[in]  enChar - specific LED char
*               - Digit '0' to '9'
*               - Char 'A','C','E','F'
*               - Decimal Point
*
*   @return  The function has no return value
*
*   This function can be called anywhere to show specific digit or char on segment LED
*
*/

void vLEDSegmentDisplay(tenLedChar enChar)
{
    switch(enChar)
    {
        case enZero: // display digit - '0'
            vGpioSetBit(LED_SEGMENT_A_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_B_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_C_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_D_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_E_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_F_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_G_PIN,  enLOW);
            break;

        case enOne: // display digit - '1'
            vGpioSetBit(LED_SEGMENT_A_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_B_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_C_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_D_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_E_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_F_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_G_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_DP_PIN, enLOW);
            break;

        case enTwo: // display digit - '2'
            vGpioSetBit(LED_SEGMENT_A_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_B_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_C_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_D_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_E_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_F_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_G_PIN, enHIGH);
            break;

        case enThree: // display digit - '3'
            vGpioSetBit(LED_SEGMENT_A_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_B_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_C_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_D_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_E_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_F_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_G_PIN, enHIGH);
            break;

        case enFour: // display digit - '4'
            vGpioSetBit(LED_SEGMENT_A_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_B_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_C_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_D_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_E_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_F_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_G_PIN, enHIGH);
            break;

        case enFive: // display digit - '5'
            vGpioSetBit(LED_SEGMENT_A_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_B_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_C_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_D_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_E_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_F_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_G_PIN, enHIGH);
            break;

        case enSix: // display digit - '6'
            vGpioSetBit(LED_SEGMENT_A_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_B_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_C_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_D_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_E_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_F_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_G_PIN, enHIGH);
            break;

        case enSeven: // display digit - '7'
            vGpioSetBit(LED_SEGMENT_A_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_B_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_C_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_D_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_E_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_F_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_G_PIN,  enLOW);
            break;

        case enEight: // display digit - '8'
            vGpioSetBit(LED_SEGMENT_A_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_B_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_C_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_D_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_E_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_F_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_G_PIN, enHIGH);
            break;

        case enNine: // display digit - '9'
            vGpioSetBit(LED_SEGMENT_A_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_B_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_C_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_D_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_E_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_F_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_G_PIN, enHIGH);
            break;

        case enChar_A: // display Char - 'A'
            vGpioSetBit(LED_SEGMENT_A_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_B_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_C_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_D_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_E_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_F_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_G_PIN, enHIGH);
            break;

        case enChar_C: // display Char - 'C'
            vGpioSetBit(LED_SEGMENT_A_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_B_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_C_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_D_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_E_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_F_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_G_PIN,  enLOW);
            break;

        case enChar_E: // display Char - 'E'
            vGpioSetBit(LED_SEGMENT_A_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_B_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_C_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_D_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_E_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_F_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_G_PIN, enHIGH);
            break;

        case enChar_F: // display Char - 'F'
            vGpioSetBit(LED_SEGMENT_A_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_B_PIN, enLOW);
            vGpioSetBit(LED_SEGMENT_C_PIN, enLOW);
            vGpioSetBit(LED_SEGMENT_D_PIN, enLOW);
            vGpioSetBit(LED_SEGMENT_E_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_F_PIN, enHIGH);
            vGpioSetBit(LED_SEGMENT_G_PIN, enHIGH);
            break;

        case enDP_ON: // display Decimal Point
            vGpioSetBit(LED_SEGMENT_DP_PIN, enHIGH);
            break;

        case enDP_OFF: // Decimal Point Display Off
            vGpioSetBit(LED_SEGMENT_DP_PIN, enLOW);
            break;

        case enDisplayOFF: // Turn Off whole display
            vGpioSetBit(LED_SEGMENT_A_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_B_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_C_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_D_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_E_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_F_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_G_PIN,  enLOW);
            vGpioSetBit(LED_SEGMENT_DP_PIN, enLOW);
            break;

        default:
            // should not come here
            break;
    }
}
#endif
