/** @file HL_sys_main.c
 *   @brief Application main file
 *   Created on: 21 June, 2022
 *   Author: Lukman
 *
 */

#include "HL_het.h"
#include "HL_adc.h"
#include "HL_rti.h"
#include "math.h"
#include "stdlib.h"
#include "HL_gio.h"
#include "HL_sys_core.h"
#include "rotary.h"
#include "can_bus.h"
#include "fee.h"
#include "ti_fee_types.h"
#include "ti_fee.h"
#include "HL_sys_common.h"
#include <sl_api.h>
#include <sl_priv.h>

#define COUNTER_EEPROM_ADD 0x01

volatile uint32_t u32SystemTimer_1ms = 0;
uint32_t nowtime_ms;
uint32_t u32move_forward_waiting_ms = 0;
uint32_t u32move_backward_waiting_ms = 0;
volatile uint32_t u32TestCounter_new;
boolean bl_tick_move_forward_time = false;
boolean bl_tick_move_backward_time = false;

uint32_t u32temp_ee_test;
uint32_t u32SpeedTimer_ms = 0;
uint32_t u32ResetTimer_ms = 0;
uint32_t u32eeprom_timer_1ms = 0;
// LED Tube Definitions
#define CLOCK_PIN 0U
#define LATCH_PIN 1U
#define DATA_PIN 2U
#define LED_PORT gioPORTB
#define DIGIT_NUM 8U
#define PIN_LOW 0U
#define PIN_HIGH 1U
#define DELAY_1MS_TICKS 27273	  // The For loop takes 11 MC, hence 300M/11 = 27272727 MC for 1 sec
#define DELAY_100MS_TICKS 2727273 // The For loop takes 11 MC, hence 300M/11 = 27272727 MC for 1 sec
#define DELAY_1US_TICKS 27		  // The For loop takes 11 MC, hence 300M/11 = 27272727 MC for 1 sec
uint8 u8PortB_dir = 0b00111111;	  // set port B bit 0 to bit 5 to output signal

#define PULSE_PER_ROUND 6U

// define start switch
#define SWITCH_PIN 0U // start switch
#define SWITCH_PORT gioPORTA
#define SWITCH_DEBOUNCE_COUNTER 30U
uint16_t switch_on_cntr = 0;
uint16_t switch_off_cntr = 0;
boolean flag_switch_on = false;

#define CLOCK2_PIN 3U
#define LATCH2_PIN 4U
#define DATA2_PIN 5U

// Define Variable//
#define BATT_LIM 1.10 // 1.20 //Minimum Battery condition fro EEPROM to save data//

// PULSE CONTROL

// #define MAX_MOVE_PULSE_NUM              2100U // equivalent to 9 rounds
#define RAMP_UP_PULSE_END 300U
#define CONST_SPEED_PULSE_END (RAMP_UP_PULSE_END + 1550U)
#define RAMP_DOWN_PULSE_END (CONST_SPEED_PULSE_END + 300U)
#define MAX_MOVE_PULSE_NUM RAMP_DOWN_PULSE_END // equivalent to 9 rounds
#define PULSE_MOVE_TIMEOUT 30000U			   // 15s timeout
#define MOTOR_COOLING_TIME_MS 8000U			   // 3s

#define MAX_MOVE_DURATION_MS 3500U		 // 3.5s
#define RAMP_UP_DURATION_MS 300U		 // 0.5s
#define RAMP_DOWN_DURATION_MS 400U		 // 0.5s
#define MOTOR_STOP_DURATION_MS 300U		 // 0.5s
#define CONST_SPEED_DURATION_MS 2500U	 // 2s --- 1Nov change from 2s to 3s
#define MOTOR_TEST_STOP_WAITING_MS 3000U // original 5s is quite cool --- 31Oct Modify
#define WHOLE_WAITING_TIME_MS (MOTOR_TEST_STOP_WAITING_MS + RAMP_UP_DURATION_MS + RAMP_DOWN_DURATION_MS + MOTOR_STOP_DURATION_MS + CONST_SPEED_DURATION_MS)

// #define MAX_MOTOR_DUTY_CYCLE        	75U
#define MOTOR_DUTYCYCLE_RAMP_MIN 60U	// 88U//72//55U//60U --- Change from 58U to 60U to remove clogging sound 31 Oct
#define MOTOR_DUTYCYCLE_RAMP_MAX 62U	// 88U//82//75U
#define MOTOR_DUTYCYCLE_CONST_SPEED 70U // 78U//82//75U
#define MOTOR_DUTYCYCLE_STOP_MAX 62U	// 78U//60U
#define MOTOR_DUTYCYCLE_STOP_MIN 0U		// 20U

#define MOTOR_PULSE_PER_ROUND 6U

#define MAX_MOTOR_PULSE_COUNT 400U // actual max 940-980
#define MIN_MOTOR_PULSE_COUNT 250U // actual 25 - 60

#define MIN_CONSTSPEED_MOTOR_SPEED_RPM 200U // min value

#define MAX_MOTOR_DUTY_CYCLE 55U
#define TEST__DELAY_TICKS 100000UL // 10000UL
adcData_t adc_data[2];			   // ADC Data Structure
#define dir_forward 1U
#define dir_backward 2U

// adcData_t *adc_data_ptr = &adc_data[2]; //ADC Data Pointer
// int NumberOfChars1, NumberOfChars2; //Declare variables

char Rotary[8];
char Temperature[8];
char Light[8];
char varb[8];
char can_tx[8];
char can_rx[8];
char Pin_pwm[8];
// char value1, value2;
int Temp, Lamp, Duty1, Duty2;
double Batt;
uint8_t txtCRLF[] = {'\r', '\n'};
uint8_t Lmp[] = {"LIGHT_SENSOR_ADC:"};
uint8_t Rot[] = {"Encoder:"};
uint8_t Varbl[] = {"Direction: Forward"};
uint8_t Varbl2[] = {"Direction: Backward"};
uint8_t Varbl3[] = {"Motor Error"};
uint8_t Mot[] = {"Motor_Pulse:"};
uint8_t Tmp[] = {"TEMP_SENSOR_ADC:"};
uint8_t can_txr[] = {"CAN Transmit :"};
uint8_t can_rxr[] = {"CAN Receive :"};
uint8_t Td[] = {"Test Cycle:"};
// Function to check error in received data can bus//
uint32_t checkPackets(uint8_t *src_packet, uint8_t *dst_packet, uint32_t psize);

// encoder//
uint32_t uRotaryLastVal;
uint32_t uRotary;
uint32_t pwm_count;
uint32_t d;
int32_t pos;
uint8_t adc_count;

// EEPROM//
uint16 Status;
uint8_t SpecialRamBlock[100];
uint8_t loop;

// motor speed
uint32_t au32Speed_array[10];
uint32_t u32Speed_rpm = 0;
uint32_t u32SpeedSum = 0;
uint32_t u32SpeedAve = 0;
uint32_t u32Timer_100ms = 0;
uint32_t u32ReadBack01 = 0;
uint32_t u32ReadBack02 = 0;
boolean blflag_speed_check = false;
uint32_t u32ErrorHappen_cntr = 0;
boolean save_eeprom = false;
boolean blflag_reset = false;
boolean blfag_stop_reset = false;
uint32_t u32Retry_cntr = 0;
uint8_t au8DispStr[51]; //"Motor Pulse Count = %5u, Test Count = %9u\r\n"

//**********************************MOTOR TWO VARIABLES**********************************//
#define COUNTERTWO_EEPROM_ADD 0x02

// Motor Two Pins
#define MOTORTWOENCODERPIN 5U
#define MOTORTWODIRPIN 7U
#define MOTORTWOPWMPIN 19U
#define FORWARD 0U
#define BACKWARD 1U


volatile uint32_t u32TestCounterTwo_new;
boolean firstResetTwo = true;
boolean flag_motor2_forward = true;
boolean	flag_motor2_stop = false;
boolean flag_motor2_backward = false;
boolean flag_motor2_pause = false;
boolean flag_motor2_error = false;

//**********************************MOTOR TWO VARIABLES END **********************************//
void Motor_status_Disp(uint32_t var01, uint32_t var02)
{
	sprintf(au8DispStr, "Motor Pulse Count = %5u, Test Count = %9u\r\n", var01, var02);
	sciDisplayText(sciREG1, au8DispStr, 51U); // display text//
}
/////**************************Adc Application*******************/////
void adc_convert(void)
{

	adcStartConversion(adcREG1, adcGROUP1); // Start ADC conversion
	while (!adcIsConversionComplete(adcREG1, adcGROUP1))
		;													  // Wait for ADC conversion
	adc_count = adcGetData(adcREG1, adcGROUP1, &adc_data[0]); // Store conversion into ADC pointer

	if (adc_count >= 2) // wait until all the conversion 2 adc channel finish//
	{
		// id    = adc_data[0].id;      //Get Id for first conversion pin 8//
		Temp = adc_data[0].value; // Get value for first conversion pin 8//
		// id    = adc_data[1].id;      //Get Id for first conversion pin 9//
		Lamp = adc_data[1].value; // Get value for first conversion pin 9//
		// value1 = (int)adc_data_ptr->value;//store adc conversion value for 1 adc channel only//

		// Duty1 =(int)(Lamp*0.038);//change duty cycle from 0~100%)//
		// Duty1 = MAX_MOTOR_DUTY_CYCLE;
		// Duty2 =(int)(Temp*0.019);//change duty cycle from 0~100%)//
		Batt = (double)(Temp * 0.00123); // get the battery value//
	}
}
/////*********************End of Adc Application*******************/////

/////*********************Encoder Application*******************/////
void CheckRotary()
{

	uRotary = getRotaryPosition(); // get rotary encoder position//
	if (uRotary != uRotaryLastVal)
	{
		uRotaryLastVal = uRotary;
		ltoa(uRotary, (char *)Rotary, 10); // convert to decimal
	}
}

/////*********************End Of Encoder Application*******************/////

/////**************************SCI Display***************************///////
// Display Can transmit//
void can_tx_disp(uint32_t var)
{
	ltoa(var, (char *)can_tx, 10);
	sciDisplayText(sciREG1, can_txr, sizeof(can_txr)); // display text//
	sciDisplayText(sciREG1, can_tx, sizeof(can_tx));   // display text//
	sciDisplayText(sciREG1, txtCRLF, sizeof(txtCRLF)); // display text//
}

// Display Can transmit//
void can_rx_disp(uint32_t var)
{

	ltoa(var, (char *)can_rx, 10);
	sciDisplayText(sciREG1, can_rxr, sizeof(can_rxr)); // display text//
	sciDisplayText(sciREG1, can_rx, sizeof(can_rx));   // display text//
	sciDisplayText(sciREG1, txtCRLF, sizeof(txtCRLF)); // display text//
}

// Display Light Sensor Reading//
void Light_Sens_Disp()
{

	ltoa(Lamp, (char *)Light, 10);					   // convert to decimal
	sciDisplayText(sciREG1, Lmp, sizeof(Lmp));		   // display text//
	sciDisplayText(sciREG1, Light, sizeof(Light));	   // display text//
	sciDisplayText(sciREG1, txtCRLF, sizeof(txtCRLF)); // display text//

	/*
	//not used//
	sciSend(sciREG1, 21, (unsigned char  *)"STEng"); //Sends '0x' hex designation chars
	sciSend(sciREG1, NumberOfChars, command); //Sends the ambient light sensor data
	sciSend(sciREG1, 8, (unsigned char *)'\r\n'); //Sends new line character
	*/
}

// Display Temperatur Sensor Reading//
void Temp_Sens_Disp()
{

	ltoa(Temp, (char *)Temperature, 10); // convert to decimal
	sciDisplayText(sciREG1, Tmp, sizeof(Tmp));
	sciDisplayText(sciREG1, Temperature, sizeof(Temperature));
	sciDisplayText(sciREG1, txtCRLF, sizeof(txtCRLF));
}

// Display Encoder counter//
void Encod_Disp()
{

	sciDisplayText(sciREG1, Rot, sizeof(Rot));		   // display text//
	sciDisplayText(sciREG1, Rotary, sizeof(Rotary));   // display text//
	sciDisplayText(sciREG1, txtCRLF, sizeof(txtCRLF)); // display text//
}

// Display Variable//
void Pin_Disp(uint32_t var)
{
	// ltoa(var,(char *)Var, 10); //convert to decimal
	// sciDisplayText(sciREG1, Var, sizeof(Var)); //display text//
	if (var == 0)
	{
		sciDisplayText(sciREG1, Varbl2, sizeof(Varbl2));   // display text backward//
		sciDisplayText(sciREG1, txtCRLF, sizeof(txtCRLF)); // display text//
	}
	else if (var == 1)
	{
		sciDisplayText(sciREG1, Varbl, sizeof(Varbl));	   // display text forward//
		sciDisplayText(sciREG1, txtCRLF, sizeof(txtCRLF)); // display text//
	}

	else if (var == 2)
	{
		sciDisplayText(sciREG1, Varbl3, sizeof(Varbl3));   // display text Motor Error//
		sciDisplayText(sciREG1, txtCRLF, sizeof(txtCRLF)); // display text//
	}
	delay(1000);
}

// Display PWM//
void Mot_Disp(uint32_t var)
{

	ltoa(var, (char *)Pin_pwm, 10);					   // convert to decimal
	sciDisplayText(sciREG1, Mot, sizeof(Mot));		   // display text//
	sciDisplayText(sciREG1, Pin_pwm, sizeof(Pin_pwm)); // display text//
	sciDisplayText(sciREG1, txtCRLF, sizeof(txtCRLF)); // display text//
													   // delay(1000);
}

void Var_Disp(uint32_t var)
{
	ltoa(var, (char *)varb, 10);
	sciDisplayText(sciREG1, Td, sizeof(Td));		   // display text//
	sciDisplayText(sciREG1, varb, sizeof(varb));	   // display text//
	sciDisplayText(sciREG1, txtCRLF, sizeof(txtCRLF)); // display text//
}
/////**********************End Of SCI Display***************************///////

// Create an array that turns all segments ON
uint8_t allON[] = {0xff, 0xff, 0xff, 0xff};
const uint8_t TMS1637_Digits[] = {
	// XGFEDCBA
	0b00111111, // 0
	0b00000110, // 1
	0b01011011, // 2
	0b01001111, // 3
	0b01100110, // 4
	0b01101101, // 5
	0b01111101, // 6
	0b00000111, // 7
	0b01111111, // 8
	0b01101111, // 9
	0b01110111, // A
	0b01111100, // b
	0b00111001, // C
	0b01011110, // d
	0b01111001, // E
	0b01110001	// F
};
#define TM1637_I2C_COMM1 0x40 // automatic data incrementing
#define TM1637_I2C_COMM2 0xC0 // Data Data1~N: Transfer display data
#define TM1637_I2C_COMM3 0x80 // display intensity
uint8_t u8TM1637_databuff[4] = {1, 2, 3, 4};
/////**********************LED Tube Display ***************************///////
/* Segment bit location(7=MSB, 0=LSB):
 *
 *    |--0--|
 *   5|     |1
 *    |--6--|
 *   4|     |2
 *    |--3--| **7
 */

#if 0
uint8_t digits_CommCathode[16]=
{
		  0x3f,0x60,0x5b,0x4f, // 0,1,2,3
		  0x66,0x6d,0x7d,0x07, // 4,5,6,7
		  0x7f,0x67,0x77,0x7c, // 8,9,A,B
		  0x39,0x5e,0x79,0x71  // C,D,E,F
};
uint8_t Segment_CommCathode[8]=
{
 0xfe,0xfd,0xfb,0xf7,
 0xef,0xdf,0xbf,0x7f
};
#endif

uint8_t digits_CommAnode[18] =
	{
		0xC0, 0xF9, 0xA4, 0xB0, // '0','1','2,'3',
		0x99, 0x92, 0x82, 0xF8, // '4','5','6,'7',
		0x80, 0x90, 0X88, 0x83, // '8','9','A,'B',
		0xC6, 0xA1, 0x86, 0x8E, // 'C','D','E,'F',
		0xFF, 0b10111111};
const uint8_t OFF_char = 0xFF;
const uint8_t DASH_SIGN = 0b01000000;
#define DIG_OFF_IND 16U
#define DASH_SIGN_IND 17U
#define E_SIGN_IND 14U
uint8_t Segment_CommAnode[8] =
	{
		0x80, 0x40, 0x20, 0x10,
		0x08, 0x04, 0x02, 0x01};

uint8_t displayTubeBuff[8] = {9, 10, 11, 12, 13, 14, 15, 0};
uint8_t au8DisplayDigit[8] = {0};
void vDelay_ticks(uint32_t u32Ticks)
{

	// Sanity Check
	if (u32Ticks < 0x01)
	{
		return;
	}

	// while loop for simple delay
	do
	{
		u32Ticks--;
	} while (u32Ticks > 0);
}

void DisplayDigit_LSB(uint8_t Digit)
{
	uint8_t i = 0;
	for (i = 0; i < 8; i++)
	{
		gioSetBit(LED_PORT, CLOCK_PIN, PIN_HIGH);
		// vDelay_ticks(DELAY_1MS_TICKS*10);
		//__asm("nop");

		if ((0x01 & Digit))
		{
			gioSetBit(LED_PORT, DATA_PIN, PIN_HIGH);
		}
		else
		{
			gioSetBit(LED_PORT, DATA_PIN, PIN_LOW);
		}
		// vDelay_ticks(DELAY_1MS_TICKS);
		vDelay_ticks(5U);
		gioSetBit(LED_PORT, CLOCK_PIN, PIN_LOW);
		Digit >>= 1;
		// vDelay_ticks(DELAY_1MS_TICKS*10);
		vDelay_ticks(600U);
	}
}
void DisplayDigit(uint8_t Digit)
{
	uint8_t i = 0;
	// gioSetBit(LED_PORT, LATCH_PIN, PIN_LOW);
	for (i = 0; i < 8; i++)
	{
		gioSetBit(LED_PORT, CLOCK_PIN, PIN_LOW);
		// vDelay_ticks(DELAY_1MS_TICKS*10);
		//__asm("nop");

		if ((0x80 & Digit))
		{
			gioSetBit(LED_PORT, DATA_PIN, PIN_HIGH);
		}
		else
		{
			gioSetBit(LED_PORT, DATA_PIN, PIN_LOW);
		}
		// vDelay_ticks(DELAY_1MS_TICKS);
		vDelay_ticks(5U);
		gioSetBit(LED_PORT, CLOCK_PIN, PIN_HIGH);
		Digit <<= 1;
		// vDelay_ticks(DELAY_1MS_TICKS*10);
		vDelay_ticks(60U);
	}
}

void DisplayDigit_02(uint8_t Digit)
{
	uint8_t i = 0;
	// gioSetBit(LED_PORT, LATCH_PIN, PIN_LOW);
	for (i = 0; i < 8; i++)
	{
		gioSetBit(LED_PORT, CLOCK2_PIN, PIN_LOW);
		// vDelay_ticks(DELAY_1MS_TICKS*10);
		//__asm("nop");

		if ((0x80 & Digit))
		{
			gioSetBit(LED_PORT, DATA2_PIN, PIN_HIGH);
		}
		else
		{
			gioSetBit(LED_PORT, DATA2_PIN, PIN_LOW);
		}
		// vDelay_ticks(DELAY_1MS_TICKS);
		vDelay_ticks(5U);
		gioSetBit(LED_PORT, CLOCK2_PIN, PIN_HIGH);
		Digit <<= 1;
		// vDelay_ticks(DELAY_1MS_TICKS*10);
		vDelay_ticks(60U);
	}
}
void Display4Segment(void)
{
	// DisplayDigit(TM1637_I2C_COMM1);
	DisplayDigit_LSB(TM1637_I2C_COMM2 + (0U & 0x03));
	DisplayDigit_LSB(TMS1637_Digits[0]);
	vDelay_ticks(1000);
	DisplayDigit_LSB(TM1637_I2C_COMM2 + (1U & 0x03));
	DisplayDigit_LSB(TMS1637_Digits[2]);
	vDelay_ticks(1000);
	DisplayDigit_LSB(TM1637_I2C_COMM2 + (2U & 0x03));
	DisplayDigit_LSB(TMS1637_Digits[4]);
	vDelay_ticks(1000);
	DisplayDigit_LSB(TM1637_I2C_COMM2 + (3U & 0x03));
	DisplayDigit_LSB(TMS1637_Digits[6]);
}
void Display8Segment(uint8_t *au8DataBuff)
{

	// vDelay_ticks(200U);
	uint8_t j = 0;

	for (j = 0; j < 8; j++)
	{
		gioSetBit(LED_PORT, LATCH_PIN, PIN_LOW);
		DisplayDigit(digits_CommAnode[displayTubeBuff[j]]);
		DisplayDigit(Segment_CommAnode[j]);
		// vDelay_ticks(10U);
		gioSetBit(LED_PORT, LATCH_PIN, PIN_HIGH);
		vDelay_ticks(100);
	}
}

void vUpdateDisplay8Digit(uint32_t u32Num)
{
	uint8_t au8String[8];
	uint8_t au8Digit;
	uint8_t index = 0;
	bool firstzero = true;
	if (u32Num > 99999999UL)
	{
		u32Num = 99999999UL; // max number can be displayed
	}
	sprintf(au8String, "%08lu", u32Num);
	// gioSetBit(LED_PORT, LATCH_PIN, PIN_LOW);
	for (index = 0; index < 8; index++)
	{

		au8Digit = (uint8_t)(au8String[index] - '0');
#if 1
		if (firstzero)
		{
			if (0U == au8Digit)
			{
				au8Digit = DIG_OFF_IND;
			}
			else
			{
				firstzero = false;
			}
		}
#endif
		gioSetBit(LED_PORT, LATCH_PIN, PIN_LOW);
		DisplayDigit(digits_CommAnode[au8Digit]);
		DisplayDigit(Segment_CommAnode[index]);
		gioSetBit(LED_PORT, LATCH_PIN, PIN_HIGH);
		// vDelay_ticks(3000U);
		vDelay_ticks(100);
	}
	firstzero = true;
}
uint8 au8SpeedData[8] = {DASH_SIGN_IND, DASH_SIGN_IND, 0, 0, 0, 0, DASH_SIGN_IND, DASH_SIGN_IND};
uint8 au8ErrorInfo[8] = {DASH_SIGN_IND, DASH_SIGN_IND, E_SIGN_IND, E_SIGN_IND, E_SIGN_IND, E_SIGN_IND, DASH_SIGN_IND, DASH_SIGN_IND};
void vUpdateDisplay8Digit_02(uint32_t u32Num)
{
	uint8_t au8String[4];
	uint8_t au8Digit;
	uint8_t index = 0;
	bool firstzero = true;
	if (u32Num > 99999999UL)
	{
		u32Num = 99999999UL; // max number can be displayed
	}
	sprintf(au8String, "%04u", u32Num);
	// gioSetBit(LED_PORT, LATCH_PIN, PIN_LOW);
	for (index = 0; index < 4; index++)
	{

		au8SpeedData[2 + index] = (uint8_t)(au8String[index] - '0');
#if 1
		if (firstzero)
		{
			if (0U == au8Digit)
			{
				au8Digit = DIG_OFF_IND;
			}
			else
			{
				firstzero = false;
			}
		}
#endif
	} // complete the data array

	for (index = 0; index < 8; index++)
	{
		gioSetBit(LED_PORT, LATCH2_PIN, PIN_LOW);
		DisplayDigit_02(digits_CommAnode[au8SpeedData[index]]);
		DisplayDigit_02(Segment_CommAnode[index]);
		gioSetBit(LED_PORT, LATCH2_PIN, PIN_HIGH);
		// vDelay_ticks(3000U);
		vDelay_ticks(100);
	}
	firstzero = true;
}

void vUpdateDisplayError_02(void)
{
	uint8_t index = 0;

	for (index = 0; index < 8; index++)
	{
		gioSetBit(LED_PORT, LATCH2_PIN, PIN_LOW);
		DisplayDigit_02(digits_CommAnode[au8ErrorInfo[index]]);
		DisplayDigit_02(Segment_CommAnode[index]);
		gioSetBit(LED_PORT, LATCH2_PIN, PIN_HIGH);
		// vDelay_ticks(3000U);
		vDelay_ticks(100);
	}
}

/////**********************LED Tube Display ***************************///////
///
///
/////*******************************Motor Application**************************//////

uint32_t u32GetTimeSliceDuration_ms(uint32_t u32start_ms)
{
	if (u32SystemTimer_1ms >= u32start_ms)
		return (u32SystemTimer_1ms - u32start_ms);
	else
		return (0xFFFFFFFF - u32start_ms + u32SystemTimer_1ms);
}
// run motor forward//
void Motor_forward()
{
	motor_dir = set_motor_dir(true);  // set pin 3 forward=1, backward=0, for motor direction//
	pwmSetDuty(hetRAM1, pwm1, Duty1); // set duty cycle to pwm signal 1 from Light Sensor port to pin NHET18//
}
// run motor backward//
void Motor_backward()
{
	motor_dir = set_motor_dir(false); // set pin 3 backward=0, backward=0, for motor direction//
	pwmSetDuty(hetRAM1, pwm1, Duty1); // set duty cycle to pwm signal 1 from Light Sensor port to pin NHET18//
}

uint32_t u32GetTime_ms(void)
{
	return u32SystemTimer_1ms;
}
// stop the motor//
void Motor_Stop()
{
	pwmSetDuty(hetRAM1, pwm1, 0); // set duty cycle to 0//
}
void Motor_move_forward_OpenLoop(void) // Not used
{
	static boolean first_run = true;
	if (first_run)
	{
		nowtime_ms = u32GetTime_ms();
		first_run = false;
	}
	// move forward
	gioSetBit(gioPORTA, 3, PIN_HIGH); // set pin 3 output as 1//
	flag_motor_forward = true;
	flag_motor_stop = false;

	uint8_t time_index = 0;
	uint32_t u32Temp_ms = 0;
	uint32_t u32TimePast_ms = u32GetTimeSliceDuration_ms(nowtime_ms);

	// ramp up region
	if (u32TimePast_ms <= RAMP_UP_DURATION_MS)
	{
		time_index = (uint8_t)(u32TimePast_ms * 0.02);
		// float fldelta = (MOTOR_DUTYCYCLE_RAMP_MAX - MOTOR_DUTYCYCLE_RAMP_MIN) *50U / RAMP_UP_DURATION_MS;
		pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_RAMP_MAX - 2 * time_index); // set duty cycle to individual //
	}
	else if (u32TimePast_ms < (CONST_SPEED_DURATION_MS + RAMP_UP_DURATION_MS) + 1)
	{
		pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_CONST_SPEED);
		// only check motor speed here
		if ((u32TimePast_ms > (RAMP_UP_DURATION_MS + 1000U)) && !blflag_speed_check) // only after 1000ms starts check
		{
			if (u32SpeedAve < MIN_CONSTSPEED_MOTOR_SPEED_RPM)
			{
				flag_motor_error = true;
			}
			blflag_speed_check = true; // check no issue
		}
	}
	else if (u32TimePast_ms < (CONST_SPEED_DURATION_MS + RAMP_UP_DURATION_MS + RAMP_DOWN_DURATION_MS + 1U))
	{
		u32Temp_ms = u32TimePast_ms - CONST_SPEED_DURATION_MS - RAMP_UP_DURATION_MS;
		time_index = (uint8_t)(u32Temp_ms * 0.02);
		pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_CONST_SPEED); // set duty cycle to individual //
	}
	else if (u32TimePast_ms < (CONST_SPEED_DURATION_MS + RAMP_UP_DURATION_MS + RAMP_DOWN_DURATION_MS + MOTOR_STOP_DURATION_MS + 1U))
	{
		u32Temp_ms = u32TimePast_ms - CONST_SPEED_DURATION_MS - RAMP_UP_DURATION_MS - RAMP_DOWN_DURATION_MS;
		time_index = (uint8_t)(u32Temp_ms * 0.02);
		// pwmSetDuty(hetRAM1, pwm1, (MOTOR_DUTYCYCLE_STOP_MAX - time_index));//set duty cycle to 0//
		pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_STOP_MAX); // set duty cycle to 0//
	}
	else if (u32TimePast_ms < WHOLE_WAITING_TIME_MS)
	{
		pwmSetDuty(hetRAM1, pwm1, 0); // set duty cycle to 0//
		flag_motor_stop = true;
		if (u32TimePast_ms > (WHOLE_WAITING_TIME_MS - 100U)) // only check during last 100ms
		{
			if (u32motor_rotate_pulse_cntr < MAX_MOTOR_PULSE_COUNT) // motor stuck error or no power
			{
				// flag_motor_error = true;
			}
		}
	}
	else
	{
		// waiting period finish, should start next cycle

		nowtime_ms = u32GetTime_ms();
		max_pos_flag = true;
		flag_motor_stop = true;
		updateLED_flag = true;
		motor_forward = false;
		blflag_speed_check = false; // prepare for next check
	}
}
void Motor_move_backward_OpenLoop(void) // Not used
{

	// move backward
	gioSetBit(gioPORTA, 3, PIN_LOW); // move backward
	flag_motor_forward = false;
	flag_motor_stop = false;
	uint8_t time_index = 0;
	uint32_t u32Temp_ms = 0;
	uint32_t u32TimePast_ms = u32GetTimeSliceDuration_ms(nowtime_ms);

	// ramp up region
	if (u32TimePast_ms <= RAMP_UP_DURATION_MS)
	{
		time_index = (uint8_t)(u32TimePast_ms * 0.02);
		pwmSetDuty(hetRAM1, pwm1, (MOTOR_DUTYCYCLE_RAMP_MAX - 2 * time_index)); // set duty cycle to individual //
	}
	else if (u32TimePast_ms < (CONST_SPEED_DURATION_MS + RAMP_UP_DURATION_MS) + 1)
	{
		pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_CONST_SPEED);
		// only check motor speed here
		if ((u32TimePast_ms > (RAMP_UP_DURATION_MS + 1000U)) && !blflag_speed_check) // only after 1000ms starts check
		{
			if (u32SpeedAve < MIN_CONSTSPEED_MOTOR_SPEED_RPM)
			{
				flag_motor_error = true;
			}
			blflag_speed_check = true; // check no issue
		}
	}
	else if (u32TimePast_ms < (CONST_SPEED_DURATION_MS + RAMP_UP_DURATION_MS + RAMP_DOWN_DURATION_MS + 1U))
	{
		u32Temp_ms = u32TimePast_ms - CONST_SPEED_DURATION_MS - RAMP_UP_DURATION_MS;
		time_index = (uint8_t)(u32Temp_ms * 0.02);
		// pwmSetDuty(hetRAM1, pwm1, (MOTOR_DUTYCYCLE_RAMP_MAX - time_index));//set duty cycle to individual //
		pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_CONST_SPEED);
	}
	else if (u32TimePast_ms < (CONST_SPEED_DURATION_MS + RAMP_UP_DURATION_MS + RAMP_DOWN_DURATION_MS + MOTOR_STOP_DURATION_MS + 1U))
	{
		u32Temp_ms = u32TimePast_ms - CONST_SPEED_DURATION_MS - RAMP_UP_DURATION_MS - RAMP_DOWN_DURATION_MS;
		time_index = (uint8_t)(u32Temp_ms * 0.02);
		// pwmSetDuty(hetRAM1, pwm1, (MOTOR_DUTYCYCLE_STOP_MAX - time_index));//set duty cycle to 0//
		pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_STOP_MAX); // set duty cycle to 0//
	}
	else if (u32TimePast_ms < WHOLE_WAITING_TIME_MS)
	{
		pwmSetDuty(hetRAM1, pwm1, 0); // set duty cycle to 0//
		flag_motor_stop = true;
		if (u32TimePast_ms > (WHOLE_WAITING_TIME_MS - 100U)) // only check during last 100ms
		{
			if (u32motor_rotate_pulse_cntr > MIN_MOTOR_PULSE_COUNT) // motor stuck error or no power
			{
				// flag_motor_error = true;
			}
		}
	}
	else
	{
		// waiting period finish, should start next cycle
		// flag_motor_backword = true;

		nowtime_ms = u32GetTime_ms();
		// if(!flag_motor_error)
		{
			u32TestCounter++;
			blflag_speed_check = false; // prepare for next speed check
		}

		updateLED_flag = true;
		flag_motor_stop = true;
		motor_forward = true;
	}
}

//// motor ramp up profile
// void Motor_Move_Torque(uint8_t direction)
//{
//     // direction check
//	static bool firstcall = true;
//	static boolean bldirection;
//	static uint32_t u32StartTime_ms = 0;
//	if(dir_forward == direction)
//	{
//		gioSetBit(gioPORTA, 3, PIN_HIGH);//move backward,set pin 3 backward=0, forward=1, for motor direction//
//		gioSetBit(gioPORTA, 3, PIN_HIGH);//move backward,set pin 3 backward=0, forward=1, for motor direction//
//	}
//	else if(dir_backward == direction)
//	{
//		// move backward
//		gioSetBit(gioPORTA, 3, PIN_LOW);//move backward,set pin 3 backward=0, forward=1, for motor direction//
//		gioSetBit(gioPORTA, 3, PIN_LOW);//move backward,set pin 3 backward=0, forward=1, for motor direction//
//	}
//	uint16_t u16Delta_dutycycle = (uint16_t) ((MOTOR_DUTYCYCLE_RAMP_MAX - MOTOR_DUTYCYCLE_RAMP_MIN) * 0.1);
//	uint16_t u16Stop_Delta = (uint16_t) ((MOTOR_DUTYCYCLE_STOP_MAX - MOTOR_DUTYCYCLE_STOP_MIN) * 0.1);
//     if(firstcall)
//     {
//     	u32StartTime_ms = u32SystemTimer_1ms;
//     	firstcall = false;
//     }
//	uint32_t u32TimePast_ms = u32GetTimeSliceDuration_ms(u32StartTime_ms);
//	uint32_t u32Temp_ms = 0;
//	// ramp up region
//	if(u32TimePast_ms <= RAMP_UP_DURATION_MS)
//	{
//		pwmSetDuty(hetRAM1, pwm1, (MOTOR_DUTYCYCLE_RAMP_MIN + (uint8_t)(u32TimePast_ms / 50U) * u16Delta_dutycycle));//set duty cycle to individual //
//	}
//	else if (u32TimePast_ms < (CONST_SPEED_DURATION_MS + RAMP_UP_DURATION_MS) + 1)
//	{
//		pwmSetDuty(hetRAM1, pwm1,MOTOR_DUTYCYCLE_CONST_SPEED);
//	}
//	else if (u32TimePast_ms < (CONST_SPEED_DURATION_MS + RAMP_UP_DURATION_MS + RAMP_DOWN_DURATION_MS + 1U))
//	{
//		u32Temp_ms = u32TimePast_ms - CONST_SPEED_DURATION_MS - RAMP_UP_DURATION_MS;
//		pwmSetDuty(hetRAM1, pwm1, (MOTOR_DUTYCYCLE_RAMP_MAX - (uint8_t)(u32Temp_ms / 50U) * u16Delta_dutycycle));//set duty cycle to individual //
//	}
//	else if(u32TimePast_ms < (CONST_SPEED_DURATION_MS + RAMP_UP_DURATION_MS + RAMP_DOWN_DURATION_MS + MOTOR_STOP_DURATION_MS + 1U))
//	{
//		u32Temp_ms = u32TimePast_ms - CONST_SPEED_DURATION_MS - RAMP_UP_DURATION_MS - RAMP_DOWN_DURATION_MS;
//		pwmSetDuty(hetRAM1, pwm1, (MOTOR_DUTYCYCLE_STOP_MAX - (uint8_t)(u32Temp_ms / 50U) * u16Stop_Delta));//set duty cycle to 0//
//	}
//	else if (u32TimePast_ms < WHOLE_WAITING_TIME_MS)
//	{
//		pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_STOP_MAX);//set duty cycle to 0//
//	}
//	else
//	{
//		//waiting period finish, should start next cycle
//		//flag_motor_backword = true;
//		timeout_flag = !timeout_flag;
//		max_pos_flag = true;
//	}
//
// }

// handling motor operation//
// void Motor_operation()
//{
//     pos= pulse_cnt;//get value of motor pulse//
//     adc_convert();//get adc value for PWM duty cycle & battery //
//
//     //pwmSetDuty(hetRAM1, pwm0, Duty2);//set duty cycle from Temperature Sensor port to pin NHET5//
//
//     if((motor_bw==true)&&(pos>MIN_PULSE)&&(motor_stop==false))//motor run backward//
//     {
//         Motor_backward();//motor move backward//
//     }
//     else if((motor_bw==false)&&(pos<MAX_PULSE)&&(motor_stop==false))//motor run forward//
//     {
//         Motor_forward();//motor move forward//
//     }
//     else if( ((motor_bw==true)&&(pos<MIN_PULSE))
//            ||((motor_bw==false)&&(pos>MAX_PULSE))
//            )
//     {
//         Motor_Stop();//stop motor because error//
//
//         motor_dir= 2;
//     }
//     else if( (motor_stop==true)&&(motor_bw==false)
//            ||(motor_stop==true)&&(motor_bw==true)
//            )
//     {
//         Motor_Stop();//stop motor because error//
//         delay(10000);
//
//         if((motor_stop==true)&&(motor_bw==false))
//         {
//           cycle_cnt +=1;//start counting cycle//
//           Pin_Disp(mdir);//Display Motor Direction //
//         }
//         else if((motor_stop==true)&&(motor_bw==true))
//         {
//           Pin_Disp(mdir);//Display Motor Direction //
//         }
//
//         motor_stop=false;
//     }
//
//     //can_dattx=pos;//assign for can bus communication//
//
//     //Handling save data to eeprom//
//     if(Batt<=BATT_LIM)//battery reading range adc from 0~3.3Volt
//     {
//       EEPROM_writeData(pos, cycle_cnt ,0x01);//write data to EEPROM//
//     }
//
//     //Display position & cycle in close, middle & open position//
//     if((pos>=(MIN_PULSE-10)) && (pos<=MIN_PULSE))
//     {
//       Mot_Disp(pos); //Display Pin PWM counter//
//       Var_Disp(cycle_cnt);//Display motor running cycle//
//     }
//     else if((pos>=(MAX_PULSE/2)) && (pos<=((MAX_PULSE/2)+20)))
//     {
//       Mot_Disp(pos); //Display Pin PWM counter//
//       Var_Disp(cycle_cnt);//Display motor running cycle//
//     }
//     else if(pos>=MAX_PULSE)
//     {
//       Mot_Disp(pos); //Display Pin PWM counter//
//       Var_Disp(cycle_cnt);//Display motor running cycle//
//     }
// }
/////***************End of Motor Application**********************//////

//******** SWITCH APPLICATION***********//
void vCheckSwitchStatus(void)
{
	if (flag_switch_on)
	{
		if (0U == gioGetBit(SWITCH_PORT, SWITCH_PIN))
		{
			switch_off_cntr++;
		}
		else
		{
			if (switch_off_cntr > 0)
				switch_off_cntr--;
		}
		if (switch_off_cntr > SWITCH_DEBOUNCE_COUNTER)
		{
			flag_switch_on = false;
			switch_off_cntr = 0U;
		}
	}
	else
	{
		if (1U == gioGetBit(SWITCH_PORT, SWITCH_PIN))
		{
			switch_on_cntr++;
		}
		else
		{
			if (switch_on_cntr > 0)
				switch_on_cntr--;
		}
		if (switch_on_cntr > SWITCH_DEBOUNCE_COUNTER)
		{
			flag_switch_on = true;
			switch_on_cntr = 0U;
		}
	}
}
//////**********************Initialization***********************/////
///
void vEEPROM_Init(void)
{
	TI_FeeModuleStatusType status = UNINIT;
	TI_Fee_Init();
	do
	{
		TI_Fee_MainFunction();
		vDelay_ticks(300U);
		status = TI_Fee_GetStatus(0);
	} while (status != IDLE);
}
void Init_all()
{
	// Initialize the system clocks
	systemInit();

	/* Initialize VIM table */
	vimInit();

	// this must enable for RS232/485 serial comm
	muxInit();

	sciInit(); // Initialize the SCI (UART) module
	adcInit(); // Initialize the ADC module
	hetInit(); // Initialize Het(PWM)//
	gioInit(); // Initialize GIO//

	// Load the timer
	// Initialize RTI module
	rtiInit();

	/* Enable RTI Compare 0,1,2 3 interrupt notification */
	/*rtiCOMPARE0 used for 1ms task schedule*/
	rtiEnableNotification(rtiREG1, rtiNOTIFICATION_COMPARE0);

	// set LED tube
	gioSetDirection(gioPORTB, u8PortB_dir); // Set GIOB port pin LED Tube direction to all output
	gioSetBit(LED_PORT, CLOCK_PIN, 0U);
	gioSetBit(LED_PORT, DATA_PIN, 0U);
	gioSetBit(LED_PORT, LATCH_PIN, 0U);
	gioSetBit(LED_PORT, CLOCK2_PIN, 0U);
	gioSetBit(LED_PORT, DATA2_PIN, 0U);
	gioSetBit(LED_PORT, LATCH2_PIN, 0U);
	TI_Fee_Init();						// Initialize FEE. This will create Virtual sectors, initialize global variables etc.//
	gioEnableNotification(gioPORTA, 1); // Enable notifications of GIO//

	canInit();							 // configuring CAN1 MB1,Msg ID-1 to transmit and CAN2 MB1 to receive //
	canEnableErrorNotification(canREG1); // enabling error interrupts CAN_Bus1//
	canEnableErrorNotification(canREG2); // enabling error interrupts CAN_Bus2//

	_enable_IRQ_interrupt_(); /* enable irq interrupt in */

	vEEPROM_Init();

	rotaryInit(false);							 // no wrap around
	rtiStartCounter(rtiREG1, rtiCOUNTER_BLOCK0); // Start RTI Counter Block 0
	pwmStart(hetRAM1, pwm0);					 // start PWM0 output//
	pwmStart(hetRAM1, pwm1);					 // start PWM1 output//
	pwmStart(hetRAM1, pwm2);					 // start PWM2 output//
	pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_CONST_SPEED);
}

//////********************** New Motor Application ***********************/////
void Motor_move_forward_pulse(void)
{
	static boolean first_run = true;
	static boolean wait_start = true;
	if (bl_tick_move_forward_time)
	{
		nowtime_ms = u32GetTime_ms();
		bl_tick_move_forward_time = false;
		i32EncPulse_cntr = 0U;
	}
	// move forward
	gioSetBit(gioPORTA, 7, PIN_HIGH); // set pin 3 output as 1//
	flag_motor2_forward = true;
	flag_motor2_stop = false;

	uint8_t time_index = 0;
	uint32_t u32Temp_ms = 0;
	uint32_t u32TimePast_ms = u32GetTimeSliceDuration_ms(nowtime_ms);

	// now check whether timeout or not
	if (u32TimePast_ms > PULSE_MOVE_TIMEOUT)
	{
		flag_motor2_error = true;
	}
	else
	{
		// ramp up region
		if (i32EncPulse_cntr <= RAMP_UP_PULSE_END)
		{
			pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_RAMP_MAX); // set duty cycle to individual //
		}
		else if (i32EncPulse_cntr < CONST_SPEED_PULSE_END)
		{
			pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_CONST_SPEED);
			// only check motor speed here
			if ((i32EncPulse_cntr > (CONST_SPEED_PULSE_END >> 1U)) && !blflag_speed_check) // only after 2000ms starts check
			{
				if (u32SpeedAve < MIN_CONSTSPEED_MOTOR_SPEED_RPM)
				{
					flag_motor2_error = true;
				}
				blflag_speed_check = true; // check no issue
			}
		}
		else if (i32EncPulse_cntr < RAMP_DOWN_PULSE_END)
		{
			pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_STOP_MAX); // set duty cycle to individual //
		}
		else
		{
			// stop motor first
			pwmSetDuty(hetRAM1, pwm1, 0); // set duty cycle to 0//
			pwmSetDuty(hetRAM1, pwm1, 0); // set duty cycle to 0//
			flag_motor2_stop = true;
			flag_motor2_forward = false;

			// updateLED_flag = true;
			// motor_forward = false;
			blflag_speed_check = false; // prepare for next check

			u32move_forward_waiting_ms = u32GetTime_ms();
		}
	}
}

void Motor_move_backward_pulse(void)
{
	// move backward
	gioSetBit(gioPORTA, 3, PIN_LOW); // move backward
	flag_motor_forward = false;
	flag_motor_stop = false;
	uint32_t u32Temp_ms = 0;
	max_pos_flag = false;
	static uint32_t u32TimePast_ms;

	if (bl_tick_move_backward_time)
	{
		nowtime_ms = u32GetTime_ms();
		bl_tick_move_backward_time = false;
	}

	u32TimePast_ms = u32GetTimeSliceDuration_ms(nowtime_ms);
	// now check whether timeout or not
	if (u32TimePast_ms > PULSE_MOVE_TIMEOUT)
	{
		flag_motor_error = true;
	}
	else
	{
		// ramp up region
		if (i32EncPulse_cntr >= CONST_SPEED_PULSE_END)
		{
			pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_RAMP_MAX); // set duty cycle to individual //
		}
		else if (i32EncPulse_cntr >= RAMP_UP_PULSE_END)
		{
			pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_CONST_SPEED);
			// only check motor speed here
			// if((u32TimePast_ms > (2000U)) && !blflag_speed_check) // only after 1000ms starts check
			if ((i32EncPulse_cntr < (CONST_SPEED_PULSE_END >> 1U)) && !blflag_speed_check) // only after 920pulses starts check
			{
				if (u32SpeedAve < MIN_CONSTSPEED_MOTOR_SPEED_RPM)
				{
					flag_motor_error = true;
				}
				blflag_speed_check = true; // check no issue
			}
		}
		else if (i32EncPulse_cntr > 3U)
		{
			pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_STOP_MAX); // set duty cycle to individual //
		}
		else
		{
			pwmSetDuty(hetRAM1, pwm1, 0); // set duty cycle to 0//
			pwmSetDuty(hetRAM1, pwm1, 0); // set duty cycle to 0//
			if (!flag_motor_error)
			{
				// u32TestCounter++;
				u32TestCounter_new++;
			}
			blflag_speed_check = false; // prepare for next speed check

			flag_motor_stop = true;
			max_pos_flag = true;

			u32move_backward_waiting_ms = u32GetTime_ms();
			// motor_forward = true;
			// i32EncPulse_cntr = 0;
		}
	}
}

// void Motor_move_forward_torque(void)
//{
//	static boolean first_run = true;
//	if(first_run)
//	{
//		nowtime_ms = u32GetTime_ms();
//		first_run = false;
//	}
//     // move forward
//	gioSetBit(gioPORTA, 3, PIN_HIGH);//set pin 3 output as 1//
//	flag_motor_forward = true;
//	flag_motor_stop = false;
//
//	uint8_t time_index = 0;
//	uint32_t u32Temp_ms = 0;
//	uint32_t u32TimePast_ms = u32GetTimeSliceDuration_ms(nowtime_ms);
//
//	// ramp up region
//	if(u32TimePast_ms <= RAMP_UP_DURATION_MS)
//	{
//		time_index = (uint8_t)(u32TimePast_ms * 0.01);
//		//float fldelta = (MOTOR_DUTYCYCLE_RAMP_MAX - MOTOR_DUTYCYCLE_RAMP_MIN) *50U / RAMP_UP_DURATION_MS;
//		pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_RAMP_MAX - time_index);//set duty cycle to individual //
//	}
//	else if (u32TimePast_ms < (CONST_SPEED_DURATION_MS + RAMP_UP_DURATION_MS) + 1)
//	{
//		pwmSetDuty(hetRAM1, pwm1,MOTOR_DUTYCYCLE_CONST_SPEED);
//		// only check motor speed here
//		if((u32TimePast_ms > (RAMP_UP_DURATION_MS + 1000U)) && !blflag_speed_check) // only after 1000ms starts check
//		{
//			if(u32SpeedAve < MIN_CONSTSPEED_MOTOR_SPEED_RPM)
//			{
//				flag_motor_error = true;
//			}
//			blflag_speed_check = true; // check no issue
//		}
//	}
//	else if (u32TimePast_ms < (CONST_SPEED_DURATION_MS + RAMP_UP_DURATION_MS + RAMP_DOWN_DURATION_MS + 1U))
//	{
//		//u32Temp_ms = u32TimePast_ms - CONST_SPEED_DURATION_MS - RAMP_UP_DURATION_MS;
//		//time_index = (uint8_t)(u32Temp_ms * 0.01);
//		pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_CONST_SPEED);//set duty cycle to individual //
//	}
//	else if(u32TimePast_ms < (CONST_SPEED_DURATION_MS + RAMP_UP_DURATION_MS + RAMP_DOWN_DURATION_MS + MOTOR_STOP_DURATION_MS + 1U))
//	{
//		u32Temp_ms = u32TimePast_ms - CONST_SPEED_DURATION_MS - RAMP_UP_DURATION_MS - RAMP_DOWN_DURATION_MS;
//		time_index = (uint8_t)(u32Temp_ms * 0.01);
//		pwmSetDuty(hetRAM1, pwm1, (MOTOR_DUTYCYCLE_STOP_MAX - time_index));//set duty cycle to 0//
//		//pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_STOP_MAX);//set duty cycle to 0//
//	}
//	else if (u32TimePast_ms < WHOLE_WAITING_TIME_MS)
//	{
//		pwmSetDuty(hetRAM1, pwm1, 0);//set duty cycle to 0//
//		flag_motor_stop = true;
//		if(u32TimePast_ms > (WHOLE_WAITING_TIME_MS - 100U)) // only check during last 100ms
//		{
//			if(u32motor_rotate_pulse_cntr < MAX_MOTOR_PULSE_COUNT) // motor stuck error or no power
//			{
//				//flag_motor_error = true;
//			}
//		}
//	}
//	else
//	{
//		//waiting period finish, should start next cycle
//
//		nowtime_ms = u32GetTime_ms();
//		max_pos_flag = true;
//		flag_motor_stop = true;
//		updateLED_flag = true;
//		motor_forward = false;
//		blflag_speed_check = false; // prepare for next check
//	}
// }
// void Motor_move_backward_torque(void) //Not used
//{
//
//     // move backward
//	gioSetBit(gioPORTA, 3, PIN_LOW);//move backward
//	flag_motor_forward = false;
//	flag_motor_stop = false;
//	uint8_t time_index = 0;
//	uint32_t u32Temp_ms = 0;
//	uint32_t u32TimePast_ms = u32GetTimeSliceDuration_ms(nowtime_ms);
//
//	// ramp up region
//	if(u32TimePast_ms <= RAMP_UP_DURATION_MS)
//	{
//		time_index = (uint8_t)(u32TimePast_ms * 0.01);
//		//float fldelta = (MOTOR_DUTYCYCLE_RAMP_MAX - MOTOR_DUTYCYCLE_RAMP_MIN) *50U / RAMP_UP_DURATION_MS;
//		pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_RAMP_MAX - time_index);//set duty cycle to individual //
//	}
//	else if (u32TimePast_ms < (CONST_SPEED_DURATION_MS + RAMP_UP_DURATION_MS) + 1)
//	{
//		pwmSetDuty(hetRAM1, pwm1,MOTOR_DUTYCYCLE_CONST_SPEED);
//		// only check motor speed here
//		if((u32TimePast_ms > (RAMP_UP_DURATION_MS + 1000U)) && !blflag_speed_check) // only after 1000ms starts check
//		{
//			if(u32SpeedAve < MIN_CONSTSPEED_MOTOR_SPEED_RPM)
//			{
//				flag_motor_error = true;
//			}
//			blflag_speed_check = true; // check no issue
//		}
//	}
//	else if (u32TimePast_ms < (CONST_SPEED_DURATION_MS + RAMP_UP_DURATION_MS + RAMP_DOWN_DURATION_MS + 1U))
//	{
//		//u32Temp_ms = u32TimePast_ms - CONST_SPEED_DURATION_MS - RAMP_UP_DURATION_MS;
//		//time_index = (uint8_t)(u32Temp_ms * 0.01);
//		pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_CONST_SPEED);//set duty cycle to individual //
//	}
//	else if(u32TimePast_ms < (CONST_SPEED_DURATION_MS + RAMP_UP_DURATION_MS + RAMP_DOWN_DURATION_MS + MOTOR_STOP_DURATION_MS + 1U))
//	{
//		u32Temp_ms = u32TimePast_ms - CONST_SPEED_DURATION_MS - RAMP_UP_DURATION_MS - RAMP_DOWN_DURATION_MS;
//		time_index = (uint8_t)(u32Temp_ms * 0.01);
//		pwmSetDuty(hetRAM1, pwm1, (MOTOR_DUTYCYCLE_STOP_MAX - time_index));//set duty cycle to 0//
//		//pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_STOP_MAX);//set duty cycle to 0//
//	}
//	else if (u32TimePast_ms < WHOLE_WAITING_TIME_MS)
//	{
//		pwmSetDuty(hetRAM1, pwm1, 0);//set duty cycle to 0//
//		flag_motor_stop = true;
//		if(u32TimePast_ms > (WHOLE_WAITING_TIME_MS - 100U)) // only check during last 100ms
//		{
//			if(u32motor_rotate_pulse_cntr < MAX_MOTOR_PULSE_COUNT) // motor stuck error or no power
//			{
//				//flag_motor_error = true;
//			}
//		}
//	}
//	else
//	{
//		//waiting period finish, should start next cycle
//		//flag_motor_backword = true;
//
//		nowtime_ms = u32GetTime_ms();
//		if(!flag_motor_error)
//		{
//			u32TestCounter++;
//
//		}
//		blflag_speed_check = false; // prepare for next speed check
//		updateLED_flag = true;
//		flag_motor_stop = true;
//		motor_forward = true;
//
//	}
//
// }

/********************************************START OF MOTOR TWO FUNCTION DEFINITIONS*********************************************/
void loadCountersFromEEPROM(void)
{
	// uint8_t loadBufferArray = {0};
	//  Loads both main counters from same block address
	TI_Fee_ReadSync(COUNTERTWO_EEPROM_ADD, 0U, &u32TestCounterTwo_new, 4U);
	// memcpy(mainCounterOne, loadBufferArray, 4U);	 // Extract first 4 bytes for mainCounterOne
	// memcpy(u32TestCounterTwo_new, loadBufferArray + 4, 4U); // Extract next 4 bytes for u32TestCounterTwo_new
}

void saveCountersToEEPROM(void)
{
	// uint8_t writeBufferArray = {0};
	// memcpy(writeBufferArray, &mainCounterOne, 4U);
	// memcpy(writeBufferArray + 4, &u32TestCounterTwo_new, 4U);
	TI_Fee_WriteSync(COUNTERTWO_EEPROM_ADD, &u32TestCounterTwo_new);
}

void SetMotorTwoDirection(uint32 dirB)
{
	gioSetBit(gioPORTA, MOTORTWODIRPIN, dirB);
}

void SetMotorTwoSpeed(uint32 spdB)
{
	pwmSetDuty(hetRAM1, pwm0, spdB);
}

void Motor_move_forward_pulse(void)
{
	static boolean first_run = true;
	static boolean wait_start = true;
	if(bl_tick_move_forward_time)
	{
		nowtime_ms = u32GetTime_ms();
		bl_tick_move_forward_time = false;
		i32EncPulse_cntr = 0U;
	}
    // move forward
	SetMotorTwoDirection(PIN_HIGH);//set pin 3 output as 1//
	flag_motor2_forward = true;
	flag_motor2_stop = false;

	uint8_t  time_index = 0;
	uint32_t u32Temp_ms = 0;
	uint32_t u32TimePast_ms = u32GetTimeSliceDuration_ms(nowtime_ms);

	// now check whether timeout or not
	if(u32TimePast_ms > PULSE_MOVE_TIMEOUT)
	{
		flag_motor2_error = true;
	}
	else
	{
		// ramp up region
		if(i32EncPulse_cntr <= RAMP_UP_PULSE_END)
		{
			SetMotorTwoSpeed(MOTOR_DUTYCYCLE_RAMP_MAX);//set duty cycle to individual //
		}
		else if (i32EncPulse_cntr < CONST_SPEED_PULSE_END)
		{
			SetMotorTwoSpeed(hetRAM1, pwm1,MOTOR_DUTYCYCLE_CONST_SPEED);
			// only check motor speed here
			if((i32EncPulse_cntr > (CONST_SPEED_PULSE_END >> 1U)) && !blflag_speed_check) // only after 2000ms starts check
			{
				if(u32SpeedAve < MIN_CONSTSPEED_MOTOR_SPEED_RPM)
				{
					flag_motor2_error = true;
				}
				blflag_speed_check = true; // check no issue
			}
		}
		else if (i32EncPulse_cntr < RAMP_DOWN_PULSE_END)
		{
			SetMotorTwoSpeed(hetRAM1, pwm1, MOTOR_DUTYCYCLE_STOP_MAX);//set duty cycle to individual //
		}
		else
		{
			// stop motor first
			SetMotorTwoSpeed(0);//set duty cycle to 0//
			SetMotorTwoSpeed(0);//set duty cycle to 0//
			flag_motor2_pause = true;
			flag_motor2_stop = true;
			flag_motor2_forward = false;

			//updateLED_flag = true;
			//motor_forward = false;
			blflag_speed_check = false; // prepare for next check

			u32move_forward_waiting_ms = u32GetTime_ms();
		}
	}


}

void Motor_move_backward_pulse(void)
{
	// move backward
		gioSetBit(gioPORTA, 3, PIN_LOW);//move backward
		flag_motor_forward = false;
		flag_motor_stop = false;
		uint32_t u32Temp_ms = 0;
		max_pos_flag = false;
		static uint32_t u32TimePast_ms;

		if(bl_tick_move_backward_time)
		{
			nowtime_ms = u32GetTime_ms();
			bl_tick_move_backward_time = false;
		}

		u32TimePast_ms = u32GetTimeSliceDuration_ms(nowtime_ms);
		// now check whether timeout or not
		if(u32TimePast_ms > PULSE_MOVE_TIMEOUT)
		{
			flag_motor_error = true;
		}
		else
		{
			// ramp up region
			if(i32EncPulse_cntr >= CONST_SPEED_PULSE_END)
			{
				pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_RAMP_MAX);//set duty cycle to individual //
			}
			else if (i32EncPulse_cntr >= RAMP_UP_PULSE_END)
			{
				pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_CONST_SPEED);
				// only check motor speed here
				//if((u32TimePast_ms > (2000U)) && !blflag_speed_check) // only after 1000ms starts check
				if((i32EncPulse_cntr < (CONST_SPEED_PULSE_END >> 1U)) && !blflag_speed_check) // only after 920pulses starts check
				{
					if(u32SpeedAve < MIN_CONSTSPEED_MOTOR_SPEED_RPM)
					{
						flag_motor_error = true;
					}
					blflag_speed_check = true; // check no issue
				}
			}
			else if (i32EncPulse_cntr > 3U)
			{
				pwmSetDuty(hetRAM1, pwm1, MOTOR_DUTYCYCLE_STOP_MAX);//set duty cycle to individual //
			}
			else
			{
				pwmSetDuty(hetRAM1, pwm1, 0);//set duty cycle to 0//
				pwmSetDuty(hetRAM1, pwm1, 0);//set duty cycle to 0//
				if(!flag_motor_error)
				{
					//u32TestCounter++;
					u32TestCounter_new++;
				}
				blflag_speed_check = false; // prepare for next speed check

				flag_motor_stop = true;
				max_pos_flag = true;

				u32move_backward_waiting_ms = u32GetTime_ms();
				//motor_forward = true;
				//i32EncPulse_cntr = 0;
			}
		}
}

/********************************************END OF MOTOR TWO FUNCTION DEFINITIONS*********************************************/
//////**********************End of Initialization***********************/////

// uint32_t u32TestCntr = 0;

void main(void)
{

	Init_all(); // Initialize all drivers//

	// if (0)
	// {
	// 	EEPROM_writeCounterData(220804, 220804, COUNTER_EEPROM_ADD); // 11 May 2023 16:04pm
	// 	vDelay_ticks(80000U);
	// }
	// EEPROM_readData(0x01);//read data from EEPROM//
	gioSetBit(gioPORTA, 3, PIN_LOW); // move backward
	gioSetBit(gioPORTA, 3, PIN_LOW); // move backward
	gioSetBit(gioPORTA, 3, PIN_LOW); // move backward
	pwmSetDuty(hetRAM1, pwm1, 90);	 // set duty cycle to 0//
	// u32temp02;
	flag_mstart = false; // flag for assigning read data from eeprom to motor variable//
	u32SystemTimer_1ms = 0;
	motor_forward = true;
	updateLED_flag = true;
	// firstzero = true;

	u32SpeedTimer_ms = u32SystemTimer_1ms;
	u32MotorEncPosition = 0;
	uint8_t u8Index = 0;
	u32SpeedAve = 0;
	u32Timer_100ms = 0;
	u32eeprom_timer_1ms = 0;
	EEPROM_readCounterData(COUNTER_EEPROM_ADD, &u32TestCounter_new, &u32temp_ee_test);
	vDelay_ticks(80000U);
	// if (u32TestCounter_new < u32temp_ee_test)
	// {
	// 	// u32TestCounter_new = u32temp_ee_test;
	// }
	// if (0)
	// {
	// 	if (u32TestCounter > u32temp_ee_test)
	// 	{
	// 		// EEPROM_writeCounterData(u32TestCounter_new, u32TestCounter_new, COUNTER_EEPROM_ADD);
	// 	}
	// 	else if (u32TestCounter_new < u32temp_ee_test)
	// 	{
	// 		// EEPROM_writeCounterData(u32temp_ee_test, u32temp_ee_test, COUNTER_EEPROM_ADD);
	// 	}
	// }

	switch_on_cntr = 0;
	switch_off_cntr = 0;
	flag_switch_on = false;
	u32motor_rotate_pulse_cntr = 0;
	save_eeprom = false;
	blflag_reset = false;
	flag_motor_error = false;
	blflag_speed_check = false;
	u32ErrorHappen_cntr = 0;
	u32Retry_cntr = 0;
	u32SpeedTimer_ms = u32SystemTimer_1ms;

	boolean first_reset = true;
	boolean reset_foward = false;
	blfag_stop_reset = false;
	max_pos_flag = false;
	bl_tick_move_forward_time = true;
	bl_tick_move_backward_time = false;
	i32EncPulse_cntr = 0;
	while (1) // Loop to acquire and do the task//
	{
		// start EEPROM writing
		if (u32eeprom_timer_1ms > 180000) // every 3mins save the eeprom data // --- change from 30mins to 3min, 03May2023
		{

			// EEPROM_writeCounterData(u32TestCounter_new, u32TestCounter_new, COUNTER_EEPROM_ADD);
			// vDelay_ticks(800U);
			u32eeprom_timer_1ms = 0; // reset the timer
			u32eeprom_timer_1ms = 0;
		}

		// motor speed measure
		if (u32GetTimeSliceDuration_ms(u32SpeedTimer_ms) > 100U) // every 100ms calculate the speed
		{
			u32Timer_100ms++;
			u32Speed_rpm = (uint32_t)(100 * u32MotorEncPosition);
			u32SpeedSum = u32SpeedSum + u32Speed_rpm;
			u8Index++;
			if (u8Index > 3)
			{
				u32SpeedAve = (uint32_t)(u32SpeedSum >> 2U);
				u8Index = 0;
				u32SpeedSum = 0;
			}

			if ((flag_motor_error) && (!blfag_stop_reset))
			{
				if (u32ErrorHappen_cntr++ > 6000) // wait for 10mins, every retry fail increase 1 min
				{
					// restart
					u32ErrorHappen_cntr = 0;
					// pwmSetDuty(hetRAM1, pwm1, 98);//most likely motor stuck here, need push motor
					// EEPROM_writeCounterData(u32TestCounter, u32TestCounter, COUNTER_EEPROM_ADD);
					// vDelay_ticks(80000U);
					/* reset here */
					// sl_systemREG1->SYSECR = ((uint32)0x2u << 14u);
					blflag_reset = true;
					flag_motor_error = false;
					first_reset = true;
					blflag_speed_check = false;
					u32ResetTimer_ms = u32SystemTimer_1ms;
					i32EncPulse_cntr = 0;
					u32eeprom_timer_1ms = 0U;
				}
			}

			u32MotorEncPosition = 0;
			u32SpeedTimer_ms = u32SystemTimer_1ms;
			// EEPROM_writeCounterData(u32TestCounter, u32SpeedAve, COUNTER_EEPROM_ADD);
		}

		if (first_reset)
		{

			if (u32GetTimeSliceDuration_ms(u32ResetTimer_ms) < 1000U)
			{
				gioSetBit(gioPORTA, 3, PIN_HIGH); // move forward
				pwmSetDuty(hetRAM1, pwm1, 90);	  // set duty cycle to 99//
			}
			else if ((u32GetTimeSliceDuration_ms(u32ResetTimer_ms) < 2001U)) // let motor run at least 1s at 100% pwm
			{
				gioSetBit(gioPORTA, 3, PIN_LOW); // move backward
				pwmSetDuty(hetRAM1, pwm1, 90);	 // set duty cycle to 99//

				if ((u32GetTimeSliceDuration_ms(u32ResetTimer_ms) > 1200U) && !blflag_speed_check)
				{
					if (u32Speed_rpm < MIN_CONSTSPEED_MOTOR_SPEED_RPM)
					{
						// blfag_stop_reset = true; // motor still stuck or no power, no need reset
						flag_motor_error = true;
					}
					blflag_speed_check = true;
				}
			}
			else // both backward and forward done
			{

				pwmSetDuty(hetRAM1, pwm1, 0); // set duty cycle to 0//
				pwmSetDuty(hetRAM1, pwm1, 0); // set duty cycle to 0//
				first_reset = false;		  // go to normal procedure
				bl_tick_move_forward_time = true;
				motor_forward = true;
				bl_tick_move_backward_time = false;
				max_pos_flag = false;
			}
		}
		/***************************************************Start of Motor One code***************************************************/
		else
		{
			// adc_convert();//get adc value for PWM duty cycle & battery //
			// blflag_speed_check = false;
			if (flag_switch_on)
			{
				if (!flag_motor_error) // If no motor error
				{
					if (motor_forward)
					{
						// Motor_move_forward_OpenLoop();
						//  Motor_move_forward_torque();
						if (false == max_pos_flag)
						{
							Motor_move_forward_pulse();
						}
						else
						{
							if ((u32GetTimeSliceDuration_ms(u32move_forward_waiting_ms) > 1000U)) // delay for 1000ms
							{
								motor_forward = false;
								blflag_speed_check = false; // prepare for next check
								max_pos_flag = false;
								bl_tick_move_backward_time = true;
								// nowtime_ms = u32GetTime_ms();
							}
						}
					}
					else
					{
						// Motor_move_backward_OpenLoop();
						// Motor_move_backward_torque();
						if (false == max_pos_flag)
						{
							Motor_move_backward_pulse();
						}
						else // max position reached, set flag for next cycle
						{
							if ((u32GetTimeSliceDuration_ms(u32move_backward_waiting_ms) > MOTOR_COOLING_TIME_MS)) // motor cooling
							{
								motor_forward = true;
								blflag_speed_check = false; // prepare for next check
								max_pos_flag = false;
								bl_tick_move_forward_time = true;
								// nowtime_ms = u32GetTime_ms();
								i32EncPulse_cntr = 0;
								updateLED_flag = true;
								if ((u32TestCounter_new % 12U) == 0) // every 12 cyles save once
								{
									EEPROM_writeCounterData(u32TestCounter_new, u32TestCounter_new, COUNTER_EEPROM_ADD);
									vDelay_ticks(800U);
								}
							}
						}
					}

					save_eeprom = true;
				}
				else
				{
					pwmSetDuty(hetRAM1, pwm1, 0); // set duty cycle to 0//
					pwmSetDuty(hetRAM1, pwm1, 0); // set duty cycle to 0//
					motor_stop = true;
				}
			}
			else
			{
				if (save_eeprom)
				{
					// EEPROM_writeCounterData(u32TestCounter_new, u32TestCounter_new, COUNTER_EEPROM_ADD);
					// vDelay_ticks(8000U);
					save_eeprom = false;
					pwmSetDuty(hetRAM1, pwm1, 0); // set duty cycle to 0//
					pwmSetDuty(hetRAM1, pwm1, 0); // set duty cycle to 0//
				}

				motor_stop = true;
			}

			if (updateLED_flag) //(max_pos_flag)
			{
				// Motor_status_Disp(i32EncPulse_cntr,u32TestCounter);
				updateLED_flag = false;
				// EEPROM_readCounterData(COUNTER_EEPROM_ADD,&u32ReadBack01, &u32TestCntr);
			}
			/***************************************************End of Motor One code***************************************************/

			/***************************************************Start of Motor Two code***************************************************/

			if (firstResetTwo)
			{
				if (u32GetTimeSliceDuration_ms(u32ResetTimer_ms) < 1000U)
				{
					gioSetBit(gioPORTA, 3, PIN_HIGH); // move forward
					pwmSetDuty(hetRAM1, pwm1, 90);	  // set duty cycle to 99//
				}
				else if ((u32GetTimeSliceDuration_ms(u32ResetTimer_ms) < 2001U)) // let motor run at least 1s at 100% pwm
				{
					SetMotorTwoDirection(BACKWARD);
					SetMotorTwoSpeed(99);
		

					if ((u32GetTimeSliceDuration_ms(u32ResetTimer_ms) > 1200U) && !blflag_speed_check)
					{
						if (u32Speed_rpm < MIN_CONSTSPEED_MOTOR_SPEED_RPM)
						{
							// blfag_stop_reset = true; // motor still stuck or no power, no need reset
							flag_motor_error = true;
						}
						blflag_speed_check = true;
					}
				}
				else // both backward and forward done
				{

					pwmSetDuty(hetRAM1, pwm1, 0); // set duty cycle to 0//
					pwmSetDuty(hetRAM1, pwm1, 0); // set duty cycle to 0//
					first_reset = false;		  // go to normal procedure
					bl_tick_move_forward_time = true;
					motor_forward = true;
					bl_tick_move_backward_time = false;
					max_pos_flag = false;
				}
				// Else not first reset
				else
				{
					// adc_convert();//get adc value for PWM duty cycle & battery //
					// blflag_speed_check = false;
					if (flag_switch_on)
					{
						if (!flag_motor_error) // If no motor error
						{
							if (motor_forward)
							{
								// Motor_move_forward_OpenLoop();
								//  Motor_move_forward_torque();
								if (false == max_pos_flag)
								{
									Motor_move_forward_pulse();
								}
								else
								{
									if ((u32GetTimeSliceDuration_ms(u32move_forward_waiting_ms) > 1000U)) // delay for 1000ms
									{
										motor_forward = false;
										blflag_speed_check = false; // prepare for next check
										max_pos_flag = false;
										bl_tick_move_backward_time = true;
										// nowtime_ms = u32GetTime_ms();
									}
								}
							}
							else
							{
								// Motor_move_backward_OpenLoop();
								// Motor_move_backward_torque();
								if (false == max_pos_flag)
								{
									Motor_move_backward_pulse();
								}
								else // max position reached, set flag for next cycle
								{
									if ((u32GetTimeSliceDuration_ms(u32move_backward_waiting_ms) > MOTOR_COOLING_TIME_MS)) // motor cooling
									{
										motor_forward = true;
										blflag_speed_check = false; // prepare for next check
										max_pos_flag = false;
										bl_tick_move_forward_time = true;
										// nowtime_ms = u32GetTime_ms();
										i32EncPulse_cntr = 0;
										updateLED_flag = true;
										if ((u32TestCounter_new % 12U) == 0) // every 12 cyles save once
										{
											EEPROM_writeCounterData(u32TestCounter_new, u32TestCounter_new, COUNTER_EEPROM_ADD);
											vDelay_ticks(800U);
										}
									}
								}
							}

							save_eeprom = true;
						}
						else
						{
							pwmSetDuty(hetRAM1, pwm1, 0); // set duty cycle to 0//
							pwmSetDuty(hetRAM1, pwm1, 0); // set duty cycle to 0//
							motor_stop = true;
						}
					}
					else
					{
						if (save_eeprom)
						{
							// EEPROM_writeCounterData(u32TestCounter_new, u32TestCounter_new, COUNTER_EEPROM_ADD);
							// vDelay_ticks(8000U);
							save_eeprom = false;
							pwmSetDuty(hetRAM1, pwm1, 0); // set duty cycle to 0//
							pwmSetDuty(hetRAM1, pwm1, 0); // set duty cycle to 0//
						}

						motor_stop = true;
					}
				}

				/***************************************************End of Motor Two code***************************************************/
				vUpdateDisplay8Digit(u32TestCounter_new); // Display MotorOne counter on DisplayOne

				if (!flag_motor_error)
				{
					vUpdateDisplay8Digit_02(u32SpeedAve); // Display MotorOne RPM on Display2; display average speed --- 31Oct change
				}
				else
				{
					vUpdateDisplayError_02(); // Else display Error Message on Display2
				}
			}
		}

		if ((u32GetTimeSliceDuration_ms(u32move_backward_waiting_ms) > MOTOR_COOLING_TIME_MS)) // motor cooling
		{
			motor_forward = true;
			blflag_speed_check = false; // prepare for next check
			max_pos_flag = false;
			bl_tick_move_forward_time = true;
			// nowtime_ms = u32GetTime_ms();
			i32EncPulse_cntr = 0;
			updateLED_flag = true;
			if ((u32TestCounter_new % 12U) == 0) // every 12 cyles save once
			{
				EEPROM_writeCounterData(u32TestCounter_new, u32TestCounter_new, COUNTER_EEPROM_ADD);
				vDelay_ticks(800U);
			}
		}
	}

	/* USER CODE BEGIN (4) */
	void rtiNotification(rtiBASE_t * rtiREG, uint32 notification)
	{
		// gioSetPort(gioPORTB, gioGetPort(gioPORTB) ^ 0b00001000);		// Toggle GIOB3
		u32SystemTimer_1ms++;
		u32eeprom_timer_1ms++;
		vCheckSwitchStatus();
	}

	/*for testing*/

	/****************Not used only for testing****************/
	/*
	pwmSetDuty(hetRAM1, pwm2, 50U);//set duty cycle and porting to pin NHET00//
	uRotary = getRotaryPosition();//Initialize rotary//
	uRotaryLastVal = uRotary;//Initialize rotary//
	*/
	/****************Not used only for testing****************/

	// Var_Disp(rd_data[0]);

	// send data through can bus//
	// can_transmit();//can_bus transmit//
	// can_receive();//can_bus receive//
	// can_rx_disp(can_datrx);//display data receive//

	// Light_Sens_Disp();//Display Light Sensor Reading//
	// Temp_Sens_Disp();//Display Temperatur Sensor Reading//

	// pwmSetDuty(hetRAM1, pwm1, Duty1);
	// CheckRotary();//read encoder//
	// Encod_Disp();//Display Encoder Reading//
	/*********/

	/*for testing*/
	// pos=getPWM(1,1);//get position//
	// Pin_Disp(pin_bit);//Display Pin PWM counter//
	/********/
