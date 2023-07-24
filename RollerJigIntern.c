/** @file HL_sys_main.c
 *   @brief Application main file
 *   @date 30=06-2023
 *   @version 1.0
 *
 *   This file contains an empty main function,
 *   which can be used for the application.
 */

/*
 * Copyright (C) 2009-2018 Texas Instruments Incorporated - www.ti.com
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**************************************************NOTES***************************************************************
 * Minimum motor PWM: 40%
 * Variables to save to EERPOM: mainCounterOne, mainCounterTwo (4 bytes each)
 *
 *
 *********************************************************************************************************************/

/* USER CODE BEGIN (0) */
// Include Files
#include "HL_het.h"
#include "HL_sys_common.h"
#include "math.h"
#include "stdlib.h"
#include "HL_gio.h"
#include "HL_system.h"
#include "HL_rti.h"
#include "ti_fee.h"
/* USER CODE END */

/* Include Files */

#include "HL_sys_common.h"

/* USER CODE BEGIN (1) */

// Pin Output States
#define LOW 0U
#define HIGH 1U

// Motor One Pins
#define MOTORONEENCODERPIN 1U
#define MOTORONEDIRPIN 3U
#define MOTORONEPWMPIN 18U
#define MOTORONEPORT gioPORTA

// Motor Two Pins
#define MOTORTWOENCODERPIN 5U
#define MOTORTWODIRPIN 7U
#define MOTORTWOPWMPIN 19U
#define MOTORTWOPORT gioPORTA

// Motor Pause Switch Pin
#define MOTORSWITCH 0U
#define MOTORSWITCHPORT gioPORTA

// DisplayOne Pinout
#define CLOCK_PIN 0U
#define LATCH_PIN 1U
#define DATA_PIN 2U
#define LED_PORT gioPORTB

// DisplayTwo Pinout
#define CLOCK2_PIN 3U
#define LATCH2_PIN 4U
#define DATA2_PIN 5U

#define E_SIGN_IND 14U
#define DIG_OFF_IND 16U
#define DASH_SIGN_IND 17U

// Motor Speed, Direction & Encoder variables
#define BACKWARD 0U
#define STOP 0U
#define FORWARD 1U
#define SLOW 40U
#define FAST 70U
#define PULSES_PER_ROTATION 12U
#define TIMER_CYCLES_PER_MINUTE 60U // 60*100ms = 1min

#define DELAY_1MS_TICKS 27273 // The For loop takes 11 MC, hence 300M/11 = 27272727 MC for 1 sec

const int mainCountersAddress = 0x1;
const int mainCountersTotalByteSize = 8U;

unsigned int mainCounterOne = 0;
unsigned int mainCounterTwo = 0;

int stateMotorOne = 1;
volatile bool startMotorOneTimer = false;
volatile int motorOneTimer = 0;

uint16 u16JobResult, Status;
Std_ReturnType oResult = E_OK;
volatile long encoderOneCounter = 0U;
volatile long encoderTwoCounter = 0U;
volatile int timerCounter = 0U;
volatile int twoSecondTimerLimit = 20U; // Number of 100ms cycles
volatile int rpmOne = 0U;
volatile int rpmTwo = 0U;
volatile bool motorErrorFlag = false;
int mainCounterAddressOffset = 0U;

uint8_t Segment_CommAnode[8] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
uint8_t digits_CommAnode[18] =
    {0xC0, 0xF9, 0xA4, 0xB0, // '0','1','2,'3',
     0x99, 0x92, 0x82, 0xF8, // '4','5','6,'7',
     0x80, 0x90, 0X88, 0x83, // '8','9','A,'B',
     0xC6, 0xA1, 0x86, 0x8E, // 'C','D','E,'F',
     0xFF, 0b10111111};
uint8 au8ErrorInfo[8] = {DASH_SIGN_IND, DASH_SIGN_IND, E_SIGN_IND, E_SIGN_IND, E_SIGN_IND, E_SIGN_IND, DASH_SIGN_IND, DASH_SIGN_IND};
uint8 au8SpeedData[8] = {DASH_SIGN_IND, DASH_SIGN_IND, 0, 0, 0, 0, DASH_SIGN_IND, DASH_SIGN_IND};

/* USER CODE END */

/** @fn void main(void)
 *   @brief Application main function
 *   @note This function is empty by default.
 *
 *   This function is called after startup.
 *   The user can use this function to implement the application.
 */

/* USER CODE BEGIN (2) */

// Function Definitions
void vDelay_ticks(uint32_t ticksBuffer)
{
    uint32_t delayTicks = ticksBuffer;

    // Check for valid function parameter input
    if (delayTicks < 0x01)
    {
        return;
    }
    // Delay implementation through decrement
    do
    {
        delayTicks--;
    } while (delayTicks > 0);
}

void SetMotorOneDirection(uint32 dirA)
{
    gioSetBit(MOTORONEPORT, MOTORONEDIRPIN, dirA);
}

void SetMotorTwoDirection(uint32 dirB)
{
    gioSetBit(MOTORTWOPORT, MOTORTWODIRPIN, dirB);
}

void SetMotorOneSpeed(uint32 spdA)
{
    pwmSetDuty(hetRAM1, pwm1, spdA);
}

void SetMotorTwoSpeed(uint32 spdB)
{
    pwmSetDuty(hetRAM1, pwm0, spdB);
}

void StartMotorsPWM(void)
{
    pwmStart(hetRAM1, pwm0); // MotorTwo PWM
    pwmStart(hetRAM1, pwm1); // MotorOne PWM
}

void EEPROMInit(void)
{
    TI_Fee_Init();
    /*Always wait after async EEPROM read or write till EEPROM is idle (Operation finished)*/
    do
    {
        TI_Fee_MainFunction();
        vDelay_ticks(300U);
        Status = TI_Fee_GetStatus(0);
    } while (Status != IDLE);
}

void loadCountersFromEEPROM(void)
{
    uint8_t loadBufferArray[mainCountersTotalByteSize] = {0};
    // Loads both main counters from same block address
    TI_Fee_ReadSync(mainCountersAddress, mainCounterAddressOffset, *loadBufferArray, mainCountersTotalByteSize);
    memcpy(mainCounterOne, loadBufferArray, 4U);     // Extract first 4 bytes for mainCounterOne
    memcpy(mainCounterTwo, loadBufferArray + 4, 4U); // Extract next 4 bytes for mainCounterTwo
}

void saveCountersToEEPROM(void)
{
    uint8_t writeBufferArray[mainCountersTotalByteSize] = {0};
    memcpy(writeBufferArray, &mainCounterOne, 4U);
    memcpy(writeBufferArray + 4, &mainCounterTwo, 4U);
    TI_Fee_WriteSync(mainCountersAddress, *writeBufferArray);
}

/*DisplayOne number display function*/
void DisplayDigit(uint8_t Digit)
{
    uint8_t i = 0;
    for (i = 0; i < 8; i++)
    {
        gioSetBit(LED_PORT, CLOCK_PIN, LOW);

        if ((0x80 & Digit))
        {
            gioSetBit(LED_PORT, DATA_PIN, HIGH);
        }
        else
        {
            gioSetBit(LED_PORT, DATA_PIN, LOW);
        }
        vDelay_ticks(5U);
        gioSetBit(LED_PORT, CLOCK_PIN, HIGH);
        Digit <<= 1;
        vDelay_ticks(60U);
    }
}

/*DisplayTwo number display function*/
void DisplayDigit_02(uint8_t Digit)
{
    uint8_t i = 0;

    for (i = 0; i < 8; i++)
    {
        gioSetBit(LED_PORT, CLOCK2_PIN, LOW);

        if ((0x80 & Digit))
        {
            gioSetBit(LED_PORT, DATA2_PIN, HIGH);
        }
        else
        {
            gioSetBit(LED_PORT, DATA2_PIN, LOW);
        }

        vDelay_ticks(5U);
        gioSetBit(LED_PORT, CLOCK2_PIN, HIGH);
        Digit <<= 1;
        vDelay_ticks(60U);
    }
}

/*DisplayOne number printing function*/
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
        gioSetBit(LED_PORT, LATCH_PIN, LOW);
        DisplayDigit(digits_CommAnode[au8Digit]);
        DisplayDigit(Segment_CommAnode[index]);
        gioSetBit(LED_PORT, LATCH_PIN, HIGH);

        vDelay_ticks(100);
    }
    firstzero = true;
}

/*DisplayTwo number printing function*/
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
        gioSetBit(LED_PORT, LATCH2_PIN, LOW);
        DisplayDigit_02(digits_CommAnode[au8SpeedData[index]]);
        DisplayDigit_02(Segment_CommAnode[index]);
        gioSetBit(LED_PORT, LATCH2_PIN, HIGH);
        // vDelay_ticks(3000U);
        vDelay_ticks(100);
    }
    firstzero = true;
}

void vUpdateDisplayError_01(void)
{
    uint8_t index = 0;

    for (index = 0; index < 8; index++)
    {
        gioSetBit(LED_PORT, LATCH_PIN, LOW);
        DisplayDigit(digits_CommAnode[au8ErrorInfo[index]]);
        DisplayDigit(Segment_CommAnode[index]);
        gioSetBit(LED_PORT, LATCH_PIN, HIGH);

        vDelay_ticks(100);
    }
}

void vUpdateDisplayError_02(void)
{
    uint8_t index = 0;

    for (index = 0; index < 8; index++)
    {
        gioSetBit(LED_PORT, LATCH2_PIN, LOW);
        DisplayDigit_02(digits_CommAnode[au8ErrorInfo[index]]);
        DisplayDigit_02(Segment_CommAnode[index]);
        gioSetBit(LED_PORT, LATCH2_PIN, HIGH);

        vDelay_ticks(100);
    }
}

/* USER CODE END */

int main(void)
{
    /* USER CODE BEGIN (3) */
   // EEPROMInit();
    hetInit();                                                // Initialize PWM High End Timer Module
    rtiInit();                                                // Initialize Timer Module
    gioInit();                                                // Initialize GIO Module
    _enable_IRQ();                                            // Enable interrupt vector
    gioEnableNotification(gioPORTA, MOTORONEENCODERPIN);      // Enable Pin interrupt
    gioEnableNotification(gioPORTA, MOTORTWOENCODERPIN);      // Enable Pin interrupt
    rtiEnableNotification(rtiREG1, rtiNOTIFICATION_COMPARE0); // Enable timer interrupt
    rtiStartCounter(rtiREG1, rtiCOUNTER_BLOCK0);              // Start timer module

    // Start PWM output & motor
    StartMotorsPWM();

    while (1)
    {
        switch (stateMotorOne)
        {
        case 1:
            // Motors forward state 3 sec
            SetMotorOneDirection(FORWARD);
            startMotorOneTimer = true;
            if (motorOneTimer <= 30)
            {
                SetMotorOneSpeed(70U);
            }
            else
            {
                startMotorOneTimer = false;
                motorOneTimer = 0;
                stateMotorOne = 2;
            }
            break;

        case 2:
            // 1 sec pause state
            startMotorOneTimer = true;
            if (motorOneTimer <= 10)
            {
                SetMotorOneSpeed(0U);
            }
            else
            {
                startMotorOneTimer = false;
                motorOneTimer = 0;
                stateMotorOne = 3;
            }
            break;

        case 3:
            // Motors backward state 3.5 sec
            SetMotorOneDirection(BACKWARD);
            startMotorOneTimer = true;
            if (motorOneTimer <= 35)
            {
                SetMotorOneSpeed(70U);
            }
            else
            {
                startMotorOneTimer = false;
                motorOneTimer = 0;
                stateMotorOne = 4;
            }
            break;

        case 4:
            // Motors 5 sec end-of-cycle pause state
            startMotorOneTimer = true;
            if (motorOneTimer <= 50)
            {
                SetMotorOneSpeed(0U);
            }
            else
            {
                startMotorOneTimer = false;
                motorOneTimer = 0;
                stateMotorOne = 1;
            }
            break;

        case 5:
            // Motor error state
            break;

        case 6:
            // Motor switch off state
            break;

        default:
            break;
        }

        // save counters to EEPROM every 12 cycles
    }

    /* USER CODE END */

    return 0;
}

/* USER CODE BEGIN (4) */

/**************************************************ISR Definition***************************************************/

void gioNotification(gioPORT_t *port, uint32 bit)
{
    if (bit == MOTORONEENCODERPIN)
    {
        encoderOneCounter++;
    }
    if (bit == MOTORTWOENCODERPIN)
    {
        encoderTwoCounter++;
    }
}

void rtiNotification(rtiBASE_t *rtiREG, uint32 notification)
{
    // Increments every 100ms
    timerCounter++;

    /************************RPM Calculation************************/
    volatile int numberOfRotationOne = encoderOneCounter / PULSES_PER_ROTATION;
    volatile int numberOfRotationTwo = encoderTwoCounter / PULSES_PER_ROTATION;
    rpmOne = TIMER_CYCLES_PER_MINUTE * numberOfRotationOne;
    rpmTwo = TIMER_CYCLES_PER_MINUTE * numberOfRotationTwo;
    encoderOneCounter = 0;
    encoderTwoCounter = 0;
    /**************************************************************/

    /***********************Two second timer***********************/
    if (timerCounter >= twoSecondTimerLimit)
    {

        twoSecondTimerLimit = 0U;
    }
    /**************************************************************/

    if (startMotorOneTimer)
    {
        motorOneTimer++;
    }
    else if(startMotorOneTimer == false)
    {
        motorOneTimer = 0;
    }
}

void esmGroupNotification(int bit)
{
    return;
}

void esmGroup2Notification(int bit)
{
    return;
}

/**************************************************End of ISR Definition***************************************************/
/* USER CODE END */
