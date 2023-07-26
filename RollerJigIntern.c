/** @file HL_sys_main.c
 *   @brief Application main file
 *   @date 30-06-2023
 *   @version 1.0
 *   Author: SIT Internship Team
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



/* USER CODE BEGIN (0) */
/**************************************************NOTES***************************************************************
 * Minimum motor PWM to run: 40%
 * Variables saved to EERPOM: mainCounterOne, mainCounterTwo (4 bytes each)
 *
 *
 * Outstanding things to do:
 * 1. Implement Pause switch
 * 2. Test 7-segment display code
 * 
 *********************************************************************************************************************/
// Include Files
#include "HL_het.h"
#include "HL_sys_common.h"
#include "math.h"
#include "stdlib.h"
#include "HL_gio.h"
#include "HL_system.h"
#include "HL_rti.h"
#include "ti_fee.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
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

// Display Pinout & variables
#define CLOCK_PIN 0U
#define LATCH_PIN 1U
#define DATA_PIN 2U
#define CLOCK2_PIN 3U
#define LATCH2_PIN 4U
#define DATA2_PIN 5U
#define LED_PORT gioPORTB
#define E_SIGN_IND 14U
#define DIG_OFF_IND 16U
#define DASH_SIGN_IND 17U

// Motor Speed, Direction & Encoder variables
#define BACKWARD 0U
#define STOP 0U
#define FORWARD 1U
#define SLOW 40U
#define FAST 70U
#define FIRST_RESET_SPEED 90U
#define NORMAL_SPEED 70U
#define FIRST_RESET_RUNTIME 100U       // 10 secs
#define FORWARD_PHASE_RUNTIME 30U      // 3 secs
#define BACKWARD_PHASE_RUNTIME 35U     // 3.5 secs
#define NORMAL_PHASE_PAUSETIME 10U     // 1 sec
#define ERROR_CHECK_TIME 10U           // 1 sec
#define END_CYCLE_PAUSETIME 50U        // 5 secs
#define ERROR_COOLDOWN_TIME 600U       // 60 secs

// Motor RPM variables
#define PULSES_PER_ROTATION 12U
#define TIMER_CYCLES_PER_MINUTE 60U // 60*100ms = 1min
#define MIN_CONSTSPEED_MOTOR_SPEED_RPM 200U
#define EEPROM_SAVING_CYCLE_INTERVAL 12U // Save to EEPROM every 12 cycles

//EEPROM variables
#define MAIN_COUNTERS_TOTAL_BYTE_SIZE 8U
const int mainCountersAddress = 0x1U;
const int mainCounterAddressOffset = 0U;
const int mainCounterByteSize = 4U;
const int mainCountersTotalByteSize = MAIN_COUNTERS_TOTAL_BYTE_SIZE;

unsigned int mainCounterOne = 0;
unsigned int mainCounterTwo = 0;

int stateMotorOne = 1;
int stateMotorTwo = 1;
volatile bool startMotorOneTimerFlag = false;
volatile bool startMotorTwoTimerFlag = false;
volatile int motorOneTimer = 0;
volatile int motorTwoTimer = 0;

uint16 u16JobResult, Status;
volatile long encoderOneCounter = 0U;
volatile long encoderTwoCounter = 0U;
volatile int twoSecondTimerLimit = 20U; // Number of 100ms cycles
volatile int rpmOne = 0U;
volatile int rpmTwo = 0U;
volatile bool motorOneErrorFlag = false;
volatile bool motorTwoErrorFlag = false;
int eepromFlush = 0;

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
    uint8_t loadBufferArray[MAIN_COUNTERS_TOTAL_BYTE_SIZE] = {0};
    // Loads both main counters from same block address
    TI_Fee_ReadSync(mainCountersAddress, mainCounterAddressOffset, (uint8_t *)loadBufferArray, mainCountersTotalByteSize);
    memcpy(&mainCounterOne, loadBufferArray, mainCounterByteSize);                       // Extract first 4 bytes for mainCounterOne
    memcpy(&mainCounterTwo, loadBufferArray + mainCounterByteSize, mainCounterByteSize); // Extract next 4 bytes for mainCounterTwo
}

void saveCountersToEEPROM(void)
{
    uint8_t writeBufferArray[MAIN_COUNTERS_TOTAL_BYTE_SIZE] = {0};
    memcpy(writeBufferArray, &mainCounterOne, mainCounterByteSize);
    memcpy(writeBufferArray + mainCounterByteSize, &mainCounterTwo, mainCounterByteSize);
    TI_Fee_WriteSync(mainCountersAddress, (uint8_t *)writeBufferArray);
}

void flushEEPROM(void)
{
    uint8_t flushBufferArray[MAIN_COUNTERS_TOTAL_BYTE_SIZE] = {0};
    memcpy(flushBufferArray, &eepromFlush, mainCounterByteSize);
    memcpy(flushBufferArray + mainCounterByteSize, &eepromFlush, mainCounterByteSize);
    TI_Fee_WriteSync(mainCountersAddress, (uint8_t *)flushBufferArray);
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
void printCounterDisplayOne(uint32_t u32Num)
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
void printCounterDisplayTwo(uint32_t u32Num)
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

        gioSetBit(LED_PORT, LATCH2_PIN, LOW);
        DisplayDigit(digits_CommAnode[au8Digit]);
        DisplayDigit(Segment_CommAnode[index]);
        gioSetBit(LED_PORT, LATCH2_PIN, HIGH);

        vDelay_ticks(100);
    }
    firstzero = true;
}

void displayErrorMotorOne(void)
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

void displayErrorMotorTwo(void)
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

/* USER CODE END */

int main(void)
{
    /* USER CODE BEGIN (3) */
    EEPROMInit();
    hetInit();                                                // Initialize PWM High End Timer Module
    rtiInit();                                                // Initialize Timer Module
    gioInit();                                                // Initialize GIO Module
    _enable_IRQ();                                            // Enable interrupt vector
    gioEnableNotification(gioPORTA, MOTORONEENCODERPIN);      // Enable Pin interrupt
    gioEnableNotification(gioPORTA, MOTORTWOENCODERPIN);      // Enable Pin interrupt
    rtiEnableNotification(rtiREG1, rtiNOTIFICATION_COMPARE0); // Enable timer interrupt
    rtiStartCounter(rtiREG1, rtiCOUNTER_BLOCK0);              // Start timer module

    // flushEEPROM();
    loadCountersFromEEPROM();
    //  Start PWM output & motor
    StartMotorsPWM();

    while (1)
    {
        switch (stateMotorOne)
        {
        case 1:
            /****************************First Reset State****************************/

            startMotorOneTimerFlag = true;

            /*********Run motor forward for 10 secs*********/
            if (motorOneTimer <= FIRST_RESET_RUNTIME)
            {
                SetMotorOneDirection(FORWARD);
                SetMotorOneSpeed(FIRST_RESET_SPEED);

                /*********Start error checking 1 sec later*********/
                if (motorOneTimer > ERROR_CHECK_TIME)
                {
                    // Compare RPM for Error
                    if (rpmOne < MIN_CONSTSPEED_MOTOR_SPEED_RPM)
                    {
                        motorOneErrorFlag = true;
                        stateMotorOne = 6;
                    }
                }
            }

            /*********Run motor backward for 10 secs*********/
            else if (motorOneTimer > FIRST_RESET_RUNTIME && motorOneTimer <= (FIRST_RESET_RUNTIME + FIRST_RESET_RUNTIME))
            {
                SetMotorOneDirection(BACKWARD);
                SetMotorOneSpeed(FIRST_RESET_SPEED);

                /*********Start error checking 1 sec later*********/
                if (motorOneTimer > (FIRST_RESET_RUNTIME + ERROR_CHECK_TIME))
                {
                    // Compare RPM for Error
                    if (rpmOne < MIN_CONSTSPEED_MOTOR_SPEED_RPM)
                    {
                        motorOneErrorFlag = true;
                        stateMotorOne = 6;
                    }
                }
            }

            /*********First Reset Done*********/
            else
            {
                startMotorOneTimerFlag = false;
                motorOneTimer = 0;
                SetMotorOneSpeed(STOP);
                stateMotorOne = 2; // Transition to next state
            }
            break;

        case 2:
            /***************************Motor Forward State***************************/

            startMotorOneTimerFlag = true;

            /*********Run motor forward for 3 secs*********/
            if (motorOneTimer <= FORWARD_PHASE_RUNTIME)
            {
                SetMotorOneDirection(FORWARD);
                SetMotorOneSpeed(NORMAL_SPEED);
                printCounterDisplayOne(mainCounterOne);
                /*********Start error checking 1 sec later*********/
                if (motorOneTimer > ERROR_CHECK_TIME)
                {
                    // Compare RPM for Error
                    if (rpmOne < MIN_CONSTSPEED_MOTOR_SPEED_RPM)
                    {
                        motorOneErrorFlag = true;
                        stateMotorOne = 6;
                    }
                }
            }
            else
            {
                startMotorOneTimerFlag = false;
                motorOneTimer = 0;
                stateMotorOne = 3; // Transition to next state
            }

            break;

        case 3:
            /*************************Motor 1 sec Pause State*************************/

            startMotorOneTimerFlag = true;

            if (motorOneTimer <= NORMAL_PHASE_PAUSETIME)
            {
                SetMotorOneSpeed(STOP);
            }
            else
            {
                startMotorOneTimerFlag = false;
                motorOneTimer = 0;
                stateMotorOne = 4; // Transition to next state
            }
            break;

        case 4:
            /***************************Motor Backward State***************************/

            startMotorOneTimerFlag = true;

            /*********Run motor backward for 3.5 secs*********/
            if (motorOneTimer <= BACKWARD_PHASE_RUNTIME)
            {
                SetMotorOneDirection(BACKWARD);
                SetMotorOneSpeed(NORMAL_SPEED);
                /*********Start error checking 1 sec later*********/
                if (motorOneTimer > ERROR_CHECK_TIME)
                {
                    // Compare RPM for Error
                    if (rpmOne < MIN_CONSTSPEED_MOTOR_SPEED_RPM)
                    {
                        motorOneErrorFlag = true;
                        stateMotorOne = 6;
                    }
                }
            }
            else
            {
                startMotorOneTimerFlag = false;
                motorOneTimer = 0;
                stateMotorOne = 5; // Transition to next state
                mainCounterOne++;
            }
            break;

        case 5:
            /*************************Motor 5 sec Pause State*************************/
            startMotorOneTimerFlag = true;
            if (motorOneTimer <= END_CYCLE_PAUSETIME)
            {
                SetMotorOneSpeed(STOP);
            }
            else
            {
                startMotorOneTimerFlag = false;
                motorOneTimer = 0;
                stateMotorOne = 2; // Transition to next state
            }
            break;

        case 6:
            /***************************Motor Error State***************************/
            startMotorOneTimerFlag = true;

            if (motorOneTimer <= ERROR_COOLDOWN_TIME)
            {
                SetMotorOneSpeed(STOP);
                displayErrorMotorOne();
            }
            else
            {
                startMotorOneTimerFlag = false;
                motorOneTimer = 0;
                stateMotorOne = 1; // Transition to next state

                motorOneErrorFlag = false;
            }

            break;

        default:
            break;
        }

        switch (stateMotorTwo)
        {
        case 1:
            /****************************First Reset State****************************/

            startMotorTwoTimerFlag = true;

            /*********Run motor forward for 10 secs*********/
            if (motorTwoTimer <= FIRST_RESET_RUNTIME)
            {
                SetMotorTwoDirection(FORWARD);
                SetMotorTwoSpeed(FIRST_RESET_SPEED);

                /*********Start error checking 1 sec later*********/
                if (motorTwoTimer > ERROR_CHECK_TIME)
                {
                    // Compare RPM for Error
                    if (rpmTwo < MIN_CONSTSPEED_MOTOR_SPEED_RPM)
                    {
                        motorTwoErrorFlag = true;
                        stateMotorTwo = 6;
                    }
                }
            }

            /*********Run motor backward for 10 secs*********/
            else if (motorTwoTimer > FIRST_RESET_RUNTIME && motorTwoTimer <= (FIRST_RESET_RUNTIME + FIRST_RESET_RUNTIME))
            {
                SetMotorTwoDirection(BACKWARD);
                SetMotorTwoSpeed(FIRST_RESET_SPEED);

                /*********Start error checking 1 sec later*********/
                if (motorTwoTimer > (FIRST_RESET_RUNTIME + ERROR_CHECK_TIME))
                {
                    // Compare RPM for Error
                    if (rpmTwo < MIN_CONSTSPEED_MOTOR_SPEED_RPM)
                    {
                        motorTwoErrorFlag = true;
                        stateMotorTwo = 6;
                    }
                }
            }

            /*********First Reset Done*********/
            else
            {
                startMotorTwoTimerFlag = false;
                motorTwoTimer = 0;
                SetMotorTwoSpeed(STOP);
                stateMotorTwo = 2; // Transition to next state
            }
            break;

        case 2:
            /***************************Motor Forward State***************************/

            startMotorTwoTimerFlag = true;

            /*********Run motor forward for 3 secs*********/
            if (motorTwoTimer <= FORWARD_PHASE_RUNTIME)
            {
                SetMotorTwoDirection(FORWARD);
                SetMotorTwoSpeed(NORMAL_SPEED);
                printCounterDisplayTwo(mainCounterTwo);
                /*********Start error checking 1 sec later*********/
                if (motorTwoTimer > ERROR_CHECK_TIME)
                {
                    // Compare RPM for Error
                    if (rpmTwo < MIN_CONSTSPEED_MOTOR_SPEED_RPM)
                    {
                        motorTwoErrorFlag = true;
                        stateMotorTwo = 6;
                    }
                }
            }
            else
            {
                startMotorTwoTimerFlag = false;
                motorTwoTimer = 0;
                stateMotorTwo = 3; // Transition to next state
            }

            break;

        case 3:
            /*************************Motor 1 sec Pause State*************************/

            startMotorTwoTimerFlag = true;

            if (motorTwoTimer <= NORMAL_PHASE_PAUSETIME)
            {
                SetMotorTwoSpeed(STOP);
            }
            else
            {
                startMotorTwoTimerFlag = false;
                motorTwoTimer = 0;
                stateMotorTwo = 4; // Transition to next state
            }
            break;

        case 4:
            /***************************Motor Backward State***************************/

            startMotorTwoTimerFlag = true;

            /*********Run motor backward for 3.5 secs*********/
            if (motorTwoTimer <= BACKWARD_PHASE_RUNTIME)
            {
                SetMotorTwoDirection(BACKWARD);
                SetMotorTwoSpeed(NORMAL_SPEED);
                /*********Start error checking 1 sec later*********/
                if (motorTwoTimer > ERROR_CHECK_TIME)
                {
                    // Compare RPM for Error
                    if (rpmTwo < MIN_CONSTSPEED_MOTOR_SPEED_RPM)
                    {
                        motorTwoErrorFlag = true;
                        stateMotorTwo = 6;
                    }
                }
            }
            else
            {
                startMotorTwoTimerFlag = false;
                motorTwoTimer = 0;
                stateMotorTwo = 5; // Transition to next state
                mainCounterTwo++;
            }
            break;

        case 5:
            /*************************Motor 5 sec Pause State*************************/
            startMotorTwoTimerFlag = true;
            if (motorTwoTimer <= END_CYCLE_PAUSETIME)
            {
                SetMotorTwoSpeed(STOP);
            }
            else
            {
                startMotorTwoTimerFlag = false;
                motorTwoTimer = 0;
                stateMotorTwo = 2; // Transition to next state
            }
            break;

        case 6:
            /***************************Motor Error State***************************/
            startMotorTwoTimerFlag = true;

            if (motorTwoTimer <= ERROR_COOLDOWN_TIME)
            {
                SetMotorTwoSpeed(STOP);
                displayErrorMotorTwo();
            }
            else
            {
                startMotorTwoTimerFlag = false;
                motorTwoTimer = 0;
                stateMotorTwo = 1; // Transition to next state

                motorTwoErrorFlag = false;
            }

            break;

        default:
            break;
        }

        // save counters to EEPROM every 12 cycles
        if (((mainCounterOne % EEPROM_SAVING_CYCLE_INTERVAL) == 0) || ((mainCounterTwo % EEPROM_SAVING_CYCLE_INTERVAL) == 0))
        {
            saveCountersToEEPROM();
        }
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

    /************************RPM Calculation************************/
    volatile int numberOfRotationOne = encoderOneCounter / PULSES_PER_ROTATION;
    volatile int numberOfRotationTwo = encoderTwoCounter / PULSES_PER_ROTATION;
    rpmOne = TIMER_CYCLES_PER_MINUTE * numberOfRotationOne;
    rpmTwo = TIMER_CYCLES_PER_MINUTE * numberOfRotationTwo;
    encoderOneCounter = 0;
    encoderTwoCounter = 0;
    /**************************************************************/

    if (startMotorOneTimerFlag)
    {
        motorOneTimer++;
    }
    else if (startMotorOneTimerFlag == false)
    {
        motorOneTimer = 0;
    }

    if (startMotorTwoTimerFlag)
    {
        motorTwoTimer++;
    }
    else
    {
        motorTwoTimer = 0;
    }
    vCheckSwitchStatus();
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
