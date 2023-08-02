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
 * 1. 
 * 2. 
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
#include "HL_sci.h"
#include "HL_sys_core.h"
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

//Speed Config
#define BACKWARD 0U
#define STOP 0U
#define FORWARD 1U
#define SLOW 40U
#define FAST 70U
#define FIRST_RESET_SPEED 99U
#define NORMAL_SPEED 62U
#define MIN_CONSTSPEED_MOTOR_SPEED_RPM 100U

//Timing Config (In terms of 100ms cycles)
#define FIRST_RESET_RUNTIME 30U    // 3 sec
#define FORWARD_PHASE_RUNTIME 30U  // 3 secs
#define NORMAL_PHASE_PAUSETIME 10U // 1 sec
#define BACKWARD_PHASE_RUNTIME 35U // 3.5 secs
#define END_CYCLE_PAUSETIME 50U    // 5 secs
#define ERROR_CHECK_TIME 10U       // 1 sec
#define ERROR_COOLDOWN_TIME 600U   // 60 secs

// Motor RPM variables
#define PULSES_PER_ROTATION 12U
#define TIMER_CYCLES_PER_MINUTE 600U // 600*100ms = 1min

// EEPROM variables
#define EEPROM_SAVING_CYCLE_INTERVAL 12U // Save to EEPROM every 12 cycles
#define MAIN_COUNTERS_TOTAL_BYTE_SIZE 8U
const int mainCountersAddress = 0x1U;
const int mainCounterAddressOffset = 0U;
const int mainCounterByteSize = 4U;
const int mainCountersTotalByteSize = MAIN_COUNTERS_TOTAL_BYTE_SIZE;

enum MotorState {
    STATE_FIRST_RESET = 1,
    STATE_FORWARD_PHASE,
    STATE_NORMAL_PHASE_PAUSE,
    STATE_BACKWARD_PHASE,
    STATE_END_OF_CYCLE_PAUSE,
    STATE_ERROR,
};


uint32_t mainCounterOne = 0;
uint32_t mainCounterTwo = 0;

MotorState stateMotorOne = STATE_FIRST_RESET;
MotorState stateMotorTwo = STATE_FIRST_RESET;
volatile bool startMotorOneTimerFlag = false;
volatile bool startMotorTwoTimerFlag = false;
volatile int motorOneTimer = 0;
volatile int motorTwoTimer = 0;
bool pauseMotorsFlag = false;
bool saveToEERPOMFlag = false;

uint16 u16JobResult, Status;
volatile long encoderOneCounter = 0U;
volatile long encoderTwoCounter = 0U;
volatile int twoSecondTimerLimit = 20U; // Number of 100ms cycles
volatile int rpmOne = 0U;
volatile int rpmTwo = 0U;
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

#define UART sciREG1
#define BREAKSIZE 2
uint8 BREAK[BREAKSIZE] = {'\n', '\r'};

#define TSIZE4 5
uint8_t TEXT4[TSIZE4] = {'R', 'P', 'M', ':', ' '};

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

void delay(void)
{
    unsigned int dummycnt = 0x0000FFU;
    do
    {
        dummycnt--;
    } while (dummycnt > 0);
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
        delay();
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
        delay();
        gioSetBit(LED_PORT, CLOCK_PIN, HIGH);
        Digit <<= 1;
        delay();
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

        delay();
        gioSetBit(LED_PORT, CLOCK2_PIN, HIGH);
        Digit <<= 1;
        delay();
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

        delay();
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
        DisplayDigit_02(digits_CommAnode[au8Digit]);
        DisplayDigit_02(Segment_CommAnode[index]);
        gioSetBit(LED_PORT, LATCH2_PIN, HIGH);

        delay();
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

        delay();
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

        delay();
    }
}

void CheckSwitchStatus(void)
{
    if (gioGetBit(MOTORSWITCHPORT, MOTORSWITCH) == HIGH)
    {
        delay();
        if (gioGetBit(MOTORSWITCHPORT, MOTORSWITCH) == HIGH)
        {
            pauseMotorsFlag = false;
        }
    }
    else if (gioGetBit(MOTORSWITCHPORT, MOTORSWITCH) == LOW)
    {
        delay();
        if (gioGetBit(MOTORSWITCHPORT, MOTORSWITCH) == LOW)
        {
            pauseMotorsFlag = true;
        }
    }
}

void sciPrintDecimal(sciBASE_t *sci, uint32_t value)
{
    char buffer[11]; // Buffer to hold the string representation of the decimal value
    int i = 0;
    snprintf(buffer, 11, "%lu", value); // Convert the uint32 value to a string

    // Print each character of the string until the null terminator
    for (i = 0; buffer[i] != '\0'; i++)
    {
        while ((sci->FLR & 0x4) == 4)
            ;                        // Wait until busy
        sciSendByte(sci, buffer[i]); // Send out text
    }
    i = 0;
}

void sciDisplayText(sciBASE_t *sci, uint8 *text, uint32 length)
{
    while (length--)
    {
        while ((UART->FLR & 0x4) == 4)
            ;                       /* wait until busy */
        sciSendByte(UART, *text++); /* send out text   */
    };
}

void wait(uint32 time)
{
    time--;
}


/* USER CODE END */

int main(void)
{
    /* USER CODE BEGIN (3) */
    hetInit();                                                // Initialize PWM High End Timer Module
    gioInit();                                                // Initialize GIO Module
    _enable_IRQ();                                            // Enable interrupt vector
    gioEnableNotification(gioPORTA, MOTORONEENCODERPIN);      // Enable Pin interrupt
    gioEnableNotification(gioPORTA, MOTORTWOENCODERPIN);      // Enable Pin interrupt


    sciInit(); /* initialize sci/sci-lin    */
               /* even parity , 2 stop bits */

    StartMotorsPWM(); // Start PWM output & motor

    while (1)
    {
        


    } // end of main while loop

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
    //Every 100ms

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
    else
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
