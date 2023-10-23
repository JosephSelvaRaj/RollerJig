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
#define P_SIGN 0b10001100 //8C
#define A_SIGN 0b10001000 //88
#define U_SIGN 0b11000001 //C1
#define S_SIGN 0b10010010 //92
#define E_SIGN 0b10000110 //86
#define PAUSE 43210 //PAUSE
#define PAUSE_DIG_OFF 5U

// Speed Config
#define BACKWARD 0U
#define STOP 0U
#define FORWARD 1U
#define SLOW 40U
#define FAST 70U
#define FIRST_RESET_SPEED 99U
uint32_t MotorOneCW_SPEED = 80;
uint32_t MotorOneCCW_SPEED = 60;
uint32_t MotorTwoCW_SPEED = 80;
uint32_t MotorTwoCCW_SPEED = 60;
#define MIN_CONSTSPEED_MOTOR_SPEED_RPM 200U
#define MotorOneFULLCYCLE 1900U
#define MotorTwoFULLCYCLE 1900U


// Timing Config (In terms of 50ms cycles)
#define FIRST_RESET_RUNTIME 30U    // 1.5 sec
#define FORWARD_PHASE_RUNTIME 60U  // 3 secs
#define NORMAL_PHASE_PAUSETIME 20U // 1 sec
#define BACKWARD_PHASE_RUNTIME 70U // 3.5 secs
#define END_CYCLE_PAUSETIME 100U    // 5 secs
#define ERROR_CHECK_TIME 20U       // 1 sec
#define STOPERROR_CHECK_TIME 40U    // 2 sec
#define ERROR_COOLDOWN_TIME 12000U   // 10 Minutes(600,000 * 50ms)
#define PAUSE_SAVING 1200U           // 1 Minute
#define Blink_Pause 100U          //5 Seconds
#define Blink_Counter 200U            //10 Seconds

// Motor RPM variables
#define PULSES_PER_ROTATION 12U
#define TIMER_CYCLES_PER_MINUTE 1200U // 60,000*50ms = 1min

//PID variables
float64 kp = 0.005;
float64 ki = 0.0005;
int imax = 3000;
int imin = -3000;
int MotorOneCWtargetRPM = 2400;
int MotorOneCCWtargetRPM = 1800;
int MotorTwoCWtargetRPM = 2400;
int MotorTwoCCWtargetRPM = 1800;
int errorRPM = 0;
int errorTwoRPM = 0;
int pidPWM = 0;
int pidPWMTwo = 0;
int MotorOnecwmax = 80U;
int MotorOnecwmin = 60U;
int MotorOneccwmax = 80U;
int MotorOneccwmin = 60U;

int MotorTwocwmax = 80U;
int MotorTwocwmin = 60U;
int MotorTwoccwmax = 80U;
int MotorTwoccwmin = 60U;
int MotorOneCWierrorRPM = 0;
int MotorOneCCWierrorRPM = 0;
int MotorTwoCWierrorRPM = 0;
int MotorTwoCCWierrorRPM = 0;
int PI[10000];
int i = 0;
int pos[10000];
int j = 0;


//PID Flags
bool MotorOneCWPID = false;
bool MotorOneCCWPID = false;
bool MotorTwoCWPID = false;
bool MotorTwoCCWPID = false;
bool RPMPrint = true;
bool PIsave = false;

//Error Check Flags
bool MotorOneCheck = false;
bool MotorTwoCheck = false;

//Error Check Variables
int MotorOneErrorCounter = 0;
int MotorTwoErrorCounter = 0;

//Motor Motion/Position
bool MotorOneMotion = false;
bool MotorTwoMotion = false;
bool MotorOneForward = false;
bool MotorOneBackward = false;
bool MotorTwoForward = false;
bool MotorTwoBackward = false;
volatile long MotorOnePosition = 0U;
volatile long MotorTwoPosition = 0U;

//Pause Timer Variable
volatile int PauseTimer = 0;
volatile int BlinkTimer =0;
bool Pause = false;

// EEPROM variables
#define EEPROM_SAVING_CYCLE_INTERVAL 12U // Save to EEPROM every 12 cycles
#define MAIN_COUNTERS_TOTAL_BYTE_SIZE 8U
const int mainCountersAddress = 0x3;
const int mainCounterAddressOffset = 0U;
const int mainCounterByteSize = 4U;
const int mainCountersTotalByteSize = MAIN_COUNTERS_TOTAL_BYTE_SIZE;

enum MotorState
{
    STATE_FIRST_RESET = 1,
    STATE_FORWARD_PHASE,
    STATE_NORMAL_PHASE_PAUSE,
    STATE_BACKWARD_PHASE,
    STATE_END_OF_CYCLE_PAUSE,
    STATE_ERROR,
};

uint32_t mainCounterOne = 0;
uint32_t mainCounterTwo = 0;
int MotorOneEepromCounter = 0;
int MotorTwoEepromCounter = 0;

int stateMotorOne = STATE_FIRST_RESET;
int stateMotorTwo = STATE_FIRST_RESET;
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
uint8_t pause_CommAnode[6]=
    {
        0x86, 0x92, 0xC1, 0x88, 0x8C, // 'E', 'S', 'U'. 'A', 'P'
        0xFF
    };
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

//Display Pause on both 7 segments
void printPauseDisplayOne(uint32_t u32Num)
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
                au8Digit = PAUSE_DIG_OFF;
            }
            else
            {
                firstzero = false;
            }
        }
#endif

        gioSetBit(LED_PORT, LATCH_PIN, LOW);
        DisplayDigit(pause_CommAnode[au8Digit]);
        DisplayDigit(Segment_CommAnode[index]);
        gioSetBit(LED_PORT, LATCH_PIN, HIGH);

        delay();
    }
    firstzero = true;
}

void printPauseDisplayTwo(uint32_t u32Num)
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
                au8Digit = PAUSE_DIG_OFF;
            }
            else
            {
                firstzero = false;
            }
        }
#endif

        gioSetBit(LED_PORT, LATCH2_PIN, LOW);
        DisplayDigit_02(pause_CommAnode[au8Digit]);
        DisplayDigit_02(Segment_CommAnode[index]);
        gioSetBit(LED_PORT, LATCH2_PIN, HIGH);

        delay();
    }
    firstzero = true;
}

//Display Error
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

void MotorOneCWPIcontrol()
{
    //P Control
    errorRPM = MotorOneCWtargetRPM - rpmOne;
    //I Control
    MotorOneCWierrorRPM += errorRPM;
    if(MotorOneCWierrorRPM > imax)
    {
        MotorOneCWierrorRPM = imax;
    }
    else if(MotorOneCWierrorRPM < imin)
    {
        MotorOneCWierrorRPM = imin;
    }
    pidPWM = MotorOneCW_SPEED + errorRPM * kp + MotorOneCWierrorRPM * ki;
    MotorOneCW_SPEED = pidPWM;

    if(MotorOneCW_SPEED > MotorOnecwmax)
    {
        MotorOneCW_SPEED = MotorOnecwmax;
    }
    else if (MotorOneCW_SPEED < MotorOnecwmin)
    {
        MotorOneCW_SPEED = MotorOnecwmin;
    }
}

void MotorOneCCWPIcontrol()
{
    //P Control
    errorRPM = MotorOneCCWtargetRPM - rpmOne;
    //I Control
    MotorOneCCWierrorRPM += errorRPM;
    if(MotorOneCCWierrorRPM > imax)
    {
        MotorOneCCWierrorRPM = imax;
    }
    else if(MotorOneCCWierrorRPM < imin)
    {
        MotorOneCCWierrorRPM = imin;
    }
    pidPWM = MotorOneCCW_SPEED + errorRPM * kp + MotorOneCCWierrorRPM * ki;
    MotorOneCCW_SPEED = pidPWM;

    if(MotorOneCCW_SPEED > MotorOneccwmax)
    {
        MotorOneCCW_SPEED = MotorOneccwmax;
    }
    else if (MotorOneCCW_SPEED < MotorOneccwmin)
    {
        MotorOneCCW_SPEED = MotorOneccwmin;
    }
}

void MotorTwoCWPIcontrol()
{
    //P Control
    errorTwoRPM = MotorTwoCWtargetRPM - rpmTwo;
    //I Control
    MotorTwoCWierrorRPM += errorTwoRPM;
    if(MotorTwoCWierrorRPM > imax)
    {
        MotorTwoCWierrorRPM = imax;
    }
    else if(MotorTwoCWierrorRPM < imin)
    {
        MotorTwoCWierrorRPM = imin;
    }
    pidPWMTwo = MotorTwoCW_SPEED + errorTwoRPM * kp + MotorTwoCWierrorRPM * ki;
    MotorTwoCW_SPEED = pidPWMTwo;

    if(MotorTwoCW_SPEED > MotorTwocwmax)
    {
        MotorTwoCW_SPEED = MotorTwocwmax;
    }
    else if (MotorTwoCW_SPEED < MotorTwocwmin)
    {
        MotorTwoCW_SPEED = MotorTwocwmin;
    }
}

void MotorTwoCCWPIcontrol()
{
    //P Control
    errorTwoRPM = MotorTwoCCWtargetRPM - rpmTwo;
    //I Control
    MotorTwoCCWierrorRPM += errorTwoRPM;
    if(MotorTwoCCWierrorRPM > imax)
    {
        MotorTwoCCWierrorRPM = imax;
    }
    else if(MotorTwoCCWierrorRPM < imin)
    {
        MotorTwoCCWierrorRPM = imin;
    }
    pidPWMTwo = MotorTwoCCW_SPEED + errorTwoRPM * kp + MotorTwoCCWierrorRPM * ki;
    MotorTwoCCW_SPEED = pidPWMTwo;

    if(MotorTwoCCW_SPEED > MotorTwoccwmax)
    {
        MotorTwoCCW_SPEED = MotorTwoccwmax;
    }
    else if (MotorTwoCCW_SPEED < MotorTwoccwmin)
    {
        MotorTwoCCW_SPEED = MotorTwoccwmin;
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

    sciInit(); /* initialize sci/sci-lin    */
               /* even parity , 2 stop bits */
    loadCountersFromEEPROM();
    StartMotorsPWM(); // Start PWM output & motor
    //mainCounterOne = 60083;
    //mainCounterTwo = 164120;
    //saveCountersToEEPROM();

    while (1)
    {
//      if (RPMPrint)
//      {
//          sciDisplayText(UART, &TEXT4[0], TSIZE4); /* send text code 3 */
//          sciPrintDecimal(UART, rpmOne);
//          sciDisplayText(UART, &BREAK[0], BREAKSIZE); /* send text code 3 */
//          sciPrintDecimal(UART, MotorOnePosition);
//          sciDisplayText(UART, &BREAK[0], BREAKSIZE); /* send text code 3 */
//          RPMPrint = false;
//      }

         CheckSwitchStatus();

        // Pause switch is off
        if (!pauseMotorsFlag)
        {
            Pause = false;
            switch(stateMotorOne)
            {
                case STATE_FIRST_RESET:
                    /****************************First Reset State****************************/

                    startMotorOneTimerFlag = true;
                    MotorOneCWPID = false;
                    MotorOneCCWPID = false;
                    //PIsave = false;
                    printCounterDisplayOne(mainCounterOne);

                    /*********Run motor forward for 1.5 secs*********/
                    if (motorOneTimer <= FIRST_RESET_RUNTIME)
                    {
                        MotorOneMotion = true;
                        MotorOneCheck = false;
                        SetMotorOneDirection(FORWARD);
                        SetMotorOneSpeed(FIRST_RESET_SPEED);

                        /*********Start error checking 1 sec later*********/
                        if (motorOneTimer > ERROR_CHECK_TIME)
                        {
                            // Compare RPM for Error
                            MotorOneCheck = true;
                        }
                    }

                    /*********Run motor backward for 1.5 secs*********/
                    else if (motorOneTimer > FIRST_RESET_RUNTIME && motorOneTimer <= (FIRST_RESET_RUNTIME + FIRST_RESET_RUNTIME))
                    {
                        SetMotorOneDirection(BACKWARD);
                        SetMotorOneSpeed(FIRST_RESET_SPEED);
                        MotorOneCheck = false;

                        /*********Start error checking 1 sec later*********/
                        if (motorOneTimer > (FIRST_RESET_RUNTIME + ERROR_CHECK_TIME))
                        {
                            // Compare RPM for Error
                            MotorOneCheck = true;
                        }
                    }

                    /*********First Reset Done*********/
                    else
                    {
                        startMotorOneTimerFlag = false;
                        MotorOneMotion = false;
                        MotorOneCheck = false;
                        motorOneTimer = 0;
                        MotorOneErrorCounter = 0;
                        SetMotorOneSpeed(STOP);
                        stateMotorOne = STATE_FORWARD_PHASE; // Transition to next state
                    }
                    break;

                case STATE_FORWARD_PHASE:
                    /***************************Motor Forward State***************************/

                    startMotorOneTimerFlag = true;
                    MotorOneCCWPID = false;
                    MotorOneForward = true;
                    MotorOneBackward = false;
                    //PIsave = true;
                    printCounterDisplayOne(mainCounterOne);

                    /*********Run motor forward for 3 secs & 1900 encoder readings*********/
                    if (MotorOnePosition < MotorOneFULLCYCLE)
                    {
                        MotorOneMotion = true;
                        SetMotorOneDirection(FORWARD);
                        SetMotorOneSpeed(MotorOneCW_SPEED);
                        MotorOneCWPID = true;
                        MotorOneCheck = false;
                        /*********Start error checking 1 sec later*********/
                        if (motorOneTimer > ERROR_CHECK_TIME && motorOneTimer < STOPERROR_CHECK_TIME)
                        {
                            // Compare RPM for Error from 1 sec to 2 sec
                            MotorOneCheck = true;
                        }
                    }
                    else
                    {
                        startMotorOneTimerFlag = false;
                        MotorOneCheck = false;
                        motorOneTimer = 0;
                        stateMotorOne = STATE_NORMAL_PHASE_PAUSE; // Transition to next state
                        MotorOneCWPID = false;
                        MotorOneErrorCounter = 0;
                    }

                    break;

                case STATE_NORMAL_PHASE_PAUSE:
                    /*************************Motor 1 sec Pause State*************************/

                    startMotorOneTimerFlag = true;
                    MotorOneCWPID = false;
                    MotorOneCCWPID = false;
                    MotorOneMotion = false;
                    MotorOneForward = false;
                    MotorOneBackward = false;
                    MotorOneCheck = false;
                    MotorOneErrorCounter = 0;
                    //PIsave = true;
                    printCounterDisplayOne(mainCounterOne);

                    if (motorOneTimer <= NORMAL_PHASE_PAUSETIME)
                    {
                        SetMotorOneSpeed(STOP);
                    }
                    else
                    {
                        startMotorOneTimerFlag = false;
                        motorOneTimer = 0;
                        stateMotorOne = STATE_BACKWARD_PHASE; // Transition to next state
                    }
                    break;

                case STATE_BACKWARD_PHASE:
                    /***************************Motor Backward State***************************/

                    startMotorOneTimerFlag = true;
                    MotorOneCWPID = false;
                    MotorOneForward = false;
                    MotorOneBackward = true;
                    //PIsave = true;
                    printCounterDisplayOne(mainCounterOne);

                    /*********Run motor backward for 3.5 secs & go back to original position*********/
                    if (MotorOnePosition > 0)
                    {
                        MotorOneMotion = true;
                        SetMotorOneDirection(BACKWARD);
                        SetMotorOneSpeed(MotorOneCCW_SPEED);
                        MotorOneCCWPID = true;
                        MotorOneCheck = false;
                        /*********Start error checking 1 sec later*********/
                        if (motorOneTimer > ERROR_CHECK_TIME && motorOneTimer < STOPERROR_CHECK_TIME)
                        {
                            // Compare RPM for Error from 1 sec to 2 sec
                            MotorOneCheck = true;
                        }
                    }

                    else
                    {
                        startMotorOneTimerFlag = false;
                        motorOneTimer = 0;
                        MotorOneCCWPID = false;
                        MotorOneCheck = false;
                        MotorOnePosition = 0;
                        MotorOneErrorCounter = 0;
                        stateMotorOne = STATE_END_OF_CYCLE_PAUSE; // Transition to next state
                        mainCounterOne++;
                        //MotorOneEepromCounter++;
                        if ((mainCounterOne % EEPROM_SAVING_CYCLE_INTERVAL == 0) && (mainCounterOne != 0))
                        {
                            saveCountersToEEPROM();
                            //MotorOneEepromCounter = 0;
                        }
                    }
                    break;

                case STATE_END_OF_CYCLE_PAUSE:
                    /*************************Motor 5 sec Pause State*************************/
                    startMotorOneTimerFlag = true;
                    MotorOneCWPID = false;
                    MotorOneCCWPID = false;
                    MotorOneMotion = false;
                    MotorOneForward = false;
                    MotorOneBackward = false;
                    MotorOneCheck = false;
                    MotorOneErrorCounter = 0;
                    //PIsave = true;
                    printCounterDisplayOne(mainCounterOne);
                    if (motorOneTimer <= END_CYCLE_PAUSETIME)
                    {
                        SetMotorOneSpeed(STOP);
                    }
                    else
                    {
                        startMotorOneTimerFlag = false;
                        motorOneTimer = 0;
                        stateMotorOne = STATE_FORWARD_PHASE; // Transition to next state
                    }
                    break;

                case STATE_ERROR:
                    /***************************Motor Error State***************************/
                    startMotorOneTimerFlag = true;
                    MotorOneCWPID = false;
                    MotorOneCCWPID = false;
                    MotorOneMotion = false;
                    MotorOneForward = false;
                    MotorOneBackward = false;
                    MotorOneCheck = false;
                    //PIsave = false;
                    MotorOneErrorCounter = 0;

                    if (motorOneTimer <= ERROR_COOLDOWN_TIME)
                    {
                        SetMotorOneSpeed(STOP);
                        displayErrorMotorOne();
                    }
                    else
                    {
                        startMotorOneTimerFlag = false;
                        motorOneTimer = 0;
                        stateMotorOne = STATE_FIRST_RESET; // Transition to next state
                    }

                    break;

                default:
                    break;
            }

            switch (stateMotorTwo)
            {
                case STATE_FIRST_RESET:
                    /****************************First Reset State****************************/

                    startMotorTwoTimerFlag = true;
                    MotorTwoCWPID = false;
                    MotorTwoCCWPID = false;
                    //PIsave = false;
                    printCounterDisplayTwo(mainCounterTwo);

                    /*********Run motor forward for 1.5 secs*********/
                    if (motorTwoTimer <= FIRST_RESET_RUNTIME)
                    {
                        MotorTwoMotion = true;
                        MotorTwoCheck = false;
                        SetMotorTwoDirection(FORWARD);
                        SetMotorTwoSpeed(FIRST_RESET_SPEED);

                        /*********Start error checking 1 sec later*********/
                        if (motorTwoTimer > ERROR_CHECK_TIME)
                        {
                            // Compare RPM for Error
                            MotorTwoCheck = true;
                        }
                   }

                    /*********Run motor backward for 1.5 secs*********/
                    else if (motorTwoTimer > FIRST_RESET_RUNTIME && motorTwoTimer <= (FIRST_RESET_RUNTIME + FIRST_RESET_RUNTIME))
                    {
                        MotorTwoCheck = false;
                        SetMotorTwoDirection(BACKWARD);
                        SetMotorTwoSpeed(FIRST_RESET_SPEED);


                        /*********Start error checking 1 sec later*********/
                        if (motorTwoTimer > (FIRST_RESET_RUNTIME + ERROR_CHECK_TIME))
                        {
                            // Compare RPM for Error
                            MotorTwoCheck = true;
                                }
                    }

                    /*********First Reset Done*********/
                    else
                    {
                        MotorTwoMotion = false;
                        MotorTwoCheck = false;
                        startMotorTwoTimerFlag = false;
                        motorTwoTimer = 0;
                        MotorTwoErrorCounter = 0;
                        SetMotorTwoSpeed(STOP);
                        stateMotorTwo = STATE_FORWARD_PHASE; // Transition to next state
                    }
                break;

                case STATE_FORWARD_PHASE:
                    /***************************Motor Forward State***************************/

                    startMotorTwoTimerFlag = true;
                    MotorTwoCCWPID = false;
                    MotorTwoForward = true;
                    MotorTwoBackward = false;
                    //PIsave = true;
                    printCounterDisplayTwo(mainCounterTwo);

                    /*********Run motor forward for 3 secs*********/
                    if (MotorTwoPosition < MotorTwoFULLCYCLE)
                    {
                        MotorTwoMotion = true;
                        SetMotorTwoDirection(FORWARD);
                        SetMotorTwoSpeed(MotorTwoCW_SPEED);
                        MotorTwoCWPID = true;
                        MotorTwoCheck = false;

                        /*********Start error checking 1 sec later*********/
                        if (motorTwoTimer > ERROR_CHECK_TIME && motorTwoTimer < STOPERROR_CHECK_TIME)
                        {
                            // Compare RPM for Error from 1 sec to 2 sec
                            MotorTwoCheck = true;
                        }
                    }
                    else
                    {
                        startMotorTwoTimerFlag = false;
                        motorTwoTimer = 0;
                        MotorTwoErrorCounter = 0;
                        stateMotorTwo = STATE_NORMAL_PHASE_PAUSE; // Transition to next state
                        MotorTwoCWPID = false;
                        MotorTwoCheck = false;
                    }

                break;

                case STATE_NORMAL_PHASE_PAUSE:
                    /*************************Motor 1 sec Pause State*************************/

                    startMotorTwoTimerFlag = true;
                    MotorTwoCWPID = false;
                    MotorTwoCCWPID = false;
                    MotorTwoMotion = false;
                    MotorTwoForward = false;
                    MotorTwoBackward = false;
                    MotorTwoCheck = false;
                    //PIsave = true;
                    MotorTwoErrorCounter = 0;
                    printCounterDisplayTwo(mainCounterTwo);

                    if (motorTwoTimer <= NORMAL_PHASE_PAUSETIME)
                    {
                        SetMotorTwoSpeed(STOP);
                    }
                    else
                    {
                        startMotorTwoTimerFlag = false;
                        motorTwoTimer = 0;
                        stateMotorTwo = STATE_BACKWARD_PHASE; // Transition to next state
                    }
                break;

                case STATE_BACKWARD_PHASE:
                    /***************************Motor Backward State***************************/

                    startMotorTwoTimerFlag = true;
                    MotorTwoCWPID = false;
                    MotorTwoForward = false;
                    MotorTwoBackward = true;
                    //PIsave = true;
                    printCounterDisplayTwo(mainCounterTwo);

                    /*********Run motor backward for 3.5 secs*********/
                    if (MotorTwoPosition > 0)
                    {
                        MotorTwoMotion = true;
                        MotorTwoCheck = false;
                        SetMotorTwoDirection(BACKWARD);
                        SetMotorTwoSpeed(MotorTwoCCW_SPEED);
                        MotorTwoCCWPID = true;
                        /*********Start error checking 1 sec later*********/
                        if (motorTwoTimer > ERROR_CHECK_TIME && motorTwoTimer < STOPERROR_CHECK_TIME

                        )
                        {
                            // Compare RPM for Error from 1 sec to 2 sec
                            MotorTwoCheck = true;
                        }
                    }

                    else
                    {
                        startMotorTwoTimerFlag = false;
                        motorTwoTimer = 0;
                        stateMotorTwo = STATE_END_OF_CYCLE_PAUSE; // Transition to next state
                        mainCounterTwo++;
                        //MotorTwoEepromCounter++;
                        MotorTwoPosition = 0;
                        MotorTwoCCWPID = false;
                        MotorTwoCheck = false;
                        MotorTwoErrorCounter = 0;
                        if ((mainCounterTwo % EEPROM_SAVING_CYCLE_INTERVAL == 0) && (mainCounterTwo != 0))
                        {
                            saveCountersToEEPROM();
                            //MotorTwoEepromCounter = 0;
                        }

                        printCounterDisplayTwo(mainCounterTwo);
                    }
                break;

                case STATE_END_OF_CYCLE_PAUSE:
                    /*************************Motor 5 sec Pause State*************************/
                    startMotorTwoTimerFlag = true;
                    MotorTwoCWPID = false;
                    MotorTwoCCWPID = false;
                    MotorTwoMotion = false;
                    MotorTwoForward = false;
                    MotorTwoBackward = false;
                    MotorTwoCheck = false;
                    //PIsave = true;
                    MotorTwoErrorCounter = 0;
                    printCounterDisplayTwo(mainCounterTwo);
                    if (motorTwoTimer <= END_CYCLE_PAUSETIME)
                    {
                        SetMotorTwoSpeed(STOP);
                    }
                    else
                    {
                        startMotorTwoTimerFlag = false;
                        motorTwoTimer = 0;
                        stateMotorTwo = STATE_FORWARD_PHASE; // Transition to next state
                    }
                break;

                case STATE_ERROR:
                    /***************************Motor Error State***************************/
                    startMotorTwoTimerFlag = true;
                    MotorTwoCWPID = false;
                    MotorTwoCCWPID = false;
                    MotorTwoMotion = false;
                    MotorTwoForward = false;
                    MotorTwoBackward = false;
                    MotorTwoCheck = false;
                    //PIsave = false;
                    MotorTwoErrorCounter = 0;

                    if (motorTwoTimer <= ERROR_COOLDOWN_TIME)
                    {
                        SetMotorTwoSpeed(STOP);
                        displayErrorMotorTwo();
                    }
                    else
                    {
                        startMotorTwoTimerFlag = false;
                        motorTwoTimer = 0;
                        stateMotorTwo = STATE_FIRST_RESET; // Transition to next state
                    }

                break;

                default:
                break;
            }
        }

        //pause switch is flipped
        else
        {
            // Stop motors
            SetMotorOneSpeed(STOP);
            SetMotorTwoSpeed(STOP);
            // off all flags
            MotorOneCWPID = false;
            MotorOneCCWPID = false;
            MotorOneMotion = false;
            MotorOneForward = false;
            MotorOneBackward = false;
            MotorOneCheck = false;

            MotorTwoCWPID = false;
            MotorTwoCCWPID = false;
            MotorTwoMotion = false;
            MotorTwoForward = false;
            MotorTwoBackward = false;
            MotorTwoCheck = false;

            startMotorOneTimerFlag = false;
            startMotorTwoTimerFlag = false;

            motorOneTimer = 0;
            motorTwoTimer = 0;

            Pause = true;
            if(PauseTimer == PAUSE_SAVING)
            {
                saveCountersToEEPROM();
                //MotorOneEepromCounter = 0;
                //MotorTwoEepromCounter = 0;
            }
            if(BlinkTimer <= Blink_Pause)
            {
                // Update display to prevent flickering
                printPauseDisplayOne(PAUSE);
                printPauseDisplayTwo(PAUSE);
            }
            else if(BlinkTimer <= Blink_Counter)
            {
                // Update display to prevent flickering
                printCounterDisplayOne(mainCounterOne);
                printCounterDisplayTwo(mainCounterTwo);
            }
            else
            {
                BlinkTimer = 0;
            }
        }




    } // end of main while loop

    /* USER CODE END */

    return 0;
}

/* USER CODE BEGIN (4) */

/**************************************************ISR Definition***************************************************/

void gioNotification(gioPORT_t *port, uint32 bit)
{
    if (bit == MOTORONEENCODERPIN && MotorOneMotion)
    {
        encoderOneCounter++;

        if(MotorOneForward)
        {
            MotorOnePosition++;
        }
        if(MotorOneBackward && MotorOnePosition > 0)
        {
            MotorOnePosition--;
        }
    }

    if (bit == MOTORTWOENCODERPIN && MotorTwoMotion)
    {
        encoderTwoCounter++;

        if(MotorTwoForward)
        {
            MotorTwoPosition++;
        }
        if(MotorTwoBackward && MotorTwoPosition > 0)
        {
            MotorTwoPosition--;
        }
    }
}

void rtiNotification(rtiBASE_t *rtiREG, uint32 notification)
{
    // Every 50ms

    /************************RPM Calculation************************/
    volatile float numberOfRotationOne = encoderOneCounter / PULSES_PER_ROTATION;
    volatile float numberOfRotationTwo = encoderTwoCounter / PULSES_PER_ROTATION;
    rpmOne = TIMER_CYCLES_PER_MINUTE * numberOfRotationOne;
    rpmTwo = TIMER_CYCLES_PER_MINUTE * numberOfRotationTwo;
    RPMPrint = true;
    encoderOneCounter = 0;
    encoderTwoCounter = 0;
    /**************************Timer Counter***********************/

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

    if (Pause)
    {
        PauseTimer++;
        BlinkTimer++;
    }
    else
    {
        PauseTimer = 0;
        BlinkTimer = 0;
    }
    /**************************Error Check*************************/
    if(MotorOneCheck)
    {
        if (rpmOne < MIN_CONSTSPEED_MOTOR_SPEED_RPM)
        {
            MotorOneErrorCounter++;
            if(MotorOneErrorCounter == 3)
            {
                startMotorOneTimerFlag = false;
                stateMotorOne = STATE_ERROR;
                MotorOneErrorCounter = 0;
            }
        }
    }
    if(MotorTwoCheck)
    {
        if (rpmTwo < MIN_CONSTSPEED_MOTOR_SPEED_RPM)
        {
            MotorTwoErrorCounter++;
            if(MotorTwoErrorCounter == 3)
            {
                startMotorTwoTimerFlag = false;
                stateMotorTwo = STATE_ERROR;
                MotorTwoErrorCounter = 0;
            }

        }
    }

    /********************PI Control********************************/
    if(MotorOneCWPID)
    {
        MotorOneCWPIcontrol();
    }
    if(MotorOneCCWPID)
    {
        MotorOneCCWPIcontrol();
    }

    if(MotorTwoCWPID)
    {
        MotorTwoCWPIcontrol();
    }
    if(MotorTwoCCWPID)
    {
        MotorTwoCCWPIcontrol();
    }

    /*if(PIsave)
    {
        PI[i] = rpmOne;
        pos[j] = MotorOnePosition;
        i++;
        j++;
    }*/
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
