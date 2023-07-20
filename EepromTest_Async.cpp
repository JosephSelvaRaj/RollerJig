/** @file HL_sys_main.c
 *   @brief Application main file
 *   @date 11-Dec-2018
 *   @version 04.07.01
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
/*********************************************FLOW OF EVENTS*********************************************/
// To test the writing to EEPROM and then loading back the correct values via "Serial Monitor"
// 1. Load counters from EEPROM
// 2. Start RTI timer
// 3. Every 1 second, increment counters
// 4. Every 60 seconds, save counters to EEPROM
/********************************************************************************************************/
/* USER CODE END */

/* Include Files */

#include "HL_sys_common.h"

/* USER CODE BEGIN (1) */
#include "HL_sys_core.h"
#include "HL_mibspi.h"
#include "HL_esm.h"
#include "HL_rti.h"
#include "HL_gio.h"
#include "HL_het.h"
#include "ti_fee.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#include "HL_system.h"
#include "HL_sci.h"

#define  TSIZE1 6
uint8  TEXT1[TSIZE1]= {'S','A','V','E','D',' '};
#define  TSIZE2 1
uint8  TEXT2[TSIZE2]= {' '};
#define  TSIZE3 2
uint8  TEXT3[TSIZE3]= {'\n','\r'};

/* USER CODE END */

/** @fn void main(void)
 *   @brief Application main function
 *   @note This function is empty by default.
 *
 *   This function is called after startup.
 *   The user can use this function to implement the application.
 */

/* USER CODE BEGIN (2) */
const int mainCounterAddressOffset = 0U;
const int mainCountersAddress = 0x01;
const int mainCountersTotalByteSize = 16U;

int EepromSaveInterval = 60000;     // Every 60 seconds
int CouterIncrementInterval = 1000; // Every 1 second
int mainCounterOne = 30U;
int mainCounterTwo = 31U;
volatile bool saveEepromFlag = false;
volatile bool incrementCounterFlag = false;
volatile int eepromTimerCounter = 0U;
volatile int incrementTimerCounter = 0U;

void loadCountersFromEEPROM(void)
{
    uint8_t loadBufferArray[16];
    // Loads both main counters from same block address
    TI_Fee_ReadSync(mainCountersAddress, mainCounterAddressOffset, (uint8_t *)loadBufferArray, mainCountersTotalByteSize);
    memcpy(&mainCounterOne, loadBufferArray, 4U);     // Extract first 4 bytes for mainCounterOne
    memcpy(&mainCounterTwo, loadBufferArray + 4, 4U); // Extract next 4 bytes for mainCounterTwo
}

void saveCountersToEEPROM(void)
{
    uint8_t writeBufferArray[16];
    memcpy(writeBufferArray, &mainCounterOne, 4U);
    memcpy(writeBufferArray + 4, &mainCounterTwo, 4U);
    TI_Fee_WriteAsync(mainCountersAddress, (uint8_t *)writeBufferArray);
    do
    {
        TI_Fee_MainFunction();
        delay();
        Status=TI_Fee_GetStatus(0);
    }
    while(Status!=IDLE);

}
/* USER CODE END */

void main(void)
{
    /* USER CODE BEGIN (3) */

    TI_Fee_Init();

    /* Initialize RTI driver */
    rtiInit();

    /* Set high end timer GIO port hetPort pin direction to all output */
    gioSetDirection(hetPORT1, 0xFFFFFFFF);

    loadCountersFromEEPROM();

    /* Enable RTI Compare 0 interrupt notification */
    rtiEnableNotification(rtiREG1, rtiNOTIFICATION_COMPARE0);

    _enable_IRQ_interrupt_();

    /* Start RTI Counter Block 0 */
    rtiStartCounter(rtiREG1, rtiCOUNTER_BLOCK0);

    /* Run forever */
    while (1)
    {
        if (incrementCounterFlag)
        {
            // Increment counters every 1 sec
            mainCounterOne++;
            mainCounterTwo++; // one value higher than mainCounterOne
            incrementCounterFlag = false;
        }

        if (saveEepromFlag)
        {
            // Save counters to EEPROM every 60 sec
            saveCountersToEEPROM();
            sciDisplayText(UART,&TEXT1[0],TSIZE1);
            sciPrintDecimal(UART, mainCounterOne);
            sciDisplayText(UART,&TEXT2[0],TSIZE2);
            sciPrintDecimal(UART, mainCounterTwo);
            sciDisplayText(UART,&TEXT3[0],TSIZE3);
            saveEepromFlag = false;
        }
    }

    /* USER CODE END */
}

/* USER CODE BEGIN (4) */

/* Note-You need to remove rtiNotification from notification.c to avoid redefinition */
void rtiNotification(rtiBASE_t *rtiREG, uint32 notification)
{
    // Increments every 1ms
    incrementTimerCounter++;
    eepromTimerCounter++;

    /***********************1 Second timer*************************/
    if (incrementTimerCounter >= CouterIncrementInterval)
    {
        incrementCounterFlag = true;
        incrementTimerCounter = 0U;
    }
    /**************************************************************/

    /***********************60 Seconds timer***********************/
    if (eepromTimerCounter >= EepromSaveInterval)
    {
        gioSetPort(hetPORT1, gioGetPort(hetPORT1) ^ 0x00000001); // Toggle HNET Pin 0 LED
        saveEepromFlag = true;
        eepromTimerCounter = 0U;
    }
    /**************************************************************/
}

int i = 0;

void sciPrintDecimal(sciBASE_t *sci, uint32_t value)
{
    char buffer[11]; // Buffer to hold the string representation of the decimal value
    snprintf(buffer, 11, "%lu", value); // Convert the uint32 value to a string
    // Print each character of the string until the null terminator
    for(i = 0; buffer[i] != '\0'; i++)
    {
        while ((sci->FLR & 0x4) == 4); // Wait until busy
        sciSendByte(sci, buffer[i]); // Send out text
    }
    i = 0;
}

void sciDisplayText(sciBASE_t *sci, uint8 *text,uint32 length)
{
    while(length--)
    {
        while ((UART->FLR & 0x4) == 4); /* wait until busy */
        sciSendByte(UART,*text++);      /* send out text   */
    };
}

/* USER CODE END */
