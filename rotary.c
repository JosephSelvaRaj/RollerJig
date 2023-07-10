/*
 *  rotary.c
 *  Handling Pulse counting based on GPIO Interrupt
 *  Created on: 21 June, 2022
 *      Author: Lukman
 */
#include "HL_sys_vim.h"
#include "rotary.h"


#define EQEP_BORDER_CHECK (EQEP2_QPOSMAX_CONFIGVALUE / 4)

uint32_t uRotary2LastVal = EQEP2_QPOSINIT_CONFIGVALUE; // no need to set volatile, only used in this function
uint32_t uRotary2 = EQEP2_QPOSINIT_CONFIGVALUE; // no need to set volatile, only used in this function
boolean bWrap;
unsigned char uRot;


void rotaryInit(boolean wrap) {
    bWrap = wrap;
    QEPInit();
    /* Enable Position Counter */
    eqepEnableCounter(eqepREG2);
    /* Enable Unit Timer. */
    eqepEnableUnitTimer(eqepREG2);

    /* Enable capture timer and capture period latch. */
    eqepEnableCapture(eqepREG2);

}

uint32_t getRotaryPosition(){

    /* Status flag is set to indicate that a new value is latched in the QCPRD register*/
    if((eqepREG2->QEPSTS & 0x80U) !=0U) {
        uRotary2 = eqepREG2->QPOSLAT;//get the eQEP Position Latch//
        // address border conditions to avoid shoot through around min or max
        if (!bWrap && (uRotary2 > EQEP2_QPOSMAX_CONFIGVALUE - EQEP_BORDER_CHECK) && (uRotary2LastVal < EQEP_BORDER_CHECK)) {
            uRotary2 = 0x0U;
            eqepREG2->QPOSCNT = 0x0U;
            eqepREG2->QPOSLAT = 0x0U;

        } else if (!bWrap && (uRotary2 < EQEP_BORDER_CHECK) && (uRotary2LastVal > EQEP2_QPOSMAX_CONFIGVALUE - EQEP_BORDER_CHECK)) {
            uRotary2 = EQEP2_QPOSMAX_CONFIGVALUE;
            eqepREG2->QPOSCNT = EQEP2_QPOSMAX_CONFIGVALUE;
            eqepREG2->QPOSLAT = EQEP2_QPOSMAX_CONFIGVALUE;
        }


        eqepREG2->QEPSTS |= 0x80U;//Clear the Status flag//
        uRotary2LastVal = uRotary2;
    }

    return uRotary2;
}


void sciDisplayText(sciBASE_t *sci, uint8_t *text, uint32_t length)
{
   while(length--)
   {
       while ((sci->FLR & 0x4) == 4); /* wait until busy */
       sciSendByte(sci,*text++);      /* send out text   */
   };
}

int32_t getPWM(boolean dir, uint16_t pin)
{
    uint32_t countt;
    static boolean bit;
    static boolean last_bit=0;

    //bit=(gioGetBit(gioPORTA, pin));//read from GIOA 1 pin//
    bit=pin;//read from GIOA 1 interrupt pin//

    if(flag_mstart==1)//assign data from eeprom only once
    {
        countt = datfee_rd1;//assingn data from eeprom for counter position//
        cycle_cnt=datfee_rd2;//assingn data from eeprom for number of cycle//
        flag_mstart=0; //flag read data from eeprom done//
    }


    if ((last_bit==0)&&(bit==1))
    {
        if(dir==1)
        {
          countt += 2;
        }
        else if(dir==0)
        {
          countt -= 2;
        }
        last_bit=bit;
    }
    else if((last_bit==1)&&(bit==0))
    {
        countt= countt;
        last_bit=bit;
    }

    return countt;
}

int32_t getHallPulse(boolean dir, uint16_t pin)
{
    uint32_t countt;
    static boolean bit;
    static boolean last_bit=0;

    //bit=(gioGetBit(gioPORTA, pin));//read from GIOA 1 pin//
    bit=pin;//read from GIOA 1 interrupt pin//

    if(flag_mstart==1)//assign data from eeprom only once
    {
        countt = u32MotorEncPosition;//assingn data from eeprom for counter position//
        cycle_cnt=u32TestCounter;//assingn data from eeprom for number of cycle//
        flag_mstart=0; //flag read data from eeprom done//
    }


    if ((last_bit==0)&&(bit==1))
    {
        if(dir==1)
        {
          countt += 2;
        }
        else if(dir==0)
        {
          countt -= 2;
        }
        last_bit=bit;
    }
    else if((last_bit==1)&&(bit==0))
    {
        countt= countt;
        last_bit=bit;
    }

    return countt;
}

uint16_t set_motor_dir(boolean dir)
{
   static boolean m_dir;

    if(dir==1)
    {
      gioSetBit(gioPORTA, 3, dir);//set pin 3 output as 1//
      m_dir = dir;
    }
    else if(dir==0)
    {
      gioSetBit(gioPORTA, 3, dir);//set pin 3 output as 0//
      m_dir = dir;
    }

    return m_dir;
}


void delay(uint32_t dly)
{
    uint32_t cnt;

    cnt= dly;
    do
    {
        cnt--;
    }
    while(cnt>0);

}


//Handle GIO High Level Interrupt//
#pragma CODE_STATE(gioHighLevelInterrupt, 32)
#pragma INTERRUPT(gioHighLevelInterrupt, IRQ)

// SourceId : GIO_SourceId_011 //
// DesignId : GIO_DesignId_011 //
// Requirements : HL_CONQ_GIO_SR12 //
void gioHighLevelInterrupt(void)
{
    uint32_t offset = gioREG->OFF1;

    if(offset!=0U)
    {
        offset = offset - 1U;

        if (offset >= 1U)
        {
          gioNotification(gioPORTA, offset);
        }
    }
}

//Handle GIO Notification for GIO Interrupt//
#pragma WEAK(gioNotification)
void gioNotification(gioPORT_t *port, uint32 bit)
{
    volatile uint32_t offset = bit;

    if(bit == 1)
    {
        if(offset>=1U)//check when rising event triggered//
        {
            pin_bit= !pin_bit;//assign to pin bit//
            u32MotorEncPosition++;
        }

        // count the pulse
        if(!flag_motor_stop && flag_motor_forward) // forward
        {
            u32motor_rotate_pulse_cntr++;
            i32EncPulse_cntr++;
        }
        else if(!flag_motor_stop && !flag_motor_forward) // backward
        {
            if(u32motor_rotate_pulse_cntr > 0) u32motor_rotate_pulse_cntr--;
            if(i32EncPulse_cntr > 0) i32EncPulse_cntr--;
        }
        if(flag_motor_stop)
        {
            //i32EncPulse_cntr = 0;
        }
        mdir= motor_dir;//assign motor direction value//
        pulse_cnt = getPWM(mdir, pin_bit);//count PWM value based on motor direction//

        if((motor_bw==true)&&(pulse_cnt<=MIN_PULSE))//check direction based on position//
        {
        motor_stop=true;
        motor_bw=false;//motor run forward//
        pulse_cnt=MIN_PULSE;

        }
        else if((motor_bw==false)&&(pulse_cnt>=MAX_PULSE))//check direction based on position//
        {
        motor_stop=true;
        motor_bw=true;//motor run backward//
        pulse_cnt=MAX_PULSE;

        }
    }
    else
    {   if(offset>=1U)//check when rising event triggered//
        {
            pin_bit= !pin_bit;//assign to pin bit//
            encoderTwoCounter++;
        }
    }
}
