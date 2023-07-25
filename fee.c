/*
 *  fee.c
 *  Handling EEPROM write & read data
 *  Created on: 06 July, 2022
 *      Author: Lukman
 */


#include "fee.h"
#include "rotary.h"
#include "string.h"
#include "stdio.h"



void EEPROM_readValue(uint8_t* buffer_o, unsigned int length, uint8_t block_number)
{
    uint16_t Status;

    TI_Fee_Read(block_number, 0, buffer_o, length);
    //TI_Fee_Read(block_number, 0, buffer_o, 0xFFFF);
    do
    {
        TI_Fee_MainFunction();
        delay(0xFF);
        Status=TI_Fee_GetStatus(0);
    }
   while(Status!=IDLE);

}

void EEPROM_readData(unsigned int address)
{

  EEPROM_readValue(rd_data, 0xFFFF, address);
  datfee_rd1= rd_data[1]; //take the last 8bit data  //
  datfee_rd1=(rd_data[3]<<8)+datfee_rd1;  //add first 8 bit data with the last 8bit data//

  datfee_rd2= rd_data[5]; //take the last 8bit data  //
  datfee_rd2=(rd_data[7]<<8)+datfee_rd2;  //add first 8 bit data with the last 8bit data//

}

void EEPROM_readCounterData(unsigned int address,uint32_t *pCounterOne, uint32_t *pCounterTwo)
{
  uint16_t Status;

  TI_Fee_ReadSync(address, 0, (uint8_t*)au8Read_buff, 8U);
  memcpy(pCounterOne,au8Read_buff, 4U);
  memcpy(pCounterTwo,  au8Read_buff+4, 4U);
}

void EEPROM_writeValue(uint8_t* buffer_i, uint8_t block_number)
{
    uint16_t Status;

    TI_Fee_WriteSync(block_number, buffer_i);
    do
    {
        TI_Fee_MainFunction();
        delay(0xFF);
        Status=TI_Fee_GetStatus(0);
    }
   while(Status!=IDLE);

}

void EEPROM_writeData(uint16_t data1, uint16_t data2, unsigned int address)
{

    wr_data[0]= (uint8_t)data1;
    wr_data[1]= data1>>8;
	//memcpy(wr_data, &u32data1, 4U);
    wr_data[2]= (uint8_t)data2;
    wr_data[3]= data2>>8;
	//memcpy(wr_data+4, &u32data2, 4U);
    EEPROM_writeValue((uint8_t*)wr_data, address);
    delay(100);
}

void EEPROM_writeCounterData(uint32_t u32data1, uint32_t u32data2, unsigned int address)
{

	memcpy(au8Write_buff, &u32data1, 4U);
	memcpy(au8Write_buff+4, &u32data2, 4U);
	TI_Fee_WriteSync(address, (uint8_t*)au8Write_buff);
	delay(100);
}


