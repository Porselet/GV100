#ifndef __POWERSUPPLUNEW_H
#define __POWERSUPPLUNEW_H
/*
#include "stdbool.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include "stdint.h"

#include "DataStruct.h"



extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

uint16_t CRC16(const uint8_t *nData, uint16_t wLength);

enum CommandsToPS
{
	NULL_CMD,
	TIMEOUT_CONF_CMD,
	BIT_POWER_ON_CMD,
	BIT_POWER_OFF_CMD,
	BIT_WORK_ON_CMD,
	BIT_WORK_OFF_CMD,
	GET_VOLT_AMP_CMD,
	SET_VOLT_AMP_CMD
};

enum CommandsToPS LAST_CMD;

enum StatesMSG
{
	IDLE_STATE_POWERSUPPLY,
	TRY_TO_SEND_MSG_STATE_POWERSUPPLY,
	SEND_MSG_STATE_POWERSUPPLY, //и ждать ответа
	GET_ANSWER_STATE_POWERSUPPLY,
	PROCESS_STATE_POWERSUPPLY
} volatile StateMSG;


bool SendCmdToPowerSupply(enum CommandsToPS cmd, struct DataStruct * values);
void onGetBytesPowerSupply();
void onSendBytesPowerSupply();
void ProcessingMsgPowerSupply(uint8_t * msg, uint8_t * size , struct DataStruct * values);




const uint8_t TimeoutConf[] =	{ 0x01, 0x10, 0x00, 0x24, 0x00, 0x01, 0x02, 0x05, 0xDC, 0xA2, 0x7D};
const uint8_t BitPowerOn[]	=	{ 0x01, 0x0F, 0x01, 0x10, 0x00, 0x01, 0x01, 0x01, 0x2F, 0x45};
const uint8_t BitPowerOff[] =	{ 0x01, 0x0F, 0x01, 0x10, 0x00, 0x01, 0x01, 0x00, 0xEE, 0x85};
const uint8_t BitWorkOn[]	 =	{ 0x01, 0x0F, 0x01, 0x11, 0x00, 0x01, 0x01, 0x01, 0x12, 0x85};
const uint8_t BitWorkOff[]	=	{ 0x01, 0x0F, 0x01, 0x11, 0x00, 0x01, 0x01, 0x00, 0xD3, 0x45};
const uint8_t GetAmpVolt[]	=	{ 0x01, 0x04, 0x00, 0x14, 0x00, 0x02, 0x31, 0xCF};
volatile	uint8_t SetAmp[] 			= { 0x01, 0x10 , 0x00 , 0x12 , 0x00 , 0x02 , 0x04 , 0x00 , 0x14 , 0x01 , 0x90 , 0x32 , 0x82};

void SendBytes(uint8_t * bytes, uint8_t size)
{
	HAL_GPIO_WritePin(DI1_GPIO_Port, DI1_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit_DMA(&huart1, bytes, size);
	while (HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_READY) {	};
	while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) {};

};

bool SendCmdToPowerSupply(enum CommandsToPS cmd, struct DataStruct * values)
{
	int16_t timer_timeout = 100;
	while (StateMSG != IDLE_STATE_POWERSUPPLY) 
	{
		vTaskDelay(1); 
		timer_timeout--; 
		if (timer_timeout < 0) 
		{
			StateMSG = IDLE_STATE_POWERSUPPLY;
			SendCmdToPowerSupply(LAST_CMD, values);
			return false;
		}
	};
	StateMSG = TRY_TO_SEND_MSG_STATE_POWERSUPPLY;
	 switch (cmd)
	 {
		 case NULL_CMD:
			  break;
		 case TIMEOUT_CONF_CMD:
			  SendBytes((uint8_t*)TimeoutConf, sizeof(TimeoutConf));
			  break;			 
		 case BIT_POWER_ON_CMD:
			  SendBytes((uint8_t*)BitPowerOn, sizeof(BitPowerOn));
			  break;
		 case BIT_POWER_OFF_CMD:
			  SendBytes((uint8_t*)BitPowerOff, sizeof(BitPowerOff));
			  break;
		 case BIT_WORK_ON_CMD:
			  SendBytes((uint8_t*)BitWorkOn, sizeof(BitWorkOn));
			  break;
		 case BIT_WORK_OFF_CMD:
			  SendBytes((uint8_t*)BitWorkOff, sizeof(BitWorkOff));
			  break;
		 case GET_VOLT_AMP_CMD:
			  SendBytes((uint8_t*)GetAmpVolt, sizeof(GetAmpVolt));
			  break;
		 case SET_VOLT_AMP_CMD:
			 	SetAmp[7] = (values->AmperageSetpoint >> 8); //msb amp;
				SetAmp[8] = (values->AmperageSetpoint & 0x00FF); //lsb amp;
				SetAmp[9] = (values->VoltageSetpoint >> 8); //msb voltage;
				SetAmp[10] = (values->VoltageSetpoint & 0x00FF); //lsb voltage;
				uint16_t crc = CRC16((uint8_t*)SetAmp, sizeof(SetAmp) - 2);
				SetAmp[12] = (crc >> 8); //msb crc;
				SetAmp[11] = (crc & 0x00FF); //lsb crc;
				SendBytes((uint8_t*)SetAmp, sizeof(SetAmp));
			 break;
	 } 
	LAST_CMD = cmd;
	return true;
};

void onSendBytesPowerSupply()
{
	StateMSG = SEND_MSG_STATE_POWERSUPPLY;
};

void onGetBytesPowerSupply()
{
	StateMSG = GET_ANSWER_STATE_POWERSUPPLY;	
};


void ProcessingMsgPowerSupply(uint8_t * msg, uint8_t * size , struct DataStruct * values)
{
	if (StateMSG != GET_ANSWER_STATE_POWERSUPPLY) return;
	StateMSG = PROCESS_STATE_POWERSUPPLY;

	if (*(size))
	{
		*(size) = 0;

		if ((msg[0]==1) && (msg[1]==4) && (msg[2]==4) )
			{
				values->Amperage = ((uint16_t)msg[3] * 256) + msg[4];
				values->Voltage = ((uint16_t)msg[5] * 256) + msg[6];
				
				if (abs(((int)values->Amperage) - ((int)values->AmperageUserSetpoint)) < 3)
				{
					values->Amperage = values->AmperageUserSetpoint;
				}
			}			
	}
	StateMSG = IDLE_STATE_POWERSUPPLY;
};
	

uint16_t CRC16(const uint8_t *nData, uint16_t wLength)
{
    static const uint16_t wCRCTable[] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040 };

    uint8_t nTemp;
    uint16_t wCRCWord = 0xFFFF;

    while (wLength--)
    {
        nTemp = *nData++ ^ wCRCWord;
        wCRCWord >>= 8;
        wCRCWord  ^= wCRCTable[(nTemp & 0xFF)];
    }
    return wCRCWord;
} // End: CRC16
	*/
	
#endif