#ifndef __POWERSUPPLYINTERFACE_H 
#define __POWERSUPPLYINTERFACE_H

#include "UARTs.h"
#include "DataStruct.h"

#define PowerSupplyUart     &huart1
#define PowerSupplyBuff   	RXBuffer1
#define PowerSupplyBuffLen  RXBufferLen1

void InitPowerSupplyInterface();

//void ProcessingMsgPowerSupply(uint8_t * msg, uint8_t * size , struct DataStruct * values);

void GetAmperageFromPowerSupply(struct DataStruct * values);
void SetAmperageToPowerSupply(struct DataStruct * values);
void StartPowerSupply();
void StopPowerSupply();
void PollPowerSupply(struct DataStruct * values);


#include "stdbool.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include "stdint.h"

#include "DataStruct.h"



uint16_t CRC16(const uint8_t *nData, uint16_t wLength);




//bool SendCmdToPowerSupply(enum CommandsToPS cmd, struct DataStruct * values);
//void onGetBytesPowerSupply();
//void onSendBytesPowerSupply();





	



#endif //__POWERSUPPLYINTERFACE_H