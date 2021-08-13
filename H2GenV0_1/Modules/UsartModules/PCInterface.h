#ifndef __PC_INTERFACE_H
#define __PC_INTERFACE_H

#include "stdint.h"
#include "DataStruct.h"
#include "stdbool.h"
#include "UARTs.h"


#define PC_RS485_UART     &huart4
#define PC_Buff   				RXBuffer4
#define PC_BuffLen  			RXBufferLen4

//#include "PowerSupply.h"

//extern void SendToPC_RAW(char * msg, uint8_t size);
extern void GeneratorSetActive(bool active);
extern void SendToNextion(char * msg);

enum CommandsPC
{
  CMD_SET_ACTIVE,
  CMD_SET_SETPOINT,
  CMD_GET_VALUES,
  CMD_GET_RAW_VALUES,
};
void SendValuesToPC(struct DataStruct * values);
void GrabMSG( uint8_t * msg, uint8_t size , struct DataStruct * values);

#pragma pack(push, 1)
struct AnswerValues		
{
	enum CommandsPC cmd;
	uint8_t size;
	uint32_t AhCounter;
	uint32_t sysCounter;
	float Press;
	float Temperature;
	float WQ;
	uint16_t AmperageSetpoint;
	uint16_t Amperage;
	uint16_t Voltage;
	char D_IN_1;
	char D_IN_2;
	char D_IN_3;
	char D_IN_4;	
	enum GENERATOR_MODES Mode;

};
#pragma pack(pop)



#endif //__PC_INTERFACE_H