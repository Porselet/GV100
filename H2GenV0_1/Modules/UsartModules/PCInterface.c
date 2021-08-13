#include "PCInterface.h"
#include "stdint.h"
#include "Config.h"

void SendToPC_RAW(char * msg, uint8_t size)
{
	SendBytesUART((uint8_t*)msg, size, PC_RS485_UART);
};

void SendValuesToPC(struct DataStruct * values)
{
	struct AnswerValues Values;
	Values.cmd = CMD_GET_VALUES;
	Values.size = sizeof(Values);
	Values.AhCounter = values->AhCounter;
	Values.Amperage = values->Amperage;
	Values.AmperageSetpoint = values->AmperageUserSetpoint;
	Values.D_IN_1 = values->D_IN_1;
	Values.D_IN_2 = values->D_IN_2;
	Values.D_IN_3 = values->D_IN_3;
	Values.D_IN_4 = values->D_IN_4;
	
	Values.Mode = values->Mode;
	Values.Press = values->Press;
	Values.sysCounter = values->sysCounter;
	Values.Temperature = values->Temperature;
	Values.Voltage = values->Voltage;
	Values.WQ = values->WQ;
	
	SendToPC_RAW((char*)((&Values)), sizeof(Values));
	
};

void SetActive(bool active)
{
	GeneratorSetActive(active);
	uint8_t AnswerMSG[] = {CMD_SET_ACTIVE, 4,0,(uint8_t)active};
	SendToPC_RAW((char*)(AnswerMSG), sizeof(AnswerMSG));
}

void SetNewAmperage(uint16_t amperage, struct DataStruct * values)
{
	values->AmperageUserSetpoint = amperage / MULT_AMP_TO_FLOW;
	values->NewAmperage = true;
	uint8_t AnswerMSG[] = {CMD_SET_SETPOINT, 4,0,(uint8_t)1};
	SendToPC_RAW((char*)(AnswerMSG), sizeof(AnswerMSG));
	
	/*char StrNextion[80]={0xFF};
  sprintf(StrNextion, "SetpointFlow=%d", amperage);
  //sprintf(StrNextion, "page 0");
	SendToNextion(StrNextion);
*/
	
}


void GrabMSG( uint8_t * msg, uint8_t size , struct DataStruct * values)
{
	if ((size != msg[1])) return;
	
	switch (msg[0])
	{
		case CMD_GET_VALUES: 
			SendValuesToPC(values);
			break;
		case CMD_SET_ACTIVE:
			SetActive(msg[2]);			
			break;
		case CMD_SET_SETPOINT: 
			SetNewAmperage(msg[2] + (((uint16_t)msg[3])*256), values);
			break;
		default:
			break;
	}
	

	
};
