#ifndef _MANAGER_H
#define _MANAGER_H
#include "stdint.h"
#include "DataStruct.h"
#include "stdbool.h"
#include "cmsis_os.h"
#include "Config.h"
#include "PID_regulator.h"
#include "PowerSupplyInterface.h"




//extern void SetVoltageAmperage(struct DataStruct * values);
//extern void StartPowerSupply();
//extern void StopPowerSupply();
//extern void SendToPC_RAW(char * msg, uint8_t size);
//extern void SendAlarmNextion(uint8_t ErrNumber);
extern void SendMsgNextion(char * msg, uint8_t size);
//extern void GetVotageAmperage(struct DataStruct * values);

/* 
void GetAmperageFromPowerSupply(struct DataStruct * values);
void SetAmperageToPowerSupply(struct DataStruct * values);
void StartPowerSupply();
void StopPowerSupply();
*/
void SendAlarmMsg(struct DataStruct * values)
{
	//SendAlarmNextion((uint32_t)values->Mode);
};


void SetVoltageAmperageLowSpeed(struct DataStruct * values, uint16_t setpointAmp)
{

		while (setpointAmp-SPEED_PS > values->AmperageSetpoint)
		{
			values->AmperageSetpoint+=SPEED_PS;
			SetAmperageToPowerSupply(values);
			GetAmperageFromPowerSupply(values);			
			if ((values->NewAmperage) && (values->Mode == MODE_NORMAL_WORK)) return;
			vTaskDelay(100);
		}
		values->AmperageSetpoint= setpointAmp;
		SetAmperageToPowerSupply(values);
		GetAmperageFromPowerSupply(values);			
};


void CheckTemp(struct DataStruct * values)
{
	if  ((values->Temperature > MIN_TEMP) && (values->Temperature < MAX_TEMP))
	{
		return;
	}
	else
	{
		values->Mode = MODE_ERROR_TEMP;
		return;
	};
};

void CheckPress(struct DataStruct * values)
{
	if (values->Press > MAX_PRESS)
	{
		values->Mode = MODE_ERROR_PRESS;
	}
};

void CheckWQ(struct DataStruct * values)
{
	if (values->WQ > MAX_WQ)
	{
		values->Mode = MODE_ERROR_WQ;
	}
};

void CheckLevelSensors(struct DataStruct * values)
{
		if ((values->D_IN_1 == 1) /*&& (values->Mode != MODE_NORMAL_WORK))*/ {values->Mode = MODE_ERROR_HIGH_HYDROGEN_LEVEL;}
		if (values->D_IN_2 == 0) {values->Mode = MODE_ERROR_LOW_HYDROGEN_LEVEL;}
		if (values->D_IN_3 == 1) {values->Mode = MODE_ERROR_HIGH_OXYGEN_LEVEL;}
		if (values->D_IN_4 == 0) {values->Mode = MODE_ERROR_LOW_OXYGEN_LEVEL;}
};

void CheckUserCommands(struct DataStruct * values)
{
	switch (values->OnOffButton)
	{
		case BTN_ON:
		{	
			break;
		}
		case BTN_OFF:
		{	
			break;
		}
		case BTN_TRY_TO_START:
		{
			values->Mode = MODE_TRY_TO_START;
			break;
		}
		case BTN_TRY_TO_STOP:
		{
			values->Mode = MODE_TRY_TO_STOP;
			break;
		}
	}

	
};



void ManagerPoll(struct DataStruct * values)
{
	CheckLevelSensors(values);
	CheckTemp(values);
	CheckPress(values);
	CheckUserCommands(values);
	CheckWQ(values);

	switch (values->Mode)
	{
		case MODE_IDLE:
			values->Pump = PUMP_OFF;
		break;
		case MODE_TRY_TO_START:
			StartPowerSupply();
			values->Mode = MODE_LEAK_CHECK1;
			values->VoltageSetpoint = MAX_VOLTAGE;
			values->OnOffButton = BTN_ON;
		break;
		case MODE_TRY_TO_STOP:
			StopPowerSupply();
			values->Mode = MODE_IDLE;
			values->Pump = PUMP_OFF;
			values->OnOffButton = BTN_OFF;
		break;
		case MODE_LEAK_CHECK1:
		{
			//values->AmperageSetpoint = LEAK_CHECK_AMPERAGE1;

			if (values->Press > LEAK_CHECK_PRESS1) 
			{
				char msg[80];
				//int size = 	sprintf(msg, "LEAK CHECK 1 PASSED");
				//SendToPC_RAW(msg, size);
				values->Mode = MODE_LEAK_CHECK2;
				break;
			}
			SetVoltageAmperageLowSpeed(values, LEAK_CHECK_AMPERAGE1);

			TickType_t xTimeWhenBlocking = xTaskGetTickCount();
			TickType_t xBlockedTime = 0;
			bool LeakCheckPass = false;
			
			while ((xBlockedTime < LEAK_CHECK_TIME1) && (!LeakCheckPass) )
			{
					xBlockedTime = xTaskGetTickCount() - xTimeWhenBlocking;
					if (values->Press > LEAK_CHECK_PRESS1) 
						{
							LeakCheckPass = true; 
						}
					if (values->OnOffButton == BTN_TRY_TO_STOP) return;
					GetAmperageFromPowerSupply(values);			
					vTaskDelay(100);
			}
			if (LeakCheckPass) 
			{
				char msg[80];
				int size = 	sprintf(msg, "TIME LEAK CHECK 1 = %d", xBlockedTime);
				//SendToPC_RAW(msg, size);
				values->Mode = MODE_LEAK_CHECK2;
			}
			else
			{
				char msg[80];
				int size = 	sprintf(msg, "TIME LEAK CHECK FAIL 1 = %d", xBlockedTime);
				//SendToPC_RAW(msg, size);
				values->Mode = MODE_ERROR_LEAK_CHECK;
			}
			break;
		}
		case MODE_LEAK_CHECK2:
		{

			if (values->Press > LEAK_CHECK_PRESS2) 
			{
				char msg[80];
				int size = 	sprintf(msg, "LEAK CHECK 2 PASSED");
				//SendToPC_RAW(msg, size);
				values->Mode = MODE_NORMAL_WORK;
				break;
			}

			SetVoltageAmperageLowSpeed(values, LEAK_CHECK_AMPERAGE2);
			TickType_t xTimeWhenBlocking = xTaskGetTickCount();
			TickType_t xBlockedTime = 0;
			bool LeakCheckPass = false;


			while ((xBlockedTime  < LEAK_CHECK_TIME2) && (!LeakCheckPass) )
			{
					xBlockedTime = xTaskGetTickCount() - xTimeWhenBlocking;
					if (values->Press > LEAK_CHECK_PRESS2) 
						{
							LeakCheckPass = true; 
						}
					if (values->OnOffButton == BTN_TRY_TO_STOP) return;
					GetAmperageFromPowerSupply(values);			
					vTaskDelay(100);
			}
			if (LeakCheckPass) 
			{
				char msg[80];
				int size = 	sprintf(msg, "TIME LEAK CHECK 2 = %d", xBlockedTime);
				//SendToPC_RAW(msg, size);
				values->Mode = MODE_NORMAL_WORK;
			}
			else
			{
				char msg[80];
				int size = 	sprintf(msg, "TIME LEAK CHECK FAIL 2 = %d", xBlockedTime);
				//SendToPC_RAW(msg, size);
				values->Mode = MODE_ERROR_LEAK_CHECK;
			}
			//values->AmperageUserSetpoint = 200;
			break;
		}
		case MODE_NORMAL_WORK:
		{
			if (values->Press < LEAK_CHECK_PRESS_IN_NORMAL_WORK) {values->Mode  = MODE_ERROR_LEAK_CHECK;}
			values->Pump = PUMP_ON;
			if ((values->NewAmperage))
			{
				values->NewAmperage = 0;
				if (values->AmperageUserSetpoint > MAX_AMPERAGE)  { values->AmperageUserSetpoint = MAX_AMPERAGE;}
				values->VoltageSetpoint = MAX_VOLTAGE;
			}
			uint16_t Setpoint = PID_Calculated(values->Press);
			if (Setpoint > values->AmperageUserSetpoint) 
      { 
			   Setpoint = values->AmperageUserSetpoint;
			} 
  		SetVoltageAmperageLowSpeed(values, Setpoint);
			//vTaskDelay(100);
			GetAmperageFromPowerSupply(values);
		}
		
		break;
		case MODE_ERROR_LEAK_CHECK:
		case MODE_ERROR_WQ:
		case MODE_ERROR_PRESS:
		case MODE_ERROR_TEMP:
		case MODE_ERROR_LOW_OXYGEN_LEVEL:
		case MODE_ERROR_HIGH_OXYGEN_LEVEL:
		case MODE_ERROR_LOW_HYDROGEN_LEVEL:
		case MODE_ERROR_HIGH_HYDROGEN_LEVEL:
		case MODE_ERROR_CONNECT_TO_POWER_SUPPLY:
		case MODE_ERROR_RESERVE_1:			
		case MODE_ERROR_RESERVE_2:
			values->Pump = PUMP_OFF;
			StopPowerSupply();
			SendAlarmMsg(values);
			while (true) {vTaskDelay(100);};
		break;
	}
};




#endif //_MANAGER_H