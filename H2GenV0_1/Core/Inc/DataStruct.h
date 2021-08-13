#ifndef __DATASTRUCT_H
#define __DATASTRUCT_H

#include "stdint.h"
 
enum PinLevel
{
	LOW,
	HIGH
};


	


struct DataStruct
{
	uint16_t A_IN_1_RAW;
	uint16_t A_IN_2_RAW;
	uint16_t A_IN_3_RAW;
	uint16_t A_IN_4_RAW;
	
	char D_IN_1;
	char D_IN_2;
	char D_IN_3;
	char D_IN_4;
	char D_IN_5;
	char D_IN_6;
	
	uint16_t Amperage;
	uint16_t Voltage;
	
	uint16_t AmperageUserSetpoint;
	uint16_t AmperageSetpoint;
	uint16_t VoltageSetpoint;
	
	double Press;
	double Temperature;
	double WQ;
	
	enum PinLevel D_Output_00;
	enum PinLevel D_Output_01;
	enum PinLevel D_Input_00;
	enum PinLevel D_Input_01;
	
	enum 	
	{
		BTN_OFF,
		BTN_TRY_TO_START,
		BTN_ON,
		BTN_TRY_TO_STOP
	} OnOffButton; 
	volatile uint8_t OnOffButton2;
	char NewAmperage;
	
	enum 
	{
		NEXTION_TRY_TO_OFF,
		NEXTION_TRY_TO_ON,
		NEXTION_ON,
		NEXTION_OFF
	} NextionSleepMode;
	
	enum
	{
		LED_NORMAL_BLINK,
		LED_FAST_BLINK,
		LED_VERY_FAST_BLINK,
	} LedMode;
	enum 
	{
		PUMP_OFF,
		PUMP_ON,
	} Pump;
	enum GENERATOR_MODES
	{
		MODE_IDLE,
		MODE_TRY_TO_START,
		MODE_LEAK_CHECK1,
		MODE_LEAK_CHECK2,
		MODE_NORMAL_WORK,
		MODE_TRY_TO_STOP,
		MODE_ERROR_LEAK_CHECK,
		MODE_ERROR_WQ,
		MODE_ERROR_PRESS,
		MODE_ERROR_TEMP,
		MODE_ERROR_LOW_OXYGEN_LEVEL,
		MODE_ERROR_HIGH_OXYGEN_LEVEL,
		MODE_ERROR_LOW_HYDROGEN_LEVEL,
		MODE_ERROR_HIGH_HYDROGEN_LEVEL,
		MODE_ERROR_CONNECT_TO_POWER_SUPPLY,
		MODE_ERROR_RESERVE_1,
		MODE_ERROR_RESERVE_2,
	}Mode;
	uint32_t AhCounter;
	uint32_t sysCounter;
};	


#endif //__DATASTRUCT_H
