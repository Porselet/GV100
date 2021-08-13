#ifndef __POWERSUPPLY_H
#define __POWERSUPPLY_H


#define BUFSIZE 40


#include "stdbool.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include "DataStruct.h"
#include "PowerSupplyNew.h"
#include "PC_Interface.h"

#include "PID_regulator.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;


extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

#define PC_Uart huart4
#define PWR_SUPPLY_UART huart1
#define NEXTION_UART huart5


#define hdma_PC_Uart_rx hdma_uart4_rx;
#define hdma_PC_Uart_tx hdma_uart4_tx;
#define hdma_NEXTION_rx hdma_uart5_rx;
#define hdma_NEXTION_tx hdma_uart5_tx;
#define hdma_PWR_SUPPLY_rx hdma_usart1_rx;
#define hdma_PWR_SUPPLY_tx hdma_usart1_tx;


extern osSemaphoreId NextionMutexHandle;
/*	const uint8_t TimeoutConf[] =  { 0x01, 0x10, 0x00, 0x24, 0x00, 0x01, 0x02, 0x05, 0xDC, 0xA2, 0x7D};
	const uint8_t BitPowerOn[]  =  { 0x01, 0x0F, 0x01, 0x10, 0x00, 0x01, 0x01, 0x01, 0x2F, 0x45};
	const uint8_t BitPowerOff[] =  { 0x01, 0x0F, 0x01, 0x10, 0x00, 0x01, 0x01, 0x00, 0xEE, 0x85};
	const uint8_t BitWorkOn[]   =  { 0x01, 0x0F, 0x01, 0x11, 0x00, 0x01, 0x01, 0x01, 0x12, 0x85};
	const uint8_t BitWorkOff[]  =  { 0x01, 0x0F, 0x01, 0x11, 0x00, 0x01, 0x01, 0x00, 0xD3, 0x45};
	const uint8_t GetAmpVolt[]  =  { 0x01, 0x04, 0x00, 0x14, 0x00, 0x02, 0x31, 0xCF};
	
	volatile  uint8_t SetAmp[] 			= { 0x01, 0x10 , 0x00 , 0x12 , 0x00 , 0x02 , 0x04 , 0x00 , 0x14 , 0x01 , 0x90 , 0x32 , 0x82};
	*/
	uint8_t RxBuffPowerSupply[BUFSIZE];
	volatile uint8_t RxBufferRS485_2[BUFSIZE];
	volatile uint8_t RxBufferNextion[BUFSIZE];
	
	uint8_t rx_buff_len=0;
	volatile uint8_t rx_buff2_len=0;
	volatile uint8_t rx_buffNextion_len=0;

	int16_t WarningTimer = 0;
	bool uart5free = true;
	bool NextionFree = true;
	
uint16_t CRC16(const uint8_t *nData, uint16_t wLength);

void GetVlotageAmperage();

 void SetVoltageAmperage(struct DataStruct * values)
{
	SendCmdToPowerSupply(SET_VOLT_AMP_CMD, values);
};

void StartPowerSupply()
{
};

void StopPowerSupply()
{
	struct DataStruct val;
	while (!SendCmdToPowerSupply(BIT_WORK_OFF_CMD, &val)) {};
	while (!SendCmdToPowerSupply(BIT_POWER_OFF_CMD, &val)) {};
};

void SendToNextion(char * msg)
{

	taskENTER_CRITICAL();
	while (!NextionFree) {vTaskDelay(10);};
	if( NextionFree )
	{
		NextionFree = false;
		for (int i = 0; i < 80-3; i++)
		{
			if (msg[i] == 0)
				{
					msg[i]=0xFF;
					msg[i+1]=0xFF;
					msg[i+2]=0xFF;
					HAL_UART_Transmit_DMA(&huart5, (uint8_t*)msg, i+3);
					while (HAL_DMA_GetState(&hdma_uart5_tx) != HAL_DMA_STATE_READY) {	};
					while (HAL_UART_GetState(&huart5) == HAL_UART_STATE_BUSY_TX) {};
					break;
				}
		}
		NextionFree = true;
	}
	taskEXIT_CRITICAL();
	
	
};


//void SendMsgNextion(char * msg, uint8_t size)
//{
////	if (NextionMutexHandle == NULL) 
////	{
////		return;
////	}
//	//return;
//		taskENTER_CRITICAL();

//	while (!NextionFree) {vTaskDelay(10);};
//	if(NextionFree )
//	{
//		NextionFree = false;
//		char ch[] = "Mode.txt=\"";
//		HAL_UART_Transmit_DMA(&huart5, (uint8_t*)ch, 10);
//		while (HAL_DMA_GetState(&hdma_uart5_tx) != HAL_DMA_STATE_READY)  {};
//		HAL_UART_Transmit_DMA(&huart5, (uint8_t*)msg, size);
//		while (HAL_DMA_GetState(&hdma_uart5_tx) != HAL_DMA_STATE_READY) {	};
//		char ch2[4] = {'"', 0xFF,0xFF,0xFF};
//		HAL_UART_Transmit_DMA(&huart5, (uint8_t*)ch2, 4);
//		while (HAL_DMA_GetState(&hdma_uart5_tx) != HAL_DMA_STATE_READY) {	};
//		while (!uart5free) { };
//		NextionFree = true;
//	}
//	
//		taskEXIT_CRITICAL();
////	else
//	//{
//	//	__NOP();
//	//}

//	
//}



void SendAlarmNextion(uint8_t ErrNumber)
{
	char StrNextion[80]={0xFF};
  sprintf(StrNextion, "sys0=%d", (uint32_t)ErrNumber);
	SendToNextion(StrNextion);
	vTaskDelay(50);

  sprintf(StrNextion, "page AlarmPage");
	SendToNextion(StrNextion);
	vTaskDelay(50);
	
};

void NextionPoll(struct DataStruct * values)
{
	

	//grab msg from nextion
	if(rx_buffNextion_len)
	{
		rx_buffNextion_len = 0;
		values->LedMode = LED_FAST_BLINK;
		if  ((RxBufferNextion[0]=='b')&&(RxBufferNextion[1]=='t')&&(RxBufferNextion[2]=='n'))
		{
			if (values->OnOffButton == BTN_ON)
			{
				values->OnOffButton = BTN_TRY_TO_STOP;
			}
			if (values->OnOffButton == BTN_OFF)
			{
				values->OnOffButton = BTN_TRY_TO_START;
			}
		}
		if ((RxBufferNextion[0]=='a')&&(RxBufferNextion[1]=='m')&&(RxBufferNextion[2]=='p'))
		{
			values->AmperageUserSetpoint = ((uint16_t)RxBufferNextion[3]+ ((uint16_t)RxBufferNextion[4]*256)) / MULT_AMP_TO_FLOW;
			values->NewAmperage = true;		
		}
		
	}
	
	switch (values->NextionSleepMode)
	{
		case NEXTION_OFF:
			return;
		case NEXTION_ON:
			break;
		case NEXTION_TRY_TO_OFF:
		{
			values->NextionSleepMode = NEXTION_OFF;
			char StrNextion[15]={0xFF};
			sprintf(StrNextion, "sleep=1");
			SendToNextion(StrNextion);
			return;
		}
		case NEXTION_TRY_TO_ON:
		{
			char StrNextion[15]={0xFF};
			sprintf(StrNextion, "sleep=0");
			SendToNextion(StrNextion);
			values->NextionSleepMode = NEXTION_ON;
			break;
		}
	}
		
		char StrNextion[80]={0xFF};
/*		if (values->WQ < 5)
		{
			sprintf(StrNextion, "page0.t0.txt=\"the best!\"");
		}
		else
		{
			sprintf(StrNextion, "page0.t0.txt=\"normal\"");
		}
		
		SendToNextion(StrNextion);
		vTaskDelay(50);
		*/
		switch (values->Mode)
		{
			case MODE_LEAK_CHECK1:
			case MODE_LEAK_CHECK2:
				sprintf(StrNextion, "va0.val=1");
				SendToNextion(StrNextion);
				vTaskDelay(50);
				break;		
			case MODE_NORMAL_WORK:
				if ((PRESS_SETPOINT - values->Press < 0.1))
					sprintf(StrNextion, "va0.val=3");
					else 
						if ((MAX_VOLTAGE - values->Voltage) < 5)
						sprintf(StrNextion, "va0.val=4");
						else
							sprintf(StrNextion, "va0.val=2");
				SendToNextion(StrNextion);
				vTaskDelay(50);
				if ((WarningTimer == 0) && (values->WQ > WARNING_WQ))
				{
					WarningTimer = WARNING_WQ_TIME;
					sprintf(StrNextion, WARNING_PAGE_WQ);
					SendToNextion(StrNextion);
					vTaskDelay(50);
				}					
				break;		
			case MODE_ERROR_LEAK_CHECK:
				sprintf(StrNextion, ALARM_PAGE_LEAK_H2);
				SendToNextion(StrNextion);
				vTaskDelay(50);
				break;		
			case MODE_ERROR_WQ:
				sprintf(StrNextion, ALARM_PAGE_WQ);
				SendToNextion(StrNextion);
				vTaskDelay(50);
				break;		
			case MODE_ERROR_PRESS:
				sprintf(StrNextion, ALARM_PAGE_PRESS);
				//SendToNextion(AlarmPagePress);
				SendToNextion(ALARM_PAGE_PRESS);
				vTaskDelay(50);
				break;		
			case MODE_ERROR_TEMP:
				sprintf(StrNextion, ALARM_PAGE_LOW_TEMP);
				SendToNextion(StrNextion);
				vTaskDelay(50);
				break;		
			case MODE_ERROR_LOW_OXYGEN_LEVEL:
				sprintf(StrNextion, ALARM_PAGE_LOW_LEVEL_O2);
				SendToNextion(StrNextion);
				vTaskDelay(50);
				break;		
			case MODE_ERROR_HIGH_OXYGEN_LEVEL:
				sprintf(StrNextion, ALARM_PAGE_HIGH_LEVEL_O2);
				SendToNextion(StrNextion);
				vTaskDelay(50);
				break;		
			case MODE_ERROR_LOW_HYDROGEN_LEVEL:
				sprintf(StrNextion, ALARM_PAGE_LOW_LEVEL_H2);
				SendToNextion(StrNextion);
				vTaskDelay(50);
				break;		
			case MODE_ERROR_HIGH_HYDROGEN_LEVEL:
				sprintf(StrNextion, ALARM_PAGE_HIGH_LEVEL_H2);
				SendToNextion(StrNextion);
				vTaskDelay(50);
				break;		
			default:
				break;
		}
		
    sprintf(StrNextion, "Work_page.Pressure_H2.val=%d", (uint32_t)(values->Press*100));
		SendToNextion(StrNextion);
		vTaskDelay(50);
		
		
    sprintf(StrNextion, "Paramerts_page.Voltage.val=%d", (uint32_t)(values->Voltage));
		SendToNextion(StrNextion);
		vTaskDelay(50);
    sprintf(StrNextion, "Paramerts_page.Current.val=%d", (uint32_t)(values->Amperage));
		SendToNextion(StrNextion);
		vTaskDelay(50);

    sprintf(StrNextion, "Paramerts_page.Time_work.val=%d", (uint32_t)(values->AhCounter/3600));
		SendToNextion(StrNextion);
		vTaskDelay(50);

    sprintf(StrNextion, "RealFlow=%d", (uint32_t)(values->Amperage*MULT_AMP_TO_FLOW));
		SendToNextion(StrNextion);
		vTaskDelay(50);
    sprintf(StrNextion, "Paramerts_page.Temp.val=%d", (uint32_t)(values->Temperature));
		SendToNextion(StrNextion);
		vTaskDelay(50);
		
		
}

void SendToPC_RAW(char * msg, uint8_t size)
{
	HAL_GPIO_WritePin(DI2_GPIO_Port, DI2_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit_DMA(&huart4, (uint8_t*)msg, size);
	while (HAL_DMA_GetState(&hdma_uart4_tx) != HAL_DMA_STATE_READY) {	};
}




//void SendToPC(struct DataStruct * values)
//{


////SendValuesToPC(values);
//return;
//	static short va = 0;
//	va++;
//	values->Amperage = va;
//	uint8_t Answer[] = {2,4,(uint8_t)(values->Amperage), (uint8_t)(values->Amperage >> 8)};
//	HAL_GPIO_WritePin(DI2_GPIO_Port, DI2_Pin, GPIO_PIN_SET);
//	HAL_UART_Transmit_DMA(&huart4, Answer, sizeof(Answer));
//	while (HAL_DMA_GetState(&hdma_uart4_tx) != HAL_DMA_STATE_READY) {	};
//	
//	
//	return;
//		char StrPC[160] = {0};
//	//	    sprintf(StrPC, "%f, %d, %f ", (float)values->Temperature, values->A_IN_3_RAW, (float)values->WQ);
//    sprintf(StrPC, "A1=%d A2=%d A3=%d A4=%d Digital=%d%d%d%d, P=%f, T=%f, WQ=%f, AhCounter=%d, sysCounter=%d", 
//		   values->A_IN_1_RAW, values->A_IN_2_RAW, values->A_IN_3_RAW, values->A_IN_4_RAW,  
//		   values->D_IN_1, values->D_IN_2, values->D_IN_3, values->D_IN_4, 
//		   (float)values->Press, (float)values->Temperature, (float)values->WQ, values->AhCounter, values->sysCounter); //*/

//		
//		for (int i = 0; i < sizeof(StrPC); i++)
//		{
//			if (StrPC[i] == 0) 
//			{ 
//				StrPC[i]=13;
//				HAL_GPIO_WritePin(DI2_GPIO_Port, DI2_Pin, GPIO_PIN_SET);
//				HAL_UART_Transmit_DMA(&huart4, (uint8_t*)StrPC, i+1);
//				while (HAL_DMA_GetState(&hdma_uart4_tx) != HAL_DMA_STATE_READY) {	};
//				break;
//			}
//		}		
//		
//}
void GetVotageAmperage(struct DataStruct * values)
{
	SendCmdToPowerSupply(GET_VOLT_AMP_CMD, values);
}

void PowerSupplyPoll(struct DataStruct * values)
{
	ProcessingMsgPowerSupply(RxBuffPowerSupply, &rx_buff_len, values);

}







#endif
