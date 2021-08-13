#include "NextionInterface.h"


#define rx_buffNextion_len RXBufferLen5
#define RxBufferNextion RXBuffer5

bool NextionFree = true;
int16_t WarningTimer = 0;

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