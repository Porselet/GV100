#ifndef __UARTS_H 
#define __UARTS_H


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



extern volatile uint8_t RXBuffer1[BUFSIZE];
extern volatile uint8_t RXBuffer4[BUFSIZE];
extern volatile uint8_t RXBuffer5[BUFSIZE];
	
extern volatile uint8_t RXBufferLen1;
extern volatile uint8_t RXBufferLen4;
extern volatile uint8_t RXBufferLen5;


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_IDLE_Callback(UART_HandleTypeDef *huart);	
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);	
void InitUartCallback(UART_HandleTypeDef *huart, void(*callBackRX)(void), void(*callBackTX)(void));
void InitUarts();
void SendBytesUART(uint8_t * msg, uint8_t size, UART_HandleTypeDef *huart);

	
	#endif //__UARTS_H