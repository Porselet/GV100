#include "UARTs.h"

volatile uint8_t RXBuffer1[BUFSIZE];
volatile uint8_t RXBuffer4[BUFSIZE];
volatile uint8_t RXBuffer5[BUFSIZE];
	
volatile uint8_t RXBufferLen1=0;
volatile uint8_t RXBufferLen4=0;
volatile uint8_t RXBufferLen5=0;


void (*CallBackRX1) (void);
void (*CallBackRX4) (void);
void (*CallBackRX5) (void);

void (*CallBackTX1) (void);
void (*CallBackTX4) (void);
void (*CallBackTX5) (void);



void HAL_UART_IDLE_Callback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
		RXBufferLen1 = BUFSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
		if (CallBackRX1 != NULL) 	CallBackRX1();
		HAL_UART_AbortReceive(&huart1);
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		HAL_UART_Receive_DMA(&huart1, (uint8_t*)RXBuffer1, BUFSIZE);
	}
	if(huart == &huart4)
	{
		__HAL_UART_DISABLE_IT(&huart4, UART_IT_IDLE);
		RXBufferLen4 = BUFSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
		if (CallBackRX4 != NULL) 	CallBackRX4();
		//HAL_UART_DMAStop(&huart1);
		//uint8_t res = HAL_UART_Transmit_DMA(&huart1, (uint8_t*)rx_buff, rx_buff_len);
		HAL_UART_AbortReceive(&huart4);
		__HAL_UART_CLEAR_IDLEFLAG(&huart4);
		__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
		HAL_UART_Receive_DMA(&huart4, (uint8_t*)RXBuffer4, BUFSIZE);
	}
	if(huart == &huart5)
	{
		__HAL_UART_DISABLE_IT(&huart5, UART_IT_IDLE);
		RXBufferLen5 = BUFSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
		if (CallBackRX5 != NULL) 	CallBackRX5();
		//HAL_UART_DMAStop(&huart1);
		//uint8_t res = HAL_UART_Transmit_DMA(&huart1, (uint8_t*)rx_buff, rx_buff_len);
		HAL_UART_AbortReceive(&huart5);
		__HAL_UART_CLEAR_IDLEFLAG(&huart5);
		__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
		HAL_UART_Receive_DMA(&huart5, (uint8_t*)RXBuffer5, BUFSIZE);
	}
}

/////////////////////////////////// full ///////////////////////////////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	  if(huart == &huart1)
	  {
		  __HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
		  //HAL_UART_DMAStop(&huart1);
		  HAL_UART_Transmit_DMA(&huart4, (uint8_t*)"Full buffer\n", 12);
		  HAL_UART_AbortReceive(&huart1);
		  __HAL_UART_CLEAR_IDLEFLAG(&huart1);
		  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		  HAL_UART_Receive_DMA(&huart1, (uint8_t*)RXBuffer1, BUFSIZE);
			//onGetBytesPowerSupply();
	  }
	  if(huart == &huart4)
	  {
		  __HAL_UART_DISABLE_IT(&huart4, UART_IT_IDLE);
		  //HAL_UART_DMAStop(&huart1);
		  HAL_UART_Transmit_DMA(&huart4, (uint8_t*)"Full buffer\n", 12);
		  HAL_UART_AbortReceive(&huart4);
		  __HAL_UART_CLEAR_IDLEFLAG(&huart4);
		  __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
		  HAL_UART_Receive_DMA(&huart4, (uint8_t*)RXBuffer4, BUFSIZE);
	  }		
	  if(huart == &huart5)
	  {
		  __HAL_UART_DISABLE_IT(&huart5, UART_IT_IDLE);
		  //HAL_UART_DMAStop(&huart1);
		  HAL_UART_Transmit_DMA(&huart4, (uint8_t*)"Full buffer\n", 12);
		  HAL_UART_AbortReceive(&huart5);
		  __HAL_UART_CLEAR_IDLEFLAG(&huart5);
		  __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
		  HAL_UART_Receive_DMA(&huart5, (uint8_t*)RXBuffer5, BUFSIZE);
	  }		
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
  {
		HAL_GPIO_WritePin(DI1_GPIO_Port, DI1_Pin, GPIO_PIN_RESET);
		if (CallBackTX1 != NULL) 	CallBackTX1();
  }     
  if(huart == &huart4)
  {
		HAL_GPIO_WritePin(DI2_GPIO_Port, DI2_Pin, GPIO_PIN_RESET);
		if (CallBackTX4 != NULL) 	CallBackTX4();
  }     
  if(huart == &huart5)
  {
		if (CallBackTX5 != NULL) 	CallBackTX5();
  }     
}	

void InitUartCallback(UART_HandleTypeDef *huart, void(*callBackRX)(void), void(*callBackTX)(void))
{
	if(huart == &huart1)
  {
		CallBackRX1 = callBackRX;
		CallBackTX1 = callBackTX;
  }     
	if(huart == &huart4)
  {
		CallBackRX4 = callBackRX;
		CallBackTX4 = callBackTX;
  }     
	if(huart == &huart5)
  {
		CallBackRX5 = callBackRX;
		CallBackTX5 = callBackTX;
  }     
	
};


void InitUarts( )
{
	
	HAL_UART_Receive_DMA(&huart1, (uint8_t*)RXBuffer1, BUFSIZE);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart4, (uint8_t*)RXBuffer4, BUFSIZE);
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart5, (uint8_t*)RXBuffer5, BUFSIZE);
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
};


void SendBytesUART(uint8_t * bytes, uint8_t size, UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		HAL_GPIO_WritePin(DI1_GPIO_Port, DI1_Pin, GPIO_PIN_SET);
		HAL_UART_Transmit_DMA(&huart1, bytes, size);
		while (HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_READY) {	vTaskDelay(5);};
		while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) {	vTaskDelay(5);};
	}
	if(huart == &huart4)
	{
		HAL_GPIO_WritePin(DI2_GPIO_Port, DI2_Pin, GPIO_PIN_SET);
		HAL_UART_Transmit_DMA(&huart4, bytes, size);
		while (HAL_DMA_GetState(&hdma_uart4_tx) != HAL_DMA_STATE_READY) {	vTaskDelay(5);};
	}
	if(huart == &huart5)
	{
		HAL_UART_Transmit_DMA(&huart5, bytes, size);
		while (HAL_DMA_GetState(&hdma_uart5_tx) != HAL_DMA_STATE_READY) {	vTaskDelay(5);};
	}
				
};
