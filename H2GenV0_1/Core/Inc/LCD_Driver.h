#ifndef __LCD_DRIVER_H
#define __LCD_DRIVER_H

#define PIN_RS    (1 << 0)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)
#define LCD_ADDR ( 0x4E)
#define LCD_DELAY_MS 5

#include "stm32f1xx_hal.h"
extern I2C_HandleTypeDef hi2c1;


void I2C_send(uint8_t data, uint8_t flags)
{
	HAL_StatusTypeDef res;
        for(;;)  {
	        res = HAL_I2C_IsDeviceReady(&hi2c1, LCD_ADDR, 1, HAL_MAX_DELAY);               
                if(res == HAL_OK) 
								{
									break;
								}                                                  
	    }
 	uint8_t up = data & 0xF0;   
  uint8_t lo = (data << 4) & 0xF0;          
                                          
	uint8_t data_arr[4];
	data_arr[0] = up|flags|BACKLIGHT|PIN_EN; 
	data_arr[1] = up|flags|BACKLIGHT;         
	data_arr[2] = lo|flags|BACKLIGHT|PIN_EN;
	data_arr[3] = lo|flags|BACKLIGHT;

	HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, data_arr, sizeof(data_arr), HAL_MAX_DELAY);
	HAL_Delay(LCD_DELAY_MS);
}

void LCD_SendString(char *str)
{
	while(*str) 
        {                                   
		I2C_send((uint8_t)(*str), 1); 
    str++;                                     
        }
}

	
#endif
