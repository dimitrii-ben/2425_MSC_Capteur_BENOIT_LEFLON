/*
 * fonctions.c
 *
 *  Created on: Nov 8, 2024
 *      Author: Nicolas SIMOND
 */

#include "../Inc/tools.h"

extern UART_HandleTypeDef hlpuart1;
uint8_t prompt[15] = "\nSTM32G431 >> ";
uint8_t message[50];

void setup() {
	strcpy((char*)message, "MSC 2024 - Capteurs\r");
	HAL_UART_Transmit(&hlpuart1, prompt, strlen((char*)prompt), HAL_MAX_DELAY);
	HAL_UART_Transmit(&hlpuart1, message, strlen((char*)message), HAL_MAX_DELAY);
}

void loop(){
	static uint32_t cnt = 0;
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	HAL_Delay(500);
	snprintf((char*)message, 10, "cnt : %d\r", (int)cnt++);
	HAL_UART_Transmit(&hlpuart1, prompt, strlen((char*)prompt), HAL_MAX_DELAY);
	HAL_UART_Transmit(&hlpuart1, message, strlen((char*)message), HAL_MAX_DELAY);
}
/**
  * @brief Make PA5 LED blink.
  * @param delay : current delay
  * @retval HAL status
  */
void test_LEDs(int delay){
	for(;;){
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
		HAL_Delay(delay);
	}

}
