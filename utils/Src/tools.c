/*
 * fonctions.c
 *
 *  Created on: Nov 8, 2024
 *      Author: Nicolas SIMOND
 */

#include "../Inc/tools.h"

//global variable from main.c
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef hlpuart1;

//global variable
uint8_t prompt[15] = "\nSTM32G431 >> ";
uint8_t message[50];


void setup() {
	strcpy((char*)message, "MSC 2024 - Capteurs\r\n");
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


/**@brief Make PA5 LED blink.
 * @param delay current delay
 * @retval HAL status
 *
 */
void test_LEDs(int delay){
	for(;;){
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
		HAL_Delay(delay);
	}

}

/**@brief Verify the the sensors available on I2C bus
 * @param None
 * @retval None
 *
 */
void verifySensor(){
	for (uint8_t i=0x00;i<0xFF;i++){
		HAL_StatusTypeDef status;
		status = HAL_I2C_IsDeviceReady(&hi2c1,i,3,100);
		if(status== HAL_OK){
			printf("[0x%02X|%d]: Peripheral found with adress [0x%02X]\r\n",i,(int)i,((uint16_t)i)>>1);
		}
		else if (status == HAL_TIMEOUT){
			printf("[0x%02X|%d]: Peripheral timed out with adress [0x%02X]\r\n",i,(int)i,((uint16_t)i)>>1);
		}
	}
}


/**@brief Verify the identity of the MPU-950 sensor
 * @param None
 * @retval uint8_t is_sensor : if 1 the sensor is detected 0 otherwise
 *
 */
uint8_t checkMPU9250Identity() {
	uint8_t who_am_i = 0; // Variable to store the read value
	// Reading WHO_AM_I register
	if (HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS << 1, WHO_AM_I_REGISTER, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, HAL_MAX_DELAY) == HAL_OK) {
		if (who_am_i == EXPECTED_WHO_AM_I) {
			printf("MPU-9250 sensor, detected with value [0x%02X]\r\n", EXPECTED_WHO_AM_I);
			return 1; // Sensor detected
		}
	}
	printf("MPU-9250 sensor, not detected\r\n");
	return 0; // Sensor not detected
}





