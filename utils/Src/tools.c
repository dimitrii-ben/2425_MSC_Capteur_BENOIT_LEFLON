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
void verifySensor(){
	for (uint8_t i=0x00;i<0xFF;i++){

		if(HAL_I2C_IsDeviceReady(&hi2c1,i,3,100)== HAL_OK){
			printf("[0x%02X|%d]: Peripheral avaible,avaible at [0x%02X]\r\n",i,(int)i,((uint16_t)i)>>1);
		}else{
			printf("verifySensor error");
			Error_Handler();
		}

	}
}
/**
 * @brief Verify the identity of the MPU-950 sensor
 * @retval uint8_t is_sensor : if 1 the sensor is detected 0 otherwise
 */
uint8_t checkMPU9250Identity() {
	uint8_t who_am_i = 0; // Variable pour stocker la valeur lue
	// Reading WHO_AM_I register
	if (HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS << 1, WHO_AM_I_REGISTER, I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, HAL_MAX_DELAY) == HAL_OK) {
		if (who_am_i == EXPECTED_WHO_AM_I) {
			printf("MPU-950 sensor, detected\r\n");
			return 1; // Sensor detected
		}
	}
	printf("MPU-950 sensor, not detected\r\n");
	return 0; // Sensor not detected
}
void I2C_Write_Register(uint8_t deviceAddress, uint8_t registerAddress, uint8_t value)
{
    uint8_t data[2];
    data[0] = registerAddress; // Register address to write to
    data[1] = value;           // Data to write to the register

    // Write the register address and value to the I2C device
    // the LSB is the R/W bit
    // Should be set to 0 if the desired action is to write 0 otherwise
    if (HAL_I2C_Master_Transmit(&hi2c1, deviceAddress << 1, data, 2, HAL_MAX_DELAY)!= HAL_OK){
    	printf("Error in writing at the registerAdress [0x%02X]",registerAddress);
    	Error_Handler();
    }
}
/**
 * @brief Reset all the registers from address1 to address2
 * @param [adress1,adress2]: both addresses in hex
 * @retval None
 */
void hardwareReset(uint8_t address1,uint8_t adress2){

	for (int currentAddr = address1;currentAddr<adress2+1;currentAddr++){
		I2C_Write_Register(MPU9250_ADDRESS, address1, RESET_VALUE);
	}

}
void setDefaultValue(){
	I2C_Write_Register(MPU9250_ADDRESS, WHO_AM_I_REGISTER,EXPECTED_WHO_AM_I);
	I2C_Write_Register(MPU9250_ADDRESS, PWR_MGMT_1,PWR_MGMT_1_DEFAULT);
}
/**
 * @brief Make PA5 LED blink.
 * @param delay : current delay
 * @retval None
 */
void InitSensors(){

}



