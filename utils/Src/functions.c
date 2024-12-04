/*
 * functions.c
 *
 *  Created on: Nov 20, 2024
 *      Author: matth
 */

#include "../Inc/functions.h"
//global variable
float RoomTemp_Offset=-1;

//global variable from main.c
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef hlpuart1;



/**@brief Function to read in a register of the I2C
 * @param deviceAddress sensor adress
 * @param registerAddress register adress
 * @param value value to write in the register
 * @retval None
 *
 */
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


/**@brief Function to read a register on the I2C
 * @param deviceAddress sensor adress
 * @param registerAddress register adress
 * @param value buffer pointer
 * @retval None
 *
 */
void I2C_Read_Register(uint8_t deviceAddress, uint8_t registerAddress, uint8_t* value)
{
	// First step: Write the register address to the I2C device
	if (HAL_I2C_Master_Transmit(&hi2c1, (deviceAddress << 1), &registerAddress, 1, HAL_MAX_DELAY) != HAL_OK) {
		printf("Error in writing the register address [0x%02X]\n", registerAddress);
		Error_Handler();
	}

	HAL_Delay(100);
	// Second step: Read the value from the specified register
	if (HAL_I2C_Master_Receive(&hi2c1, (deviceAddress << 1) | 1, value, 1, HAL_MAX_DELAY) != HAL_OK) {
		printf("Error in reading the register value from [0x%02X]\n", registerAddress);
		Error_Handler();
	}
}


/**@brief Reset all the registers from address1 to address2
 * @param [adress1,adress2]: both addresses in hex
 * @retval None
 *
 */
void hardwareReset(uint8_t address1,uint8_t adress2){

	for (int currentAddr = address1;currentAddr<=adress2;currentAddr++){
		I2C_Write_Register(MPU9250_ADDRESS, address1, RESET_VALUE);
	}
}


/**@brief Set default value of 2 particular register different of 0x00
 * @param None
 * @retval None
 *
 */
void setDefaultValue(){
	I2C_Write_Register(MPU9250_ADDRESS, WHO_AM_I_REGISTER,EXPECTED_WHO_AM_I);
	I2C_Write_Register(MPU9250_ADDRESS, PWR_MGMT_1,PWR_MGMT_1_DEFAULT);
}


/**
 * @brief Choose the clock for the synchronisation
 * @param None
 * @retval None
 *
 */
void clockSelection(){
	I2C_Write_Register(MPU9250_ADDRESS, PWR_MGMT_1, CLKSEL_VALUE);
}


/**@brief Initialize full sensor registers and choosing clock
 * @param None
 * @retval None
 *
 */
void InitSensors(){
	hardwareReset(FIRST_SENSOR_ADRESS, LAST_SENSOR_ADRESS);
	setDefaultValue();
	HAL_Delay(100);
	clockSelection();
	getTempOffset(TEMP_INIT_AVG_REPETITION);
}


/**@brief Measure and display of the temperature on the console
 * @param None
 * @retval None
 *
 */
uint16_t rawTempMeasure(){
	uint8_t temperatureLowByte;
	uint8_t temperatureHighByte;
	uint16_t temperatureRaw;
	I2C_Read_Register(MPU9250_ADDRESS, TEMP_OUT_L, &temperatureLowByte);
	I2C_Read_Register(MPU9250_ADDRESS, TEMP_OUT_H, &temperatureHighByte);
	// Combine the high and low bytes into a 16-bit signed value
	temperatureRaw = (int16_t)((temperatureHighByte << 8) | temperatureLowByte);
	printf("TEMP_OUT_L: 0x%02X, TEMP_OUT_H: 0x%02X\r\n", temperatureLowByte, temperatureHighByte);
	// Print the raw temperature value
	printf("Raw Temperature: %d\r\n", temperatureRaw);
	return temperatureRaw;
}
/**@brief Compute the temperature offset, through an average
 * @param repetition : the number of petition
 * @retval RoomTemp_Offset
 */
void getTempOffset(int repetition){
	if (RoomTemp_Offset<0){
		uint16_t RoomTemp_Offset=0;
		for(int i =0; i <repetition;i++){
			RoomTemp_Offset+=rawTempMeasure();
		}
		RoomTemp_Offset=RoomTemp_Offset/repetition;
	}

}

/**@brief Compute the real value of the I2C received value of temparture
 * @param I2C_Value
 * @retval TEMP_degC
 */
uint16_t tempCalibration(uint16_t I2C_Value){
	float TEMP_degC;
	TEMP_degC = ((float)I2C_Value-RoomTemp_Offset)/(float)TEMP_SENSITIVITY + (float)TEMP_DATASHEET_OFFSET;
	return TEMP_degC;
}

float TempMeasure(){
	uint16_t temp = rawTempMeasure();
	float calibrated_temperature = tempCalibration(temp);
	printf("Calibrated Temperature: %f\r\n",calibrated_temperature);
	return calibrated_temperature;
}
