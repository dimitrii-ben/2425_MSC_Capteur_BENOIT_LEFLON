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
		RoomTemp_Offset=0;
		for(int i =0; i <repetition;i++){
			RoomTemp_Offset+= (float)rawTempMeasure();
		}
		RoomTemp_Offset=RoomTemp_Offset/repetition;
	}

}

/**@brief Compute the real value of the I2C received value of temparture
 * @param I2C_Value
 * @retval TEMP_degC
 */
float tempCalibration(uint16_t I2C_Value){
	float TEMP_degC;
	TEMP_degC = ((float)I2C_Value-RoomTemp_Offset)/TEMP_SENSITIVITY + TEMP_DATASHEET_OFFSET;
	return TEMP_degC;
}

float TempMeasure(){
	uint16_t temp = rawTempMeasure();
	float calibrated_temperature = tempCalibration(temp);
	printf("Calibrated Temperature: %f\r\n",calibrated_temperature);
	return calibrated_temperature;
}
/**@brief Get FS (full scale) register value
 * @param register_sel_mask current register map to acess __FS_SELL value
 * @param config_addr current CONFIG_ADDR for the current component [Gyroscope,Accelerometer]
 * @retval FS : full scale register value
 */

uint8_t getFS(uint8_t component){

	uint8_t config_reg_value;
	if (component == GYROSCOPE_COMP){
		I2C_Read_Register(MPU9250_ADDRESS, GYRO_FS_SEL_ADDR, &config_reg_value);
		return (config_reg_value & GYRO_FS_SEL_MASK)>>3;
	}
	else if (component == ACCELEROMETER_COMP){
		I2C_Read_Register(MPU9250_ADDRESS, ACC_FS_SEL_ADDR, &config_reg_value);
		return (config_reg_value & ACC_FS_SEL_MASK)>>3;
	}
	return -1;

}
short getSensitivity(uint8_t component){
	if (component == GYROSCOPE_COMP){
		return GET_GYRO_SENS(getFS(component));
	}
	else if (component ==ACCELEROMETER_COMP){
		return GET_ACC_SENS(getFS(component));
	}
	return -1;
}
/**@brief combine two 8 bits word (LSB and MSB)
 * @param regLowByte : LSB
 * @param regHighByte: MSB
 * @retval short
 */
short combineLH(int8_t regLowByte,int8_t regHighByte){
	return (int16_t)((regHighByte << 8) | regLowByte);

}
/**@brief Calibrate the I2C accelerometer/gyrometer received value
 * @param accValue : the acceleration value (raw)
 * @retval None
 */
double calibrateValue(short accValue,double sensitivity){
	return ((double)accValue/sensitivity);
}
/**@brief Measure the current component value
 * @param accel_table : A reserved memory space for the x,y,z value of the accelerometer
 * @retval None
 */
void componentMeasure(uint8_t current_component,double* component_table){
	uint8_t L_x,L_y,L_z,H_x,H_y,H_z;
	int16_t raw_x,raw_y,raw_z;
	short current_sensitivity = getSensitivity(current_component);
	double x,y,z;
	//x
	I2C_Read_Register(MPU9250_ADDRESS, ACCEL_XOUT_L, &L_x);
	I2C_Read_Register(MPU9250_ADDRESS, ACCEL_XOUT_H, &H_x);
	//y
	I2C_Read_Register(MPU9250_ADDRESS, ACCEL_YOUT_L, &L_y);
	I2C_Read_Register(MPU9250_ADDRESS, ACCEL_YOUT_H, &H_y);
	//z
	I2C_Read_Register(MPU9250_ADDRESS, ACCEL_ZOUT_L, &L_z);
	I2C_Read_Register(MPU9250_ADDRESS, ACCEL_ZOUT_H, &H_z);
	//combining LSB and MSB
	raw_x = combineLH(L_x,H_x);
	raw_y = combineLH(L_y,H_y);
	raw_z = combineLH(L_z,H_z);
	//calibrating raw values
	x= calibrateValue(raw_x,current_sensitivity);
	y= calibrateValue(raw_y, current_sensitivity);
	z= calibrateValue(raw_z, current_sensitivity);

	component_table[0]=x;
	component_table[1]=y;
	component_table[2]=z;
	//printf("[XOUT_L,XOUT_H]:[%d,%d]|-|[YOUT_L,YOUT_H]:[%d,%d]|-|[YOUT_L,YOUT_H]:[%d,%d]\r\n",L_x,L_y,L_z,H_x,H_y,H_z);
	//printf("raw_x=%d|-|raw_y=%d|-|raw_z=%d\r\n",raw_x,raw_y,raw_z);
	printf("X=%f|-|Y=%f|-|Z=%f\r\n",x,y,z);

}
float getVectorNorm(double* vector_table){
	return sqrt(pow(vector_table[0],2)+pow(vector_table[1],2)+pow(vector_table[2],2));
}
/*
 * ACCELEROMETER
 */

/**@brief Measure the accelerometer value
 * @param gyro_table : A reserved memory space for the x,y,z value of the accelerometer
 * @retval None
 */
void AccMeasure(double* acc_table){
	componentMeasure(ACCELEROMETER_COMP,acc_table);
}

/*
 * GYROSCOPE
 */

/**@brief Measure the gyroscope value
 * @param gyro_table : A reserved memory space for the x,y,z value of the gyroscope
 * @retval None
 */
void GyroMeasure(double* gyro_table){
	componentMeasure(GYROSCOPE_COMP,gyro_table);
}

/*
 *  MAGNETOMETER
 */
// pour communiquer avec le magnetometre AK il faut configurer le bus auxiliere I2C_SL0 il y a trois etape pour effectuer une lecture/une ecriture:
//[1]Configurer l'adresse I2C_SLV0_ADDR courante (le composant qu'on va vouloir utiliser lors du processus ici pour le magnetometre on communiquera avec l'AK8963)
//[2]Configurer l'adresse I2C_SLV0_REG courante (le registre qu'on va vouloir ecrire/lire lors du processus, par exemple WHO_AM_I)
//[3]Configurer l'adresse I2C_SLV0_CTRL, pour gerer les acces ou l'on pourra moduler la longueur des mots a lire
void ConfigureI2CSlave(uint8_t slave_addr, uint8_t is_read) {
	uint8_t value = (slave_addr << 1) | (is_read ? 1 : 0);  // Adresse avec bit R/W (LSB)
	I2C_Write_Register(MPU9250_ADDRESS, I2C_SLV0_ADDR, value);
}

void SetSlaveRegister(uint8_t reg_addr) {
	I2C_Write_Register(MPU9250_ADDRESS, I2C_SLV0_REG, reg_addr);
}

void ConfigureSlaveControl(uint8_t data_length) {
	uint8_t value = 0x80 | (data_length & 0x0F);  // Activer et définir la longueur
	I2C_Write_Register(MPU9250_ADDRESS, I2C_SLV0_CTRL, value);
}

uint8_t ReadMagnetometerWhoAmI() {
	// Configurer l'accès au magnétomètre
	ConfigureI2CSlave(AK8963_ADDR, 1);        // Adresse AK8963 en mode lecture
	SetSlaveRegister(WHO_AM_I_AK8963);        // Registre WHO_AM_I
	ConfigureSlaveControl(1);          // Lire 1 octet

	// Lire le résultat via le registre du MPU-9250
	uint8_t who_am_i = 0;
	I2C_Read_Register(MPU9250_ADDRESS, I2C_SLV0_DO, &who_am_i);  // Lecture depuis I2C_SLV0_DO
	return who_am_i;
}
//CNTL1 REG CONFIGURATION
void ConfigureMagnetometer(I2C_HandleTypeDef *hi2c) {
    uint8_t config = 0x16;  // Continuous Measurement Mode 2, 16-bit output
    I2C_Write_Register(AK8963_ADDR << 1, AK8963_CNTL, config);
    HAL_Delay(10);  // Délai pour laisser le magnétomètre s'initialiser
}

// Fonction pour lire les données magnétiques ajustées
void MagMeasure(I2C_HandleTypeDef *hi2c, double *mag_data) {
    uint8_t raw_data[6] = {0};     // Données brutes magnétiques (6 octets)
    uint8_t asa[3] = {0};          // Valeurs d'ajustement de sensibilité (ASAX, ASAY, ASAZ)
    int16_t raw_magnetic[3] = {0}; // Données brutes magnétiques en 16 bits

    // Lire les valeurs ASA (ajustement de sensibilité)
    HAL_I2C_Mem_Read(hi2c, AK8963_ADDR << 1, AK8963_ASAX, 1, &asa[0], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(hi2c, AK8963_ADDR << 1, AK8963_ASAY, 1, &asa[1], 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(hi2c, AK8963_ADDR << 1, AK8963_ASAZ, 1, &asa[2], 1, HAL_MAX_DELAY);

    // Lire les données brutes magnétiques (6 octets consécutifs)
    HAL_I2C_Mem_Read(hi2c, AK8963_ADDR << 1, AK8963_XOUT_L, 1, raw_data, 6, HAL_MAX_DELAY);

    // Combiner les octets pour obtenir les valeurs brutes par axe
    raw_magnetic[0] = (int16_t)((raw_data[1] << 8) | raw_data[0]);  // Axe X
    raw_magnetic[1] = (int16_t)((raw_data[3] << 8) | raw_data[2]);  // Axe Y
    raw_magnetic[2] = (int16_t)((raw_data[5] << 8) | raw_data[4]);  // Axe Z

    // Ajuster les données brutes en utilisant les valeurs ASA
    for (int i = 0; i < 3; i++) {
        mag_data[i] = raw_magnetic[i] * (((asa[i] - 128) * 0.5 / 128) + 1);
    }
}

