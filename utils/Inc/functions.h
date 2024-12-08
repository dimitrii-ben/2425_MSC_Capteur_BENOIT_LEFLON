/*
 * functions.h
 *
 *  Created on: Nov 20, 2024
 *      Author: matth
 */

#ifndef INC_FUNCTIONS_H_
#define INC_FUNCTIONS_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_i2c.h"

//define
#define MPU9250_ADDRESS 0x68
#define WHO_AM_I_REGISTER 0x75
#define EXPECTED_WHO_AM_I 0x71
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_1_DEFAULT 0x01
#define FIRST_SENSOR_ADRESS 0x00
#define LAST_SENSOR_ADRESS 0x7E
#define CLKSEL_VALUE 0x01
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define TEMP_DATASHEET_OFFSET 21
#define RESET_VALUE 0
#define TEMP_SENSITIVITY 333.87
#define TEMP_INIT_AVG_REPETITION 10
//ACCELEROMETER
#define ACCELEROMETER_COMP 0
#define ACC_FS_SEL_ADDR 0x1B
#define ACC_FS_SEL_MASK 0x18
//offset
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E
//measurements
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
//sensitivity macro
// Define a table using macros
#define GET_ACC_SENS(FS_SEL) (FS_SEL == 0 ? 16384 : \
                        FS_SEL == 1 ? 8192 : \
                        FS_SEL == 2 ? 4096 : \
                        FS_SEL == 3 ? 2048 : \
                        -1)  // Return -1 if key is invalid

//GYROSCOPE
#define GYROSCOPE_COMP 1
#define GYRO_FS_SEL_ADDR 0x1B
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define GET_GYRO_SENS(FS_SEL) (FS_SEL == 0 ? 131 : \
                        FS_SEL == 1 ? 65.5 : \
                        FS_SEL == 2 ? 32.8 : \
                        FS_SEL == 3 ? 16.4 : \
                        -1)  // Return -1 if key is invalid
#define GYRO_FS_SEL_MASK 0x18
// MPU TO MAGNETOMETER
#define AK8963_ADDR 0x0C //in reading mode
/***************************************************
 *      REGISTER MAP MAGNETOMETER AK8963
 **************************************************/

#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value
#define AK_ADD 			0x18

//
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV0_DO      0x63
//function definition
void I2C_Write_Register(uint8_t, uint8_t, uint8_t);
void I2C_Read_Register(uint8_t, uint8_t, uint8_t*);
void hardwareReset(uint8_t, uint8_t);
void clockSelection();
void InitSensors();
uint16_t rawTempMeasure();
float tempCalibration(uint16_t);
void getTempOffset(int repetition);
float TempMeasure();
uint8_t getFS(uint8_t component);
void componentMeasure(uint8_t current_component,double* component_table);
short getSensitivity(uint8_t component);
double calibrateValue(short accValue,double sensitivity);
short combineLH(int8_t regHighByte,int8_t regLowByte);
extern void Error_Handler(void);
float getVectorNorm(double* vector_table);
/*
 * ACCELEROMETER
 */
void AccMeasure(double* acc_table);
/*
 * GYROSCOPE
 */
void GyroMeasure(double*);

/*
 * MAGNETOMETER
 */
void ConfigureI2CSlave(uint8_t slave_addr, uint8_t is_read);
void SetSlaveRegister(uint8_t reg_addr);
void ConfigureSlaveControl(uint8_t data_length);
uint8_t ReadMagnetometerWhoAmI();
void MagMeasure(I2C_HandleTypeDef *hi2c, double *mag_data);
#endif /* INC_FUNCTIONS_H_ */


