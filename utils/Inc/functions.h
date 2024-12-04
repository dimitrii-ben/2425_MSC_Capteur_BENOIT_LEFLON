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
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_i2c.h"

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
void I2C_Write_Register(uint8_t, uint8_t, uint8_t);
void I2C_Read_Register(uint8_t, uint8_t, uint8_t*);
void hardwareReset(uint8_t, uint8_t);
void clockSelection();
void InitSensors();
uint16_t rawTempMeasure();
uint16_t tempCalibration(uint16_t);
void getTempOffset(int repetition);
float TempMeasure();
extern void Error_Handler(void);
#endif /* INC_FUNCTIONS_H_ */
