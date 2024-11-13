/*
 * fonctions.h
 *
 *  Created on: Nov 8, 2024
 *      Author: Nicolas SIMOND
 */

#ifndef INC_TOOLS_H_
#define INC_TOOLS_H_
#define MPU9250_ADDRESS 0x68
#define WHO_AM_I_REGISTER 0x75
#define EXPECTED_WHO_AM_I 0x71
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_i2c.h"
void setup();
void loop();
void test_LEDs(int);
void verifySensor();
uint8_t checkMPU9250Identity();
#endif /* INC_TOOLS_H_ */
