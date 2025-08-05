/*
 * lis2mdl_driver.h
 *
 *  Created on: Aug 1, 2025
 *      Author: yosif
 */

#ifndef INC_LIS2MDL_DRIVER_H_
#define INC_LIS2MDL_DRIVER_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"

// I2C address for LIS2MDL (7-bit shifted left for HAL)
#define LIS2MDL_I2C_ADDR      (0x3C)

// Initialize the magnetometer (if any register setup is needed)
void LIS2MDL_Init(void);

// Read raw 16-bit counts from OUTX_L..OUTZ_H
void LIS2MDL_ReadRaw(int16_t *x, int16_t *y, int16_t *z);

// Read and convert raw counts to µT
void LIS2MDL_ReadMagnetic(float *x_uT, float *y_uT, float *z_uT);

#endif // INC_LIS2MDL_DRIVER_H_

