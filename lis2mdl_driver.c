/*
 * lis2mdl_driver.c
 *
 *  Created on: Aug 1, 2025
 *      Author: yosif
 */

#include "lis2mdl_driver.h"

extern I2C_HandleTypeDef hi2c1;

void LIS2MDL_Init(void)
{
    // Optionally configure control registers here via HAL_I2C_Mem_Write
    // e.g., continuous conversion mode, output data rate, etc.
}

void LIS2MDL_ReadRaw(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t data[6];
    // Read six registers starting at OUTX_L (0x68)
    HAL_I2C_Mem_Read(&hi2c1, LIS2MDL_I2C_ADDR, 0x68, 1, data, 6, HAL_MAX_DELAY);

    *x = (int16_t)(data[1] << 8 | data[0]);
    *y = (int16_t)(data[3] << 8 | data[2]);
    *z = (int16_t)(data[5] << 8 | data[4]);
}

void LIS2MDL_ReadMagnetic(float *x_uT, float *y_uT, float *z_uT)
{
    int16_t rx, ry, rz;
    LIS2MDL_ReadRaw(&rx, &ry, &rz);
    // Sensitivity 0.15 µT/LSB for ±4 gauss range
    *x_uT = rx * 0.15f;
    *y_uT = ry * 0.15f;
    *z_uT = rz * 0.15f;
}

