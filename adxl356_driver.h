/*
 * adxl356_driver.h
 *
 *  Created on: Aug 5, 2025
 *      Author: yosif
 */

#ifndef INC_ADXL356_DRIVER_H_
#define INC_ADXL356_DRIVER_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_adc.h"

// Initialize ADC channels for ADXL356 if any setup is required
void ADXL356_Init(void);

// Read raw 12-bit ADC counts for X, Y, Z axes
void ADXL356_ReadRaw(uint16_t *x_raw, uint16_t *y_raw, uint16_t *z_raw);

// Convert a single raw count to voltage (0–3.3 V)
void ADXL356_CalcVoltage(uint16_t raw, float *voltage);

// Convert a single voltage to acceleration in g (10 g range, 0.9 V zero-g, 0.08 V/g)
void ADXL356_CalcAccelG(float voltage, float *accel_g);

// High-level: read and return all three axes in g
void ADXL356_ReadAccelG(float *x_g, float *y_g, float *z_g);

#endif // INC_ADXL356_DRIVER_H_
