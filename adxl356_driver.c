/*
 * adxl356_driver.c
 *
 *  Created on: Aug 5, 2025
 *      Author: yosif
 */

#include "adxl356_driver.h"

// External ADC handle from main.c / stm32l4xx_hal_msp.c
extern ADC_HandleTypeDef hadc1;

void ADXL356_Init(void)
{
    // If you need any ADC channel configuration beyond MX-generated code,
    // you can do it here. Otherwise, leave empty.
}

void ADXL356_ReadRaw(uint16_t *x_raw, uint16_t *y_raw, uint16_t *z_raw)
{
    // Ensure previous conversion is halted
    HAL_ADC_Stop(&hadc1);

    // Start a fresh conversion sequence
    HAL_ADC_Start(&hadc1);
    // XOUT (rank 1)
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
        *x_raw = HAL_ADC_GetValue(&hadc1);
    // YOUT (rank 2)
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
        *y_raw = HAL_ADC_GetValue(&hadc1);
    // ZOUT (rank 3)
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
        *z_raw = HAL_ADC_GetValue(&hadc1);
}

void ADXL356_CalcVoltage(uint16_t raw, float *voltage)
{
    // 12-bit ADC: raw ∈ [0,4095], Vref = 3.3 V
    *voltage = (raw / 4095.0f) * 3.3f;
}

void ADXL356_CalcAccelG(float voltage, float *accel_g)
{
    // 0.9 V = 0 g, sensitivity = 0.08 V/g for ±10 g range
    *accel_g = (voltage - 0.9f) / 0.08f;
}

void ADXL356_ReadAccelG(float *x_g, float *y_g, float *z_g)
{
    uint16_t xr, yr, zr;
    float xv, yv, zv;

    // 1) Read raw ADC counts
    ADXL356_ReadRaw(&xr, &yr, &zr);

    // 2) Convert raw → voltage
    ADXL356_CalcVoltage(xr, &xv);
    ADXL356_CalcVoltage(yr, &yv);
    ADXL356_CalcVoltage(zr, &zv);

    // 3) Convert voltage → g
    ADXL356_CalcAccelG(xv, x_g);
    ADXL356_CalcAccelG(yv, y_g);
    ADXL356_CalcAccelG(zv, z_g);
}

