/*
 * adxl312_driver.c
 *
 *  Created on: Jul 31, 2025
 *      Author: yosif
 */
// Src/adxl312_driver.c
#include "adxl312_driver.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_spi.h"
extern SPI_HandleTypeDef hspi1;
// Initialize into measurement mode
void ADXL312_Init(void)
{
    uint8_t tx[2] = { 0x2D, 0x08 };   // POWER_CTL reg, Measure bit
    ADXL312_CS_LOW();
    HAL_Delay(1);
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    ADXL312_CS_HIGH();
    HAL_Delay(10);
}

// Read a single register via SPI (MSB=1 for read)
uint8_t ADXL312_ReadRegister(uint8_t reg)
{
    uint8_t tx[2] = { reg | 0x80, 0x00 };
    uint8_t rx[2] = {0};

    ADXL312_CS_LOW();
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    ADXL312_CS_HIGH();

    return rx[1];
}

// Burst-read X, Y, Z (0x32 start, multibyte)
void ADXL312_ReadXYZ(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t tx[7] = { 0x32 | 0xC0 };
    uint8_t rx[7] = {0};

    ADXL312_CS_LOW();
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 7, HAL_MAX_DELAY);
    ADXL312_CS_HIGH();

    *x = (int16_t)((rx[2] << 8) | rx[1]);
    *y = (int16_t)((rx[4] << 8) | rx[3]);
    *z = (int16_t)((rx[6] << 8) | rx[5]);
}


