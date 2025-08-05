/*
 * mag_calibration.c
 *
 *  Created on: Aug 1, 2025
 *      Author: yosif
 */

#include "mag_calibration.h"
#include "lis2mdl_driver.h"

int16_t mag_offset_x = 0;
int16_t mag_offset_y = 0;
int16_t mag_offset_z = 0;
bool magCalibrationComplete = false;

static uint16_t count = 0;
static int32_t sum_x = 0, sum_y = 0, sum_z = 0;

void calibrateMagnetometer(void)
{
    int16_t rx, ry, rz;
    LIS2MDL_ReadRaw(&rx, &ry, &rz);

    sum_x += rx;
    sum_y += ry;
    sum_z += rz;
    count++;

    if (count >= MAG_CALIB_SAMPLES) {
        mag_offset_x = sum_x / count;
        mag_offset_y = sum_y / count;
        mag_offset_z = sum_z / count;
        magCalibrationComplete = true;
        // Reset if you want to recalibrate later:
        // count = sum_x = sum_y = sum_z = 0;
    }
}

void getCalibratedMag(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                      float *out_x_uT, float *out_y_uT, float *out_z_uT)
{
    int16_t cx = raw_x - mag_offset_x;
    int16_t cy = raw_y - mag_offset_y;
    int16_t cz = raw_z - mag_offset_z;

    *out_x_uT = cx * 0.15f;
    *out_y_uT = cy * 0.15f;
    *out_z_uT = cz * 0.15f;
}

