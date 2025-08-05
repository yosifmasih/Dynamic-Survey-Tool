/*
 * mag_calibration.h
 *
 *  Created on: Aug 1, 2025
 *      Author: yosif
 */

#ifndef INC_MAG_CALIBRATION_H_
#define INC_MAG_CALIBRATION_H_

#include <stdint.h>
#include <stdbool.h>

#define MAG_CALIB_SAMPLES  50

extern int16_t mag_offset_x;
extern int16_t mag_offset_y;
extern int16_t mag_offset_z;
extern bool magCalibrationComplete;

// Run hard-iron calibration; call repeatedly until magCalibrationComplete is true
void calibrateMagnetometer(void);

// Apply offsets and convert to µT
void getCalibratedMag(int16_t raw_x, int16_t raw_y, int16_t raw_z,
                      float *out_x_uT, float *out_y_uT, float *out_z_uT);

#endif // INC_MAG_CALIBRATION_H_

