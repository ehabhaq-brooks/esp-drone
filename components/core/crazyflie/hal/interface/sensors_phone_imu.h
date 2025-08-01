#pragma once

/**
 * @file sensors_phone_imu.h
 * @brief Header file for the Phone IMU sensor interface.
 *
 * This file contains the declarations for the Phone IMU sensor functions,
 * including initialization, data acquisition, and reading sensor values.
 *
 *
 */

#include "sensors.h"
// Initialize the Phone IMU sensor
void sensorsPhoneImuInit(void);

// Acquire the latest accelerometer & gyro readings
void sensorsPhoneImuAcquire(sensorData_t* sensors, const uint32_t tick);

// Blocking wait until a new packet arrives
void sensorsPhoneImuWaitDataReady(void);

// Non-blocking read of gyro values
bool sensorsPhoneImuReadGyro(Axis3f* gyro);

// Non-blocking read of accelerometer values
bool sensorsPhoneImuReadAcc(Axis3f* acc);

// Always returns true: assume phone IMU is pre-calibrated
bool sensorsPhoneImuAreCalibrated(void);

// Manufacturing test for the phone IMU, always returns true
bool sensorsPhoneImuManufacturingTest(void);

// Non-blocking read of accelerometer values
bool sensorsPhoneImuReadAcc(Axis3f* acc);

// Non-blocking read of magnetometer values
bool sensorsPhoneImuTest(void);

// Always returns true: assume phone IMU is pre-calibrated
bool ssensorsPhoneImuManufacturingTest(void);

// Non-blocking read of magnetometer values
bool sensorsPhoneImuReadMag(Axis3f *mag);

// Non-blocking read of barometer values
bool sensorsPhoneImuReadBaro(baro_t *baro);

// Set the accelerometer mode
void sensorsPhoneImuSetAccMode(accModes accMode);