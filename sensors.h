/*
 * sensors.h
 *
 *  Created on: 2021Äê12ÔÂ19ÈÕ
 *      Author: jingye_lab_win
 */

#ifndef SENSORS_H_
#define SENSORS_H_

#include <msp430.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#define I2CADDR_IR          0x5a     // i2c address of ir sensor
#define I2CADDR_HR          0x57     // i2c address of heart rate sensor
#define I2CADDR_ACC         0x1d     // i2c address of accelerometer sensor

//******************************************************************************
// Define some registers for HR sensor
//******************************************************************************
#define REG_INTR_STATUS_1    0x00
#define REG_INTR_STATUS_2    0x01
#define REG_INTR_ENABLE_1    0x02
#define REG_INTR_ENABLE_2    0x03
#define REG_FIFO_WR_PTR    0x04
#define REG_OVF_COUNTER    0x05
#define REG_FIFO_RD_PTR    0x06
#define REG_FIFO_DATA    0x07
#define REG_FIFO_CONFIG    0x08
#define REG_MODE_CONFIG    0x09
#define REG_SPO2_CONFIG    0x0A
#define REG_LED1_PA    0x0C
#define REG_LED2_PA    0x0D
#define REG_PILOT_PA    0x10
#define REG_MULTI_LED_CTRL1    0x11
#define REG_MULTI_LED_CTRL2    0x12
#define REG_TEMP_INTR    0x1F
#define REG_TEMP_FRAC    0x20
#define REG_TEMP_CONFIG    0x21
#define REG_PROX_INT_THRESH    0x30
#define REG_REV_ID    0xFE
#define REG_PART_ID    0xFF

void initInfraredSensor();
void readInfraredSensor(float *ob_float, float *amb_float);

void initAccSensor();
void readAccSensor(float *acc_x_float, float *acc_y_float, float *acc_z_float);

void resetHRSensor();
void setupHRSensor();
void initHRSensor();
void shutdownHRSensor();
int getDataPresentHRSensor();
void readFIFOHRSensor(long *hr_red, long *hr_ir);
void hr_buf_switch(long int *res, long int *dst, long index, long len);
void computeHeartRate(long int * hr_ir, long int * hr_red, long index, float *heartrate);
void computeHRAve(float *hr_buff, float *hr_ave);

#endif /* SENSORS_H_ */
