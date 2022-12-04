/*
 * sensors.c
 *
 *  Created on: 2021Äê12ÔÂ19ÈÕ
 *      Author: jingye_lab_win
 */

#include "sensors.h"
#include "I2C_com.h"
#include "UART_com.h"
#include "hr_calc.h"

//******************************************************************************
// Infrared sensor functions ***************************************************
//******************************************************************************

void initInfraredSensor()
{
    uint8_t data = 0x01;
    I2C_Master_WriteReg(I2CADDR_IR, 0x07, &data, 1);    //object temp register
    I2C_Master_WriteReg(I2CADDR_IR, 0x06, &data, 1);    //ambient temp register
}

void readInfraredSensor(float *ob_float, float *amb_float)
{
    uint8_t ob_temp[3] = {0};   //store object temperature
    uint8_t amb_temp[3] = {0};  //store ambient temperature
    I2C_Master_ReadReg(I2CADDR_IR, 0x07, 3);            //read object temp
    CopyArray(ReceiveBuffer, ob_temp, 3);
    I2C_Master_ReadReg(I2CADDR_IR, 0x06, 3);            //read ambient temp
    CopyArray(ReceiveBuffer, amb_temp, 3);
    *ob_float = (ob_temp[1]<<8 | ob_temp[0])*0.02-0.01-273.15;
    *amb_float = (amb_temp[1]<<8 | amb_temp[0])*0.02-0.01-273.15;
}


//******************************************************************************
// Accelerometer sensor functions **********************************************
//******************************************************************************
void initAccSensor()
{
    uint8_t data0 = 0x00;
    I2C_Master_WriteReg(I2CADDR_ACC, 0x2A, &data0, 1);  // control register (0x2a),
                                                        // 0x00 for standby mode
    uint8_t data1 = 0x01;
    I2C_Master_WriteReg(I2CADDR_ACC, 0x2A, &data1, 1);  // 0x01 for active mode

    uint8_t data2 = 0x00;
    I2C_Master_WriteReg(I2CADDR_ACC, 0x0E, &data2, 1);  // configuration register (0x2a),
                                                        // 0x00 for +/- 2g
}

void readAccSensor(float *acc_x_float, float *acc_y_float, float *acc_z_float)
{
    uint8_t acc_temp[7]={0};
    I2C_Master_ReadReg(I2CADDR_ACC, 0x00, 7);
    CopyArray(ReceiveBuffer, acc_temp, 7);
    *acc_x_float = (acc_temp[1] << 8 | acc_temp[2])/16;
    if(*acc_x_float > 2047) acc_x_float -= 4096;
    *acc_y_float = (acc_temp[3] << 8 | acc_temp[4])/16;
    if(*acc_y_float > 2047) acc_y_float -= 4096;
    *acc_z_float = (acc_temp[5] << 8 | acc_temp[6])/16;
    if(*acc_z_float > 2047) acc_z_float -= 4096;
}


//******************************************************************************
// Heart rate sensor functions *************************************************
//******************************************************************************

void resetHRSensor()
{
    uint8_t data = 0x40;
    I2C_Master_WriteReg(I2CADDR_HR, REG_MODE_CONFIG, &data, 1);    // clear all settings
}

void setupHRSensor()
{
    uint8_t data1 = 0xc0;
    uint8_t data2 = 0x00;
    I2C_Master_WriteReg(I2CADDR_HR, REG_INTR_ENABLE_1, &data1, 1);
    I2C_Master_WriteReg(I2CADDR_HR, REG_INTR_ENABLE_2, &data2, 1);

    I2C_Master_WriteReg(I2CADDR_HR, REG_FIFO_WR_PTR, &data2, 1);
    I2C_Master_WriteReg(I2CADDR_HR, REG_OVF_COUNTER, &data2, 1);
    I2C_Master_WriteReg(I2CADDR_HR, REG_FIFO_RD_PTR, &data2, 1);

    uint8_t data3 = 0x4f;
    I2C_Master_WriteReg(I2CADDR_HR, REG_FIFO_CONFIG, &data3, 1);

    uint8_t data4 = 0x03;
    I2C_Master_WriteReg(I2CADDR_HR, REG_MODE_CONFIG, &data4, 1);

    uint8_t data5 = 0x27;
    // used to specify frequency: bit4-bit2
    // 0b 0010 0111 = 0x27  FS = 100/4 = 25Hz
    // 0b 0010 0011 = 0x23  FS =  50/4 = 12.5Hz
    // 0b 0010 1011 = 0x2b  FS = 200/4 = 50Hz
    // 0b 0010 1111 = 0x2f  FS = 400/4 = 100Hz
    if (FS == 12.5) data5 = 0x23;
    if (FS == 25) data5 = 0x27;
    if (FS == 50) data5 =0x2b;
    if (FS == 100) data5 =0x2f;
    I2C_Master_WriteReg(I2CADDR_HR, REG_SPO2_CONFIG, &data5, 1);

    uint8_t data6 = 0x24;
    uint8_t data7 = 0x7f;
    I2C_Master_WriteReg(I2CADDR_HR, REG_LED1_PA, &data6, 1);
    I2C_Master_WriteReg(I2CADDR_HR, REG_LED2_PA, &data6, 1);
    I2C_Master_WriteReg(I2CADDR_HR, REG_PILOT_PA, &data7, 1);
}

void initHRSensor()
{
    resetHRSensor();
    I2C_Master_ReadReg(I2CADDR_HR, REG_INTR_STATUS_1, 1);
    setupHRSensor();
}

void shutdownHRSensor()
{
    uint8_t data = 0x80;
    I2C_Master_WriteReg(I2CADDR_HR, REG_MODE_CONFIG, &data, 1);
}

int getDataPresentHRSensor()
{
    uint8_t read_ptr[1] = {0}, write_ptr[1] = {0};
    I2C_Master_ReadReg(I2CADDR_HR, REG_FIFO_RD_PTR, 1);
    CopyArray(ReceiveBuffer, read_ptr, 1);
    I2C_Master_ReadReg(I2CADDR_HR, REG_FIFO_WR_PTR, 1);
    CopyArray(ReceiveBuffer, write_ptr, 1);
    if (read_ptr == write_ptr){
        return 0;
    }
    else{
        int num_samples = 0;
        num_samples = write_ptr - read_ptr;
        if (num_samples < 0)
            num_samples += 32;
        return num_samples;
    }
}

void readFIFOHRSensor(long *hr_red, long *hr_ir)
{
    uint8_t temp[6] = {0};
    long red_h=0,red_l=0,ir_h=0,ir_l=0;

    uint8_t reg_INTR1[1] = {0}, reg_INTR2[1] = {0};
    I2C_Master_ReadReg(I2CADDR_HR, REG_INTR_STATUS_1, 1);
    CopyArray(ReceiveBuffer, reg_INTR1, 1);
    I2C_Master_ReadReg(I2CADDR_HR, REG_INTR_STATUS_2, 1);
    CopyArray(ReceiveBuffer, reg_INTR2, 1);
    I2C_Master_ReadReg(I2CADDR_HR, REG_FIFO_DATA, 6);
    CopyArray(ReceiveBuffer, temp, 6);

    red_h = temp[0] & 0x03;
    red_l = temp[1] << 8 | temp[2];
    ir_h = temp[3] & 0x03;
    ir_l = temp[4] << 8 | temp[5];
    *hr_red = red_h << 16 | red_l;
    *hr_ir = ir_h << 16 | ir_l;
}

void hr_buf_switch(long int *res, long int *dst, long index, long len)
{
    volatile long i;
    for (i=0;i<len;i++)
    {
        dst[i] = res[(i+index)%len];
    }
}

void computeHeartRate(long int * hr_ir, long int * hr_red, long index, float *heartrate)
{
    long int hr_red_buf[BUFFER_SIZE] = {0};
    long int hr_ir_buf[BUFFER_SIZE] = {0};

    volatile long hr_result = 0;
    volatile int hr_valid = 0;

    hr_buf_switch(hr_ir, hr_ir_buf, index, BUFFER_SIZE);
    hr_buf_switch(hr_red, hr_red_buf, index, BUFFER_SIZE);
    maxim_heart_rate_and_oxygen_saturation(hr_ir_buf, BUFFER_SIZE, hr_red_buf, &hr_result, &hr_valid);
    *heartrate = hr_result;
}

void computeHRAve(float *hr_buff, float *hr_ave)
{
    uint8_t count = 0;
    uint8_t index_temp;
    for(index_temp=0;index_temp<8;index_temp++)
    {
        if(hr_buff[index_temp] != 0)
        {
            count ++;
            *hr_ave += hr_buff[index_temp];
        }
    }
    if(count != 0)
    {
        *hr_ave = *hr_ave/count;
    }
    else
    {
        *hr_ave = 0;
    }
}
