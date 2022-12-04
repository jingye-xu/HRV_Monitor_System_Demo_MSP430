//******************************************************************************
//
//    IoT project: The msp430fr5994 will be a master device to communicate with
//    other i2c slave devices.
//
//    Note: the SCL and SDA wires should be connected to a 4.7k pull-up resistor
//
//                                     /|\ /|\
//                   MSP430FR5994      4.7k |
//                 -----------------    |  4.7k
//            /|\ |             P7.1|---+---|-- I2C Clock (UCB2SCL)
//             |  |                 |       |
//             ---|RST          P7.0|-------+-- I2C Data (UCB2SDA)
//                |                 |
//                |                 |
//                |                 |
//                |                 |
//                |                 |
//                |                 |
//    SMCLK = 8MHz (important)
//            for i2c, we want 1kHz signal
//            for uart, we want 9600 bit/s
//    Author: Jingye Xu
//
//******************************************************************************

#include <msp430.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "model.h"
#include "hr_calc.h"
#include "UART_com.h"
#include "I2C_com.h"
#include "sensors.h"
#include "timer_counter.h"


//******************************************************************************
// Define
//******************************************************************************

#define I2CPINS           0x0003           // p7.0 and p7.1 pins as I2C
#define ACLK              0x0100           // Timer_A ACLK source
#define UP                0x0010           // Timer_A UP mode
#define FRAM2_HR_index    0x10000          // FRAM2 HR index
#define FRAM2_HR_data     0x10004          // FRAM2 HR data
#define FRAM2_HR_length   25600            // FRAM2 HR data length 25600*4Byte = 100KB

#define FRAM2_HRV_index   0x29004          // FRAM2 HRV index
#define FRAM2_HRV_data    0x29008          // FRAM2 HRV data
#define FRAM2_HRV_length  25600            // FRAM2 HRV data length 25600*4Byte = 100KB

// need to change when frequency changes
#define PERIOD_INT        (8000000/FS)     // period time depending on frequency of signal process
#define PERIOD_WAIT       8000000          // period wait time after pressing buttons
#define UCB2BW_VALUE      80               // fSCL = SMCLK/UCB2BW_VALUE = ~100kHz

#pragma PERSISTENT(HR_buff)                // store hr_buff in FRAM
float HR_buff[8] = {0};

#pragma PERSISTENT(HR_buff_index)          // store hr_buff_index in FRAM

uint8_t HR_buff_index = 0;

//******************************************************************************
// Device Initialization *******************************************************
//******************************************************************************

void initGPIO()
{
    // I2C pins
    P7SEL0 |= BIT0 | BIT1;
    P7SEL1 &= ~(BIT0 | BIT1);

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;
}

void initI2C()
{
    UCB2CTLW0 = UCSWRST;                      // Enable SW reset
    UCB2CTLW0 |= UCMODE_3 | UCMST | UCSSEL__SMCLK; // I2C master mode, SMCLK
    UCB2BRW = UCB2BW_VALUE;
    UCB2CTLW0 &= ~UCSWRST;                    // Clear SW reset, resume operation
    UCB2IE |= UCNACKIE;
}


//******************************************************************************
// Main ************************************************************************
//******************************************************************************
void main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	initGPIO();                 // initial p7.0 and p7.1 to I2C functionally ports
	initUART();
	initI2C();                  // initial I2C settings
	setup_timer0();             // setup timer0

	P7OUT = I2CPINS;            // try to use internal ...
	P7REN = I2CPINS;            // pull-up resistors

	P5OUT |= BIT6;              // pull up the button
	P5REN |= BIT6;              // use switch s1 to control the flow

	P1DIR = BIT0;               // enable the p1.0 (red led) as the indicator
	P1OUT = 0x0000;             // initial value is off
	
	long hr_red[BUFFER_SIZE] = {0};  // store hr red raw data
	long hr_ir[BUFFER_SIZE] = {0};   // store hr ir raw data
	long counter1 = 0;
	float *FRAM2_HR_pointer = (float *)FRAM2_HR_data;
	unsigned long *FRAM2_HR_index_pointer = (unsigned long *)FRAM2_HR_index;
//	if (*FRAM2_HR_index_pointer >= FRAM2_HR_length)
	*FRAM2_HR_index_pointer = 0;

	float *FRAM2_HRV_pointer = (float *)FRAM2_HRV_data;
	unsigned long *FRAM2_HRV_index_pointer = (unsigned long *)FRAM2_HRV_index;
//	if (*FRAM2_HRV_index_pointer >= FRAM2_HRV_length)
	*FRAM2_HRV_index_pointer = 0;

	long index = 0;
	uint8_t exportHRindex = 0;
	uint8_t exportHRVindex = 0;

//	float hr_out[1] = {0};
//	float hr_in[1] = {76.427678725};
//	PredictHR(&hr_in[0], &hr_out[0], 1);
//
//	float hrv_out[1] = {0};
//	float hrv_in[HRV_COUNT] = {85.53125,86.333333,85.925,86.375,87.166667,85.5,84.78125,83.625,80.8125,78.725,79.5,81.5,82.375,86.1875,89.8125,90.375,91.0,90.6875,88.15,83.7425,79.555375,81.12499999999999,85.18124999999998,89.44031249999998,88.791667,86.775,87.875,86.78125,82.4421875,78.320078125,79.25,80.81250000000001,83.375,84.84375000000001,87.8125,89.0,85.5625,81.59375,80.291667,81.125,81.90624999999999,80.1875,76.78125,77.46875,81.34218750000002,84.666667,86.29999999999998,89.62500000000001,91.791667,91.1,88.3125,85.03125,83.833333,81.583333,80.5,81.625,83.15625,84.25,83.416667,84.40625};
//	PredictHRV(&hrv_in[0],&hrv_out[0], 1);

//	DTHRV(&hrv_in[0],&hrv_out[0], 1);

	__no_operation();

	__delay_cycles(PERIOD_WAIT);

	while(1)
	{
        while((P5IN & BIT6) == 0)
        {
            __delay_cycles(PERIOD_WAIT);   // set a delay to avoid pushing button repeatedly in a short time, will change to timer in the future

            uart_tx_chars("start!", 7, 1);
            initHRSensor();

            while(1)
            {
                readFIFOHRSensor(&hr_red[index], &hr_ir[index]);

//                if(hr_red[index] < 100000 || hr_ir[index] < 100000){
//                    uart_tx_chars("error!", 7, 1);
//                    continue;
//                }

                index = (index + 1) % BUFFER_SIZE;          //increment hr raw data index

                if (index == 0) exportHRindex = 1;

//                 every second calculate one HR
                if ((index == (int)FS || index == (int)(FS*2) || index == (int)(FS*3) || index == 0) && exportHRindex == 1)
                {

                    float heartrate = 0;
                    computeHeartRate(hr_ir, hr_red, index, &heartrate);
                    if(heartrate != -999)
                    {
                        HR_buff[HR_buff_index] = heartrate;
                        HR_buff_index = (HR_buff_index + 1) % BUFFER_SIZE;
                    }
                    float hr_ave = 0, hr_ave_predicted = 0;
                    computeHRAve(HR_buff, &hr_ave);
                    counter = 0;
                    PredictHR(&hr_ave, &hr_ave_predicted, 1);
                    counter1 = counter;
                    uart_tx_chars("t", 2, 0);
                    uart_tx_float(counter1, 1);

                    if(hr_ave != 0)
                    {
                        FRAM2_HR_pointer[*FRAM2_HR_index_pointer] = hr_ave_predicted;
                        *FRAM2_HR_index_pointer = (*FRAM2_HR_index_pointer + 1) % FRAM2_HR_length;
                        exportHRVindex = 1;

//                        uart_tx_float(hr_ave, 0);
                        uart_tx_chars("h", 2, 0);
                        uart_tx_float(hr_ave_predicted, 1);

                        // every HRV_COUNT calculate one HRV
                        if (*FRAM2_HR_index_pointer % HRV_COUNT == 0 && exportHRVindex == 1)
                        {
                            float hrv_predicted = 0.0;
                            counter = 0;
                            PredictHRV(&FRAM2_HR_pointer[*FRAM2_HR_index_pointer - HRV_COUNT], &hrv_predicted, 1);
                            counter1 = counter;
                            uart_tx_chars("i", 2, 0);
                            uart_tx_float(counter1, 1);
                            FRAM2_HRV_pointer[*FRAM2_HRV_index_pointer] = hrv_predicted;
                            *FRAM2_HRV_index_pointer = (*FRAM2_HRV_index_pointer + 1) % FRAM2_HRV_length;

                            uart_tx_chars("v", 2, 0);
                            uart_tx_float(hrv_predicted, 1);

                        }
                    }
                }

                if ((P5IN & BIT6) == 0)
                {
                    __delay_cycles(PERIOD_WAIT);   // set a delay to avoid pushing button repeatedly in a short time, will change to timer in the future
                    break;
                }
                __delay_cycles(PERIOD_INT);

            }
            shutdownHRSensor();

            uart_tx_chars("end!", 5, 1);
        }
	}
//    __bis_SR_register(LPM0_bits + GIE);
}
