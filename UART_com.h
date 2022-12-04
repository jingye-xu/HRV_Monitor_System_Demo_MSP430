/*
 * UART_com.h
 *
 *  Created on: 2021Äê12ÔÂ19ÈÕ
 *      Author: jingye_lab_win
 */

#ifndef UART_COM_H_
#define UART_COM_H_

#include <msp430.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

//******************************************************************************
// UART Communication Via USB Configuration ************************************
//******************************************************************************
#define UART_CLK_SEL 0x0080             // Specifies accurate SMCLK clock for UART
#define BR0_FOR_9600 0x34               // Value required to use 9600 baud
#define BR1_FOR_9600 0x00               // Value required to use 9600 baud
#define CLK_MOD 0x4911                  // Microcontroller will "clean-up" clock signal

// need to change when frequency changes
#define CSCTL1_VALUE 0x0046             // Specifies frequency of DCO, bit6=1 for high, 0 for low
                                        // bit 3-1: 0:1/1, 1:2.67/5.33, 2:3.5/7, 3:4/8, 4:5.33/16
#define CSCTL3_VALUE 0x0000             // use to specify clock source divider bit6-4 for SMCLK
                                        // 0:/1, 1:/2, 2:/4, 3:/8, 4:/16, 5:/32
#define UCA0BRW_VALUE 52                // SMCLK/16/9600

void uart_clock_signals(void);          //Select Clock Signals
void assign_pins_to_uart(void);         // Used to Give UART Control of Appropriate Pins
void uart_9600_baud(void);              //Specify UART Baud Rate
void initUART(void);

void uart_tx_chars(volatile char *tx_buff, uint8_t count, uint8_t newline);
void uart_tx_float(float f, uint8_t newline);
void uart_tx_long(long l, uint8_t newline);

#endif /* UART_COM_H_ */
