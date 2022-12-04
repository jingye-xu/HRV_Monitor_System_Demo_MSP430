/*
 * UART_com.c
 *
 *  Created on: 2021Äê12ÔÂ19ÈÕ
 *      Author: jingye_lab_win
 */


#include "UART_com.h"

//Select Clock Signals
void uart_clock_signals(void)
{
    CSCTL0_H = CSKEY_H;                 // Unlock CS registers
    CSCTL1 = CSCTL1_VALUE;              // Specifies frequency of main clock
    CSCTL2 = 0x0133;                    // Assigns additional clock signals
    CSCTL3 = CSCTL3_VALUE;                    // Use clocks at intended frequency, do not slow them down
    CSCTL0_H = 0;
}

// Used to Give UART Control of Appropriate Pins
void assign_pins_to_uart(void)
{
    // Configure GPIO
    P2SEL0 &= ~(BIT0 | BIT1);           // set SEL1 as 1 and SEL0 as 0 to use
    P2SEL1 |= (BIT0 | BIT1);            // USCI_A0 UART operation
}

//Specify UART Baud Rate
void uart_9600_baud(void)
{
    UCA0CTLW0 = UCSWRST;                    // Put UART into SoftWare ReSeT1
    UCA0CTLW0 = UCA0CTLW0 | UART_CLK_SEL;   // Specifies clock source for UART
    UCA0BRW = UCA0BRW_VALUE;
    UCA0MCTLW = CLK_MOD;                    // "Cleans" clock signal
    UCA0CTLW0 = UCA0CTLW0 & (~UCSWRST);     // Takes UART out of SoftWare ReSeT
}

void initUART(void)
{
    uart_clock_signals();
    assign_pins_to_uart();
    uart_9600_baud();
}

void uart_tx_chars(volatile char *tx_buff, uint8_t count, uint8_t newline)
{
    volatile int c=0;
    while(c< count - 1){
        while (!(UCA0IFG&UCTXIFG));   // USCI_A0 TX buffer ready for new data?
        UCA0TXBUF = tx_buff[c];       //Send the UART message out pin P3.4 by resign it to the buffer UCA0TXBUF
        c++;
    }
    if (newline == 1)                    // newline == 1, newline
    {
        while (!(UCA0IFG&UCTXIFG));
        UCA0TXBUF = '\n';
    }
    else                               // newline != 1, space
    {
        while (!(UCA0IFG&UCTXIFG));
        UCA0TXBUF = ' ';
    }
}

void uart_tx_float(float f, uint8_t newline)
{
    char s[8];
    sprintf(s,"%-7.3f",f);
    unsigned int c=0;
    while(c<7){
        while (!(UCA0IFG&UCTXIFG));
        UCA0TXBUF = s[c];
        c++;
    }
    if (newline == 1)                    // newline == 1, newline
    {
        while (!(UCA0IFG&UCTXIFG));
        UCA0TXBUF = '\n';
    }
    else                               // newline != 1, space
    {
        while (!(UCA0IFG&UCTXIFG));
        UCA0TXBUF = ' ';
    }
}

void uart_tx_long(long l, uint8_t newline)
{
    char l_buff[10] = {0};
    ltoa(l, l_buff, 10);
    uart_tx_chars(l_buff, 11, newline);
}
