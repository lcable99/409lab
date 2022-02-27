//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//      uart.c
//      Created on: May 21, 2021
//      Author: Logan
//      Requirements: SMCLK = 24MHz
//      Spec: Baudrate = 115.2KHz
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "msp.h"
#include "uart.h"

void UART_print_string(char sendstring[])
{
    uint8_t iter = 0;
    while(sendstring[iter] != NULL)
    {
        UART_send_data(sendstring[iter]);
        iter++;
    }
}

void UART_esc_code(char sendstring[])
{
    uint8_t iter = 0;
    UART_send_data(ESC);
    while(sendstring[iter] != NULL)
    {
        UART_send_data(sendstring[iter]);
        iter++;
    }
}

void UART_send_data(uint8_t data)
{
    while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));    //wait until TXBUF is empty
    EUSCI_A0->TXBUF = data;
}

void UART_init(void)
{
    //Initialize UART
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST;     //put UART into software reset

    //Use SMCLK for BRCLK
    EUSCI_A0->CTLW0 =   EUSCI_A_CTLW0_SSEL__SMCLK
                        |EUSCI_A_CTLW0_SWRST;

    //Set clock prescaler of baud generator
    EUSCI_A0->BRW = UCBR_VAL;

    //Oversampling mode enable, Set BRF(1st modulation) to BRF_VAL,
    //Set BRS(2nd modulation) to BRS_VAL
    EUSCI_A0->MCTLW =   EUSCI_A_MCTLW_OS16
                        |(UCBRF_VAL<<EUSCI_A_MCTLW_BRF_OFS)
                        |(UCBRS_VAL<<EUSCI_A_MCTLW_BRS_OFS);

    //Initialize TX and RX pins
    Puart->SEL0 |= (TXD|RXD);
    Puart->SEL1 &= ~(TXD|RXD);
    Puart->DIR |= TXD;
    Puart->DIR &= ~RXD;

    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;     //end UART reset

    //Interrupts
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;    //enable RX to trigger interrupt
    NVIC->ISER[0] = NVIC_ISR_UART_A0;   //Enable ISR for UART
    __enable_irq();

    //Clear terminal, set terminal colors
    UART_esc_code(CLEAR_SCREEN);
    UART_esc_code(TEXT_CYAN);
    UART_esc_code(BOLD);
    UART_esc_code(RESET_CURSOR);
}
