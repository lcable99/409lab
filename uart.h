//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//      uart.h
//      Created on: May 21, 2021
//      Author: Logan
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifndef UART_H_
#define UART_H_

#define UCBR_VAL 13
#define UCBRF_VAL 0
#define UCBRS_VAL 0x25

//ASCII Characters
#define ESC 0x1B
#define NULL '\0'
#define ASCII_NUM_OFFSET 0x30
#define DECIMAL 0x2E

//ESC Commands
#define CLEAR_SCREEN "[2J"
#define CLEAR_LINE "[2K"
#define RESET_CURSOR "[H"
#define DISABLE_ATTRIBUTES "[m"
#define BLINK "[5m"
#define BOLD "[1m"
#define CURSOR_DOWN_1 "[1B"
#define CURSOR_DOWN_2 "[2B"
#define CURSOR_DOWN_3 "[3B"
#define CURSOR_DOWN_4 "[4B"
#define CURSOR_DOWN_5 "[5B"
#define CURSOR_DOWN_6 "[6B"
#define CURSOR_DOWN_7 "[7B"
#define CURSOR_DOWN_8 "[8B"
#define CURSOR_DOWN_9 "[9B"

#define CURSOR_RIGHT_5 "[5C"
#define CURSOR_LEFT_5 "[5D"
#define CURSOR_LEFT_15 "[15D"
#define TEXT_RED    "[31m"
#define TEXT_BLUE   "[34m"
#define TEXT_GREEN  "[32m"
#define TEXT_WHITE  "[37m"
#define TEXT_CYAN "[36m"

#define Puart P1
#define RXD BIT2
#define TXD BIT3
#define NVIC_ISR_UART_A0 (1<<16)

void UART_init(void);
void UART_print_string(char sendstring[]);
void UART_send_data(uint8_t character);
void UART_esc_code(char sendstring[]);



#endif /* UART_H_ */
