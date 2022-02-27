//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//      main.c
//      Created on: Feb. 23, 2021
//      Author: Logan
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "msp.h"
#include "uart.h"
#include "clock.h"
#include <stdio.h>

#define TRIG_DUR 600
#define FRAME_DUR 60000 //100Hz new data
#define FRAME_FREQ 6000000
#define SOUNDSPEED 343

void timer_init(void);
void pin_init(void);
void util_uart_print_distance(uint32_t number);

uint8_t received = 0;
uint16_t framecycles;
uint32_t distance;  //in micrometers
char str[80];

void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	HIGH_FREQ_CLOCK();
    UART_init();
    timer_init();
    pin_init();

    //Printing to terminal
    UART_print_string("Distance:");

    while(1)
    {
        if(received == 1)    //calculate distance
        {

            received = 0;
            //calculate distance below
            distance = SOUNDSPEED*framecycles/FRAME_FREQ*1000000;
            UART_esc_code(CLEAR_LINE);
            UART_esc_code(RESET_CURSOR);
            UART_print_string("Distance:");
            util_uart_print_distance(distance);
            UART_print_string("m");
        }
    }
}

//Input in micrometers
void util_uart_print_distance(uint32_t number)
{
    UART_send_data(number/(1000000)+ASCII_NUM_OFFSET);
    UART_send_data(DECIMAL);
    UART_send_data((number/(100000))%10+ASCII_NUM_OFFSET);
    UART_send_data((number/(10000))%10+ASCII_NUM_OFFSET);
    UART_send_data((number/(1000))%10+ASCII_NUM_OFFSET);
}


void pin_init(void)
{
    //Set GPIO output
        //Pin set for trigger
    P2->SEL0 &= ~BIT7;
    P2->SEL1 &= ~BIT7;
    P2->DIR |= BIT7;
    P2->OUT |= BIT7;   //set high

        //Pin set for tone decoder
    P3->SEL0 &= ~BIT7;
    P3->SEL1 &= ~BIT7;  //gpio mode
    P3->DIR &= ~BIT7;   //input
    P3->IES |= BIT7;    //high to low transition interrupt
    P3->IE |= BIT7;     //enable interrupt
    NVIC->ISER[1] = BIT5;  //enable P3 interrupt ISR
}

void PORT3_IRQHandler(void)
{
    P3->IFG &= ~BIT7;   //clear flag
    if(received != 1)
    {
        framecycles = TIMER_A1->R;  //record cycles for distance measurement
        received = 1;
    }
}

void TA0_0_IRQHandler(void)
{
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;  //clear flag
    P2->OUT |= BIT7;   //set back to high
    TIMER_A0 -> CCR[0] = 0;
}

void TA1_0_IRQHandler(void)
{
    TIMER_A1->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;  //clear flag
    TIMER_A0 -> CCR[0] = TRIG_DUR-1;
    P2->OUT &= ~BIT7;   //set low
}


void timer_init(void)
{
    //Set up interrupts for TimerA0
        //Used to control trigger
    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE;                 //Interrupt enable for CCR0
    //Set CCR0 value to trigger duration
    //TIMER_A0->CCR[0] = TRIG_DUR-1;
    TIMER_A0 -> CCR[0] = 0;
    NVIC->ISER[0] = BIT8;                                   //Enable ISR for TimerA0 CCR0
    __enable_irq();                                         //Global interrupt enable

    //Timer_A0 in compare mode
    //Set up TimerA0 to run on SMCLK, w/ divide by 4, in UP mode (using CCR0)
    //Timer off originally
    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK|
                    TIMER_A_CTL_ID__4|
                    TIMER_A_CTL_MC__UP;


    //Timer_A1 in compare mode
    //Set up TimerA1 to run on SMCLK, w/ divide by 4, in UP mode (using CCR0)
    TIMER_A1->CTL = TIMER_A_CTL_SSEL__SMCLK|
                    TIMER_A_CTL_ID__4|
                    TIMER_A_CTL_MC__UP;
    TIMER_A1->CCTL[0] = TIMER_A_CCTLN_CCIE;                 //Interrupt enable for CCR0
    TIMER_A1->CCR[0] = FRAME_DUR-1;
    NVIC->ISER[0] = 1<<10;                                  //Enable ISR for TimerA1 CCR0


    /*
    //Timer_A1 in Capture Mode for Frequency Measuring from Comparator
    //Used for frequency measuring
    TIMER_A1->CCTL[3] = TIMER_A_CCTLN_CCIE                 //Interrupt enable for CCR3
                        |TIMER_A_CCTLN_CM__RISING           //Capture on rising edge
                        |TIMER_A_CCTLN_CCIS__CCIB           //Capture on CCIB signal
                        |TIMER_A_CCTLN_CAP                  //Capture mode
                        |TIMER_A_CCTLN_SCS;                  //Synchronize capture signal

    //Enable ISR for TimerA1 CCR3 and TAIFG for comparator 1
    NVIC->ISER[0] = 1<<11;
    */

    //Start Timer
    //Set up TimerA1 to run on SMCLK, w/ divide by 4, in CONT mode, //en overflow interrupt
    /*
    TIMER_A1->CTL = TIMER_A_CTL_SSEL__SMCLK|
                    TIMER_A_CTL_ID__4|
                    TIMER_A_CTL_MC__CONTINUOUS|
                    TIMER_A_CTL_IE;
                    */
}
