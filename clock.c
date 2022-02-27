//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//      clock.c
//      Created on: Apr 25, 2021
//      Author: Logan
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "msp.h"
#include "clock.h"

//Function to use HI_FREQ_OSCILLATOR for MCLK = 48MHz, SMCLK = 24MHz
//Note: Max Freq. for SMCLK is 24MHz
void HIGH_FREQ_CLOCK(void)
{
    /* Transition to VCORE Level 1: AM0_LDO --> AM1_LDO */
    while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
        PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_AMR_1;
    while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
    /* Configure Flash wait-state to 1 for both banks 0 & 1 */
    FLCTL->BANK0_RDCTL = (FLCTL->BANK0_RDCTL &
         ~(FLCTL_BANK0_RDCTL_WAIT_MASK)) | FLCTL_BANK0_RDCTL_WAIT_1;
    FLCTL->BANK1_RDCTL  = (FLCTL->BANK0_RDCTL &
         ~(FLCTL_BANK1_RDCTL_WAIT_MASK)) | FLCTL_BANK1_RDCTL_WAIT_1;

    /* Configure HFXT to use 48MHz crystal, source to MCLK */
    PJ->SEL0 |= BIT2 | BIT3;        // Configure PJ.2/3 for HFXT function
    PJ->SEL1 &= ~(BIT2 | BIT3);

    CS->KEY = CS_KEY_VAL;       // Unlock CS module for register access
    CS->CTL2 |= CS_CTL2_HFXT_EN | CS_CTL2_HFXTFREQ_6 | CS_CTL2_HFXTDRIVE;
    while(CS->IFG & CS_IFG_HFXTIFG)     // Wait for HFXT to start up
        CS->CLRIFG |= CS_CLRIFG_CLR_HFXTIFG;

    /*
    CS->CTL1 = CS->CTL1 & ~(CS_CTL1_SELM_MASK | CS_CTL1_DIVM_MASK) //Clear SELM & DIVM
                        | CS_CTL1_SELM__HFXTCLK;                   //Select MCLK = HFXT
    CS->KEY = 0;
*/

    //Clear SELM, DIVM, SELS, DIVS
    CS->CTL1 = CS->CTL1 & ~(CS_CTL1_SELM_MASK | CS_CTL1_DIVM_MASK | CS_CTL1_SELS_MASK | CS_CTL1_DIVS_MASK)
                        | CS_CTL1_SELM__HFXTCLK                     //Select MCLK = HFXT
                        | CS_CTL1_SELS__HFXTCLK                     //Select SMCLK = HFXT/2
                        | CS_CTL1_DIVS__2;

    CS->KEY = 0;

}

//Function used to adjust DCO to a nominal frequency 1.5 to 24MHz
//Also sets up MCLK and SMCLK to use DCO (writes 0 to everything else)
void set_DCO(uint8_t freq_set)
{
    CS->KEY = CS_KEY_VAL;
    //Set DCO Freq.
    switch(freq_set)
    {
        case(0):
            CS->CTL0 = CS_CTL0_DCORSEL_0;           //Nominal Frequency 1.5MHz
            break;
        case(1):
            CS->CTL0 = CS_CTL0_DCORSEL_1;           //Nominal Frequency 3MHz
            break;
        case(2):
            CS->CTL0 = CS_CTL0_DCORSEL_2;           //Nominal Frequency 6MHz
            break;
        case(3):
            CS->CTL0 = CS_CTL0_DCORSEL_3;           //Nominal Frequency 12MHz
            break;
        case(4):
            CS->CTL0 = CS_CTL0_DCORSEL_4;           //Nominal Frequency 24MHz
            break;
        default:
            CS->CTL0 = CS_CTL0_DCORSEL_1;           //Default Nominal Frequency 3MHz
            break;
    }

    //DCO_tune();

    //Set MCLK to DCO and set MCLK_divider to 1
    //Set SMCLK to DCO and set SMCLK_divider to 1
    CS->CTL1 =  CS_CTL1_SELM__DCOCLK|
                CS_CTL1_DIVM__1|
                CS_CTL1_SELS__DCOCLK|
                CS_CTL1_DIVS__1;

    CS->KEY = 0;
}

void DCO_tune(void)
{
    int16_t tune = DCO_TUNE;
    CS->CTL0 |= CS_CTL0_DCOTUNE_MASK&tune;
}

void MCLK_PIN_INIT(void)
{
    //Initializes MCLK to output on P4.3
    P4->SEL0 |= BIT3;
    P4->SEL1 &= ~BIT3;
    P4->DIR |= BIT3;
}
