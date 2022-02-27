//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//      clock.h
//      Created on: Apr 25, 2021
//      Author: Logan
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#ifndef CLOCK_H_
#define CLOCK_H_

#define CYCLES 3000000
#define FREQ_1_5_MHz 0
#define FREQ_3_MHz 1
#define FREQ_6_MHz 2
#define FREQ_12_MHz 3
#define FREQ_24_MHz 4
#define DCO_TUNE -4

void set_DCO(uint8_t freq_set);
void MCLK_PIN_INIT(void);
void DCO_tune(void);
void HIGH_FREQ_CLOCK(void);

#endif /* CLOCK_H_ */
