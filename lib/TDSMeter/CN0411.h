#ifndef _CN0411_H_
#define _CN0411_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "AD5683.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

namespace CN0411 {
const uint32_t CN0411_SUCCESS = 0;
const int32_t CN0411_FAILURE = -1;
const uint32_t CS_AD7124 = 0;
const uint32_t CS_AD5683 = 1;

const uint32_t PWM_SYSCALIB_AIN7 = 1;
const uint32_t PWM_SYSCALIB_AIN8 = 2;
const uint32_t PWM_CONVERSION = 3;
const uint32_t PWM_FREQ_94 = 94;
const uint32_t PWM_FREQ_2400 = 2400;

const uint32_t PWM_CLR = 0;
const uint32_t PWM_SET = 1;

static inline uint32_t ADC_SET_CH(uint32_t x) { return x << 15; }

}  // namespace CN0411

/******************************************************************************/
/************************** Variable Declaration ******************************/
/******************************************************************************/

/******************************************************************************/
/************************** Function Declaration ******************************/
/******************************************************************************/

void CN0411_pwm_freq(uint16_t freq);
void CN0411_pwm_gen(void);
void CN0411_init(void);

#endif /* _CN0411_H_ */