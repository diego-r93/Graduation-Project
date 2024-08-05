//  bits freq min freq max (Hz)
//  1    19532    ?
//  2    9766     10019569
//  3    4883     5009784
//  4    2442     2504892
//  5    1221     1252446
//  6    611      626223
//  7    306      313111
//  8    153      156555
//  9    77       78277
//  10   39       39138
//  11   20       19569
//  12   10       9784
//  13   5        4892
//  14   3        2446

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "CN0411.h"

#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Communication.h"

using namespace CN0411;

/******************************************************************************/
/*************************** Variable Definitions *****************************/
/******************************************************************************/

uint8_t pwm_status;

/******************************************************************************/
/************************* Function Definitions *******************************/
/******************************************************************************/

/**
 * Set PWM frequency
 *
 * @param freq - frequency value to be set.
 * @return none
 */
void CN0411_pwm_freq(uint16_t freq) {
   switch (freq) {
      case PWM_FREQ_94:
         ledcSetup(4, 94, 12);  // Canal 4, 94 Hz, 8 bits de resolução
         break;
      case PWM_FREQ_2400:
         ledcSetup(4, 2400, 12);  // Canal 4, 2400 Hz, 8 bits de resolução
         break;
      default:
         break;
   }
}

/**
 * Generate PWM

 * @return none
*/
void CN0411_pwm_gen(void) {
   switch (pwm_status) {
      case PWM_SYSCALIB_AIN7:
         digitalWrite(18, LOW);   // PWM3
         digitalWrite(17, HIGH);  // PWM2
         digitalWrite(14, HIGH);  // PWM1
         break;
      case PWM_SYSCALIB_AIN8:
         digitalWrite(18, HIGH);  // PWM3
         digitalWrite(17, LOW);   // PWM2
         digitalWrite(14, LOW);   // PWM1
         break;
      case PWM_CONVERSION:
         ledcAttachPin(14, 4);
         ledcWrite(4, 2048);  // 50% duty cycle
         break;
      default:
         break;
   }
}

/**
 * CN0411 Initialization
 *
 * @param cn0411_dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
void CN0411_init(void) {
   int32_t ret;

   /* Initialize PWM */
   pinMode(18, OUTPUT);
   pinMode(17, OUTPUT);
   pinMode(14, OUTPUT);

   CN0411_pwm_freq(PWM_FREQ_94);
   pwm_status = PWM_CONVERSION;
   CN0411_pwm_gen();

   SPI_Init();

   /* Initial Setup ADC */

   /* Initial Setup DAC */

   /* Setup ADC */

   /* Set DAC output value */
}