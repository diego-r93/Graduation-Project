#ifndef TIMER_H_
#define TIMER_H_

#include <Arduino.h>

#include "freeRTOSTimerController.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

typedef uint32_t timer_ticks_t;

void timer_start(void);
void timer_sleep(timer_ticks_t ticks);
void timer_sleep_5uS(timer_ticks_t ticks);

#endif  // TIMER_H_