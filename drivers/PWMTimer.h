/*
 * pwmTimers.h
 *
 *  This file is part of Jaqueza-NC a Non-PRU port of Remora
 *  (https://github.com/scottalford75/Remora) a free, opensource LinuxCNC
 *  component and Programmable Realtime Unit (PRU)
 *
 *  Copyright (C) 2021
 *
 */


#ifndef PWMTIMER_H_
#define PWMTIMER_H_

#include "config/stm32plus.h"
#include "config/timer.h"
#include <cmath>

using namespace stm32plus;

/* We'll use TIM4 for PWM using channels 1:4 Port D12:D13 */
//typedef Timer4<
//    Timer4InternalClockFeature,       // the timer clock source is APB1
////    TimerChannel1Feature<>,           // we're going to use channel 1
////    TimerChannel2Feature<>,
//    TimerChannel3Feature<>,
////    TimerChannel4Feature<>,
//    Timer4GpioFeature<          // we want to output to GPIO
//      TIMER_REMAP_FULL,         // the GPIO output will be remap to PD12:PD15
////      TIM4_CH1_OUT              // PD12
////      TIM4_CH2_OUT,             // PD13
//      TIM4_CH3_OUT             // PD14
////      TIM4_CH3_OUT              // PD15
//    >
//> PWMTimer_t;

/* The PWM outputs is on PD14. */
typedef Timer4<
  Timer4InternalClockFeature,   // timer clock source is APB1
  TimerChannel3Feature<>,
  Timer4GpioFeature<
    TIMER_REMAP_FULL,
    TIM4_CH3_OUT
  >
> PWMTimer_t;



#endif /* PWMTIMER_H_ */
