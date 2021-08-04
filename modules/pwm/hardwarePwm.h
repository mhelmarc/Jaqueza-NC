#ifndef HARDWAREPWM_H
#define HARDWAREPWM_H

#include "modules/module.h"
#include "drivers/PWMTimer.h"

//TODO make this usable with other timer PWM

class HardwarePWM: public Module {
  private:
    enum {
      PWM_BASE_FREQUENCY = 6000000
    };

    /* PWM pin is defined in the timer configuration */
//		int pwmMax;					        // maximum PWM output
//		int pwmSP;					        // PWM setpoint as a percentage of maxPwm

    PWMTimer_t pwmPin;                      // PWM out object

    volatile float *ptrPwmPeriod {NULL}; 	// pointer to the data source
    volatile float *ptrPwmPulseWidth; 	    // pointer to the data source

    uint32_t frequency;                     // frequency (Hz)
    float pwmPulseWidth { 0.0 };            // Pulse width (%)
    int pwmPulseWidth_us {0};               // Pulse width (us)

    bool variablePeriod { false };

  public:

    HardwarePWM(volatile float&, uint32_t);
    HardwarePWM(volatile float&, volatile float&, uint32_t);

    virtual void update(void);          // Module default interface
    virtual void slowUpdate(void);      // Module default interface
};

#endif

