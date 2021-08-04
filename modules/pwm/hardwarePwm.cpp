#include "config/stm32plus.h"
#include "drivers/SerialComm.h"
#include "hardwarePwm.h"

HardwarePWM::HardwarePWM(volatile float &ptrPwmPulseWidth, uint32_t freq) :
    ptrPwmPulseWidth(&ptrPwmPulseWidth), frequency(freq) {

  xputs(" Creating Hardware PWM\n");

  uint16_t period;
  float f = static_cast<float>(frequency);
  period = floorf(PWM_BASE_FREQUENCY / f);

  variablePeriod = false;

  this->pwmPin.setTimeBaseByFrequency(PWM_BASE_FREQUENCY, period - 1);

  /* Initialize PWM channel in edge-aligned mode */
  this->pwmPin.initCompareForPwmOutput();
  /* Enable the timer. */
  this->pwmPin.enablePeripheral();

}


/* TODO */
HardwarePWM::HardwarePWM(volatile float &ptrPwmPeriod, volatile float &ptrPwmPulseWidth, uint32_t freq) :
ptrPwmPeriod(&ptrPwmPeriod), ptrPwmPulseWidth(&ptrPwmPulseWidth), frequency(freq)
{
//  cout << "Creating variable frequency Hardware PWM at pin " << this->pin << endl;
//  xputs("Creating variable frequency Hardware PWM\n");
//  uint16_t period;
//  float f = static_cast<float>(frequency);
//  period = floorf(PWM_BASE_FREQUENCY / f);
//
//  variablePeriod = true;
//
//  this->pwmPin.setTimeBaseByFrequency(PWM_BASE_FREQUENCY, ptrPwmPeriod - 1);
//
//  /* Initialize PWM channel in edge-aligned mode (TIM_OCMode_PWM1). */
//  this->pwmPin.initCompareForPwmOutput();
//  /* Enable the timer. The PWM outputs is on PD14. */
//  this->pwmPin.enablePeripheral();

}

void HardwarePWM::update() {
//  if (variablePeriod) {
//    if (*(this->ptrPwmPeriod) != 0
//        && (*(this->ptrPwmPeriod) != this->pwmPeriod)) {
//      // PWM period has changed
//      this->pwmPeriod = *(this->ptrPwmPeriod);
//      this->pwmPin->period_us(this->pwmPeriod);
//      this->pwmPulseWidth_us = (this->pwmPeriod * this->pwmPulseWidth) / 100.0;
//      this->pwmPin->pulsewidth_us(this->pwmPulseWidth_us);
//    }
//  }

//  if (*(this->ptrPwmPulseWidth) != this->pwmPulseWidth) {
    // PWM duty has changed
//    this->pwmPulseWidth = *(this->ptrPwmPulseWidth);
//    this->pwmPulseWidth_us = (this->pwmPeriod * this->pwmPulseWidth) / 100.0;
    this->pwmPin.setDutyCycle(*(this->ptrPwmPulseWidth));
//  }

  return;
}

void HardwarePWM::slowUpdate() {
  return;
}
