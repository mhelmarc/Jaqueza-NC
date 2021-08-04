#include "config/stm32plus.h"
#include "config/gpio.h"
#include "resetPin.h"

ResetPin::ResetPin(volatile bool &ptrReset, const stm32plus::GpioPinRef& pin) :
    ptrReset(&ptrReset) {
  /* Pin used for this is an INPUT, mode is already set before calling this class */
  this->pin = pin;
}

void ResetPin::update() {

  *(this->ptrReset) = this->pin.read();

}

void ResetPin::slowUpdate() {
  return;
}
