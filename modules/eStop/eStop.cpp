#include "config/stm32plus.h"
#include "config/gpio.h"
#include "eStop.h"


eStop::eStop(volatile int32_t &ptrTxHeader, const stm32plus::GpioPinRef& pin) :
    ptrTxHeader(&ptrTxHeader)
{
  /* Pin used for this is an INPUT, mode is already set before calling this class */
  this->pin = pin;
}



void eStop::update()
{
    if (this->pin.read() == true)
    {
        *ptrTxHeader = PRU_ESTOP;
    }
    else {
        *ptrTxHeader = PRU_DATA;
    }
}

void eStop::slowUpdate()
{
	return;
}
