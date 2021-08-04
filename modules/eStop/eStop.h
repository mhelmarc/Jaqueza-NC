#ifndef ESTOP_H
#define ESTOP_H

#include "modules/module.h"
#include "../../configuration.h"


class eStop: public Module {

  private:

    volatile int32_t *ptrTxHeader;
    stm32plus::GpioPinRef pin;
  public:

    eStop(volatile int32_t&, const stm32plus::GpioPinRef&);

    virtual void update(void);
    virtual void slowUpdate(void);
};

#endif
