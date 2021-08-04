#ifndef MODULES_RESETPIN_H_
#define MODULES_RESETPIN_H_

#include "modules/module.h"

class ResetPin : public Module {
  private:

    volatile bool *ptrReset;     // pointer to the data source
    stm32plus::GpioPinRef pin;

  public:

    ResetPin(volatile bool&, const stm32plus::GpioPinRef&);
    virtual void update(void);
    virtual void slowUpdate(void);

};

#endif /* MODULES_RESETPIN_H_ */
