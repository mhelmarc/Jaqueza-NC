/*
 * configuration.cpp
 *
 *  This file is part of Jaqueza-NC a Non-PRU port of Remora
 *  (https://github.com/scottalford75/Remora) a free, opensource LinuxCNC
 *  component and Programmable Realtime Unit (PRU)
 *
 *  Copyright (C) 2021
 *
 */


#include "configuration.h"
#include "remora.h"

#include "drivers/SerialComm.h"

#include "modules/module.h"
#include "modules/blink/blink.h"
#include "modules/alarm/alarm.h"
#include "modules/resetPin/resetPin.h"
#include "modules/stepgen/stepgen.h"
#include "modules/digitalPin/digitalPin.h"
#include "modules/eStop/eStop.h"
#include "modules/switch/switch.h"
#include "modules/pwm/hardwarePwm.h"

#include "thread/pruThread.h"

//#include "utils/utils.h"

/* These are declared in main, but we need access to it here */
extern PRUThread<ServoThreadTimer_t> *servoThread;
extern PRUThread<BaseThreadTimer_t> *baseThread;

/* Pointers to data */
volatile int32_t* ptrTxHeader;
volatile bool*    ptrPRUreset;
volatile int32_t* ptrJointFreqCmd[JOINTS];
volatile int32_t* ptrJointFeedback[JOINTS];
volatile uint8_t* ptrJointEnable;
volatile float*   ptrSetPoint[VARIABLES];
volatile float*   ptrProcessVariable[VARIABLES];
volatile uint8_t* ptrInputs;
volatile uint8_t* ptrOutputs;

/* Blinker */
void makeBlinker( const GpioPinRef led, uint8_t frequency) {
  Module *blinker = new Blink(led, static_cast<uint32_t>(PRU_SERVOFREQ), frequency);
  servoThread->registerModule(blinker);
}

/* Alarm */
void makeAlarm(const stm32plus::GpioPinRef& pin, volatile uint32_t* freq, volatile bool* status) {
  Module *alarm = new Alarm(pin, static_cast<uint32_t>(PRU_SERVOFREQ), freq, status);
  servoThread->registerModule(alarm);
}

/* EStop */
void makeEStopper(const GpioPinRef estop_pin) {
  ptrTxHeader = &txData.data.header;
  Module *estop = new eStop(*ptrTxHeader, estop_pin);
  servoThread->registerModule(estop);
}


/* Reset Pin */
void makeResetter(volatile bool *pReset,  const GpioPinRef resetPin) {
  ptrPRUreset = pReset;
  Module *resetter = new ResetPin(*ptrPRUreset, resetPin);
  servoThread->registerModule(resetter);
}


/* Step generators */
void makeStepGenerator(int joint, const GpioPinRef step, const GpioPinRef direction,
    const GpioPinRef enable) {

  // pointers to data source and feedback location
  ptrJointFreqCmd[joint] = &rxData.data.jointFreqCmd[joint];
  ptrJointEnable = &rxData.data.jointEnable;

  ptrJointFeedback[joint] = &txData.data.jointFeedback[joint];

  Module *stepgen = new Stepgen(static_cast<int32_t>(PRU_BASEFREQ), joint, step,
      direction, enable, STEPBIT, *ptrJointFreqCmd[joint],
      *ptrJointFeedback[joint], *ptrJointEnable);

  baseThread->registerModule(stepgen);
}


/* Limit pins */
void makeLimitPin(int bit, bool invert, const GpioPinRef pin) {
    ptrInputs = &txData.data.inputs;
    Module *limitPin = new DigitalPin(*ptrInputs, 0, pin, bit, invert);
    servoThread->registerModule(limitPin);
}


/* Spindle PWM */
//FIXME not working yet, PWM is working but the module not yet.
void initPwm(uint8_t index, uint32_t frequency) {
  ptrSetPoint[index] = &rxData.data.setPoint[index];
  Module *pwm = new HardwarePWM(*ptrSetPoint[index], frequency);
  servoThread->registerModule(pwm);
}




