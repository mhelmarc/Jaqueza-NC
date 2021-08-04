#ifndef STEPGEN_H
#define STEPGEN_H

#include "modules/module.h"

class Stepgen: public Module {
  private:

    int jointNumber;              	// LinuxCNC joint number
    int mask;

    stm32plus::GpioPinRef step, direction, enable;  // physical pins connections

    bool isEnabled;        	// flag to enable the step generator
    bool isForward;        	// current diretion

    int32_t stepBit; // position in the DDS accumulator that triggers a step pulse
    int32_t frequencyCommand; // the joint frequency command generated by LinuxCNC
    volatile int32_t *ptrFrequencyCommand; // pointer to the data source where to get the frequency command
    int32_t rawCount; // current position raw count - not currently used - mirrors original stepgen.c
    volatile int32_t *ptrFeedback;      // pointer where to put the feedback
    volatile uint8_t *ptrJointEnable;
    int32_t DDSaccumulator;       // Direct Digital Synthesis (DDS) accumulator
    float frequencyScale;		  	// frequency scale
    int32_t DDSaddValue;		  	// DDS accumulator add vdd value

  public:

    Stepgen(int32_t, int, const stm32plus::GpioPinRef&, const stm32plus::GpioPinRef&,
        const stm32plus::GpioPinRef&, int, volatile int32_t&, volatile int32_t&,
        volatile uint8_t&); // constructor

    virtual void update(void);           // Module default interface
    virtual void slowUpdate(void);
    void makePulses();
    void setEnabled(bool);
};

#endif
