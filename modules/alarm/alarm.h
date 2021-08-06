/*
 * alarm.h
 *	  This file is part of jaqueza-netnc
 *
 */


#ifndef MODULES_ALARM_ALARM_H_
#define MODULES_ALARM_ALARM_H_


#include "modules/module.h"

class Alarm : public Module
{

    private:
        bool        bState;
        uint32_t    periodCount;
        uint32_t    tick;
        volatile uint32_t*   frequency;
        uint32_t    threadFrequency;
        volatile bool*       alarmIsOn;
        stm32plus::GpioPinRef beeperPin; // class object members - Pin objects

    public:

        Alarm(const stm32plus::GpioPinRef&, uint32_t, volatile uint32_t*, volatile bool*);

        virtual void update(void);
        virtual void slowUpdate(void);
};




#endif /* MODULES_ALARM_ALARM_H_ */
