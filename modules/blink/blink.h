#ifndef BLINK_H
#define BLINK_H

#include "modules/module.h"


class Blink : public Module
{

	private:
		bool 		bState;
		uint32_t 	periodCount;
		uint32_t 	blinkCount;

		stm32plus::GpioPinRef blinkPin;	// class object members - Pin objects

	public:

		Blink(const stm32plus::GpioPinRef&, uint32_t, uint32_t);

		virtual void update(void);
		virtual void slowUpdate(void);
};

#endif
