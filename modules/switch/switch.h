#ifndef SWITCH_H
#define SWITCH_H

#include "modules/module.h"


class Switch : public Module
{

	private:

		volatile float* ptrPV; 			// pointer to the data source
		float 			PV;
		float 			SP;
		bool			mode;			// 0 switch off, 1 switch on

		stm32plus::GpioPinRef     pin;


	public:

		Switch(float, volatile float&, const stm32plus::GpioPinRef&, bool);

		virtual void update(void);
		virtual void slowUpdate(void);
};

#endif
