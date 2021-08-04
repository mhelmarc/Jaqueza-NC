/*
 * alarm.cpp
 *	  This file is part of jaqueza-netnc
 *    Copyright (C) 2021 Mhel Marcelo
 *
 */

#include "config/stm32plus.h"
#include "config/gpio.h"
#include "alarm.h"


Alarm::Alarm(const stm32plus::GpioPinRef& Pin, uint32_t threadFreq,
   volatile uint32_t* freq, volatile bool* alarm) {

  this->frequency = freq;
  this->threadFrequency = threadFreq;
  this->periodCount = 0;
  this->tick = 0;
  this->bState = false;
  this->beeperPin = Pin;
  this->beeperPin.setState(bState);
  this->alarmIsOn = alarm;
}


void Alarm::update(void) {
  if (this->alarmIsOn) {
    this->periodCount = this->threadFrequency / *(this->frequency);
    ++this->tick;
    if (this->tick >= (this->periodCount / 2)) {
      this->beeperPin.setState(this->bState ^= true);
      this->tick = 0;
    }
  }
}


void Alarm::slowUpdate(void) {
  return;
}
