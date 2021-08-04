#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#define PRU_BASEFREQ    	100000 //24000   // PRU Base thread ISR update frequency (hz)
#define PRU_SERVOFREQ       1000            // PRU Servo thread ISR update freqency (hz)
#define STEPBIT     		22            	// bit location in DDS accum
#define STEP_MASK   		(1L<<STEPBIT)

#define JSON_BUFF_SIZE	    10000			// Jason dynamic buffer size

//#define JOINTS			  8				// Number of joints - set this the same as LinuxCNC HAL compenent. Max 8 joints
//#define VARIABLES           6            	// Number of command values - set this the same as the LinuxCNC HAL compenent

#define PRU_DATA		    0x64617461 	    // "data" SPI payload
#define PRU_READ            0x72656164      // "read" SPI payload
#define PRU_WRITE           0x77726974      // "writ" SPI payload
#define PRU_ESTOP           0x65737470      // "estp" SPI payload


// Serial configuration
/*
  USART1 on PortA
  TX    PA9
  RX    PA10
 */
#define PC_BAUD             115200          // UART baudrate

#define LOOP_TIME           100             // msec
#define SPI_ERR_MAX         5
// PRU reset will occur in SPI_ERR_MAX * LOOP_TIME = 0.5sec

/*
 LED on PE0

 SPI1 on Port A4:A7 on STM32F4
 NSS    PA4
 SCK    PA5
 MiSO   PA6
 MOSI   PA7

 Step / Direction / Enable pins PD0:PD11
         Step   Dir   Enable
 X-axis: PD0    PD1   PD2
 Y-axis: PD3    PD4   PD5
 Z-axis: PD6    PD7   PD8
 A-axis: PD9    PD10  PD11

 Limit pins on PE1:PE6
 PE1 Xmin, PE2 Xmax
 PE3 Ymin, PE4 Ymax
 PE5 Zmin, PE6 Zmax

 Spindle PWM output
 PD14

*/

#include "config/stm32plus.h"
#include "config/gpio.h"

/* Step and Direction output pins on PORT D0:D11 */
enum {
  STEP_PIN = 0, DIRECTION_PIN = 1, ENABLE_PIN = 2
};

void makeEStopper(const stm32plus::GpioPinRef);
void makeBlinker( const stm32plus::GpioPinRef, uint8_t);
void makeResetter(volatile bool*,  const stm32plus::GpioPinRef);
void makeStepGenerator(int, const  stm32plus::GpioPinRef, const  stm32plus::GpioPinRef,  const stm32plus::GpioPinRef);
void makeLimitPin(int, bool, const stm32plus::GpioPinRef);
void initPwm(uint8_t, uint32_t);

#endif
