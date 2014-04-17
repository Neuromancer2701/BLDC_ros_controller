// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
*
* Atmel Corporation
*
* - File              : BLDC.h
* - Compiler          : IAR EWAAVR 2.28a/3.10c
*
* - Support mail      : avr@atmel.com
*
* - Supported devices : ATmega48/88/168
*
* - AppNote           : AVR444 - Sensorless control of three-phase brushless
'                       DC motors with ATmega48.
*
* - Description       : Example of how to use the ATmega48 for sensorless
*                       control of a three phase brushless DC motor.
*SS
* $Revision: 1.1 $
* $Date: Monday, October 10, 2005 11:15:46 UTC $
*****************************************************************************/

#ifndef __BLDC_H__
#define __BLDC_H__

class BLDC
{
public:
	BLDC();
	virtual ~BLDC();

	void init();
	void setSpeed(unsigned long _speed);
	void Control();

private:
	unsigned long speed;
};



#endif  // File guard
