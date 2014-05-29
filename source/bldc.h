// This file has been prepared for Doxygen automatic documentation generation.
//Seth M King

#ifndef __BLDC_H__
#define __BLDC_H__

//Profiles will use C++11 enum class so I clased it in #ifdef block for the time being
#ifdef profiles

class enum MotorType			//class for different motor profiles
{
	Default = 0,
	Apex    = 1,
	E-FliteOutrunner = 2,
	MAX_TYPE = E-FliteOutrunner + 1
};


class Motor
{
	double nominalVoltage;
	double maxCurrent;
}

#endif

class BLDC
{
public:
	BLDC();
	virtual ~BLDC();

	void init();
	void setSpeed(unsigned long _speed);
	void Control();
	void SetCommutationState(unsigned char state);
	unsigned char AnalogData(unsigned char mux);

private:
	unsigned long speed;
};



#endif 
