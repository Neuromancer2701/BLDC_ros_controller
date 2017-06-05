// This file has been prepared for Doxygen automatic documentation generation.
//Seth M King

#ifndef __BLDC_H__
#define __BLDC_H__

#include <stdint.h>

enum commumationStates
{
    State1   = 0x04,
    State2   = 0x06,
    State3   = 0x02,
    State4   = 0x03,
    State5   = 0x01,
    State6   = 0x05
};

enum FET_PINS
{
    AH = 4,	 //! Port pin connected to phase A, high side enable switch. Arduino Pin 12
    AL = 12, //! Port pin connected to phase A, low side enable switch.  Arduino Pin 4

    BH = 9,	 //! Port pin connected to phase B, high side enable switch. Arduino Pin 9
    BL = 8,	 //! Port pin connected to phase B, low side enable switch. Arduino Pin 8

    CH = 5,	 //! Port pin connected to phase C, high side enable switch. Arduino Pin 5
    CL = 13	 //! Port pin connected to phase C, low side enable switch.	Arduino Pin 13
};

enum HALL_PINS
{
    HALL1 = 2,	 //! Port pin connected to phase A, high side enable switch. Arduino Pin 2
    HALL2 = 1, //! Port pin connected to phase A, low side enable switch.  Arduino Pin 1
    HALL3 = 0	 //! Port pin connected to phase B, high side enable switch. Arduino Pin 0
};



class BLDC
{
public:
	BLDC();
	virtual ~BLDC();

	void setSpeed(uint8_t _speed) {speed = _speed;}
	void Control();
	void SetCommutationState();

    volatile unsigned short getCommunationState() { return CommunationState; }

    unsigned short* getRawHallData();

    void Reverse(){forward = false;}
    void Forward(){forward = true;}

    enum constants
    {
        NUMBER_HALLS = 3,
    };

private:
	unsigned char speed;
	bool forward;
    volatile unsigned short RawHallData[NUMBER_HALLS];
    volatile unsigned short CommunationState;



	void ReadHalls();
};



#endif 
