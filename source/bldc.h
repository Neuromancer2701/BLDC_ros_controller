// This file has been prepared for Doxygen automatic documentation generation.
//Seth M King

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
	void SetCommutationState(unsigned short state);

    volatile unsigned short getCommunationState() { return CommunationState; }
    volatile unsigned short getHallIndex() { return HallIndex; }

    void Reverse(){forward = false;}
    void Forward(){forward = true;}
    unsigned short LookupIndex();

    enum constants
    {
        HALL_THRESHOLD = 1000,
        NUMBER_HALLS = 3,
        HALL_STATES = 8
    };

private:
	unsigned long speed;
	bool forward;
    volatile bool HallStates[NUMBER_HALLS];
    volatile unsigned short RawHallData[NUMBER_HALLS];

    volatile unsigned short HallIndex;
    volatile unsigned short CommunationState;

	void ReadADCHalls();
};

static const unsigned short LookupTable[2][BLDC::HALL_STATES] = {
        {0, 5, 1, 6, 3, 4, 2, 0},
        {0, 2, 4, 3, 6, 1, 5, 0}
};




#endif 
