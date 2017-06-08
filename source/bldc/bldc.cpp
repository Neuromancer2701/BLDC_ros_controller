#include "Arduino.h"
#include "bldc.h"
#include "SoftPWM.h"


BLDC::BLDC()
{
    speed = 0;
    forward = true;
    CommunationState = 0;

    pinMode(HALL1, INPUT);
    pinMode(HALL2, INPUT);
    pinMode(HALL3, INPUT);

    pinMode(AL, OUTPUT);
    pinMode(BL, OUTPUT);
    pinMode(CL, OUTPUT);
    digitalWrite(AL, 0);
    digitalWrite(BL, 0);
    digitalWrite(CL, 0);

    SoftPWMBegin();
    SoftPWMSetPercent(AH, 0);
    SoftPWMSetPercent(BH, 0);
    SoftPWMSetPercent(CH, 0);
}

BLDC::~BLDC()
{

}

void BLDC::ReadHalls()
{
    RawHallData[0] = digitalRead(HALL1);
    RawHallData[1] = digitalRead(HALL2);
    RawHallData[2] = digitalRead(HALL3);
}


void BLDC::Control()
{
    setSpeed(50);
    SetCommutationState();
}

void BLDC::SetCommutationState()
{
    uint8_t AH_duty = 0;
	uint8_t AL_duty = 0;

    uint8_t BH_duty = 0;
    uint8_t BL_duty = 0;

    uint8_t CH_duty = 0;
    uint8_t CL_duty = 0;


    ReadHalls();
    CommunationState = ((RawHallData[2] << 2) | (RawHallData[1] << 1) | RawHallData[1]);
		switch(CommunationState)
		{

			case State1:
                 if(forward)
                 {
                    AH_duty = speed;
                    CL_duty = 1;
                 }
                 else
                 {
                     CH_duty = speed;
                     AL_duty = 1;
                 }
				 break;
			case State2:
                 if(forward)
                 {
                    BH_duty = speed;
                    CL_duty = 1;
                 }
                 else
                 {
                    CH_duty = speed;
                    BL_duty = 1;
                 }
                 break;
			case State3:
                 if(forward)
                 {
                    BH_duty = speed;
                    AL_duty = 1;
                 }
                 else
                 {
                    AH_duty = speed;
                    BL_duty = 1;
                 }
                 break;
			case State4:
                 if(forward)
                 {
                    CH_duty = speed;
                    AL_duty = 1;
                 }
                 else
                 {
                    AH_duty = speed;
                    CL_duty = 1;
                 }
                 break;
			case State5:
                 if(forward)
                 {
                    CH_duty = speed;
                    BL_duty = 1;
                 }
                 else
                 {
                    BH_duty = speed;
                    CL_duty = 1;
                 }
                 break;
			case State6:
                if(forward)
                {
                    AH_duty = speed;
                    BL_duty = 1;
                }
                else
                {
                    BH_duty = speed;
                    AL_duty = 1;
                }
                break;

            default:
                break;
		
		}

	SoftPWMSetPercent(AH, AH_duty);
	SoftPWMSetPercent(BH, BH_duty);
	SoftPWMSetPercent(CH, CH_duty);
    digitalWrite(AL, AL_duty);
    digitalWrite(BL, BL_duty);
    digitalWrite(CL, CL_duty);
}

unsigned short *BLDC::getRawHallData(){
    return (unsigned short *)RawHallData;
}





