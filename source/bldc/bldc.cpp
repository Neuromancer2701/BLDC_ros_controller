#include "Arduino.h"
#include "bldc.h"
#include "SoftPWM.h"


BLDC::BLDC()
{
    speed = 0;
    counter = 0;
    forward = true;
    currentCommunationState = State6;
    newCommunationState =  State6;
    sprintf(data,"nothing \0");


    pinMode(HALL1, INPUT);
    pinMode(HALL2, INPUT);
    pinMode(HALL3, INPUT);

    pinMode(AH, OUTPUT);
    pinMode(BH, OUTPUT);
    pinMode(CH, OUTPUT);
    pinMode(AL, OUTPUT);
    pinMode(BL, OUTPUT);
    pinMode(CL, OUTPUT);
    digitalWrite(AH, 0);
    digitalWrite(BH, 0);
    digitalWrite(CH, 0);
    digitalWrite(AL, 0);
    digitalWrite(BL, 0);
    digitalWrite(CL, 0);


    //SoftPWMBegin();
    //SoftPWMSetPercent(AH, 0);
    //SoftPWMSetPercent(BH, 0);
    //SoftPWMSetPercent(CH, 0);
}

BLDC::~BLDC()
{

}

void BLDC::ReadHalls()
{
    RawHallData[HALL1_INDEX] = digitalRead(HALL1);
    RawHallData[HALL2_INDEX] = digitalRead(HALL2);
    RawHallData[HALL3_INDEX] = digitalRead(HALL3);
}


void BLDC::Control()
{
    setSpeed(50);
    CalculateCommutationState();
}

void BLDC::CalculateCommutationState()
{
    uint8_t AH_duty = 0;
	uint8_t AL_duty = 0;

    uint8_t BH_duty = 0;
    uint8_t BL_duty = 0;

    uint8_t CH_duty = 0;
    uint8_t CL_duty = 0;


    ReadHalls();
    newCommunationState = (commumationStates)((RawHallData[HALL1_INDEX] << HALL1_SHIFT) | (RawHallData[HALL2_INDEX] << HALL2_SHIFT) | RawHallData[HALL3_INDEX]);

    if(newCommunationState == currentCommunationState)
    {
        return;
    }
    else
    {
        currentCommunationState = newCommunationState;
        counter;
    }

		switch(currentCommunationState)
		{

			case State1:
                 if(forward)
                 {
                     PORTD = 0x10;
                     PORTB = 0x20;
                    //AH_duty = speed;
                    //CL_duty = 1;
                 }
                 else
                 {
                     //CH_duty = speed;
                     //AL_duty = 1;
                 }
				 break;
			case State2:
                 if(forward)
                 {
                     PORTD = 0x00;
                     PORTB = 0x22;
                    //BH_duty = speed;
                    //CL_duty = 1;
                 }
                 else
                 {
                    //CH_duty = speed;
                    //BL_duty = 1;
                 }
                 break;
			case State3:
                 if(forward)
                 {
                     PORTD = 0x00;
                     PORTB = 0x12;
                 }
                 else
                 {
                    //AH_duty = speed;
                    //BL_duty = 1;
                 }
                 break;
			case State4:
                 if(forward)
                 {
                     PORTD = 0x20;
                     PORTB = 0x10;
                 }
                 else
                 {
                    //AH_duty = speed;
                    //CL_duty = 1;
                 }
                 break;
			case State5:
                 if(forward)
                 {
                     PORTD = 0x20;
                     PORTB = 0x01;
                    //CH_duty = speed;
                    //BL_duty = 1;
                 }
                 else
                 {
                    //BH_duty = speed;
                    //CL_duty = 1;
                 }
                 break;
			case State6:
                if(forward)
                {
                    PORTD = 0x10;
                    PORTB = 0x01;
                    //AH_duty = speed;
                    //BL_duty = 1;
                }
                else
                {
                    //BH_duty = speed;
                    //AL_duty = 1;
                }
                break;

            default:
                PORTD = 0x00;
                PORTB = 0x00;
                break;
		
		}

#if 0
    sprintf(data,"AH:%d BH:%d CH:%d AL:%d BL:%d Cl:%d Halls: %d %d %d",
    AH_duty, BH_duty, CH_duty, AL_duty, BL_duty, CL_duty, RawHallData[HALL1_INDEX], RawHallData[HALL2_INDEX], RawHallData[HALL3_INDEX]);

    digitalWrite(AL, 0);
    digitalWrite(BL, 0);
    digitalWrite(CL, 0);
    SoftPWMSetPercent(AH, 0);
    SoftPWMSetPercent(BH, 0);
    SoftPWMSetPercent(CH, 0);

	SoftPWMSetPercent(AH, AH_duty);
	SoftPWMSetPercent(BH, BH_duty);
	SoftPWMSetPercent(CH, CH_duty);
    digitalWrite(AL, AL_duty);
    digitalWrite(BL, BL_duty);
    digitalWrite(CL, CL_duty);
#endif
}

int *BLDC::getRawHallData(){
    return (int *)RawHallData;
}









