#include "Arduino.h"
#include "bldc.h"
#include "SoftPWM.h"


#define UL  PINB5	//! Port pin connected to phase U, low side enable switch.  Arduino Pin 13
#define UH  PINB4	//! Port pin connected to phase U, high side enable switch. Arduino Pin 12
#define VL  PINB3	//! Port pin connected to phase V, high side enable switch. Arduino Pin 11
#define VH  PINB2	//! Port pin connected to phase V, high side enable switch. Arduino Pin 10
#define WL  PINB1	//! Port pin connected to phase W, low side enable switch.	Arduino Pin 9
#define WH  PINB0	//! Port pin connected to phase W, high side enable switch. Arduino Pin 8
#define UL_Pin  13
#define UH_Pin  12
#define VL_Pin  11
#define VH_Pin  10
#define WL_Pin  9
#define WH_Pin  8




static short DRIVE_PATTERN_STEP1 = ((1 << UL) | (1 << VH));	//! Drive pattern for commutation step 1,
static short DRIVE_PATTERN_STEP2 = ((1 << UL) | (1 << WH));	//! Drive pattern for commutation step 2,
static short DRIVE_PATTERN_STEP3 = ((1 << VL) | (1 << WH));	//! Drive pattern for commutation step 3,
static short DRIVE_PATTERN_STEP4 = ((1 << VL) | (1 << UH));	//! Drive pattern for commutation step 4,
static short DRIVE_PATTERN_STEP5 = ((1 << WL) | (1 << UH));	//! Drive pattern for commutation step 5,
static short DRIVE_PATTERN_STEP6 = ((1 << WL) | (1 << VH));	//! Drive pattern for commutation step 6,


#define EXTERNAL_REF_VOLTAGE      	  	((4930UL * 10) / 43)	//! External reference voltage in milliVolts.
#define SHUNT_RESISTANCE          		1	//! Current measurement shunt value in milliOhm.
#define ADC_RESOLUTION   				256	//! The ADC resolution used.

#define P_REG_K_P 64	//! P-regulator proportional gain.
#define P_REG_SCALING 65536	//! P-regulator scaling factor. The result is divided by this number.
#define MAX_COMMUTATION_STEPS 7
#define MIN_COMMUTATION_STEPS 0

#define MAX_ADC_DATA 5
#define MIN_ADC_DATA 0



BLDC::BLDC()
{
    speed = 0;
    forward = true;
    HallIndex = 0;
    CommunationState = 0;
}

BLDC::~BLDC()
{

}

void BLDC::init()
{
    pinMode(UL_Pin, OUTPUT);
    pinMode(VL_Pin, OUTPUT);
    pinMode(WL_Pin, OUTPUT);
    digitalWrite(UL_Pin,0);
    digitalWrite(VL_Pin,0);
    digitalWrite(WL_Pin,0);

    SoftPWMBegin();
    SoftPWMSetPercent(VH_Pin,0);
    SoftPWMSetPercent(WH_Pin,0);
    SoftPWMSetPercent(UH_Pin,0);

}


void BLDC::ReadADCHalls()
{
    RawHallData[0] = analogRead(ADC_HALL_1);
    RawHallData[1] = analogRead(ADC_HALL_2);
    RawHallData[2] = analogRead(ADC_HALL_3);
    for(int i = 0; i < NUMBER_HALLS; i++)
	{
        HallStates[i] = (RawHallData[i] > HALL_THRESHOLD);
        Serial.print("Hall States: ");
        Serial.println(HallStates[i]);
    }
}

unsigned short BLDC::LookupIndex()
{
    unsigned short index = 0;
    for(int i = 0; i < NUMBER_HALLS; i++)
    {
        index |= ((unsigned short)HallStates[i]) << i;
    }
    HallIndex = index;

    return LookupTable[(unsigned int)forward][HallIndex];
}

void BLDC::setSpeed(unsigned long _speed)
{
    speed = _speed;
}

void BLDC::Control()
{
    ReadADCHalls();
    SetCommutationState(LookupIndex());
}

void BLDC::SetCommutationState(unsigned short state)
{
	char UL_duty = 0;
	char UH_duty = 0;
	char VL_duty = 0;
	char VH_duty = 0;
	char WL_duty = 0;
	char WH_duty = 0;


    CommunationState = state;
	if( (state >= MIN_COMMUTATION_STEPS) && (state < MAX_COMMUTATION_STEPS))
	{
		switch(driveTable[state])
		{
            case 0:
                 UL_duty = 0;
                 UH_duty = 0;
                 VL_duty = 0;
                 VH_duty = 0;
                 WL_duty = 0;
                 WH_duty = 0;
                 break;

			case DRIVE_PATTERN_STEP1_CCW:
				 UL_duty = 1;
				 VH_duty = 75;
				 break;
			case DRIVE_PATTERN_STEP2_CCW:
				 UL_duty = 1;
				 WH_duty = 75;
				 break;
			case DRIVE_PATTERN_STEP3_CCW:
				 VL_duty = 1;
				 WH_duty = 75;
				 break;
			case DRIVE_PATTERN_STEP4_CCW:
				 VL_duty = 1;
				 UH_duty = 75;
				 break;
			case DRIVE_PATTERN_STEP5_CCW:
				 WL_duty = 1;
				 UH_duty = 75;
				 break;
			case DRIVE_PATTERN_STEP6_CCW:
				 WL_duty = 1;
				 VH_duty = 75;
				 break;
		
		}
	}


	SoftPWMSetPercent(UH_Pin, UH_duty);
	SoftPWMSetPercent(VH_Pin, VH_duty);
	SoftPWMSetPercent(WH_Pin, WH_duty);
    digitalWrite(UL_Pin, UL_duty);
    digitalWrite(VL_Pin, VL_duty);
    digitalWrite(WL_Pin, WL_duty);

}





