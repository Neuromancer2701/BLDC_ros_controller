#include "Arduino.h"
#include "bldc.h"
#include "SoftPWM.h"

SOFTPWM_DEFINE_CHANNEL(AH, DDRD, PORTD, PORTD4);  //Arduino pin 4
SOFTPWM_DEFINE_CHANNEL(BH, DDRB, PORTB, PORTB1);  //Arduino pin 9
SOFTPWM_DEFINE_CHANNEL(CH, DDRD, PORTD, PORTD5);  //Arduino pin 5

SOFTPWM_DEFINE_OBJECT_WITH_PWM_LEVELS(3, 100);
SOFTPWM_DEFINE_EXTERN_OBJECT_WITH_PWM_LEVELS(3, 100);

commumationStates startSeqeunce[BLDC::COMMUTATION_STATES] = {State1, State2, State3, State4, State5, State6};

BLDC::BLDC()
{
    targetSpeed = 0;
    currentSpeed = 0;
    accelerate = true;
    cycleCounter = 0;
    forward = true;
    started = false;
    currentCommunationState = State6;
    newCommunationState =  State6;
    sprintf(data,"nothing \0");

    pinMode(HALL1, INPUT);
    pinMode(HALL2, INPUT);
    pinMode(HALL3, INPUT);

    //pinMode(AH, OUTPUT);
    //pinMode(BH, OUTPUT);
    //pinMode(CH, OUTPUT);
    pinMode(AL, OUTPUT);
    pinMode(BL, OUTPUT);
    pinMode(CL, OUTPUT);
    //digitalWrite(AH, 0);
    //digitalWrite(BH, 0);
    //digitalWrite(CH, 0);
    digitalWrite(AL, 0);
    digitalWrite(BL, 0);
    digitalWrite(CL, 0);


    // begin with 60hz pwm frequency
    Palatis::SoftPWM.begin(PWN_FREQUENCY);

    // print interrupt load for diagnostic purposes
    Palatis::SoftPWM.printInterruptLoad();
}

BLDC::~BLDC()
{

}

void BLDC::ReadHalls()
{
    RawHallData[HALL1_INDEX] = digitalRead(HALL1);
    RawHallData[HALL2_INDEX] = digitalRead(HALL2);
    RawHallData[HALL3_INDEX] = digitalRead(HALL3);
    newCommunationState = (commumationStates)((RawHallData[HALL1_INDEX] << HALL1_SHIFT) | (RawHallData[HALL2_INDEX] << HALL2_SHIFT) | RawHallData[HALL3_INDEX]);
}


void BLDC::Control()
{
    setSpeed(50);
    StartMotor();
    ReadHalls();
    CalculateCommutationState();
}

void BLDC::FullCycleTest()
{
    newCommunationState = State1;
    CalculateCommutationState();
    delay(1000);

    newCommunationState = State2;
    CalculateCommutationState();
    delay(1000);

    newCommunationState = State3;
    CalculateCommutationState();
    delay(1000);

    newCommunationState = State4;
    CalculateCommutationState();
    delay(1000);

    newCommunationState = State5;
    CalculateCommutationState();
    delay(1000);

    newCommunationState = State6;
    CalculateCommutationState();
    delay(1000);

}


void BLDC::CalculateCommutationState()
{
    uint8_t AH_duty = 0;
	uint8_t AL_duty = 0;

    uint8_t BH_duty = 0;
    uint8_t BL_duty = 0;

    uint8_t CH_duty = 0;
    uint8_t CL_duty = 0;

    if(!started)
        return;

    if(newCommunationState == currentCommunationState)
    {
        return;
    }
    else
    {
        currentCommunationState = newCommunationState;
        cycleCounter++;

        if(accelerate)
        {
            if(currentSpeed < targetSpeed)
                currentSpeed++;
        }
        else
        {
            if(currentSpeed > MIN_DUTY)
                currentSpeed--;
        }

        PORTD = 0x00; //clear io to give a bit of rest time between states. To prevent shoot through.
        PORTB = 0x00;
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
                     PORTD = 0x20;
                     PORTB = 0x10;
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
                     PORTD = 0x20;
                     PORTB = 0x01;
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
                     PORTD = 0x10;
                     PORTB = 0x01;
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
                     PORTD = 0x10;
                     PORTB = 0x20;
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
                     PORTD = 0x00;
                     PORTB = 0x22;

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
                    PORTD = 0x00;
                    PORTB = 0x12;
                    //BH_duty = speed;
                    //AL_duty = 1;
                }
                break;

            default:
                PORTD = 0x00;
                PORTB = 0x00;
                break;
		
		}


    sprintf(data,"PORTB: %02x PORTD: %02x Forward: %s", PORTB, PORTD, forward ? "true" : "false");

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

int *BLDC::getRawHallData()
{
    return (int *)RawHallData;
}

void BLDC::StartMotor()
{
    ReadHalls();
    commumationStates startState = newCommunationState;
    int stateIndex = findIndex(startState);
    currentCommunationState = startSeqeunce[(stateIndex + 1) % COMMUTATION_STATES]; // make sure the two starting states are different.

    CalculateCommutationState();
    delay(100);
    ReadHalls();

    if(newCommunationState != startState)
    {
        started = true;
        return;
    }

    newCommunationState = startSeqeunce[(stateIndex - 1) % COMMUTATION_STATES];
    CalculateCommutationState();
    delay(100);
    ReadHalls();

    if(newCommunationState != startState)
    {
        started = true;
        return;
    }

    newCommunationState = startSeqeunce[(stateIndex + 1) % COMMUTATION_STATES];
    CalculateCommutationState();
    delay(100);
    ReadHalls();

    if(newCommunationState != startState)
    {
        started = true;
        return;
    }
}

int BLDC::findIndex(commumationStates state)
{
    for(int i = 0;i < COMMUTATION_STATES; i++)
    {
        if(startSeqeunce[i] == state)
            return i;
    }
}






