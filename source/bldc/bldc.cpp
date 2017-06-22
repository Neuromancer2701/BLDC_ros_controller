#include "Arduino.h"
#include "bldc.h"
#include "SoftPWM.h"


SOFTPWM_DEFINE_CHANNEL(AH_INDEX, DDRD, PORTD, PORTD4);  //Arduino pin 4
SOFTPWM_DEFINE_CHANNEL(BH_INDEX, DDRB, PORTB, PORTB1);  //Arduino pin 9
SOFTPWM_DEFINE_CHANNEL(CH_INDEX, DDRD, PORTD, PORTD5);  //Arduino pin 5

SOFTPWM_DEFINE_OBJECT_WITH_PWM_LEVELS(3, 100);
SOFTPWM_DEFINE_EXTERN_OBJECT_WITH_PWM_LEVELS(3, 100);

commumationStates startSeqeunce[BLDC::COMMUTATION_STATES] = {State1, State2, State3, State4, State5, State6};

BLDC::BLDC():
        pub_velocity("velocity",&velocity_msg),
        sub_start("start", &BLDC::StartMotor, this ),
        sub_pwm("pwm", &BLDC::ros_setSpeed, this )
{
    targetSpeed = 0;
    currentSpeed = 0;
    accelerate = true;
    cycleCounter = 0;
    forward = true;
    started = false;
    currentCommunationState = State6;
    newCommunationState =  State6;
    velocity = 0.0;
    previousTime = 0;
    currentTime = 0;
    rosUpdateTime = millis() + ROS_UPDATE_PERIOD;


    pinMode(HALL1, INPUT);
    pinMode(HALL2, INPUT);
    pinMode(HALL3, INPUT);

    pinMode(AL, OUTPUT);
    pinMode(BL, OUTPUT);
    pinMode(CL, OUTPUT);
    digitalWrite(AL, 0);
    digitalWrite(BL, 0);
    digitalWrite(CL, 0);

    nodeHandle.initNode();
    nodeHandle.subscribe(sub_start);
    nodeHandle.subscribe(sub_pwm);

}

BLDC::~BLDC()
{

}

void BLDC::initPWM()
{
    // begin with 500 pwm frequency
    Palatis::SoftPWM.begin(500);

    // print interrupt load for diagnostic purposes
    Palatis::SoftPWM.printInterruptLoad();

    Palatis::SoftPWM.allOff();
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

    ReadHalls();
    if(started)
    {
        CalculateCommutationState();
    }

    ProcessRosMessages();
}

void BLDC::FullCycleTest()
{
    setSpeed(50);
    unsigned long Delay = 1000;

    newCommunationState = State1;
    CalculateCommutationState();
    delay(Delay);

    newCommunationState = State2;
    CalculateCommutationState();
    delay(Delay);

    newCommunationState = State3;
    CalculateCommutationState();
    delay(Delay);


    newCommunationState = State4;
    CalculateCommutationState();
    delay(Delay);

    newCommunationState = State5;
    CalculateCommutationState();
    delay(Delay);

    newCommunationState = State6;
    CalculateCommutationState();
    delay(Delay);

}


void BLDC::CalculateCommutationState()
{

    int highSideIndex = 0;
    unsigned short lowSide = 0;

    if(newCommunationState == currentCommunationState)
    {
        return;
    }
    else
    {
        currentCommunationState = newCommunationState;
        cycleCounter++;

        previousTime = currentTime;
        currentTime = millis();
        velocity = TWO_PI * (RADIUS/(double)1000) * ((1/(double)CYCLES_PER_REV)/((currentTime - previousTime)/(double)1000));

        Serial.print("velocity: ");
        Serial.println(velocity);

#if 0
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

#endif
        PORTB = 0x00; //clear io to give a bit of rest time between states. To prevent shoot through.
        Palatis::SoftPWM.allOff();
    }

		switch(currentCommunationState)
		{

			case State1:
                 if(forward)
                 {
                     highSideIndex = AH_INDEX;
                     lowSide = CL_HIGH_PORTB;
                 }
                 else
                 {
                     highSideIndex = CH_INDEX;
                     lowSide = AL_HIGH_PORTB;
                 }
				 break;

			case State2:
                 if(forward)
                 {
                     highSideIndex = BH_INDEX;
                     lowSide = CL_HIGH_PORTB;
                 }
                 else
                 {
                     highSideIndex = CH_INDEX;
                     lowSide = BL_HIGH_PORTB;
                 }
                 break;

			case State3:
                 if(forward)
                 {
                     highSideIndex = BH_INDEX;
                     lowSide = AL_HIGH_PORTB;
                 }
                 else
                 {
                     highSideIndex = AH_INDEX;
                     lowSide = BL_HIGH_PORTB;
                 }
                 break;
			case State4:
                 if(forward)
                 {
                     highSideIndex = CH_INDEX;
                     lowSide = AL_HIGH_PORTB;
                 }
                 else
                 {
                     highSideIndex = AH_INDEX;
                     lowSide = CL_HIGH_PORTB;
                 }
                 break;
			case State5:
                 if(forward)
                 {
                     highSideIndex = CH_INDEX;
                     lowSide = BL_HIGH_PORTB;;
                 }
                 else
                 {
                     highSideIndex = BH_INDEX;
                     lowSide = CL_HIGH_PORTB;
                 }
                 break;
			case State6:
                if(forward)
                {
                    highSideIndex = AH_INDEX;
                    lowSide = BL_HIGH_PORTB;
                }
                else
                {
                    highSideIndex = BH_INDEX;
                    lowSide = AL_HIGH_PORTB;
                }
                break;

            default:
                PORTB = 0x00;
                Palatis::SoftPWM.allOff();
                break;
		
		}

    //sprintf(data,"state: %d cycle count: %d velocity: %05d", currentCommunationState, cycleCounter,(int)(velocity * 1000));
    //sprintf(data,"hideIndex: %d PORTB: %02x Forward: %s", highSideIndex, lowSide, forward ? "true" : "false");

    //Serial.println(data);

    Palatis::SoftPWM.set(highSideIndex, targetSpeed);
    PORTB = lowSide;

}

int *BLDC::getRawHallData()
{
    return (int *)RawHallData;
}

void BLDC::ros_setSpeed(const std_msgs::UInt8 &cmd_msg)
{
    setSpeed(cmd_msg.data);
}

void BLDC::StartMotor(const std_msgs::Bool& cmd_msg)
{

    if(cmd_msg.data)
    {
        started = true;
        ReadHalls();
        currentCommunationState = startSeqeunce[(findIndex(newCommunationState) + 1) % COMMUTATION_STATES]; // make sure the two starting states are different.
    }
    else
    {
        currentCommunationState = newCommunationState = State1;
        started = false;
    }


#if 0

    CalculateCommutationState();



    delay(100);
    ReadHalls();
    if(newCommunationState != startState)
    {
        Serial.println("SUCCESS");
        started = true;
        return;
    }


    newCommunationState = startSeqeunce[(currentStateIndex - 1) % COMMUTATION_STATES];
    Serial.print("Try  before current state:");
    Serial.println(newCommunationState);
    CalculateCommutationState();
    delay(100);
    ReadHalls();
    if(newCommunationState != startState)
    {
        Serial.println("SUCCESS");
        started = true;
        return;
    }

    newCommunationState = startSeqeunce[(currentStateIndex + 1) % COMMUTATION_STATES];
    Serial.print("Try after current state:");
    Serial.println(newCommunationState);
    CalculateCommutationState();
    delay(100);
    ReadHalls();
    if(newCommunationState != startState)
    {
        Serial.println("SUCCESS");
        started = true;
        return;
    }
#endif
}

int BLDC::findIndex(commumationStates state)
{
    for(int i = 0;i < COMMUTATION_STATES; i++)
    {
        if(startSeqeunce[i] == state)
            return i;
    }
}

void BLDC::ProcessRosMessages()
{

    if(rosUpdateTime > millis())
    {
        velocity_msg.data = (float)velocity;
        pub_velocity.publish(&velocity_msg);

        rosUpdateTime = millis() + ROS_UPDATE_PERIOD;
    }

    nodeHandle.spinOnce();
}




