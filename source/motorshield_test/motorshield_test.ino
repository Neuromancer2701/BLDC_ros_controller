#include <SoftPWM.h>
#include <SoftPWM_timer.h>

#include <bldc.h>

BLDC Motor;

char* format = "Communation State: %d\n";

char* Halls_format = "Halls: %02x %02x %02x\n";
unsigned short state = 0;

void setup() 
{
  Serial.begin(57600);
  Serial.println("After Motor init");

}

void loop() 
{
    char buffer[256];
    Motor.Forward();
    Motor.Control();


    unsigned short* halls = Motor.getRawHallData();
    snprintf(buffer,sizeof(buffer),Halls_format, halls[2], halls[1], halls[0]);
    Serial.println(buffer);

    /*
    Serial.print("State: ");
    Serial.println(state);
    Motor.SetCommutationState(state++);
    */
    delay(100);
}
