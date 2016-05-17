#include <SoftPWM.h>
#include <SoftPWM_timer.h>

#include <bldc.h>

BLDC Motor;

char* format = "Hall State: %d Communation State: %d\n";
unsigned short state = 0;

void setup() 
{
  Serial.begin(57600);
  Motor.init();
  Serial.println("After Motor init");

}

void loop() 
{
    /*char buffer[256];
    Motor.Control();

    snprintf(buffer,sizeof(buffer),format, Motor.getHallIndex(), Motor.getCommunationState());
    Serial.print(buffer);
    */
    
    Serial.print("State: ");
    Serial.println(state);
    Motor.SetCommutationState(state++);

    delay(5000);
}
