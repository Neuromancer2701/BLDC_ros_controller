
#include <BLDC.h>

BLDC Motor;

void setup() 
{
  Motor.init();
}

void loop() 
{
  unsigned long speed_var =  25;
  
  Motor.setSpeed(speed_var);
  Motor.Control();
}
