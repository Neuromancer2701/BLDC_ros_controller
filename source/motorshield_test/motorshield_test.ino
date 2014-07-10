
#include <bldc.h>

BLDC Motor;
unsigned char state;

void setup() 
{
  Serial.begin(57600);
  Motor.init();
  Serial.println("After Motor init");
  state = 0;
}

void loop() 
{
  Serial.print("State: ");
  Serial.println(state);
  Motor.SetCommutationState(state++);
  
  if(state >= 6)
  {
    state = 0;
  }
  
  delay(100);
}
