#include <SoftPWM.h>
#include <SoftPWM_timer.h>

#include <bldc.h>

BLDC Motor;
unsigned char state;
bool A;
bool B;
bool C;
unsigned char halls;

void setup() 
{
  Serial.begin(57600);
  Motor.init();
  Serial.println("After Motor init");
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
  
  Motor.GetHallData(A, B, C, halls);
  Serial.print("Hall Effect A: ");
  Serial.println( A );
  Serial.print("Hall Effect B: ");
  Serial.println( B );
  Serial.print("Hall Effect C: ");
  Serial.println( C );

  Serial.print("Halls: ");
  Serial.println( halls, HEX );  
  
  delay(1000);
}
