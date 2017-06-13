
#include <bldc.h>

BLDC Motor;

char* format = "Communation State: %d\n";

char* Halls_format = "Halls: %02x %02x %02x";
unsigned short state = 0;
char buffer[256];


void setup() 
{
  Serial.begin(57600);
  Serial.println("After Motor init");
  Motor.Forward();   ///Reverse

  delay(5000);

}

void loop() 
{

    Motor.Control();
    //Serial.println(Motor.stringData());
    delay(10);

    //Motor.FullCycleTest();


}
