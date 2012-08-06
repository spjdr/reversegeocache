#include <EEPROM.h>
#include <EEPROMAnything.h>
#include <PWMServo.h>

PWMServo lock; // SERVO

float lat = 55.676371;  // target lattitude (degrees)
float lon = 9.378122;  // target longitude (degrees)

int LSERVO = 10;

void setup()
{	

  EEPROM.write(0,0); // read attempts
  EEPROM.write(1,0);   // read solved
  EEPROM.write(2,1);     // read open
  EEPROM_writeAnything(5,lat);
  EEPROM_writeAnything(10,lon);
  
  lock.attach(LSERVO);   
  for(unsigned int pos = 175; pos>=1; pos-=1) 
  {
        Serial.println(pos,DEC); 
        lock.write(pos); delay(15);    
  }
  lock.detach();
}

void loop()
{
  
}
