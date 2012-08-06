#include <NewSoftSerial.h>
#include <TinyGPS.h>
#include <LiquidCrystal.h>
#include <PWMServo.h>
#include <math.h>
#include <EEPROM.h>
#include <EEPROMAnything.h>

// ID of the settings block
#define CONFIG_VERSION "ls1"

TinyGPS gps;
NewSoftSerial nss(6,7); // GPS COMMUNICATION
LiquidCrystal lcd(12, 11, 5, 4, 3, 2); // LCD
PWMServo lock; // SERVO

// ROUTINES

void gpsdump(TinyGPS &gps); // DUMPING GPS DATA
bool feedgps(); // FEED THE GPS
void printfloat(double f, int digits = 2); // PRINT A FLOAT
double earthdistance(float lat1, float lon1, float lat2, float lon2); // calculate distance between two points on the surface of Earth
bool attemptsolution(float lat1, float lon1, float lat2, float lon2); // returns true if attempt succeeds
void turnoff();
void loadpuzzle();
void program();

// STATE VARS LOADED FROM EEPROM

int attempts; // number of attempts read from EEPROM (1..50)
int solved;   // is the puzzle solved? (0 = unsolved, 1 = solved)
int open;     // is the box open? (0 = closed, 1 = open)

// SETUP

int OFF = 8; // OFF pin for the power switch
int LSERVO = 10; // SERVO pin for the lock

// GPS LOCATION SOLVING RIDDLE

float lat = 0.0;  // target lattitude (degrees)
float lon = 0.0;  // target longitude (degrees)

void setup()
{	
  pinMode(OFF, OUTPUT);      // sets the digital pin as output
  
  Serial.begin(115200);
  nss.begin(4800);
  lcd.begin(16,2);           // initialize LCD 

  Serial.print("Welcome to the Reverse Geocache v. 0.01"); 
  Serial.println();
  Serial.print("Sizeof(gpsobject) = "); 
  Serial.println(sizeof(TinyGPS));
  Serial.println();

  // LOAD EEPROM DATA
  loadpuzzle();
  
  attempts = EEPROM.read(0); // read attempts
  solved = EEPROM.read(1);   // read solved
  open = EEPROM.read(2);     // read open

  Serial.print("Attempts: ");
  Serial.println(attempts);
  Serial.print("Solved: ");
  Serial.println(solved);
  Serial.print("Open: ");
  Serial.println(open);

  // PRINT WELCOME
  lcd.print("Welcome");      // Welcome message
  Serial.println("Welcome");
  delay(4000);

  // TEST REVERSE GEO CACHE STATE
  // OPEN BUT UNSOLVED
  if (open==1 && solved==0)
  {
    Serial.println("Box is open but unsolved");

    lcd.clear();
    lcd.print("Puzzle unsolved.");
    lcd.setCursor(0,1);
    lcd.print("Locking box.");

    puzzlelock(0);

    EEPROM.write(0,1); attempts = 1; // reset counter 
    EEPROM.write(2,0); open = 0; // close box

    delay(4000);

    turnoff();
  }

  // IF ALL ATTEMPTS HAS BEEN USED
  if (attempts > 50 && solved==0 && open==0)
  { 
    Serial.print("Puzzle unsolved");
    Serial.println();

    lcd.clear();
    lcd.print("50 attempts used");
    delay(5000);
    lcd.clear();

    lcd.print("Box sealed");
    lcd.setCursor(0,1);
    lcd.print("forever");

    delay(4000);

    turnoff();
  }
  
  // OPEN AND SOLVED
  if (open==1 && solved==1)
  {
    Serial.print("Box is open and solved.");
    Serial.println();

    lcd.clear();
    lcd.print("Puzzle solved.");
    delay(3000);
  }
  
  // CLOSED AND UNSOLVED
  if (open==0 && solved==0 && attempts < 50)
  {  
    Serial.println("Box is closed and unsolved.");
    
    // "normal" procedure
    lcd.clear();
    lcd.print("Attempt ");
    lcd.print(attempts);
    lcd.print(" of 50");
  }
}

void loop()
{
  bool newdata = false;
  unsigned long start = millis();

  // Every 5 seconds we print an update
  while (millis() - start < 5000)
  {
    if (feedgps())
      newdata = true;
  }

  // Abort if GPS fix takes longer than a minute
  if (millis() - start > 60000)
  {
    lcd.clear();
    lcd.print("No signal found.");
    Serial.println("No signal found.");
    
    delay(5000);
    lcd.clear();

    turnoff();
  }

  // If GPS fixed, proceed as normal
  if (newdata)
  {
    //gpsdump(gps);	

    lcd.clear();

    float flat, flon;
    double distance;
    unsigned long age;

    gps.f_get_position(&flat, &flon, &age);

    // Test if puzzle is solved
    if ( attemptsolution(lat,lon,flat,flon) )
    {
      lcd.print("Congratulations!");
      Serial.println("Congratulations!");
      delay(5000);
      
      lcd.clear();
      lcd.print("You solved the");
      lcd.setCursor(0,1);
      lcd.print("puzzle.");
      Serial.println("You solved the puzzle.");

      puzzlelock(1);
      
      EEPROM.write(1,1); solved = 1,
      EEPROM.write(2,1); open = 1;

      delay(5000);

      turnoff();
    }
    else   
    {
      distance = earthdistance(lat,lon,flat,flon);

      printfloat(distance);
      Serial.println(" km to target.");
      
      // Display hint
      if (distance > 1)
      {
        lcd.print((int) round(distance));
        lcd.print(" km to target");  
      }
      else
      {
        lcd.print((int) round(distance*1000));
        lcd.print(" m to target"); 
      }

      EEPROM.write(0,attempts+1); attempts = attempts + 1; // increase attempts counter

      delay(5000);

      turnoff();
    }
  }
}

// Test wether or not you have solved th puzzle.
bool attemptsolution(float lat1, float lon1, float lat2, float lon2)
{
  float a1 = lat1*0.0174532925;
  float b1 = lon1*0.0174532925;

  float a2 = lat2*0.0174532925;
  float b2 = lon2*0.0174532925;

  return (fabs(a1-a2)+fabs(b1-b2))*6378 < 0.1;
}

// Calculates distance in kilometers
double earthdistance(float lat1, float lon1, float lat2, float lon2)
{
  float a1 = lat1*0.0174532925;
  float b1 = lon1*0.0174532925;

  float a2 = lat2*0.0174532925;
  float b2 = lon2*0.0174532925;

  float dist = acos(cos(a1)*cos(b1)*cos(a2)*cos(b2) + cos(a1)*sin(b1)*cos(a2)*sin(b2) + sin(a1)*sin(a2)) * 6378;
  
  if (dist > 5)
  {
     return dist; 
  }
  else
  {
     return sqrt(fabs(a1-a2)*fabs(a1-a2)+fabs(b1-b2)*fabs(b1-b2))*6378;
  }
}

void printfloat(double number, int digits)
{
  // Handle negative numbers
  if (number < 0.0)
  {
    Serial.print('-');
    number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;

  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long) number;
  double remainder = number - (double) int_part;
  Serial.print(int_part);

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    Serial.print("."); 

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    Serial.print(toPrint);
    remainder -= toPrint; 
  } 
}

void gpsdump(TinyGPS &gps)
{
  long lat, lon;
  float flat, flon;
  unsigned long age, date, time, chars;
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned short sentences, failed;

  gps.get_position(&lat, &lon, &age);
  Serial.print("Lat/Long(10^-5 deg): "); 
  Serial.print(lat); 
  Serial.print(", "); 
  Serial.print(lon); 
  Serial.print(" Fix age: "); 
  Serial.print(age); 
  Serial.println("ms.");

  feedgps(); // If we don't feed the gps during this long routine, we may drop characters and get checksum errors

  gps.f_get_position(&flat, &flon, &age);
  Serial.print("Lat/Long(float): "); 
  printfloat(flat, 5); 
  Serial.print(", "); 
  printfloat(flon, 5);
  Serial.print(" Fix age: "); 
  Serial.print(age); 
  Serial.println("ms.");

  feedgps();

  gps.get_datetime(&date, &time, &age);
  Serial.print("Date(ddmmyy): "); 
  Serial.print(date); 
  Serial.print(" Time(hhmmsscc): "); 
  Serial.print(time);
  Serial.print(" Fix age: "); 
  Serial.print(age); 
  Serial.println("ms.");

  feedgps();

  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  Serial.print("Date: "); 
  Serial.print(static_cast<int>(month)); 
  Serial.print("/"); 
  Serial.print(static_cast<int>(day)); 
  Serial.print("/"); 
  Serial.print(year);
  Serial.print("  Time: "); 
  Serial.print(static_cast<int>(hour)); 
  Serial.print(":"); 
  Serial.print(static_cast<int>(minute)); 
  Serial.print(":"); 
  Serial.print(static_cast<int>(second)); 
  Serial.print("."); 
  Serial.print(static_cast<int>(hundredths));
  Serial.print("  Fix age: ");  
  Serial.print(age); 
  Serial.println("ms.");

  feedgps();

  Serial.print("Alt(cm): "); 
  Serial.print(gps.altitude()); 
  Serial.print(" Course(10^-2 deg): "); 
  Serial.print(gps.course()); 
  Serial.print(" Speed(10^-2 knots): "); 
  Serial.println(gps.speed());
  Serial.print("Alt(float): "); 
  printfloat(gps.f_altitude()); 
  Serial.print(" Course(float): "); 
  printfloat(gps.f_course()); 
  Serial.println();
  Serial.print("Speed(knots): "); 
  printfloat(gps.f_speed_knots()); 
  Serial.print(" (mph): ");  
  printfloat(gps.f_speed_mph());
  Serial.print(" (mps): "); 
  printfloat(gps.f_speed_mps()); 
  Serial.print(" (kmph): "); 
  printfloat(gps.f_speed_kmph()); 
  Serial.println();

  feedgps();

  gps.stats(&chars, &sentences, &failed);
  Serial.print("Stats: characters: "); 
  Serial.print(chars); 
  Serial.print(" sentences: "); 
  Serial.print(sentences); 
  Serial.print(" failed checksum: "); 
  Serial.println(failed);
}

bool feedgps()
{
  while (nss.available())
  {
    if (gps.encode(nss.read()))
      return true;
  }
  return false;
}

// TURNOFF THE BOARD

void turnoff()
{
  unsigned long start = millis(); 

  lcd.clear();
  lcd.print("Powering off");
  Serial.println("Powering off");

  // Every 5 seconds we print an update
  while (millis() - start < 5000)
  {
      if (Serial.available())
      {
        char input = Serial.read(); 
        if (input == 'P')
        {
           program(); 
        }
      }
  }

  lcd.clear();
  digitalWrite(OFF,HIGH);

  Serial.println("Powered OFF");
}

void program()
{
  Serial.println("Entering programming mode.");
  while(true)
  {
    if (Serial.available()){
      char input = Serial.read(); 
    
      if (input == 'R')
      {
        EEPROM.write(0,0);
        EEPROM.write(1,0);
        
        puzzlelock(0);
        
        Serial.println("Reverse GeoCache reset.");
      }
      else if (input == 'O')
      {
        puzzlelock(1);
        
        Serial.println("Reverse GeoCache opened.");
      } 
      else if (input == 'C')
      {
        puzzlelock(0); 
       
        Serial.println("Reverse GeoCache closed.");
      }
    }
  }
}

void puzzlelock(int degree)
{ 
  lcd.noDisplay();
  nss.end();
  delay(500);

  lock.attach(LSERVO);   
  if (degree < 1)
  {
     for(unsigned int pos = 175; pos>=1; pos-=1) 
     {
        Serial.println(pos,DEC); 
        lock.write(pos); delay(15);    
     }
     EEPROM.write(2,0); open = 0;
  }
  else if (degree > 0)
  {
     for(unsigned int pos = 5; pos<=175; pos+=1)
     {
       Serial.println(pos,DEC);
       lock.write(pos); delay(15);
     }
     EEPROM.write(2,1); open = 1;
  } 
  lock.detach();
  nss.begin(4800);
  lcd.display(); 
}

void loadpuzzle() {
  EEPROM_readAnything(5, lat);
  EEPROM_readAnything(10, lon);
}


