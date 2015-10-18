#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <SD.h>
#include <SPI.h>


//Visual studio has different imports
#ifdef _MSC_VER
#include <Wire.h>
#include <EEPROM\EEPROM.h>
#include <SoftwareSerial\SoftwareSerial.h>
#else
#include <SoftwareSerial.h>
#include <EEPROM.h>
#endif
//#include <SFE_BMP180.h>

//Dummy Vars
unsigned long currGlobal;
unsigned long prevGlobal;

//Prev Loc Data
unsigned long pLat;
unsigned long pLongit;
unsigned long pCurrALT;

//Location Data
unsigned long lat = 4003890;
unsigned long longit = 7501685;
unsigned long currALT = 1;

unsigned long maxALT;

//Timing data
unsigned int startTime;
unsigned int sanityCheckTime = 0;
boolean initDone;

//SD interface variables
int cs_pin = 53;
int sd_pow_pin = 9;

#define SENTENCE_SIZE 75

char sentence[SENTENCE_SIZE];

//unsigned long minLong = 7247880; // if we want to include detkin in bdry box

unsigned long minLong = 7537170; //change before flight
unsigned long maxLong = 7619290; //Chambersburg, PA
unsigned long minLat = 3985070;  //South of Chester, PA
unsigned long maxLat = 4068060;  //Nazareth, PA

unsigned int calibrateTime = 10 * 1000; //milliseconds until performs sanityCheck

SoftwareSerial GPSSerial(10, 11);

#define runLength 64

int time = 0;

void setup() {
  // put your setup code here, to run once:
  initGPS();
  updateGPS();
  delay(10000);
  updateGPS();
  delay(1000);
  updateGPS();
  delay(1000);
  updateGPS();
  Serial.begin(9600);

  //SD stuff
  SPI.begin();
  pinMode(cs_pin, OUTPUT);
  pinMode(sd_pow_pin, OUTPUT);
  digitalWrite(sd_pow_pin, HIGH);
    if(!SD.begin(cs_pin)){
    Serial.println("Could not connect with SD");
  }
  else{
    Serial.println("Could connect with SD");
  }
  //SD end

  startTime = millis();
  initDone = true;
}

void loop() {
   updateGPS();


  if (currALT > maxALT)
  {
    maxALT = currALT;
  }
}


void initGPS() {
  maxALT = 0;
  GPSSerial.begin(9600);
}

void updateGPS()
{
  static int i = 0;

  while (GPSSerial.available())
  {
    char ch = GPSSerial.read();
    if (ch != '\n' && i < SENTENCE_SIZE)
    {
      sentence[i] = ch;
      i++;
    }
    else
    {
      sentence[i] = '\0';
      i = 0;
      readGPS();
    }
  }  
}

//RETRIEVE & PARSE DATA FROM GPS
void readGPS() {
  char field[20];
  getField(field, 0);
  if (strcmp(field, "$GPGGA") == 0)
  {
    unsigned long latDEG = 0;
    unsigned long latMIN = 0;
    lat = 0;
    unsigned long longDEG = 0;
    unsigned long longMIN = 0;
    longit = 0;
    Serial.println("---");
    Serial.print("Lat: ");
    getField(field, 2); // Latitude number
    for (int i = 0; i < 2; i++)
    {
      latDEG = 10 * latDEG + (field[i] - '0');
    }
    for (int i = 2; i < 9; i++)
    {
      if (field[i] != '.')
      {
        latMIN = 10 * latMIN + (field[i] - '0');
      }
    }
    lat = (latDEG * 100000 + latMIN / 6);
    Serial.print(field);
    getField(field, 3); // N
    Serial.print(field);
    Serial.print(" Long: ");
    getField(field, 4); // Longitude number
    for (int i = 0; i < 3; i++)
    {
      longDEG = 10 * longDEG + (field[i] - '0');
    }
    for (int i = 3; i < 10; i++)
    {
      if (field[i] != '.')
      {
        longMIN = 10 * longMIN + (field[i] - '0');
      }
    }
    longit = (longDEG * 100000 + longMIN / 6);
    Serial.print(field);
    getField(field, 5); // W
    Serial.print(field);
    Serial.print(" Alt: ");
    for(int i=0;i<10;i++)
      field[i]=0;
    getField(field, 9); // Altitude number
    currALT = 0;
    for (int i = 0; i < 8; i++)
    {
      if (field[i] <='9' && field[i]>='0')
      {
        currALT = 10 * currALT + (field[i] - '0');
      }
    }
    currALT = currALT / 10; //Marcos* -- Not sure if we should do this, loses the last decimal point, I know it's not important
    Serial.print(field); //182.2
    getField(field, 10); // Meters
    Serial.println(field); //m
    Serial.print("Lat: ");
    Serial.print(lat);
    Serial.print(" Long: ");
    Serial.print(longit);
    Serial.print(" Current Alt: ");
    Serial.println(currALT);

    //print data to SD
    GPSSD();
  }
}


//CHECK IF THE BALLOON IS IN THE BOUNDARY BOX
boolean inBdryBox() {
  if ((lat <= maxLat) && (lat >= minLat) && (longit <= maxLong) && (longit >= minLong) && (currALT <= 29000)) 
  {
    return true;
  }
  else
  {
    return false;
  }
}

//CHECK IF THE BALLOON HAS DESCENDED FROM ITS PEAK ALTITUDE
boolean isFalling() {
  if (currALT + 500 < maxALT)
  {
    return true;
  }
  else
  {
    return false;
  }
}

//RETRIEVE ROW OF DATA FROM GPS MODULE
void getField(char* buffer, int index) {
  int sentencePos = 0;
  int fieldPos = 0;
  int commaCount = 0;
  while (sentencePos < SENTENCE_SIZE)
  {
    if (sentence[sentencePos] == ',')
    {
      commaCount++;
      sentencePos++;
    }
    if (commaCount == index)
    {
      buffer[fieldPos] = sentence[sentencePos];
      fieldPos++;
    }
    sentencePos++;
  }
  buffer[fieldPos] = '\0';
}
 
 
void GPSSD()
{
  if (GPSSerial.available())
  {
     String latstr = String(lat);
     String longitstr = String(longit);
     String currALTstr = String(currALT);
     String dataString = latstr + " " + longitstr + " " + currALTstr;
     File dataFile = SD.open("gps.txt", FILE_WRITE);

     SD.open("gps.txt", FILE_WRITE);
     if (dataFile) {
          dataFile.println(dataString);
          dataFile.flush(); 
          dataFile.close();   
        }  
        // if the file isn't open, pop up an error:
        else {
          Serial.println("error opening gps.txt");
        }       
  }      
}

