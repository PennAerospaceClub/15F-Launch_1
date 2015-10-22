/*
 PAC Flight Code
 @version: 10/22/15
 @author Antonio Menarde
 
Section 1: Declarations
  1.1 Sanity Check
  1.2 LED Declarations
  1.3 Temperature Analog-in Declarations
  1.4 SD Declarations
  1.5 IMU Declarations
  1.6 GPS Declarations
  1.7 inDryBox Utility
  1.8 Nichrome Declarations
  1.9 Timing data

Section 2: Setup
  2.1 Initializations
  2.2 GPS Setup
  2.3 Nichrome Setup
  2.4 IMU Setup
  2.5 LED Setup

Section 3: Loop
  3.1 GPS Section
  3.2 IMU Section
  3.3 Sanity and Nichrome

Section 4: Functions
  4.1 IMU
  4.2 GPS
    4.2.1 Interfacing GPS
    4.2.2 GPS Boundary Box
    4.2.3 GPS Falling
  4.3 Nichrome
  4.4 Temperature Sensors: Current 
  4.5 Sanity
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <Time.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <SD.h>
#include <SPI.h>
#include <SoftwareSerial.h>

//Section 1: Declarations

//1.1 Sanity Check
boolean sane = false;

//1.2 LED Declarations
const int LED_GREEN = 36;
const int LED_YELLOW = 38;
const int LED_RED = 40;

//1.3 Temperature Analog-in Declarations
const int TEMP1_PIN = A0;
const int TEMP2_PIN = A1;

//1.4 SD Declarations
int cs_pin = 53;
int sd_pow_pin = 9;

//1.5 IMU Declarations
/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

//1.6 GPS Declarations
SoftwareSerial GPSSerial(10, 11);
  //Boundary Box UPDATE DAY OF LAUNCH WITH MOST RECENT SIMULATION
unsigned long minLong = 0;
unsigned long maxLong = 0;
unsigned long minLat = 0;
unsigned long maxLat = 0;
unsigned long maxAlt = 50000; //measures in meters
  //Location Data
unsigned long lat = -1; 
unsigned long longit = -1;
unsigned long currAlt = -1; //altitude is in tens of meters CHECK

//1.7 inDryBox Utility
#define SENTENCE_SIZE 75
char sentence[SENTENCE_SIZE];

//1.8 Nichrome Declarations
const int NICHROME_GATE_PIN = 32;
const int NICHROME_EXPERIMENT_PIN = 34;
boolean nichromeStarted = false;
unsigned long nichromeEndTime = 0xFFFFFFFFL;
boolean nichromeFinished = false;

//1.9 Timing data
unsigned int startTime;
unsigned int sanityCheckTime = 0;
boolean initDone;
unsigned int calibrateTime = 10000; //milliseconds until performs sanityCheck

//Section 2: Setup
void setup() {
 //2.1 Initializations
  Serial.begin(9600); //115200
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
  
//2.2 GPS Setup
  initGPS();
  updateGPS();
  delay(10000);
  updateGPS();
  delay(1000);
  updateGPS();
  delay(1000);
  updateGPS();

//2.3 Nichrome Setup
  initNichrome();

//2.4 IMU Setup
  initIMU();

//2.5 LED Setup
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  
  Serial.println("SETUP DONE");

}

//Section 3: Loop
void loop() {
//3.1 GPS Section
  updateGPS();
  updateMaxAlt();
  
//3.2 IMU Section
  runIMU();

//3.3 Sanity and Nichrome
  if(!sane){
    sane = sanityCheck();
  } 
  else{
  //Hey should we burn the nichrome or nah?
  nichromeCheck();
  updateNichrome();
  }

}

//Section 4: Functions
//4.1 IMU
/* Initialise the sensors */
void initIMU(){
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
    
  }
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
}

/* Get a new sensor event */
void runIMU()
{
  sensors_event_t event;

  /* Display time */
  Serial.print(hour()); Serial.print(":"); Serial.print(minute()); Serial.print(":"); Serial.print(second()); Serial.print(", ");
    
  /* Display the results (acceleration is measured in m/s^2) */
  accel.getEvent(&event);
  Serial.print(event.acceleration.x); Serial.print(", ");
  Serial.print(event.acceleration.y); Serial.print(", ");
  Serial.print(event.acceleration.z); Serial.print(", ");
  
  
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  mag.getEvent(&event);
  Serial.print(event.magnetic.x); Serial.print(", ");
  Serial.print(event.magnetic.y); Serial.print(", ");
  Serial.print(event.magnetic.z); Serial.print(", ");

  /* Display the results (gyrocope values in rad/s) */
  gyro.getEvent(&event);
  Serial.print(event.gyro.x); Serial.print(", ");
  Serial.print(event.gyro.y); Serial.print(", ");
  Serial.print(event.gyro.z); Serial.print(", ");

  /* Display the pressure sensor results (barometric pressure is measure in hPa) */
  bmp.getEvent(&event);
  if (event.pressure)
  {
    /* Display atmospheric pressure in hPa */
    Serial.print(event.pressure);
    Serial.print(", ");
    /* Display ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
    Serial.print(temperature);
    Serial.print(F(", "));
    /* Then convert the atmospheric pressure, SLP and temp to altitude    */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                        event.pressure,
                                        temperature)); 
  }

  Serial.println(F(""));
  delay(100);
}
  
void displaySensorDetails(void)
{
  sensor_t sensor;
  
  accel.getSensor(&sensor);
  Serial.println(F("----------- ACCELEROMETER ----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" m/s^2"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  gyro.getSensor(&sensor);
  Serial.println(F("------------- GYROSCOPE -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" rad/s"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  mag.getSensor(&sensor);
  Serial.println(F("----------- MAGNETOMETER -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" uT"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  bmp.getSensor(&sensor);
  Serial.println(F("-------- PRESSURE/ALTITUDE ---------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" hPa"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" hPa"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" hPa"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  
  delay(500);
}

//4.2 GPS
//4.2.1 Interfacing GPS
void initGPS() {
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

void updateMaxAlt()
{
  if (currAlt > maxAlt){
    maxAlt = currAlt;
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
    currAlt = 0;
    for (int i = 0; i < 8; i++)
    {
      if (field[i] <='9' && field[i]>='0')
      {
        currAlt = 10 * currAlt + (field[i] - '0');
      }
    }
    currAlt = currAlt / 10; //Marcos* -- Not sure if we should do this, loses the last decimal point, I know it's not important
    Serial.print(field); //182.2
    getField(field, 10); // Meters
    Serial.println(field); //m
    Serial.print("Lat: ");
    Serial.print(lat);
    Serial.print(" Long: ");
    Serial.print(longit);
    Serial.print(" Current Alt: ");
    Serial.println(currAlt);

    //print data to SD
    GPSSD();
  }
  }
  
//PRINT GPS DATA TO SD
void GPSSD()
{
  if (GPSSerial.available())
  {
     String latstr = String(lat);
     String longitstr = String(longit);
     String currALTstr = String(currAlt);
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

//4.2.2 GPS Boundary Box
//CHECK IF THE BALLOON IS IN THE BOUNDARY BOX
boolean inBdryBox() {
  if ((lat <= maxLat) && (lat >= minLat) && (longit <= maxLong) && (longit >= minLong) && (currAlt <= 29000)) 
  {
    return true;
  }
  else
  {
    return false;
  }
}

//RETRIEVE ROW OF DATA FROM GPS MODULE
void getField(char* buffer, int index)
{
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

//4.2.3 GPS Falling
//CHECK IF THE BALLOON HAS DESCENDED FROM ITS PEAK ALTITUDE
boolean isFalling() {
  if (currAlt + 500 < maxAlt)
  {
    return true;
  }
  else
  {
    return false;
  }
}

//4.3 Nichrome
void nichromeCheck()
{
  if (!inBdryBox())
  {
    Serial.println("Outside Bdry Box!");
  }
  if (isFalling())
  {
    Serial.println("Falling!");
  }
  if (!inBdryBox() && !isFalling())
  {
    startNichrome();
  }
  updateNichrome();
}


void initNichrome()
{
  pinMode(NICHROME_GATE_PIN, OUTPUT);
  digitalWrite(NICHROME_GATE_PIN, LOW);
  TCCR2B &= B11111001;//increase PWM frequency
}

void updateNichrome()
{
  if (nichromeStarted && !nichromeFinished && nichromeEndTime < millis()) {
    Serial.println("NICHROME DEACTIVATING");
    digitalWrite(NICHROME_GATE_PIN, LOW);
    nichromeFinished = true;
  }
}

void startNichrome()
{
  if (!nichromeStarted) {
    Serial.println("NICHROME ACTIVATING");
    nichromeStarted = true;
    digitalWrite(NICHROME_GATE_PIN, HIGH);// 128//This duty cycle is an estimate. You might need to increase it. Test.
    nichromeEndTime = millis() + 5000;//Again, 5000 ms is an estimate... <- CHANGE!!!
  }
}

//4.4 Temperature Sensors: Current 
void readTempVoltage()
{
  float tempVoltage1 = analogRead(TEMP1_PIN);
  float tempVoltage2 = analogRead(TEMP2_PIN);

  File dataFile = SD.open("temperature.txt", FILE_WRITE);

  SD.open("temperature.txt", FILE_WRITE);
  if (dataFile) {
     dataFile.println((String)tempVoltage1 + "," + (String)tempVoltage2);
     dataFile.flush(); 
     dataFile.close();   
 }  
    // if the file isn't open, pop up an error:
   else {
     Serial.println("error opening temperature.txt");
 }       
}

//4.5 Sanity
boolean sanityCheck()
{
  Serial.println("Doing Sanitycheck");
  boolean bdryBool = true;
  boolean fallingBool = true;
  boolean gpsBool = true;
  
  if (!inBdryBox()){
    bdryBool =  false;
  }
  if (isFalling()){
    fallingBool =  false;
  }
  if ((currAlt > maxAlt) || (currAlt == -1) || (lat == -1) || (longit == -1)){
    gpsBool =  false;
  }

  sanitySerial_SD_LED(bdryBool, fallingBool, gpsBool);

  if(bdryBool && fallingBool && gpsBool){
    return true;
  }  
  else{
    return false;
  }  
}

//RED LIGHT: GPS NOT UPDATING
//YELLOW LIGHT: THINKS IT'S FALLING
//GREEN LIGHT: THINKS IT'S OUTSIDE BOUNDARY BOX
void sanitySerial_SD_LED(boolean bdryBool, boolean fallingBool, boolean gpsBool)
{
     boolean myBdryBool = bdryBool;
     boolean myFallingBool = fallingBool;
     boolean myGpsBool = gpsBool;

     File dataFile = SD.open("gps.txt", FILE_WRITE);

     SD.open("sanity.txt", FILE_WRITE);
     if (dataFile) {
          if(!myBdryBool){
            Serial.println("Sanity: Outside of Boundary Box");
            dataFile.println("Sanity: Outside of Boundary Box");
            digitalWrite(LED_GREEN, HIGH);
          }  
          else{
            Serial.println("Sanity: Inside of Boundary Box");
            dataFile.println("Sanity: Inside of Boundary Box");
            digitalWrite(LED_GREEN, LOW);
          }  
          if(!myFallingBool){
            Serial.println("Sanity: Falling (bad)");
            dataFile.println("Sanity: Falling (bad)");
            digitalWrite(LED_YELLOW, HIGH);
          }  
          else{
            Serial.println("Sanity: Not falling (good)");
            dataFile.println("Sanity: Not falling (good)");
            digitalWrite(LED_YELLOW, LOW);
          }   
          if(!myGpsBool){
            Serial.println("Sanity: GPS not updating");
            dataFile.println("Sanity: GPS not updating");
            digitalWrite(LED_RED, HIGH);
          }  
          else{
            Serial.println("Sanity: GPS updating");
            dataFile.println("Sanity: GPS updating");
            digitalWrite(LED_RED, LOW);
          }       
          dataFile.flush(); 
          dataFile.close();   
        }  
        else {
          Serial.println("error opening sanity.txt");
        }       
}

