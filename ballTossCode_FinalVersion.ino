//------------COMMUNICATION PROTOCOLS----------//

// Load the Wire library, which will give us
// commands to interface with I2C
// communication protocol:
#include <Wire.h>

// Load the SPI library, which will give us
// commands to interface with the SPI
// communication protocol:
#include <SPI.h>
#include <math.h>
//---------------------------------------------//

//----------------SENSOR LIBRAIES--------------//

// Load the BME280 library, which will give us
// commands to interface to the BME280 sensor:
#include "SparkFunBME280.h"
BME280 mySensor;

// Load the SD library, which will give us
// commands to interface to the SD card:
#include <SD.h>
//---------------------------------------------//


//-------------CONSTANT VARIABLES--------------//

const String camperDataFile = "TEST.csv"; // name must not exceed 8 characters, not including the ".csv"
const float P_0 = 101397;   // averaged pressure in grassy area at Rainier Vista UW on 06/27/2019 (1 hPa = 100 Pa)
const float T_0 = 17 + 273; // initial temperature needs to be googled (?°C + 273 to convert to kelvin)
const float R   = 286.724;  // normalized gas constant for earths atmosphere (universal gas constant 8.315 J/(mol*K))
const float g   = 9.807;    // acceleration due to gravity (m/s^2)
//---------------------------------------------//


//--------------CONSTANT PINS------------------//

const int pinCS = 10; // Pin 10 on Arduino Uno
//---------------------------------------------//

// Initialize pressure, temperature, and height-above-sea-level
float pressure    = 0;
float temperature = 0;
float height      = 0;
float answer      = 0;
File sensorData;
unsigned long time;

//==========================================================================================//
void setup() {
  Serial.begin(9600);
  
  //pinMode(pinCS, OUTPUT); // library takes care of SS
  
  // SD Card Initialization
  if (SD.begin())
  {
    Serial.println("SD card is ready to use.");
  } else
  {
    Serial.println("SD card initialization failed");
    return;
  }
  
  // Check to see if the file exists:
  if (SD.exists(camperDataFile)) {
    // delete the file:
    SD.remove(camperDataFile);
  }
  // If the file doesn't exist:
  if (SD.exists(camperDataFile)==0) {
    // Create/Open file // File sensorData;
    sensorData = SD.open(camperDataFile, FILE_WRITE); 
    sensorData.close(); // close the file
  }
  
  Serial.println("Reading basic values from BME280");
  Serial.println();
  Wire.begin();

  if (mySensor.beginI2C() == false) //Begin communication over I2C
  {
    Serial.println("The sensor did not respond. Please check wiring.");
    while(1); //Freeze
  }
}

void loop() {
   // Create/Open file  
   sensorData = SD.open(camperDataFile, FILE_WRITE);

  // if the file opened okay, write to it:
  if (sensorData) {
    //Serial.println("Writing to file...");

      //---------------TIME--------------------------
      //Serial.print(" Time: ");
      time = millis();
      //Serial.println(time);

      //--------------Raw Pressure-------------------
      //Serial.print(" Pressure: ");
      pressure = mySensor.readFloatPressure(); // Pressure is unsigned 32 bit int in Q24.8 format (24 int bits & 8 fractional bits). Output of “24674867” represents 24674867/256 = 96386.2 Pa
      Serial.println(pressure, 0);
      
      //------------Raw Temperature-------------------
      //Serial.print(" Temperature: ");
      temperature = (mySensor.readTempF() - 32) / 1.8; // convert to °C
      //Serial.println(temperature, 0);
      
      //--------------Data String--------------------
      //Serial.print(" Height: ");
      height = getHeight(pressure); 
      Serial.println(height, 0);
      //Serial.print(" ");
      answer = mySensor.readFloatAltitudeMeters();
      //Serial.println(answer, 0);

    // Write to file
    sensorData.print(time);
    sensorData.print(",");
    sensorData.print((String)height);
    sensorData.print(",");
    sensorData.println((String)answer);
    sensorData.close(); // close the file
    //Serial.println("Done.");
  }
  // if the file didn't open, print an error:
  else {
    Serial.println("error opening text file: test.txt");
  }

  //Serial.println();
  delay(100);
}

/* 
The barometric formula, sometimes called the exponential atmosphere or isothermal atmosphere,
is a formula used to model how the pressure (or density) of the air changes with altitude.
The hypsometric formula, also known as the thickness equation, correlates the ratio of atmospheric
pressures to the equivalent thickness of an atmospheric layer under the assumptions of constant 
temperature and gravity. The troposphere is the first 6-10 km. Shotout to Dr.Qiangfu for the help.
h = R*T_0/g[-ln(pressure/P_0)] where R = 8.315/(29*10^-3)
*/
float getHeight(float pressure) {
  return (R * T_0 / g) * -(log(pressure / P_0));
}