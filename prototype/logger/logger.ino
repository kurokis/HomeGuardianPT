/*
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4
 */

#include <SPI.h>
#include <SD.h>

const int chipSelect = 4;

int read_tof();
int read_sonar();

void setup()
{
  //////////////////// SD card setup begin
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  //////////////////// SD card setup end

  // TODO: write setup code for ToF sensor

  // TODO: write setup code for sonar
  
}

void loop()
{
  // make a string for assembling the data to log:
  String dataString = "";

  // Get run time
  unsigned long current_time = millis();
  
  // Read sensor data
  int tof_reading = read_tof();
  int sonar_reading = read_sonar();

  // Create comma-delimited string from sensor data
  dataString += String(current_time);
  dataString += ",";
  dataString += String(tof_reading);
  dataString += ",";
  dataString += String(sonar_reading);

  /*
  // Sample code
  // read three sensors and append to the string:
  for (int analogPin = 0; analogPin < 3; analogPin++) {
    int sensor = analogRead(analogPin);
    dataString += String(sensor);
    if (analogPin < 2) {
      dataString += ",";
    }
  }
  */
  
  //////////////////// SD card logging begin
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
  //////////////////// SD card logging end
}

int read_tof(){
  // TODO: write code for reading ToF data

  return 0;
}
int read_sonar(){
  // TODO: write code for reading sonar data
  
  return 0;
}

