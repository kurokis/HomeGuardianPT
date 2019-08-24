
/*
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4
 *
 * ToF (VL53L0X) attached to following pins:
 ** SDA 
 ** SCL
 ** 3.3V
 ** GND
 * 
 * Sonar (HC-SR04) attached to following pins:
 ** VCC - 5V
 ** Trig - pin 3
 ** Echo - pin 9
 ** GND
 * 
 * DC motor driver
 * PWMA - pin 5
 * VCC
 * VM - motor voltage
 * GND
 * 
 * Servo motor for rudder
 * signal (yellow) - pin 6
 * VCC 5V
 * GND
 */

#include <SPI.h> // SD
#include <SD.h> // SD
#include <Wire.h> // I2C - ToF
#include <VL53L0X.h>// I2C - ToF

// Constants: pins
const int chipSelect = 4;
const int sonarTrigPin = 3;
const int sonarEchoPin = 9;
//const int switchPin = 2;
const int DCmotor_pin = 5; //to DC motor driver
const int servo_pin = 6; // signal pin of rudder servo
//const int LEDPin = 8;
const unsigned long sonarTimeout = 30000; // microseconds, 0.03s=>max range 5.1m

// Variables: control parameters
int rudder_angle = 90; //0~180 90= 0deg(center), 0=left max, 180=right max
int motor_velocity = 100; // 0~255

// Functions
uint16_t read_tof();
uint16_t read_sonar();

// Sensors
VL53L0X tof;

// Servos
Servo rudder_servo;

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

  // Setup code for ToF sensor
  Wire.begin();
  tof.init();
  tof.setTimeout(500);
  tof.startContinuous(20);

  // Setup code for sonar
  pinMode(sonarTrigPin, OUTPUT);
  pinMode(sonarEchoPin, INPUT); 

  // Setup code for servo
  rudder_servo.attach(servo_pin);
}

void loop()
{
  // make a string for assembling the data to log:
  String dataString = "";

  // Get run time
  unsigned long current_time = millis();
  
  // Read sensor data
  uint16_t tof_reading = read_tof();
  uint16_t sonar_reading = read_sonar();

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

  // Steer
  rudder_servo.write(rudder_angle);

  // Throttle
  analogWrite(DCmotor_pin, motor_velocity);
  
  
  //////////////////// SD card logging begin
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    //Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
  //////////////////// SD card logging end
}

uint16_t read_tof(){
  // Get range in millimeters
  uint16_t tof_mm = tof.readRangeContinuousMillimeters();
  return tof_mm;
}
uint16_t read_sonar(){
  // Get range in millimeters

  // Send pulse (duration: 10 microseconds)
  digitalWrite(sonarTrigPin, LOW); 
  delayMicroseconds(2); 
  digitalWrite(sonarTrigPin, HIGH); //超音波を出力
  delayMicroseconds(10);
  digitalWrite(sonarTrigPin, LOW);

  // Read pulse and calculate distance
  double duration = pulseIn(sonarEchoPin, HIGH, sonarTimeout); //センサからの入力
  uint16_t distance = 65535; // distance in mm
  if (duration > 0) {
    duration = duration/2; // duration for one-way in microseconds
    distance = (uint16_t)(duration/1000000*340*1000); // distance in mm (speed of sound: 340m/s)
  }
  
  return distance;
}

