#include<Servo.h>

/*
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

int DCmotor_pin = 5; //to DC motor driver
int servo_pin = 6; // signal pin of rudder servo
int rudder_angle = 0; //0~180 90= 0deg(center), 0=left max, 180=right max
int motor_velocity = 0; // 0~255
Servo rudder_servo;

void setup() {
  Serial.begin(9600);
  rudder_servo.attach(servo_pin);

}

void loop() {



  rudder_servo.write(rudder_angle++);
  if(rudder_angle>180)
  {
    rudder_angle=0;
  }

  motor_velocity=255;
  analogWrite(DCmotor_pin, motor_velocity);
 
  delay(10);
}
