# Note: run this code with
# sudo python3 motor.py

import wiringpi as w # install with: pip3 install wiringpi

# Parameters
MOTORL_INA = 23 # GPIO 23 = Pin 16
MOTORL_INB = 24 # GPIO 24 = Pin 18
MOTORL_PWM = 18 # GPIO 18 = Pin 12
#MOTORR_INA = 5 # GPIO 5 = Pin 29
#MOTORR_INB = 6 # GPIO 6 = Pin 31
#MOTORR_PWM = 27 # GPIO 27 = Pin 13

w.wiringPiSetupGpio()
w.pinMode(MOTORL_INA, w.GPIO.OUTPUT)
w.pinMode(MOTORL_INB, w.GPIO.OUTPUT)
w.pinMode(MOTORL_PWM, w.GPIO.PWM_OUTPUT)

# Set PWM Mode
# balanced: PWM_MODE_BAL
# mark-space: PWM_MODE_MS
w.pwmSetMode(w.PWM_MODE_MS)

# Set PWM range and divisor(clock)
# 1.92MHz/divisor = range*freq
# range:1024, divisor:15 => freq:125Hz
w.pwmSetRange(1024)
w.pwmSetClock(15)


w.digitalWrite(MOTORL_INA, 1)
w.digitalWrite(MOTORL_INB, 0)
  
duty = 50

while True:
  w.pwmWrite(MOTORL_PWM, int(duty*10))
  print(duty)
