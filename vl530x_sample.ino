#include <Wire.h>
#include <VL53L0X.h>

 
VL53L0X gVL530X; 

void setup() {
  Serial.begin(9600);
  Wire.begin();
  //initialize vl530x
  gVL530X.init();
  gVL530X.setTimeout(1000);
  //star measurement. set sampling interval [ms]
  gVL530X.startContinuous(20);
  pinMode(ledPin, OUTPUT);   // 出力に設定
}

void loop() {
  int dist;   //センサの出力[mm]を保存する変数
  dist = gVL530X.readRangeContinuousMillimeters();  //センサーから距離[mm]
  Serial.print(dist);
  Serial.print("[mm]\n");    
  
  if (gVL530X.timeoutOccurred()){
    Serial.print("タイムアウトが起きました\n");
  }
  
  
}
