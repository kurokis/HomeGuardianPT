#include <SoftwareSerial.h>
SoftwareSerial mySerial(2, 3); // RX, TX

int LED = 13;
int count = 0;

void setup(){
  Serial.begin(115200);  
  mySerial.begin(115200); // ソフトウェアシリアルの初期化

  pinMode(LED, OUTPUT);
}

void loop(){

  mySerial.listen();
  if(mySerial.available()>0)
  {
  int value=mySerial.read();
    Serial.print((char)value);
    }
  

  delay(1);
}
