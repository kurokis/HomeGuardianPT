void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(Serial.available()){
    byte in = Serial.read();
    unsigned int a = (unsigned int) in;
    Serial.write(a);
  }
}
