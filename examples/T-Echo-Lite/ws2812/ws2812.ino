#include <Arduino.h>
#include <Adafruit_TinyUSB.h>

void setup() {
  Serial.begin(115200);
  delay(3000);
}

void loop() {
  Serial.println("HELLO FROM T-ECHO LITE");
  delay(1000);
}