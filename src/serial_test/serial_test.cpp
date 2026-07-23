#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include "t_echo_lite_config.h"

void setup()
{
    Serial.begin(115200);
    delay(3000);

    pinMode(RT9080_EN, OUTPUT);
    digitalWrite(RT9080_EN, HIGH);

    Serial.println("T-Echo Lite serial test");
    Serial.println("USB serial is alive at 115200 baud.");
}

void loop()
{
    static uint32_t counter = 0;
    Serial.printf("serial_test tick=%lu millis=%lu\n", counter++, millis());
    delay(1000);
}
