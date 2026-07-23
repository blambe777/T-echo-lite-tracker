#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include "TinyGPSPlus.h"

void displayInfo();

// Build wrapper for the unmodified LilyGO GPS example.
#include "../../examples/T-Echo-Lite/GPS/GPS.ino"
