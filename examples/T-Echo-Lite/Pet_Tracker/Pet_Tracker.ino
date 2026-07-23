#include <Arduino.h>
#include <bluefruit.h>
#include <RadioLib.h>
#include <TinyGPS++.h>
#include <GxEPD2_BW.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>

// ==========================================
// HARDWARE PIN DEFINITIONS (LilyGO T-Echo Lite)
// ==========================================
#define PIN_BAT_READ_EN   31  // HIGH = enable battery measurement circuit
#define PIN_BAT_ADC       A2  // Analog pin reading battery voltage
#define PIN_GPS_EN        34  // HIGH = Power VCC to L76K GPS Module
#define PIN_USER_BUTTON   0   // Hardware button 2
#define PIN_IMU_INT       42  // Motion interrupt from ICM20948

// LoRa SPI & Control Pins (SX1262)
#define PIN_LORA_NSS      24
#define PIN_LORA_DIO1     20
#define PIN_LORA_NRST     25
#define PIN_LORA_BUSY     17

// E-Paper SPI & Control Pins 
#define PIN_EPD_CS        8
#define PIN_EPD_DC        28
#define PIN_EPD_RST       29
#define PIN_EPD_BUSY      30

// ==========================================
// INSTANCE INITIALIZATIONS
// ==========================================
// 1.54" E-Paper instance for the T-Echo Lite panel
GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> display(GxEPD2_154_D67(PIN_EPD_CS, PIN_EPD_DC, PIN_EPD_RST, PIN_EPD_BUSY));

// LoRa Radio instance via RadioLib
SX1262 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO1, PIN_LORA_NRST, PIN_LORA_BUSY);

// GPS parsing library
TinyGPSPlus gps;

// BLE Services
BLEDis  bledis;  
BLEBas  blebas;  

// Global States & Timers
volatile bool buttonTriggered = false;
volatile bool motionTriggered = false;
bool isHomeBLEConnected = false;
unsigned long lastBatteryUpdate = 0;
unsigned long lastLoRaPingTime = 0;

// LoRa Settings (Matches regional ISM 868MHz / 915MHz configurations)
const float LORA_FREQ = 868.0; // Change to 915.0 if you are outside Europe/UK
const float LORA_BW   = 125.0;
const uint8_t LORA_SF = 9;
const uint8_t LORA_CR = 7;
const int8_t LORA_TX_POWER = 22; // +22dBm max punch through dense brush

// ==========================================
// HARDWARE INTERRUPT SERVICE ROUTINES
// ==========================================
void button_ISR() {
  buttonTriggered = true;
}

void motion_ISR() {
  motionTriggered = true;
}

// ==========================================
// UTILITY FUNCTIONS
// ==========================================
uint8_t get_battery_level() {
  digitalWrite(PIN_BAT_READ_EN, HIGH);
  delay(5); // Stabilize analog trace
  int rawADC = analogRead(PIN_BAT_ADC);
  digitalWrite(PIN_BAT_READ_EN, LOW); // Cut connection to stop constant leakage
  
  // 12-bit ADC mapping over internal reference
  float measuredV = (rawADC / 4095.0) * 3.6 * 2.0; 
  int pct = map(measuredV * 100, 340, 420, 0, 100);
  return (uint8_t)constrain(pct, 0, 100);
}

void update_ui(String stateMessage, uint8_t batPct) {
  display.init(0);
  display.setRotation(1);
  display.setTextColor(GxEPD_BLACK);
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    
    // Header Row
    display.setFont(&FreeSansBold12pt7b);
    display.setCursor(10, 30);
    display.print("ROXY"); 
    
    display.setFont(&FreeSans9pt7b);
    display.setCursor(145, 30);
    display.print(String(batPct) + "%");
    display.drawFastHLine(0, 42, 200, GxEPD_BLACK);
    
    // Core Status text block
    display.setCursor(10, 80);
    display.print("Status: ");
    display.setCursor(10, 105);
    display.print(stateMessage);
    
    // Permanent Emergency Contact Info
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(10, 155);
    display.print("IF FOUND CALL:");
    display.setCursor(10, 180);
    display.print("+353 XX XXX XXXX");
  } while (display.nextPage());
  
  display.powerOff(); // Turn off display driver bias to achieve absolute 0mA static hold
}

void ble_connect_callback(uint16_t conn_handle) {
  isHomeBLEConnected = true;
  digitalWrite(PIN_GPS_EN, LOW); // Kill power line to L76K instantly
  radio.sleep();                 // Drop SX1262 down to 1.6µA state
  update_ui("SAFE AT HOME", get_battery_level());
}

void ble_disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  isHomeBLEConnected = false;
  motionTriggered = true; // Force-wakes tracking loop out of standby
}

void setup() {
  pinMode(PIN_BAT_READ_EN, OUTPUT);
  pinMode(PIN_GPS_EN, OUTPUT);
  digitalWrite(PIN_GPS_EN, LOW); // Start with GPS cold-isolated
  
  pinMode(PIN_USER_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_USER_BUTTON), button_ISR, FALLING);
  
  pinMode(PIN_IMU_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT), motion_ISR, FALLING);

  display.init(0);
  update_ui("INITIALIZING...", get_battery_level());

  int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, 0x12, LORA_TX_POWER);
  if (state == RADIOLIB_ERR_NONE) {
    radio.sleep();
  } else {
    update_ui("LORA ERROR", get_battery_level());
    while (1);
  }

  Serial1.begin(9600, SERIAL_8N1);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.Periph.setConnectCallback(ble_connect_callback);
  Bluefruit.Periph.setDisconnectCallback(ble_disconnect_callback);

  bledis.setManufacturer("DIY Custom");
  bledis.setModel("T-Echo Dog v1");
  bledis.begin();

  blebas.begin();
  blebas.write(get_battery_level());

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(blebas);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.start(0);

  update_ui("SEARCHING BASE", get_battery_level());
}

void loop() {
  if (buttonTriggered) {
    buttonTriggered = false;
    uint8_t currentBat = get_battery_level();
    String msg = isHomeBLEConnected ? "HOME (FORCED REF)" : "ACTIVE SEARCH";
    update_ui(msg, currentBat);
    if (!isHomeBLEConnected) {
      motionTriggered = true;
    }
  }

  if (isHomeBLEConnected) {
    if (millis() - lastBatteryUpdate > 900000) {
      uint8_t bat = get_battery_level();
      blebas.write(bat);
      update_ui("SAFE AT HOME", bat);
      lastBatteryUpdate = millis();
    }
    sd_app_evt_wait();
  } else {
    if (motionTriggered) {
      motionTriggered = false;
      digitalWrite(PIN_GPS_EN, HIGH);
      unsigned long gpsSearchStart = millis();
      bool hasLock = false;
      
      while (millis() - gpsSearchStart < 60000) {
        while (Serial1.available() > 0) {
          if (gps.encode(Serial1.read())) {
            if (gps.location.isValid() && gps.location.age() < 2000) {
              hasLock = true;
              break;
            }
          }
        }
        if (hasLock) break;
      }

      if (hasLock) {
        uint8_t bat = get_battery_level();
        uint8_t payload[11];
        int32_t latFixed = gps.location.lat() * 1000000;
        int32_t lngFixed = gps.location.lng() * 1000000;
        int16_t altFixed = gps.altitude.meters();
        memcpy(&payload[0], &latFixed, 4);
        memcpy(&payload[4], &lngFixed, 4);
        memcpy(&payload[8], &altFixed, 2);
        payload[10] = bat;

        radio.transmit(payload, 11);
        radio.sleep();

        String coords = String(gps.location.lat(), 4) + ", " + String(gps.location.lng(), 4);
        update_ui("LORA TX OK\n" + coords, bat);
      } else {
        uint8_t bat = get_battery_level();
        uint8_t errPayload[1] = { bat };
        radio.transmit(errPayload, 1);
        radio.sleep();
        update_ui("GPS NO LOCK\nSEND KEEPALIVE", bat);
      }

      digitalWrite(PIN_GPS_EN, LOW);
      lastLoRaPingTime = millis();
    }

    if (millis() - lastLoRaPingTime > 60000) {
      motionTriggered = true;
    }
    delay(200);
  }
}
