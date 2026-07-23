#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <bluefruit.h>
#include "RadioLib.h"
#include "TinyGPSPlus.h"
#include "t_echo_lite_config.h"

// User settings. Change these values, rebuild, and upload.
static const bool TRACKER_HAS_DISPLAY = false;
static const char DOG_NAME[] = "Clodagh-Test";
static const float LORA_FREQUENCY_MHZ = 868.0;
static const uint8_t LORA_SYNC_WORD = 0x12; // Standard private LoRa sync word; compatible with SX1262 and SX1276.
static const uint32_t GPS_FRESH_FIX_MAX_AGE_MS = 30000;
static const bool BLE_STATUS_LED_ENABLED = false;
static const bool GPS_RAW_NMEA_DEBUG = true;
static const uint32_t GPS_RAW_NMEA_WINDOW_MS = 120000;
static const uint32_t BUTTON_DEBOUNCE_MS = 40;
static const uint32_t BUTTON_LONG_PRESS_MS = 1500;
static const uint32_t BUTTON_VERY_LONG_PRESS_MS = 5000;
static const uint32_t COMMAND_RECEIVE_WINDOW_MS = 2000;
static const uint32_t HOME_SEND_INTERVAL_MS = 3600000;
static const uint32_t MONITORING_SEND_INTERVAL_MS = 3600000;
static const uint32_t ACTIVITY_SEND_INTERVAL_MS = 10000;
static const uint32_t SEARCH_SEND_INTERVAL_MS = 30000;
static const uint32_t EMERGENCY_SEND_INTERVAL_MS = 5000;
static const uint32_t EMERGENCY_AUTO_EXIT_MS = 600000;
static const uint32_t BLE_HOME_LOST_MS = 300000;

enum TrackerMode
{
    MODE_HOME = 0,
    MODE_MONITORING,
    MODE_ACTIVITY,
    MODE_SEARCH,
    MODE_EMERGENCY
};

struct LocationSnapshot
{
    bool valid;
    double latitude;
    double longitude;
    double altitudeMeters;
    uint32_t ageMs;
    uint32_t charsProcessed;
    uint32_t satellites;
    float hdop;
    bool fresh;
};

// BLE services follow the repository BLE_Uart example.
BLEDfu bledfu;
BLEDis bledis;
BLEBas blebas;
BLEUart bleuart;

// GPS parser and peripheral instances follow the LilyGO GPS example.
TinyGPSPlus gps;

SPIClass LoRa_SPI(NRF_SPIM3, SX1262_MISO, SX1262_SCLK, SX1262_MOSI);
SX1262 radio = new Module(SX1262_CS, SX1262_DIO1, SX1262_RST, SX1262_BUSY, LoRa_SPI);

volatile bool emergencyRequested = false;
bool bleConnected = false;
bool radioReady = false;
uint32_t packetCounter = 0;
uint32_t lastLocationSendMs = 0;
uint32_t lastGpsDebugMs = 0;
uint32_t lastRadioInitAttemptMs = 0;
uint32_t buttonPressedAtMs = 0;
uint32_t lastButtonChangeMs = 0;
uint32_t lastLedPatternMs = 0;
uint32_t ledPulseUntilMs = 0;
uint32_t lastHomeBleSeenMs = 0;
uint32_t modeStartedMs = 0;
bool lastButtonLevel = HIGH;
bool stableButtonLevel = HIGH;
bool buttonLongPressHandled = false;
bool buttonSearchPressHandled = false;
int lastLoRaState = RADIOLIB_ERR_NONE;
float lastBatteryVoltage = 0.0f;
uint8_t lastBatteryPercent = 0;
LocationSnapshot lastLocation = {false, 0.0, 0.0, 0.0, 0, 0, 0, 0.0f, false};
String lastPacketEvent = "NONE";
uint32_t lastTxMs = 0;
String deviceId = "unknown";
uint32_t bootMs = 0;
String gpsRawLine = "";
TrackerMode currentMode = MODE_ACTIVITY;
bool gpsPowered = true;
bool homeBleSeen = false;
uint32_t appliedCommandCounter = 0;

void emergencyButtonISR()
{
    emergencyRequested = true;
}

uint8_t readBatteryPercent(float voltage);
void sendLocationPacket(const char *eventName);

const char *modeName(TrackerMode mode)
{
    switch (mode)
    {
    case MODE_HOME:
        return "HOME";
    case MODE_MONITORING:
        return "MONITORING";
    case MODE_ACTIVITY:
        return "ACTIVITY";
    case MODE_SEARCH:
        return "SEARCH";
    case MODE_EMERGENCY:
        return "EMERGENCY";
    default:
        return "UNKNOWN";
    }
}

TrackerMode modeFromName(const String &name, TrackerMode fallback)
{
    if (name == "HOME")
        return MODE_HOME;
    if (name == "MONITORING" || name == "MONITOR")
        return MODE_MONITORING;
    if (name == "ACTIVITY")
        return MODE_ACTIVITY;
    if (name == "SEARCH")
        return MODE_SEARCH;
    if (name == "EMERGENCY")
        return MODE_EMERGENCY;
    return fallback;
}

uint32_t modeSendIntervalMs(TrackerMode mode)
{
    switch (mode)
    {
    case MODE_HOME:
        return HOME_SEND_INTERVAL_MS;
    case MODE_MONITORING:
        return MONITORING_SEND_INTERVAL_MS;
    case MODE_ACTIVITY:
        return ACTIVITY_SEND_INTERVAL_MS;
    case MODE_SEARCH:
        return SEARCH_SEND_INTERVAL_MS;
    case MODE_EMERGENCY:
        return EMERGENCY_SEND_INTERVAL_MS;
    default:
        return MONITORING_SEND_INTERVAL_MS;
    }
}

bool modeWantsGps(TrackerMode mode)
{
    return mode != MODE_HOME;
}

void setLed(uint32_t pin, bool on)
{
    digitalWrite(pin, on ? LOW : HIGH);
}

void allLedsOff()
{
    setLed(LED_1, false);
    setLed(LED_2, false);
    setLed(LED_3, false);
}

void pulseLed(uint32_t pin, uint16_t durationMs = 80)
{
    allLedsOff();
    setLed(pin, true);
    ledPulseUntilMs = millis() + durationMs;
}

void setGpsPower(bool on)
{
    if (gpsPowered == on)
    {
        return;
    }

    gpsPowered = on;
    if (gpsPowered)
    {
        pinMode(GPS_RT9080_EN, OUTPUT);
        digitalWrite(GPS_RT9080_EN, HIGH);
        pinMode(GPS_WAKE_UP, OUTPUT);
        digitalWrite(GPS_WAKE_UP, HIGH);
        Serial2.setPins(GPS_UART_RX, GPS_UART_TX);
        Serial2.begin(9600);
        Serial.println("GPS power ON");
    }
    else
    {
        Serial2.end();
        digitalWrite(GPS_WAKE_UP, LOW);
        pinMode(GPS_WAKE_UP, INPUT_PULLDOWN);
        digitalWrite(GPS_RT9080_EN, LOW);
        pinMode(GPS_RT9080_EN, INPUT_PULLDOWN);
        gpsRawLine = "";
        Serial.println("GPS power OFF");
    }
}

void applyMode(TrackerMode mode, const char *reason)
{
    if (currentMode == mode && modeStartedMs != 0)
    {
        return;
    }

    currentMode = mode;
    modeStartedMs = millis();
    setGpsPower(modeWantsGps(currentMode));
    Serial.print("Mode changed to ");
    Serial.print(modeName(currentMode));
    Serial.print(" reason=");
    Serial.println(reason);
}

// The LilyGO SX126x example uses these two RF switch pins for TX/RX routing.
void setLoRaTransmitSwitch(bool transmitting)
{
    digitalWrite(SX1262_RF_VC1, transmitting ? HIGH : LOW);
    digitalWrite(SX1262_RF_VC2, transmitting ? LOW : HIGH);
}

bool startRadio()
{
    lastRadioInitAttemptMs = millis();
    radioReady = false;

    pinMode(SX1262_RF_VC1, OUTPUT);
    pinMode(SX1262_RF_VC2, OUTPUT);
    setLoRaTransmitSwitch(false);

    LoRa_SPI.begin();
    LoRa_SPI.setClockDivider(SPI_CLOCK_DIV2);

    lastLoRaState = radio.begin();
    if (lastLoRaState != RADIOLIB_ERR_NONE)
    {
        Serial.print("SX1262 init failed, code=");
        Serial.println(lastLoRaState);
        return false;
    }

    int state = radio.setFrequency(LORA_FREQUENCY_MHZ);
    if (state == RADIOLIB_ERR_NONE)
        state = radio.setBandwidth(125.0);
    if (state == RADIOLIB_ERR_NONE)
        state = radio.setSpreadingFactor(9);
    if (state == RADIOLIB_ERR_NONE)
        state = radio.setCodingRate(6);
    if (state == RADIOLIB_ERR_NONE)
        state = radio.setSyncWord(LORA_SYNC_WORD);
    if (state == RADIOLIB_ERR_NONE)
        state = radio.setOutputPower(22);
    if (state == RADIOLIB_ERR_NONE)
        state = radio.setCurrentLimit(140);
    if (state == RADIOLIB_ERR_NONE)
        state = radio.setPreambleLength(16);
    if (state == RADIOLIB_ERR_NONE)
        state = radio.setCRC(false);

    lastLoRaState = state;
    if (lastLoRaState != RADIOLIB_ERR_NONE)
    {
        Serial.print("SX1262 config failed, code=");
        Serial.println(lastLoRaState);
        return false;
    }

    radio.sleep();
    radioReady = true;
    Serial.println("SX1262 ready");
    return true;
}

// Battery measurement follows the LilyGO Battery_Measurement example:
// enable the divider, read with the 3.0 V internal reference, then disable it.
float readBatteryVoltage()
{
    digitalWrite(BATTERY_MEASUREMENT_CONTROL, HIGH);
    delay(5);
    const uint16_t raw = analogRead(BATTERY_ADC_DATA);
    digitalWrite(BATTERY_MEASUREMENT_CONTROL, LOW);

    const float adcVoltage = ((float)raw * (3000.0f / 4096.0f)) / 1000.0f;
    lastBatteryVoltage = adcVoltage * 2.0f;
    lastBatteryPercent = readBatteryPercent(lastBatteryVoltage);
    return lastBatteryVoltage;
}

uint8_t readBatteryPercent(float voltage)
{
    const int percent = map((int)(voltage * 1000.0f), 3300, 4200, 0, 100);
    return (uint8_t)constrain(percent, 0, 100);
}

// CRC-16/CCITT-FALSE over the ASCII packet body.
uint16_t crc16Ccitt(const String &body)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < body.length(); i++)
    {
        crc ^= ((uint16_t)body[i] << 8);
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

String fixed6(double value)
{
    return String(value, 6);
}

String fixed1(double value)
{
    return String(value, 1);
}

String fieldAt(const String &text, uint8_t index)
{
    uint8_t current = 0;
    int start = 0;
    for (int i = 0; i <= text.length(); i++)
    {
        if (i == text.length() || text[i] == ',')
        {
            if (current == index)
            {
                return text.substring(start, i);
            }
            current++;
            start = i + 1;
        }
    }
    return "";
}

void serviceGps()
{
    if (!gpsPowered)
    {
        return;
    }

    while (Serial2.available() > 0)
    {
        const char c = (char)Serial2.read();
        gps.encode(c);

        if (GPS_RAW_NMEA_DEBUG && millis() - bootMs <= GPS_RAW_NMEA_WINDOW_MS)
        {
            if (c == '\n')
            {
                gpsRawLine.trim();
                if (gpsRawLine.length() > 0)
                {
                    Serial.print("GPS raw: ");
                    Serial.println(gpsRawLine);
                }
                gpsRawLine = "";
            }
            else if (c != '\r' && gpsRawLine.length() < 120)
            {
                gpsRawLine += c;
            }
        }
    }
}

LocationSnapshot readLocationSnapshot()
{
    serviceGps();

    LocationSnapshot snapshot;
    if (!gpsPowered)
    {
        snapshot.valid = false;
        snapshot.latitude = 0.0;
        snapshot.longitude = 0.0;
        snapshot.altitudeMeters = 0.0;
        snapshot.ageMs = 0;
        snapshot.charsProcessed = gps.charsProcessed();
        snapshot.satellites = 0;
        snapshot.hdop = 0.0f;
        snapshot.fresh = false;
        return snapshot;
    }

    snapshot.ageMs = gps.location.age();
    snapshot.satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;
    snapshot.hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 0.0f;
    snapshot.fresh = gps.location.isValid() && snapshot.ageMs <= GPS_FRESH_FIX_MAX_AGE_MS;
    snapshot.valid = snapshot.fresh;
    snapshot.latitude = snapshot.valid ? gps.location.lat() : 0.0;
    snapshot.longitude = snapshot.valid ? gps.location.lng() : 0.0;
    snapshot.altitudeMeters = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
    snapshot.charsProcessed = gps.charsProcessed();
    return snapshot;
}

void logGpsDebug()
{
    if (millis() - lastGpsDebugMs < 5000)
    {
        return;
    }

    lastGpsDebugMs = millis();
    LocationSnapshot location = readLocationSnapshot();
    lastLocation = location;

    Serial.print("GPS debug: fix=");
    Serial.print(location.valid ? "YES" : "NO");
    Serial.print(" chars=");
    Serial.print(location.charsProcessed);
    Serial.print(" sats=");
    Serial.print(location.satellites);
    Serial.print(" hdop=");
    Serial.print(location.hdop);
    Serial.print(" age_ms=");
    Serial.print(location.ageMs);
    Serial.print(" fresh=");
    Serial.print(location.fresh ? "YES" : "NO");
    if (location.valid)
    {
        Serial.print(" lat=");
        Serial.print(location.latitude, 6);
        Serial.print(" lon=");
        Serial.print(location.longitude, 6);
    }
    Serial.println();
}

// Packet format transmitted over LoRa.
// Base fields:
// DT1,<device_id>,<dog>,<counter>,<event>,<fix>,<lat>,<lon>,<alt_m>,<battery_mv>
//
// Battery-test extension fields appended after battery_mv:
// <mode>,<interval_s>,<uptime_s>,<battery_pct>,<radio_ready>,<last_tx_code>,<gps_chars>,<display_on>,<ble_connected>
//
// GPS quality extension fields appended after ble_connected:
// <gps_age_ms>,<gps_sats>,<gps_hdop>
//
// event is PERIODIC or EMERGENCY. fix is 1 only when GPS has a fresh location
// updated within GPS_FRESH_FIX_MAX_AGE_MS, otherwise 0. This avoids publishing
// stale coordinates to Home Assistant maps.
// radio_ready/display_on/ble_connected are 1 or 0. last_tx_code is the previous
// RadioLib transmit/init status code, where 0 means OK.
// crc16 is CRC-16/CCITT-FALSE of the bytes before the asterisk.
String buildPacket(const LocationSnapshot &location, float batteryVoltage, const char *eventName)
{
    String body = "DT1,";
    body += deviceId;
    body += ",";
    body += DOG_NAME;
    body += ",";
    body += String(packetCounter);
    body += ",";
    body += eventName;
    body += ",";
    body += location.valid ? "1" : "0";
    body += ",";
    body += location.valid ? fixed6(location.latitude) : "0.000000";
    body += ",";
    body += location.valid ? fixed6(location.longitude) : "0.000000";
    body += ",";
    body += location.valid ? fixed1(location.altitudeMeters) : "0.0";
    body += ",";
    body += String((uint16_t)(batteryVoltage * 1000.0f));
    body += ",";
    body += modeName(currentMode);
    body += ",";
    body += String(modeSendIntervalMs(currentMode) / 1000);
    body += ",";
    body += String(millis() / 1000);
    body += ",";
    body += String(readBatteryPercent(batteryVoltage));
    body += ",";
    body += radioReady ? "1" : "0";
    body += ",";
    body += String(lastLoRaState);
    body += ",";
    body += String(location.charsProcessed);
    body += ",";
    body += TRACKER_HAS_DISPLAY ? "1" : "0";
    body += ",";
    body += bleConnected ? "1" : "0";
    body += ",";
    body += String(location.ageMs);
    body += ",";
    body += String(location.satellites);
    body += ",";
    body += String(location.hdop, 2);

    char crcText[8];
    snprintf(crcText, sizeof(crcText), "%04X", crc16Ccitt(body));
    return body + "*" + crcText;
}

bool parseCommandPacket(const String &raw)
{
    const int star = raw.lastIndexOf('*');
    if (star < 0 || star + 5 > raw.length())
    {
        return false;
    }

    const String body = raw.substring(0, star);
    const String crcText = raw.substring(star + 1);
    const uint16_t sentCrc = (uint16_t)strtoul(crcText.c_str(), nullptr, 16);
    if (sentCrc != crc16Ccitt(body))
    {
        Serial.println("Command ignored: CRC mismatch");
        return false;
    }

    if (fieldAt(body, 0) != "CMD1")
    {
        return false;
    }

    const String targetId = fieldAt(body, 1);
    if (targetId != deviceId && targetId != "*")
    {
        Serial.print("Command ignored: target=");
        Serial.println(targetId);
        return false;
    }

    const String modeText = fieldAt(body, 2);
    const uint32_t commandCounter = (uint32_t)fieldAt(body, 3).toInt();
    if (commandCounter != 0 && commandCounter == appliedCommandCounter)
    {
        Serial.println("Command ignored: already applied");
        return false;
    }

    appliedCommandCounter = commandCounter;
    applyMode(modeFromName(modeText, currentMode), "LORA_COMMAND");
    pulseLed(LED_2, 250);
    return true;
}

void listenForCommand()
{
    if (!radioReady)
    {
        return;
    }

    String command;
    setLoRaTransmitSwitch(false);
    const int state = radio.receive(command, COMMAND_RECEIVE_WINDOW_MS);
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.print("LoRa RX command: ");
        Serial.println(command);
        parseCommandPacket(command);
    }
    else if (state != RADIOLIB_ERR_RX_TIMEOUT)
    {
        Serial.print("LoRa command receive state=");
        Serial.println(state);
    }
}

void updateDisplay(const char *status, const LocationSnapshot &location, float batteryVoltage)
{
    if (!TRACKER_HAS_DISPLAY)
    {
        Serial.print("Status ");
        Serial.print(status);
        Serial.print(" battery=");
        Serial.print(batteryVoltage, 2);
        Serial.print("V gps=");
        Serial.println(location.valid ? "fix" : "no-fix");
    }
}

const char *displayStatusForEvent(const char *eventName, const LocationSnapshot &location)
{
    if (strcmp(eventName, "EMERGENCY") == 0)
    {
        return "HELP";
    }

    if (!location.valid)
    {
        return "GPS WAIT";
    }

    return "TRACKING";
}

void startBleAdvertising()
{
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(bleuart);
    Bluefruit.Advertising.addService(blebas);
    Bluefruit.ScanResponse.addName();
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0);
}

void connectCallback(uint16_t connHandle)
{
    (void)connHandle;
    bleConnected = true;
    homeBleSeen = true;
    lastHomeBleSeenMs = millis();
    Serial.println("BLE connected");
}

void disconnectCallback(uint16_t connHandle, uint8_t reason)
{
    (void)connHandle;
    bleConnected = false;
    Serial.print("BLE disconnected, reason=0x");
    Serial.println(reason, HEX);
}

void updateModeAutomation()
{
    if (bleConnected)
    {
        homeBleSeen = true;
        lastHomeBleSeenMs = millis();
    }

    if (currentMode == MODE_EMERGENCY && millis() - modeStartedMs >= EMERGENCY_AUTO_EXIT_MS)
    {
        applyMode(MODE_SEARCH, "EMERGENCY_TIMEOUT");
    }

    if (currentMode == MODE_HOME && homeBleSeen && millis() - lastHomeBleSeenMs >= BLE_HOME_LOST_MS)
    {
        applyMode(MODE_SEARCH, "BLE_HOME_LOST");
        sendLocationPacket("MODE");
    }
}

void updateLedPattern()
{
    const uint32_t now = millis();
    if (ledPulseUntilMs && now < ledPulseUntilMs)
    {
        return;
    }
    if (ledPulseUntilMs && now >= ledPulseUntilMs)
    {
        ledPulseUntilMs = 0;
        allLedsOff();
    }

    uint32_t periodMs = 30000;
    uint32_t ledPin = LED_2;
    uint16_t pulseMs = 60;

    switch (currentMode)
    {
    case MODE_HOME:
        periodMs = 30000;
        ledPin = LED_2;
        break;
    case MODE_MONITORING:
        periodMs = 15000;
        ledPin = LED_2;
        break;
    case MODE_ACTIVITY:
        periodMs = 10000;
        ledPin = LED_1;
        break;
    case MODE_SEARCH:
        periodMs = 3000;
        ledPin = LED_1;
        break;
    case MODE_EMERGENCY:
        periodMs = 1000;
        ledPin = (now / 1000) % 2 ? LED_1 : LED_2;
        pulseMs = 120;
        break;
    }

    if (now - lastLedPatternMs >= periodMs)
    {
        lastLedPatternMs = now;
        pulseLed(ledPin, pulseMs);
    }
}

void sendLocationPacket(const char *eventName)
{
    LocationSnapshot location = readLocationSnapshot();
    lastLocation = location;
    const float batteryVoltage = readBatteryVoltage();
    const uint8_t batteryPercent = readBatteryPercent(batteryVoltage);
    blebas.write(batteryPercent);

    String packet = buildPacket(location, batteryVoltage, eventName);
    packetCounter++;

    Serial.print("LoRa TX ");
    Serial.print(eventName);
    Serial.print(": ");
    Serial.println(packet);

    if (bleConnected)
    {
        bleuart.println(packet);
    }

    if (!radioReady)
    {
        Serial.println("LoRa not ready, retrying SX1262 init");
        startRadio();
    }

    if (radioReady)
    {
        LoRa_SPI.begin();
        setLoRaTransmitSwitch(true);
        lastLoRaState = radio.transmit(packet);
        setLoRaTransmitSwitch(false);
        listenForCommand();
        radio.sleep();
        Serial.print("LoRa TX result=");
        Serial.println(lastLoRaState);
    }
    else
    {
        lastLoRaState = RADIOLIB_ERR_UNKNOWN;
        Serial.println("LoRa skipped because radio init failed");
    }

    lastPacketEvent = eventName;
    lastTxMs = millis();
    updateDisplay(displayStatusForEvent(eventName, location), location, batteryVoltage);
    lastLocationSendMs = millis();
}

void cycleDisplayScreen()
{
    TrackerMode next = MODE_ACTIVITY;
    if (currentMode == MODE_ACTIVITY)
    {
        next = MODE_MONITORING;
    }
    else if (currentMode == MODE_MONITORING)
    {
        next = MODE_HOME;
    }
    else
    {
        next = MODE_ACTIVITY;
    }
    applyMode(next, "BUTTON_SHORT");
    sendLocationPacket("MODE");
}

void handleButton()
{
    const bool level = digitalRead(nRF52840_BOOT);
    const uint32_t now = millis();

    if (level != lastButtonLevel)
    {
        lastButtonLevel = level;
        lastButtonChangeMs = now;
    }

    if ((now - lastButtonChangeMs) < BUTTON_DEBOUNCE_MS || level == stableButtonLevel)
    {
        if (stableButtonLevel == LOW && !buttonSearchPressHandled && (now - buttonPressedAtMs) >= BUTTON_LONG_PRESS_MS)
        {
            buttonSearchPressHandled = true;
            applyMode(MODE_SEARCH, "BUTTON_LONG");
            sendLocationPacket("MODE");
        }

        if (stableButtonLevel == LOW && !buttonLongPressHandled && (now - buttonPressedAtMs) >= BUTTON_VERY_LONG_PRESS_MS)
        {
            buttonLongPressHandled = true;
            emergencyRequested = true;
        }
        return;
    }

    stableButtonLevel = level;
    if (stableButtonLevel == LOW)
    {
        buttonPressedAtMs = now;
        buttonLongPressHandled = false;
        buttonSearchPressHandled = false;
    }
    else if (!buttonLongPressHandled)
    {
        cycleDisplayScreen();
    }
}

void setup()
{
    Serial.begin(115200);
    delay(3000);
    bootMs = millis();
    Serial.println("T-Echo Lite dog tracker starting");

    // Main 3.3 V rail and user-visible LEDs. LilyGO examples show these LEDs
    // are active-low, so HIGH keeps them off for cleaner battery testing.
    pinMode(RT9080_EN, OUTPUT);
    digitalWrite(RT9080_EN, HIGH);
    pinMode(LED_1, OUTPUT);
    digitalWrite(LED_1, HIGH);
    pinMode(LED_2, OUTPUT);
    digitalWrite(LED_2, HIGH);
    pinMode(LED_3, OUTPUT);
    digitalWrite(LED_3, HIGH);

    // Battery ADC setup copied from the LilyGO battery example.
    pinMode(BATTERY_ADC_DATA, INPUT);
    pinMode(BATTERY_MEASUREMENT_CONTROL, OUTPUT);
    digitalWrite(BATTERY_MEASUREMENT_CONTROL, LOW);
    analogReference(AR_INTERNAL_3_0);
    analogReadResolution(12);

    // Boot button is the repository-defined nRF52840_BOOT pin.
    pinMode(nRF52840_BOOT, INPUT_PULLUP);

    // GPS power, wake, and UART setup copied from the LilyGO GPS example.
    pinMode(RT9080_EN, OUTPUT);
    digitalWrite(RT9080_EN, HIGH);
    pinMode(GPS_RT9080_EN, OUTPUT);
    digitalWrite(GPS_RT9080_EN, HIGH);
    pinMode(GPS_1PPS, INPUT);
    pinMode(GPS_WAKE_UP, OUTPUT);
    digitalWrite(GPS_WAKE_UP, HIGH);
    Serial2.setPins(GPS_UART_RX, GPS_UART_TX);
    Serial2.begin(9600);

    Serial.println("E-paper display disabled in firmware");

    // LoRa setup copied from LilyGO's SX126x examples, with EU 868 MHz.
    startRadio();

    // BLE advertises using DOG_NAME and provides UART/debug plus battery level.
    // Disable the default Bluefruit advertising/connection blink. It is useful
    // during bring-up, but it adds an avoidable load during battery tests.
    Bluefruit.autoConnLed(BLE_STATUS_LED_ENABLED);
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    Bluefruit.begin();
    Bluefruit.setTxPower(8);
    Bluefruit.setName(DOG_NAME);
    Bluefruit.Periph.setConnectCallback(connectCallback);
    Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

    bledfu.begin();
    bledis.setManufacturer("LILYGO Industries");
    bledis.setModel("T-Echo-Lite Dog Tracker");
    bledis.begin();
    blebas.begin();
    bleuart.begin();
    startBleAdvertising();

    deviceId = String(getMcuUniqueID());
    Serial.print("Device ID: ");
    Serial.println(deviceId);
    Serial.print("BLE name: ");
    Serial.println(DOG_NAME);
    Serial.print("LoRa frequency MHz: ");
    Serial.println(LORA_FREQUENCY_MHZ, 3);
    applyMode(currentMode, "BOOT_DEFAULT");

    LocationSnapshot location = readLocationSnapshot();
    lastLocation = location;
    updateDisplay("STARTING", location, readBatteryVoltage());
    sendLocationPacket("PERIODIC");
}

void loop()
{
    serviceGps();
    logGpsDebug();
    handleButton();

    if (emergencyRequested)
    {
        emergencyRequested = false;
        Serial.println("Emergency button pressed");
        applyMode(MODE_EMERGENCY, "EMERGENCY_BUTTON");
        sendLocationPacket("EMERGENCY");
    }

    updateModeAutomation();
    updateLedPattern();

    if (millis() - lastLocationSendMs >= modeSendIntervalMs(currentMode))
    {
        sendLocationPacket("PERIODIC");
    }

    if (!radioReady && millis() - lastRadioInitAttemptMs >= 15000)
    {
        Serial.println("Retrying SX1262 init");
        startRadio();
    }

    while (Serial.available())
    {
        const char c = (char)Serial.read();
        if (bleConnected)
        {
            bleuart.write((uint8_t)c);
        }
    }

    while (bleuart.available())
    {
        Serial.write((char)bleuart.read());
    }

    delay(10);
}
