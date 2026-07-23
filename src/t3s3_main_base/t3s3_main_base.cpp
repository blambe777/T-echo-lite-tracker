#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <PubSubClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RadioLib.h>
#include <time.h>

#if __has_include("../local_config.h")
#include "../local_config.h"
#endif

#ifndef BASE_WIFI_SSID
#define BASE_WIFI_SSID "CHANGE_ME"
#endif
#ifndef BASE_WIFI_PASSWORD
#define BASE_WIFI_PASSWORD ""
#endif
#ifndef BASE_MQTT_HOST
#define BASE_MQTT_HOST "homeassistant.local"
#endif
#ifndef BASE_MQTT_USER
#define BASE_MQTT_USER ""
#endif
#ifndef BASE_MQTT_PASSWORD
#define BASE_MQTT_PASSWORD ""
#endif
#ifndef BASE_OTA_PASSWORD
#define BASE_OTA_PASSWORD "CHANGE_ME"
#endif

// Home Wi-Fi settings. Leave WIFI_SSID as CHANGE_ME to use setup/status AP mode.
static const char *WIFI_SSID = BASE_WIFI_SSID;
static const char *WIFI_PASSWORD = BASE_WIFI_PASSWORD;
static const char *SETUP_AP_NAME = "Clodagh-T3S3-Base";

// MQTT/Home Assistant settings. If your broker is not on homeassistant.local,
// change MQTT_HOST to the Home Assistant or Mosquitto broker IP address.
static const char *MQTT_HOST = BASE_MQTT_HOST;
static const uint16_t MQTT_PORT = 1883;
static const char *MQTT_USER = BASE_MQTT_USER;
static const char *MQTT_PASSWORD = BASE_MQTT_PASSWORD;
static const char *MQTT_CLIENT_ID = "clodagh_t3s3_base";
static const char *MQTT_BASE_TOPIC = "clodagh_tracker";
static const char *MQTT_BASE_STATION_TOPIC = "clodagh_t3s3_base";
static const char *MQTT_DISCOVERY_PREFIX = "homeassistant";

// Web OTA settings. Browse to http://<base-ip>/update and log in with these.
static const char *OTA_USER = "admin";
static const char *OTA_PASSWORD = BASE_OTA_PASSWORD;

// Time settings for Home Assistant timestamps. POSIX TZ keeps Ireland on
// GMT/IST automatically; the fallback string is used until NTP sync completes.
static const char *TIME_ZONE = "GMT0IST,M3.5.0/1,M10.5.0";
static const char *NTP_SERVER_1 = "pool.ntp.org";
static const char *NTP_SERVER_2 = "time.google.com";

// Collar LoRa settings. These must match src/dog_tracker/dog_tracker.cpp.
static const float LORA_FREQUENCY_MHZ = 868.0;
static const float LORA_BANDWIDTH_KHZ = 125.0;
static const uint8_t LORA_SPREADING_FACTOR = 9;
static const uint8_t LORA_CODING_RATE = 6;
static const uint8_t LORA_SYNC_WORD = 0x12; // Standard private LoRa sync word; compatible with SX1262 and SX1276.
static const uint16_t LORA_PREAMBLE_LENGTH = 16;
// LilyGO T3-S3 radio pins from LilyGO LoRa Series T3-S3 hardware docs.
// This base unit is the SX1276 868/915 MHz revision. These pins match LilyGO's
// official T3-S3-SX1276 PingPong example.
static const int LORA_SCK = 5;
static const int LORA_MISO = 3;
static const int LORA_MOSI = 6;
static const int LORA_CS = 7;
static const int LORA_RESET = 8;
static const int LORA_DIO0 = 9;
static const int LORA_DIO1 = 33;

static const int OLED_SDA = 18;
static const int OLED_SCL = 17;
static const int OLED_RST = -1;
static const int USER_LED = 37;

static const int SCREEN_WIDTH = 128;
static const int SCREEN_HEIGHT = 64;
static const int OLED_ADDRESS = 0x3C;

SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RESET, LORA_DIO1);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);
WebServer server(80);
WiFiClient mqttWifiClient;
PubSubClient mqttClient(mqttWifiClient);

struct CollarPacket
{
    bool valid = false;
    bool crcOk = false;
    String raw = "";
    String deviceId = "";
    String dogName = "";
    uint32_t counter = 0;
    String event = "";
    bool gpsFix = false;
    String latitude = "";
    String longitude = "";
    String altitudeM = "";
    uint16_t batteryMv = 0;
    String trackerMode = "UNKNOWN";
    uint32_t intervalS = 0;
    uint32_t trackerUptimeS = 0;
    uint8_t batteryPercent = 0;
    bool trackerRadioReady = false;
    int16_t trackerLastTxCode = 0;
    uint32_t gpsChars = 0;
    bool trackerDisplayOn = false;
    bool trackerBleConnected = false;
    uint32_t gpsAgeMs = 0;
    uint32_t gpsSatellites = 0;
    String gpsHdop = "";
    int16_t rssi = 0;
    float snr = 0.0;
    uint32_t receivedAtMs = 0;
};

CollarPacket latestPacket;
uint32_t receivedPackets = 0;
uint32_t badPackets = 0;
int lastRadioState = RADIOLIB_ERR_NONE;
volatile bool radioPacketFlag = false;
String lastBadPacket = "";
String lastBadReason = "NONE";
int16_t lastBadRssi = 0;
float lastBadSnr = 0.0;
bool wifiStationMode = false;
String ipText = "0.0.0.0";
bool mqttDiscoveryPublished = false;
uint32_t mqttLastConnectAttempt = 0;
uint32_t mqttLastHeartbeat = 0;
uint32_t radioLastMaintenanceMs = 0;
uint32_t radioLastTimeoutLogMs = 0;

void setRadioPacketFlag()
{
    radioPacketFlag = true;
}
bool wifiHasCredentials = false;
bool fallbackApStarted = false;
uint32_t wifiLastRetry = 0;
uint32_t wifiLastStatusCheck = 0;

void drawOled();

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

bool parsePacket(const String &raw, CollarPacket &out)
{
    out = CollarPacket();
    out.raw = raw;
    out.rssi = (int16_t)radio.getRSSI();
    out.snr = radio.getSNR();
    out.receivedAtMs = millis();

    const int star = raw.lastIndexOf('*');
    if (star < 0 || star + 5 > raw.length())
    {
        return false;
    }

    const String body = raw.substring(0, star);
    const String crcText = raw.substring(star + 1);
    char *endPtr = nullptr;
    const uint16_t sentCrc = (uint16_t)strtoul(crcText.c_str(), &endPtr, 16);
    out.crcOk = sentCrc == crc16Ccitt(body);

    if (fieldAt(body, 0) != "DT1")
    {
        return false;
    }

    out.deviceId = fieldAt(body, 1);
    out.dogName = fieldAt(body, 2);
    out.counter = fieldAt(body, 3).toInt();
    out.event = fieldAt(body, 4);
    out.gpsFix = fieldAt(body, 5) == "1";
    out.latitude = fieldAt(body, 6);
    out.longitude = fieldAt(body, 7);
    out.altitudeM = fieldAt(body, 8);
    out.batteryMv = (uint16_t)fieldAt(body, 9).toInt();
    out.trackerMode = fieldAt(body, 10);
    if (out.trackerMode.length() == 0)
    {
        out.trackerMode = "UNKNOWN";
    }
    out.intervalS = (uint32_t)fieldAt(body, 11).toInt();
    out.trackerUptimeS = (uint32_t)fieldAt(body, 12).toInt();
    out.batteryPercent = (uint8_t)fieldAt(body, 13).toInt();
    out.trackerRadioReady = fieldAt(body, 14) == "1";
    out.trackerLastTxCode = (int16_t)fieldAt(body, 15).toInt();
    out.gpsChars = (uint32_t)fieldAt(body, 16).toInt();
    out.trackerDisplayOn = fieldAt(body, 17) == "1";
    out.trackerBleConnected = fieldAt(body, 18) == "1";
    out.gpsAgeMs = (uint32_t)fieldAt(body, 19).toInt();
    out.gpsSatellites = (uint32_t)fieldAt(body, 20).toInt();
    out.gpsHdop = fieldAt(body, 21);
    out.valid = out.crcOk && out.dogName.length() > 0;
    return out.valid;
}

String mqttTopic(const String &name)
{
    return String(MQTT_BASE_TOPIC) + "/" + name;
}

String baseMqttTopic(const String &name)
{
    return String(MQTT_BASE_STATION_TOPIC) + "/" + name;
}

String jsonEscape(const String &value)
{
    String escaped;
    escaped.reserve(value.length() + 8);
    for (size_t i = 0; i < value.length(); i++)
    {
        const char c = value[i];
        if (c == '"' || c == '\\')
        {
            escaped += '\\';
        }
        escaped += c;
    }
    return escaped;
}

String printablePacket(const String &value)
{
    String out;
    const char hex[] = "0123456789ABCDEF";
    for (size_t i = 0; i < value.length(); i++)
    {
        const uint8_t c = (uint8_t)value[i];
        if (c >= 32 && c <= 126)
        {
            out += (char)c;
        }
        else
        {
            out += "\\x";
            out += hex[(c >> 4) & 0x0F];
            out += hex[c & 0x0F];
        }
    }
    return out;
}

String badPacketReason(const String &raw)
{
    const int star = raw.lastIndexOf('*');
    if (star < 0)
    {
        return "NO_STAR";
    }
    if (star + 5 > raw.length())
    {
        return "SHORT_CRC";
    }

    const String body = raw.substring(0, star);
    if (fieldAt(body, 0) != "DT1")
    {
        return "NOT_DT1";
    }

    const String crcText = raw.substring(star + 1, star + 5);
    const uint16_t sentCrc = (uint16_t)strtoul(crcText.c_str(), nullptr, 16);
    const uint16_t calculatedCrc = crc16Ccitt(body);
    if (sentCrc != calculatedCrc)
    {
        char expected[5];
        snprintf(expected, sizeof(expected), "%04X", calculatedCrc);
        return String("CRC_FAIL_EXPECT_") + expected;
    }

    if (fieldAt(body, 2).length() == 0)
    {
        return "NO_DOG_NAME";
    }

    return "UNKNOWN_PARSE_FAIL";
}

void mqttPublishString(const String &topic, const String &value, bool retained = false)
{
    if (mqttClient.connected())
    {
        mqttClient.publish(topic.c_str(), value.c_str(), retained);
    }
}

String currentIsoTimestamp()
{
    struct tm timeInfo;
    if (!getLocalTime(&timeInfo, 50))
    {
        return String("uptime-") + String(millis() / 1000) + "s";
    }

    char buffer[32];
    strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S%z", &timeInfo);
    return String(buffer);
}

void mqttPublishSensorConfig(const char *objectId, const char *name, const char *stateTopic,
                             const char *deviceClass = nullptr, const char *unit = nullptr,
                             const char *stateClass = nullptr)
{
    String topic = String(MQTT_DISCOVERY_PREFIX) + "/sensor/clodagh_tracker/" + objectId + "/config";
    String payload = "{";
    payload += "\"name\":\"";
    payload += name;
    payload += "\",\"unique_id\":\"clodagh_tracker_";
    payload += objectId;
    payload += "\",\"state_topic\":\"";
    payload += stateTopic;
    payload += "\",\"availability_topic\":\"";
    payload += mqttTopic("availability");
    payload += "\",\"payload_available\":\"online\",\"payload_not_available\":\"offline\"";
    if (deviceClass)
    {
        payload += ",\"device_class\":\"";
        payload += deviceClass;
        payload += "\"";
    }
    if (unit)
    {
        payload += ",\"unit_of_measurement\":\"";
        payload += unit;
        payload += "\"";
    }
    if (stateClass)
    {
        payload += ",\"state_class\":\"";
        payload += stateClass;
        payload += "\"";
    }
    payload += ",\"device\":{\"identifiers\":[\"clodagh_tracker_base\"],\"name\":\"Clodagh Tracker Base\",\"manufacturer\":\"LILYGO\",\"model\":\"T3-S3 SX1276\"}}";
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
}

void mqttPublishBaseSensorConfig(const char *objectId, const char *name, const char *stateTopic,
                                 const char *deviceClass = nullptr, const char *unit = nullptr,
                                 const char *stateClass = nullptr)
{
    String topic = String(MQTT_DISCOVERY_PREFIX) + "/sensor/clodagh_t3s3_base/" + objectId + "/config";
    String payload = "{";
    payload += "\"name\":\"";
    payload += name;
    payload += "\",\"unique_id\":\"clodagh_t3s3_base_";
    payload += objectId;
    payload += "\",\"state_topic\":\"";
    payload += stateTopic;
    payload += "\",\"availability_topic\":\"";
    payload += baseMqttTopic("availability");
    payload += "\",\"payload_available\":\"online\",\"payload_not_available\":\"offline\"";
    if (deviceClass)
    {
        payload += ",\"device_class\":\"";
        payload += deviceClass;
        payload += "\"";
    }
    if (unit)
    {
        payload += ",\"unit_of_measurement\":\"";
        payload += unit;
        payload += "\"";
    }
    if (stateClass)
    {
        payload += ",\"state_class\":\"";
        payload += stateClass;
        payload += "\"";
    }
    payload += ",\"device\":{\"identifiers\":[\"clodagh_t3s3_base\"],\"name\":\"Clodagh T3-S3 Base\",\"manufacturer\":\"LILYGO\",\"model\":\"T3-S3 SX1276\"}}";
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
}

void mqttPublishBinarySensorConfig(const char *objectId, const char *name, const char *stateTopic,
                                   const char *deviceClass = nullptr)
{
    String topic = String(MQTT_DISCOVERY_PREFIX) + "/binary_sensor/clodagh_tracker/" + objectId + "/config";
    String payload = "{";
    payload += "\"name\":\"";
    payload += name;
    payload += "\",\"unique_id\":\"clodagh_tracker_";
    payload += objectId;
    payload += "\",\"state_topic\":\"";
    payload += stateTopic;
    payload += "\",\"payload_on\":\"ON\",\"payload_off\":\"OFF\",\"availability_topic\":\"";
    payload += mqttTopic("availability");
    payload += "\"";
    if (deviceClass)
    {
        payload += ",\"device_class\":\"";
        payload += deviceClass;
        payload += "\"";
    }
    payload += ",\"device\":{\"identifiers\":[\"clodagh_tracker_base\"],\"name\":\"Clodagh Tracker Base\",\"manufacturer\":\"LILYGO\",\"model\":\"T3-S3 SX1276\"}}";
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
}

void mqttPublishDeviceTrackerConfig()
{
    String topic = String(MQTT_DISCOVERY_PREFIX) + "/device_tracker/clodagh_tracker/location/config";
    String payload = "{";
    payload += "\"name\":\"Clodagh Location\",";
    payload += "\"unique_id\":\"clodagh_tracker_location\",";
    payload += "\"source_type\":\"gps\",";
    payload += "\"json_attributes_topic\":\"";
    payload += mqttTopic("location");
    payload += "\",\"availability_topic\":\"";
    payload += mqttTopic("availability");
    payload += "\",\"payload_available\":\"online\",\"payload_not_available\":\"offline\",";
    payload += "\"device\":{\"identifiers\":[\"clodagh_tracker_base\"],\"name\":\"Clodagh Tracker Base\",\"manufacturer\":\"LILYGO\",\"model\":\"T3-S3 SX1276\"}}";
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
}

void mqttPublishBaseBinarySensorConfig(const char *objectId, const char *name, const char *stateTopic,
                                       const char *deviceClass = nullptr)
{
    String topic = String(MQTT_DISCOVERY_PREFIX) + "/binary_sensor/clodagh_t3s3_base/" + objectId + "/config";
    String payload = "{";
    payload += "\"name\":\"";
    payload += name;
    payload += "\",\"unique_id\":\"clodagh_t3s3_base_";
    payload += objectId;
    payload += "\",\"state_topic\":\"";
    payload += stateTopic;
    payload += "\",\"payload_on\":\"ON\",\"payload_off\":\"OFF\",\"availability_topic\":\"";
    payload += baseMqttTopic("availability");
    payload += "\"";
    if (deviceClass)
    {
        payload += ",\"device_class\":\"";
        payload += deviceClass;
        payload += "\"";
    }
    payload += ",\"device\":{\"identifiers\":[\"clodagh_t3s3_base\"],\"name\":\"Clodagh T3-S3 Base\",\"manufacturer\":\"LILYGO\",\"model\":\"T3-S3 SX1276\"}}";
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
}

void publishBaseStationDiscovery()
{
    mqttPublishBaseSensorConfig("ip_address", "Clodagh T3 Base IP", baseMqttTopic("ip_address").c_str());
    mqttPublishBaseSensorConfig("received_packets", "Clodagh T3 Base Packets", baseMqttTopic("received_packets").c_str(), nullptr, nullptr, "total_increasing");
    mqttPublishBaseSensorConfig("bad_packets", "Clodagh T3 Base Bad Packets", baseMqttTopic("bad_packets").c_str(), nullptr, nullptr, "total_increasing");
    mqttPublishBaseSensorConfig("radio_state", "Clodagh T3 Base Radio State", baseMqttTopic("radio_state").c_str());
    mqttPublishBaseSensorConfig("last_bad_reason", "Clodagh T3 Base Last Bad Reason", baseMqttTopic("last_bad_reason").c_str());
    mqttPublishBaseSensorConfig("last_bad_packet", "Clodagh T3 Base Last Bad Packet", baseMqttTopic("last_bad_packet").c_str());
    mqttPublishBaseSensorConfig("uptime", "Clodagh T3 Base Uptime", baseMqttTopic("uptime").c_str(), "duration", "s", "measurement");
    mqttPublishBaseSensorConfig("last_packet_age", "Clodagh T3 Base Last Packet Age", baseMqttTopic("last_packet_age").c_str(), "duration", "s", "measurement");
    mqttPublishBaseSensorConfig("last_rssi", "Clodagh T3 Base Last RSSI", baseMqttTopic("last_rssi").c_str(), "signal_strength", "dBm", "measurement");
    mqttPublishBaseSensorConfig("last_snr", "Clodagh T3 Base Last SNR", baseMqttTopic("last_snr").c_str(), nullptr, "dB", "measurement");
    mqttPublishBaseSensorConfig("last_update", "Clodagh T3 Base Last Update", baseMqttTopic("last_update").c_str(), "timestamp");
    mqttPublishBaseBinarySensorConfig("wifi", "Clodagh T3 Base WiFi", baseMqttTopic("wifi").c_str(), "connectivity");
    mqttPublishBaseBinarySensorConfig("lora_receiving", "Clodagh T3 Base LoRa Receiving", baseMqttTopic("lora_receiving").c_str(), "connectivity");

    mqttPublishBaseSensorConfig("tracker_dog_name", "Clodagh T3 Tracker Dog Name", baseMqttTopic("tracker_dog_name").c_str());
    mqttPublishBaseSensorConfig("tracker_event", "Clodagh T3 Tracker Event", baseMqttTopic("tracker_event").c_str());
    mqttPublishBaseSensorConfig("tracker_packet_counter", "Clodagh T3 Tracker Packet Counter", baseMqttTopic("tracker_packet_counter").c_str());
    mqttPublishBaseSensorConfig("tracker_battery_mv", "Clodagh T3 Tracker Battery", baseMqttTopic("tracker_battery_mv").c_str(), "voltage", "mV", "measurement");
    mqttPublishBaseSensorConfig("tracker_battery_percent", "Clodagh T3 Tracker Battery Percent", baseMqttTopic("tracker_battery_percent").c_str(), "battery", "%", "measurement");
    mqttPublishBaseSensorConfig("tracker_mode", "Clodagh T3 Tracker Mode", baseMqttTopic("tracker_mode").c_str());
    mqttPublishBaseSensorConfig("tracker_interval", "Clodagh T3 Tracker Interval", baseMqttTopic("tracker_interval").c_str(), nullptr, "s", "measurement");
    mqttPublishBaseSensorConfig("tracker_uptime", "Clodagh T3 Tracker Uptime", baseMqttTopic("tracker_uptime").c_str(), "duration", "s", "measurement");
    mqttPublishBaseSensorConfig("tracker_gps_chars", "Clodagh T3 Tracker GPS Characters", baseMqttTopic("tracker_gps_chars").c_str(), nullptr, nullptr, "total_increasing");
    mqttPublishBaseSensorConfig("tracker_gps_age", "Clodagh T3 Tracker GPS Age", baseMqttTopic("tracker_gps_age").c_str(), "duration", "ms", "measurement");
    mqttPublishBaseSensorConfig("tracker_gps_satellites", "Clodagh T3 Tracker GPS Satellites", baseMqttTopic("tracker_gps_satellites").c_str(), nullptr, nullptr, "measurement");
    mqttPublishBaseSensorConfig("tracker_gps_hdop", "Clodagh T3 Tracker GPS HDOP", baseMqttTopic("tracker_gps_hdop").c_str(), nullptr, nullptr, "measurement");
    mqttPublishBaseSensorConfig("tracker_latitude", "Clodagh T3 Tracker Latitude", baseMqttTopic("tracker_latitude").c_str());
    mqttPublishBaseSensorConfig("tracker_longitude", "Clodagh T3 Tracker Longitude", baseMqttTopic("tracker_longitude").c_str());
    mqttPublishBaseSensorConfig("tracker_last_seen", "Clodagh T3 Tracker Last Seen", baseMqttTopic("tracker_last_seen").c_str(), "timestamp");
    mqttPublishBaseSensorConfig("tracker_raw_packet", "Clodagh T3 Tracker Raw Packet", baseMqttTopic("tracker_raw_packet").c_str());
    mqttPublishBaseBinarySensorConfig("tracker_gps_fix", "Clodagh T3 Tracker GPS Fix", baseMqttTopic("tracker_gps_fix").c_str());
    mqttPublishBaseBinarySensorConfig("tracker_radio_ready", "Clodagh T3 Tracker Radio Ready", baseMqttTopic("tracker_radio_ready").c_str());
    mqttPublishBaseBinarySensorConfig("tracker_ble", "Clodagh T3 Tracker BLE Connected", baseMqttTopic("tracker_ble").c_str(), "connectivity");
    Serial.println("T3-S3 base Home Assistant MQTT discovery published");
}

void publishHomeAssistantDiscovery()
{
    mqttPublishSensorConfig("dog_name", "Clodagh Dog Name", mqttTopic("dog_name").c_str());
    mqttPublishSensorConfig("event", "Clodagh Event", mqttTopic("event").c_str());
    mqttPublishSensorConfig("packet_counter", "Clodagh Packet Counter", mqttTopic("packet_counter").c_str());
    mqttPublishSensorConfig("battery_mv", "Clodagh Battery", mqttTopic("battery_mv").c_str(), "voltage", "mV", "measurement");
    mqttPublishSensorConfig("battery_percent", "Clodagh Battery Percent", mqttTopic("battery_percent").c_str(), "battery", "%", "measurement");
    mqttPublishSensorConfig("tracker_mode", "Clodagh Tracker Mode", mqttTopic("tracker_mode").c_str());
    mqttPublishSensorConfig("tracker_interval", "Clodagh Tracker Interval", mqttTopic("tracker_interval").c_str(), nullptr, "s", "measurement");
    mqttPublishSensorConfig("tracker_uptime", "Clodagh Tracker Uptime", mqttTopic("tracker_uptime").c_str(), "duration", "s", "measurement");
    mqttPublishSensorConfig("tracker_last_tx_code", "Clodagh Last TX Code", mqttTopic("tracker_last_tx_code").c_str());
    mqttPublishSensorConfig("gps_chars", "Clodagh GPS Characters", mqttTopic("gps_chars").c_str(), nullptr, nullptr, "total_increasing");
    mqttPublishSensorConfig("gps_age", "Clodagh GPS Age", mqttTopic("gps_age").c_str(), "duration", "ms", "measurement");
    mqttPublishSensorConfig("gps_satellites", "Clodagh GPS Satellites", mqttTopic("gps_satellites").c_str(), nullptr, nullptr, "measurement");
    mqttPublishSensorConfig("gps_hdop", "Clodagh GPS HDOP", mqttTopic("gps_hdop").c_str(), nullptr, nullptr, "measurement");
    mqttPublishSensorConfig("latitude", "Clodagh Latitude", mqttTopic("latitude").c_str());
    mqttPublishSensorConfig("longitude", "Clodagh Longitude", mqttTopic("longitude").c_str());
    mqttPublishSensorConfig("rssi", "Clodagh LoRa RSSI", mqttTopic("rssi").c_str(), "signal_strength", "dBm", "measurement");
    mqttPublishSensorConfig("snr", "Clodagh LoRa SNR", mqttTopic("snr").c_str(), nullptr, "dB", "measurement");
    mqttPublishSensorConfig("last_seen", "Clodagh Last Seen", mqttTopic("last_seen").c_str(), "timestamp");
    mqttPublishSensorConfig("raw_packet", "Clodagh Raw Packet", mqttTopic("raw_packet").c_str());
    mqttPublishBinarySensorConfig("gps_fix", "Clodagh GPS Fix", mqttTopic("gps_fix").c_str());
    mqttPublishBinarySensorConfig("tracker_radio_ready", "Clodagh Tracker Radio Ready", mqttTopic("tracker_radio_ready").c_str());
    mqttPublishBinarySensorConfig("tracker_display", "Clodagh Tracker Display", mqttTopic("tracker_display").c_str());
    mqttPublishBinarySensorConfig("tracker_ble", "Clodagh Tracker BLE Connected", mqttTopic("tracker_ble").c_str(), "connectivity");
    mqttPublishDeviceTrackerConfig();
    mqttDiscoveryPublished = true;
    Serial.println("Home Assistant MQTT discovery published");
}

void publishBaseStationStatus()
{
    if (!mqttClient.connected())
    {
        return;
    }

    const uint32_t lastPacketAgeS = latestPacket.valid ? (millis() - latestPacket.receivedAtMs) / 1000 : 0;
    mqttPublishString(baseMqttTopic("availability"), "online", true);
    mqttPublishString(baseMqttTopic("ip_address"), ipText, true);
    mqttPublishString(baseMqttTopic("received_packets"), String(receivedPackets), true);
    mqttPublishString(baseMqttTopic("bad_packets"), String(badPackets), true);
    mqttPublishString(baseMqttTopic("radio_state"), String(lastRadioState), true);
    mqttPublishString(baseMqttTopic("last_bad_reason"), lastBadReason, true);
    mqttPublishString(baseMqttTopic("last_bad_packet"), printablePacket(lastBadPacket), true);
    mqttPublishString(baseMqttTopic("uptime"), String(millis() / 1000), true);
    mqttPublishString(baseMqttTopic("last_packet_age"), String(lastPacketAgeS), true);
    mqttPublishString(baseMqttTopic("last_rssi"), latestPacket.valid ? String(latestPacket.rssi) : "0", true);
    mqttPublishString(baseMqttTopic("last_snr"), latestPacket.valid ? String(latestPacket.snr, 1) : "0.0", true);
    mqttPublishString(baseMqttTopic("last_update"), currentIsoTimestamp(), true);
    mqttPublishString(baseMqttTopic("wifi"), WiFi.status() == WL_CONNECTED ? "ON" : "OFF", true);
    mqttPublishString(baseMqttTopic("lora_receiving"), lastRadioState == RADIOLIB_ERR_NONE ? "ON" : "OFF", true);
}

void publishPacketToMqtt(const CollarPacket &packet)
{
    if (!mqttClient.connected())
    {
        return;
    }

    mqttPublishString(mqttTopic("dog_name"), packet.dogName, true);
    mqttPublishString(mqttTopic("event"), packet.event, true);
    mqttPublishString(mqttTopic("packet_counter"), String(packet.counter), true);
    mqttPublishString(mqttTopic("gps_fix"), packet.gpsFix ? "ON" : "OFF", true);
    mqttPublishString(mqttTopic("latitude"), packet.latitude, true);
    mqttPublishString(mqttTopic("longitude"), packet.longitude, true);
    mqttPublishString(mqttTopic("battery_mv"), String(packet.batteryMv), true);
    mqttPublishString(mqttTopic("battery_percent"), String(packet.batteryPercent), true);
    mqttPublishString(mqttTopic("tracker_mode"), packet.trackerMode, true);
    mqttPublishString(mqttTopic("tracker_interval"), String(packet.intervalS), true);
    mqttPublishString(mqttTopic("tracker_uptime"), String(packet.trackerUptimeS), true);
    mqttPublishString(mqttTopic("tracker_radio_ready"), packet.trackerRadioReady ? "ON" : "OFF", true);
    mqttPublishString(mqttTopic("tracker_last_tx_code"), String(packet.trackerLastTxCode), true);
    mqttPublishString(mqttTopic("gps_chars"), String(packet.gpsChars), true);
    mqttPublishString(mqttTopic("gps_age"), String(packet.gpsAgeMs), true);
    mqttPublishString(mqttTopic("gps_satellites"), String(packet.gpsSatellites), true);
    mqttPublishString(mqttTopic("gps_hdop"), packet.gpsHdop, true);
    mqttPublishString(mqttTopic("tracker_display"), packet.trackerDisplayOn ? "ON" : "OFF", true);
    mqttPublishString(mqttTopic("tracker_ble"), packet.trackerBleConnected ? "ON" : "OFF", true);
    mqttPublishString(mqttTopic("rssi"), String(packet.rssi), true);
    mqttPublishString(mqttTopic("snr"), String(packet.snr, 1), true);
    const String lastSeen = currentIsoTimestamp();
    mqttPublishString(mqttTopic("last_seen"), lastSeen, true);
    mqttPublishString(mqttTopic("raw_packet"), packet.raw, true);

    mqttPublishString(baseMqttTopic("tracker_dog_name"), packet.dogName, true);
    mqttPublishString(baseMqttTopic("tracker_event"), packet.event, true);
    mqttPublishString(baseMqttTopic("tracker_packet_counter"), String(packet.counter), true);
    mqttPublishString(baseMqttTopic("tracker_battery_mv"), String(packet.batteryMv), true);
    mqttPublishString(baseMqttTopic("tracker_battery_percent"), String(packet.batteryPercent), true);
    mqttPublishString(baseMqttTopic("tracker_mode"), packet.trackerMode, true);
    mqttPublishString(baseMqttTopic("tracker_interval"), String(packet.intervalS), true);
    mqttPublishString(baseMqttTopic("tracker_uptime"), String(packet.trackerUptimeS), true);
    mqttPublishString(baseMqttTopic("tracker_gps_chars"), String(packet.gpsChars), true);
    mqttPublishString(baseMqttTopic("tracker_gps_age"), String(packet.gpsAgeMs), true);
    mqttPublishString(baseMqttTopic("tracker_gps_satellites"), String(packet.gpsSatellites), true);
    mqttPublishString(baseMqttTopic("tracker_gps_hdop"), packet.gpsHdop, true);
    mqttPublishString(baseMqttTopic("tracker_latitude"), packet.latitude, true);
    mqttPublishString(baseMqttTopic("tracker_longitude"), packet.longitude, true);
    mqttPublishString(baseMqttTopic("tracker_last_seen"), lastSeen, true);
    mqttPublishString(baseMqttTopic("tracker_raw_packet"), packet.raw, true);
    mqttPublishString(baseMqttTopic("tracker_gps_fix"), packet.gpsFix ? "ON" : "OFF", true);
    mqttPublishString(baseMqttTopic("tracker_radio_ready"), packet.trackerRadioReady ? "ON" : "OFF", true);
    mqttPublishString(baseMqttTopic("tracker_ble"), packet.trackerBleConnected ? "ON" : "OFF", true);

    if (packet.gpsFix)
    {
        String locationJson = "{";
        locationJson += "\"latitude\":" + packet.latitude + ",";
        locationJson += "\"longitude\":" + packet.longitude + ",";
        locationJson += "\"altitude\":" + packet.altitudeM + ",";
        locationJson += "\"gps_accuracy\":25,";
        locationJson += "\"gps_age_ms\":" + String(packet.gpsAgeMs) + ",";
        locationJson += "\"gps_satellites\":" + String(packet.gpsSatellites) + ",";
        locationJson += "\"gps_hdop\":\"" + jsonEscape(packet.gpsHdop) + "\",";
        locationJson += "\"battery\":" + String(packet.batteryPercent) + ",";
        locationJson += "\"source_type\":\"gps\",";
        locationJson += "\"last_seen\":\"" + jsonEscape(lastSeen) + "\",";
        locationJson += "\"packet_counter\":" + String(packet.counter) + ",";
        locationJson += "\"rssi\":" + String(packet.rssi) + ",";
        locationJson += "\"snr\":" + String(packet.snr, 1);
        locationJson += "}";
        mqttPublishString(mqttTopic("location"), locationJson, true);
    }

    String json = "{";
    json += "\"dog_name\":\"" + jsonEscape(packet.dogName) + "\",";
    json += "\"device_id\":\"" + jsonEscape(packet.deviceId) + "\",";
    json += "\"event\":\"" + jsonEscape(packet.event) + "\",";
    json += "\"counter\":" + String(packet.counter) + ",";
    json += "\"gps_fix\":";
    json += packet.gpsFix ? "true" : "false";
    json += ",\"latitude\":\"" + jsonEscape(packet.latitude) + "\",";
    json += "\"longitude\":\"" + jsonEscape(packet.longitude) + "\",";
    json += "\"battery_mv\":" + String(packet.batteryMv) + ",";
    json += "\"battery_percent\":" + String(packet.batteryPercent) + ",";
    json += "\"tracker_mode\":\"" + jsonEscape(packet.trackerMode) + "\",";
    json += "\"tracker_interval_s\":" + String(packet.intervalS) + ",";
    json += "\"tracker_uptime_s\":" + String(packet.trackerUptimeS) + ",";
    json += "\"tracker_radio_ready\":";
    json += packet.trackerRadioReady ? "true" : "false";
    json += ",\"tracker_last_tx_code\":" + String(packet.trackerLastTxCode) + ",";
    json += "\"gps_chars\":" + String(packet.gpsChars) + ",";
    json += "\"gps_age_ms\":" + String(packet.gpsAgeMs) + ",";
    json += "\"gps_satellites\":" + String(packet.gpsSatellites) + ",";
    json += "\"gps_hdop\":\"" + jsonEscape(packet.gpsHdop) + "\",";
    json += "\"tracker_display_on\":";
    json += packet.trackerDisplayOn ? "true" : "false";
    json += ",\"tracker_ble_connected\":";
    json += packet.trackerBleConnected ? "true" : "false";
    json += ",";
    json += "\"rssi\":" + String(packet.rssi) + ",";
    json += "\"snr\":" + String(packet.snr, 1) + ",";
    json += "\"last_seen\":\"" + jsonEscape(lastSeen) + "\"";
    json += "}";
    mqttPublishString(mqttTopic("state"), json, true);
}

void handleMqtt()
{
    if (!wifiStationMode || WiFi.status() != WL_CONNECTED)
    {
        return;
    }

    if (!mqttClient.connected())
    {
        if (millis() - mqttLastConnectAttempt < 5000)
        {
            return;
        }
        mqttLastConnectAttempt = millis();

        Serial.print("Connecting to MQTT ");
        Serial.print(MQTT_HOST);
        Serial.print(":");
        Serial.println(MQTT_PORT);

        bool connected = false;
        if (strlen(MQTT_USER) > 0)
        {
            connected = mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD,
                                           mqttTopic("availability").c_str(), 0, true, "offline");
        }
        else
        {
            connected = mqttClient.connect(MQTT_CLIENT_ID, mqttTopic("availability").c_str(), 0, true, "offline");
        }

        if (connected)
        {
            Serial.println("MQTT connected");
            mqttClient.publish(mqttTopic("availability").c_str(), "online", true);
            mqttClient.publish(baseMqttTopic("availability").c_str(), "online", true);
            publishHomeAssistantDiscovery();
            publishBaseStationDiscovery();
            publishBaseStationStatus();
            if (latestPacket.valid)
            {
                publishPacketToMqtt(latestPacket);
            }
        }
        else
        {
            Serial.print("MQTT failed, rc=");
            Serial.println(mqttClient.state());
        }
    }

    mqttClient.loop();

    if (millis() - mqttLastHeartbeat > 30000)
    {
        mqttLastHeartbeat = millis();
        mqttPublishString(mqttTopic("availability"), "online", true);
        publishBaseStationStatus();
    }
}

void startFallbackAp()
{
    if (fallbackApStarted)
    {
        return;
    }

    WiFi.softAP(SETUP_AP_NAME);
    fallbackApStarted = true;
    Serial.print("Fallback AP available: ");
    Serial.print(SETUP_AP_NAME);
    Serial.print(" ");
    Serial.println(WiFi.softAPIP());
}

void startTimeSync()
{
    static bool configured = false;
    if (configured)
    {
        return;
    }

    configTzTime(TIME_ZONE, NTP_SERVER_1, NTP_SERVER_2);
    configured = true;
    Serial.println("NTP time sync started");
}

void updateWiFiStatus()
{
    const bool connected = WiFi.status() == WL_CONNECTED;
    if (connected)
    {
        const String newIp = WiFi.localIP().toString();
        if (!wifiStationMode || ipText != newIp)
        {
            wifiStationMode = true;
            ipText = newIp;
            Serial.print("WiFi connected: ");
            Serial.println(ipText);
            startTimeSync();
            drawOled();
        }
        return;
    }

    if (wifiStationMode)
    {
        mqttClient.disconnect();
        mqttDiscoveryPublished = false;
        Serial.println("WiFi disconnected; fallback AP remains available");
    }

    wifiStationMode = false;
    ipText = fallbackApStarted ? WiFi.softAPIP().toString() : "0.0.0.0";
}

void handleWiFi()
{
    if (!wifiHasCredentials)
    {
        return;
    }

    if (millis() - wifiLastStatusCheck > 2000)
    {
        wifiLastStatusCheck = millis();
        updateWiFiStatus();
    }

    if (WiFi.status() != WL_CONNECTED && millis() - wifiLastRetry > 30000)
    {
        wifiLastRetry = millis();
        Serial.print("Retrying WiFi SSID ");
        Serial.println(WIFI_SSID);
        WiFi.disconnect(false, false);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
}

void drawOled()
{
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("Clodagh T3 Base");

    display.setCursor(0, 10);
    display.print(wifiStationMode ? "WiFi " : "AP ");
    display.print(ipText);
    display.print(mqttClient.connected() ? " M" : "");

    display.setCursor(0, 22);
    if (latestPacket.valid)
    {
        display.print(latestPacket.dogName);
        display.print(" #");
        display.print(latestPacket.counter);

        display.setCursor(0, 34);
        display.print(latestPacket.event);
        display.print(latestPacket.gpsFix ? " FIX" : " NO GPS");

        display.setCursor(0, 46);
        display.print(latestPacket.batteryMv);
        display.print("mV R");
        display.print(latestPacket.rssi);
        display.print(" S");
        display.print(latestPacket.snr, 1);
    }
    else
    {
        display.print("Waiting for LoRa...");
        display.setCursor(0, 34);
        display.print("RX ");
        display.print(receivedPackets);
        display.print(" bad ");
        display.print(badPackets);
        display.setCursor(0, 46);
        display.print("Radio ");
        display.print(lastRadioState);
    }

    display.display();
}

String htmlPage()
{
    String html = "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
    html += "<meta http-equiv='refresh' content='5'><title>Clodagh Base</title>";
    html += "<style>body{font-family:Arial,sans-serif;margin:24px;line-height:1.35}code{background:#eee;padding:2px 4px}.ok{color:#087a28}.bad{color:#b00020}</style>";
    html += "</head><body><h1>Clodagh Base Station</h1>";
    html += "<p><b>Mode:</b> ";
    html += wifiStationMode ? "Home Wi-Fi" : "Setup AP";
    html += " <b>IP:</b> ";
    html += ipText;
    html += "</p><p><b>Packets:</b> ";
    html += receivedPackets;
    html += " <b>Bad:</b> ";
    html += badPackets;
    html += " <b>Radio:</b> ";
    html += lastRadioState;
    html += " <b>Bad reason:</b> ";
    html += lastBadReason;
    html += "</p><p><a href='/update'>Firmware update</a></p>";

    if (latestPacket.valid)
    {
        html += "<h2>Latest Packet</h2><p class='ok'>CRC OK</p>";
        html += "<p><b>Dog:</b> " + latestPacket.dogName + "<br>";
        html += "<b>Event:</b> " + latestPacket.event + "<br>";
        html += "<b>Counter:</b> " + String(latestPacket.counter) + "<br>";
        html += "<b>GPS:</b> ";
        html += latestPacket.gpsFix ? "fix" : "no fix";
        html += "<br><b>Lat/Lon:</b> " + latestPacket.latitude + ", " + latestPacket.longitude + "<br>";
        html += "<b>Battery:</b> " + String(latestPacket.batteryMv) + " mV<br>";
        html += "<b>RSSI/SNR:</b> " + String(latestPacket.rssi) + " dBm / " + String(latestPacket.snr, 1) + " dB<br>";
        html += "<b>Age:</b> " + String((millis() - latestPacket.receivedAtMs) / 1000) + " s</p>";
        html += "<p><b>Raw:</b><br><code>" + latestPacket.raw + "</code></p>";
    }
    else
    {
        html += "<h2>No collar packet received yet</h2>";
        html += "<p><b>Last bad reason:</b> " + lastBadReason + "<br>";
        html += "<b>Last bad RSSI/SNR:</b> " + String(lastBadRssi) + " dBm / " + String(lastBadSnr, 1) + " dB</p>";
        if (lastBadPacket.length() > 0)
        {
            html += "<p><b>Last bad packet:</b><br><code>" + printablePacket(lastBadPacket) + "</code></p>";
        }
    }

    html += "</body></html>";
    return html;
}

bool otaAuthenticate()
{
    if (server.authenticate(OTA_USER, OTA_PASSWORD))
    {
        return true;
    }
    server.requestAuthentication();
    return false;
}

String otaPage()
{
    String html = "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
    html += "<title>Clodagh Base OTA</title>";
    html += "<style>body{font-family:Arial,sans-serif;margin:24px;line-height:1.35}input,button{font-size:16px;margin-top:12px}</style>";
    html += "</head><body><h1>Clodagh Base Firmware Update</h1>";
    html += "<p>Select the PlatformIO firmware file named <code>firmware.bin</code>.</p>";
    html += "<form method='POST' action='/update' enctype='multipart/form-data'>";
    html += "<input type='file' name='firmware' accept='.bin' required><br>";
    html += "<button type='submit'>Upload firmware</button>";
    html += "</form><p><a href='/'>Back</a></p></body></html>";
    return html;
}

void startWebServer()
{
    server.on("/", []()
              { server.send(200, "text/html", htmlPage()); });
    server.on("/raw", []()
              { server.send(200, "text/plain", latestPacket.raw.length() ? latestPacket.raw : "no packet"); });
    server.on("/update", HTTP_GET, []()
              {
                  if (!otaAuthenticate())
                  {
                      return;
                  }
                  server.send(200, "text/html", otaPage());
              });
    server.on("/update", HTTP_POST, []()
              {
                  if (!otaAuthenticate())
                  {
                      return;
                  }
                  const bool ok = !Update.hasError();
                  server.send(200, "text/plain", ok ? "Update complete. Rebooting..." : "Update failed.");
                  delay(500);
                  if (ok)
                  {
                      ESP.restart();
                  }
              },
              []()
              {
                  if (!otaAuthenticate())
                  {
                      return;
                  }
                  HTTPUpload &upload = server.upload();
                  if (upload.status == UPLOAD_FILE_START)
                  {
                      Serial.print("OTA upload start: ");
                      Serial.println(upload.filename);
                      mqttPublishString(mqttTopic("availability"), "offline", true);
                      mqttPublishString(baseMqttTopic("availability"), "offline", true);
                      radio.standby();
                      if (!Update.begin(UPDATE_SIZE_UNKNOWN))
                      {
                          Update.printError(Serial);
                      }
                  }
                  else if (upload.status == UPLOAD_FILE_WRITE)
                  {
                      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
                      {
                          Update.printError(Serial);
                      }
                  }
                  else if (upload.status == UPLOAD_FILE_END)
                  {
                      if (Update.end(true))
                      {
                          Serial.print("OTA upload complete, bytes: ");
                          Serial.println(upload.totalSize);
                      }
                      else
                      {
                          Update.printError(Serial);
                      }
                  }
              });
    server.begin();
}

void startWiFi()
{
    wifiHasCredentials = String(WIFI_SSID) != "CHANGE_ME" && strlen(WIFI_SSID) > 0;
    WiFi.persistent(false);
    WiFi.setSleep(false);

    if (wifiHasCredentials)
    {
        WiFi.mode(WIFI_AP_STA);
        startFallbackAp();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        Serial.print("Connecting to WiFi");
        for (uint8_t i = 0; i < 60 && WiFi.status() != WL_CONNECTED; i++)
        {
            Serial.print(".");
            delay(500);
        }
        Serial.println();

        if (WiFi.status() == WL_CONNECTED)
        {
            updateWiFiStatus();
            return;
        }

        Serial.println("WiFi not connected yet; continuing in AP+STA retry mode");
        updateWiFiStatus();
        return;
    }

    WiFi.mode(WIFI_AP);
    startFallbackAp();
    wifiStationMode = false;
    ipText = WiFi.softAPIP().toString();
}

void startDisplay()
{
    Wire.begin(OLED_SDA, OLED_SCL);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS))
    {
        Serial.println("OLED init failed");
        return;
    }
    display.clearDisplay();
    display.display();
}

void startRadio()
{
    pinMode(USER_LED, OUTPUT);
    digitalWrite(USER_LED, LOW);

    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    Serial.print("SX1276 initializing... ");
    lastRadioState = radio.begin(LORA_FREQUENCY_MHZ);
    if (lastRadioState == RADIOLIB_ERR_NONE)
    {
        lastRadioState = radio.setBandwidth(LORA_BANDWIDTH_KHZ);
    }
    if (lastRadioState == RADIOLIB_ERR_NONE)
    {
        lastRadioState = radio.setSpreadingFactor(LORA_SPREADING_FACTOR);
    }
    if (lastRadioState == RADIOLIB_ERR_NONE)
    {
        lastRadioState = radio.setCodingRate(LORA_CODING_RATE);
    }
    if (lastRadioState == RADIOLIB_ERR_NONE)
    {
        lastRadioState = radio.setSyncWord(LORA_SYNC_WORD);
    }
    if (lastRadioState == RADIOLIB_ERR_NONE)
    {
        lastRadioState = radio.setPreambleLength(LORA_PREAMBLE_LENGTH);
    }
    if (lastRadioState == RADIOLIB_ERR_NONE)
    {
        // The tracker packet has its own CRC-16 after the asterisk.
        lastRadioState = radio.setCRC(false);
    }
    if (lastRadioState == RADIOLIB_ERR_NONE)
    {
        radio.setPacketReceivedAction(setRadioPacketFlag);
        radioPacketFlag = false;
        lastRadioState = radio.startReceive();
    }

    Serial.println(lastRadioState);
    if (lastRadioState != RADIOLIB_ERR_NONE)
    {
        Serial.print("SX1276 init failed: ");
        Serial.println(lastRadioState);
        return;
    }

    Serial.println("SX1276 listening for collar packets");
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println();
    Serial.println("LilyGO T3-S3 Clodagh main base station");

    startDisplay();
    drawOled();
    startWiFi();
    startWebServer();
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setBufferSize(1024);
    startRadio();
    drawOled();
}

void handleRadio()
{
    const uint32_t now = millis();

    if (lastRadioState != RADIOLIB_ERR_NONE)
    {
        if (now - radioLastMaintenanceMs > 30000)
        {
            radioLastMaintenanceMs = now;
            Serial.println("LoRa radio not ready, restarting");
            startRadio();
            drawOled();
        }
        return;
    }

    if (radioPacketFlag)
    {
        radioPacketFlag = false;
        String packet;
        const int state = radio.readData(packet);
        if (state == RADIOLIB_ERR_NONE)
        {
            receivedPackets++;
            CollarPacket parsed;
            if (parsePacket(packet, parsed))
            {
                latestPacket = parsed;
                Serial.print("RX OK: ");
                Serial.println(packet);
                publishPacketToMqtt(latestPacket);
                publishBaseStationStatus();
                digitalWrite(USER_LED, !digitalRead(USER_LED));
            }
            else
            {
                badPackets++;
                lastBadPacket = packet;
                lastBadReason = badPacketReason(packet);
                lastBadRssi = (int16_t)radio.getRSSI();
                lastBadSnr = radio.getSNR();
                Serial.print("RX bad: ");
                Serial.print(lastBadReason);
                Serial.print(" RSSI=");
                Serial.print(lastBadRssi);
                Serial.print(" SNR=");
                Serial.print(lastBadSnr, 1);
                Serial.print(" DATA=");
                Serial.println(printablePacket(packet));
                publishBaseStationStatus();
            }
            drawOled();
        }
        else
        {
            Serial.print("LoRa read error: ");
            Serial.println(state);
        }

        lastRadioState = radio.startReceive();
        if (lastRadioState != RADIOLIB_ERR_NONE)
        {
            Serial.print("LoRa restart receive failed: ");
            Serial.println(lastRadioState);
        }
    }

    if (now - radioLastTimeoutLogMs > 30000)
    {
        radioLastTimeoutLogMs = now;
        Serial.print("LoRa waiting, packets=");
        Serial.print(receivedPackets);
        Serial.print(" bad=");
        Serial.println(badPackets);
    }

    const bool noPacketYet = !latestPacket.valid;
    const bool stalePacket = latestPacket.valid && (now - latestPacket.receivedAtMs > 180000);
    if ((noPacketYet || stalePacket) && now - radioLastMaintenanceMs > 180000)
    {
        radioLastMaintenanceMs = now;
        Serial.println("LoRa maintenance restart");
        startRadio();
        drawOled();
    }
}

void loop()
{
    server.handleClient();
    handleWiFi();
    handleMqtt();
    handleRadio();

    static uint32_t lastOledRefresh = 0;
    if (millis() - lastOledRefresh > 5000)
    {
        lastOledRefresh = millis();
        drawOled();
    }
}
