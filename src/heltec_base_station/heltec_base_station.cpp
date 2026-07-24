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
static const char *SETUP_AP_NAME = "Clodagh-Base";

// MQTT/Home Assistant settings. If your broker is not on homeassistant.local,
// change MQTT_HOST to the Home Assistant or Mosquitto broker IP address.
static const char *MQTT_HOST = BASE_MQTT_HOST;
static const uint16_t MQTT_PORT = 1883;
static const char *MQTT_USER = BASE_MQTT_USER;
static const char *MQTT_PASSWORD = BASE_MQTT_PASSWORD;
static const char *MQTT_CLIENT_ID = "clodagh_heltec_base";
static const char *MQTT_BASE_TOPIC = "clodagh_base";
static const char *MQTT_TRACKER_1_TOPIC = "clodagh_tracker_1";
static const char *MQTT_TRACKER_2_TOPIC = "clodagh_tracker_2";
static const char *MQTT_DISCOVERY_PREFIX = "homeassistant";

// Known collar IDs. These let Home Assistant show each collar as its own tidy
// device instead of mixing all received packets into the base station device.
static const char *TRACKER_1_DEVICE_ID = "8193456441F2C48E"; // Current Clodagh-Test collar.
static const char *TRACKER_2_DEVICE_ID = "9DFA83D5A49304E0"; // Original Clodagh collar.

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
static const float LORA_TCXO_VOLTAGE = 1.8;
static const uint32_t GPS_MIN_SATELLITES = 6;
static const float GPS_MAX_HDOP = 2.00f;

// Heltec WiFi LoRa 32 V3 pins from Heltec board definitions.
static const int LORA_NSS = 8;
static const int LORA_DIO1 = 14;
static const int LORA_RESET = 12;
static const int LORA_BUSY = 13;
static const int LORA_SCK = 9;
static const int LORA_MISO = 11;
static const int LORA_MOSI = 10;

static const int OLED_SDA = 17;
static const int OLED_SCL = 18;
static const int OLED_RST = 21;
static const int VEXT_ENABLE = 36; // LOW enables external/OLED power on Heltec V3.

static const int SCREEN_WIDTH = 128;
static const int SCREEN_HEIGHT = 64;
static const int OLED_ADDRESS = 0x3C;

SPIClass loraSpi(FSPI);
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RESET, LORA_BUSY, loraSpi);
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
bool wifiStationMode = false;
String ipText = "0.0.0.0";
bool mqttDiscoveryPublished = false;
uint32_t mqttLastConnectAttempt = 0;
uint32_t mqttLastHeartbeat = 0;
bool wifiHasCredentials = false;
bool fallbackApStarted = false;
uint32_t wifiLastRetry = 0;
uint32_t wifiLastStatusCheck = 0;
String pendingTrackerMode[2] = {"", ""};
uint32_t pendingCommandCounter[2] = {0, 0};
uint32_t commandCounter = 0;

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

String trackerTopic(uint8_t slot, const String &name)
{
    return String(slot == 2 ? MQTT_TRACKER_2_TOPIC : MQTT_TRACKER_1_TOPIC) + "/" + name;
}

uint8_t trackerSlotForPacket(const CollarPacket &packet)
{
    if (packet.deviceId == TRACKER_2_DEVICE_ID)
    {
        return 2;
    }
    return 1;
}

String trackerDeviceId(uint8_t slot)
{
    return slot == 2 ? String("clodagh_tracker_2") : String("clodagh_tracker_1");
}

String trackerName(uint8_t slot)
{
    return slot == 2 ? String("Clodagh Tracker 2") : String("Clodagh Tracker 1");
}

const char *trackerKnownDeviceId(uint8_t slot)
{
    return slot == 2 ? TRACKER_2_DEVICE_ID : TRACKER_1_DEVICE_ID;
}

bool validModeName(const String &mode)
{
    return mode == "HOME" || mode == "MONITORING" || mode == "ACTIVITY" ||
           mode == "SEARCH" || mode == "EMERGENCY";
}

String buildCommandPacket(uint8_t slot)
{
    String body = "CMD1,";
    body += trackerKnownDeviceId(slot);
    body += ",";
    body += pendingTrackerMode[slot - 1];
    body += ",";
    body += String(pendingCommandCounter[slot - 1]);

    char crcText[8];
    snprintf(crcText, sizeof(crcText), "%04X", crc16Ccitt(body));
    return body + "*" + crcText;
}

bool packetHasAccurateGps(const CollarPacket &packet)
{
    return packet.gpsFix && packet.gpsSatellites >= GPS_MIN_SATELLITES &&
           packet.gpsHdop.toFloat() > 0.0f && packet.gpsHdop.toFloat() <= GPS_MAX_HDOP;
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

String deviceJson(const String &identifier, const String &name, const String &manufacturer, const String &model)
{
    String payload = "\"device\":{\"identifiers\":[\"";
    payload += identifier;
    payload += "\"],\"name\":\"";
    payload += name;
    payload += "\",\"manufacturer\":\"";
    payload += manufacturer;
    payload += "\",\"model\":\"";
    payload += model;
    payload += "\"}";
    return payload;
}

void mqttPublishSensorConfig(const String &componentId, const String &objectId, const String &name,
                             const String &stateTopic, const String &availabilityTopic,
                             const String &deviceIdentifier, const String &deviceName,
                             const char *deviceClass = nullptr, const char *unit = nullptr,
                             const char *stateClass = nullptr)
{
    String topic = String(MQTT_DISCOVERY_PREFIX) + "/sensor/" + componentId + "/" + objectId + "/config";
    String payload = "{";
    payload += "\"name\":\"";
    payload += name;
    payload += "\",\"unique_id\":\"";
    payload += componentId;
    payload += "_";
    payload += objectId;
    payload += "\",\"state_topic\":\"";
    payload += stateTopic;
    payload += "\",\"availability_topic\":\"";
    payload += availabilityTopic;
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
    payload += ",";
    payload += deviceJson(deviceIdentifier, deviceName, deviceIdentifier == "clodagh_base" ? "Heltec" : "LILYGO",
                          deviceIdentifier == "clodagh_base" ? "WiFi LoRa 32 V3" : "T-Echo Lite");
    payload += "}";
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
}

void mqttPublishBinarySensorConfig(const String &componentId, const String &objectId, const String &name,
                                   const String &stateTopic, const String &availabilityTopic,
                                   const String &deviceIdentifier, const String &deviceName,
                                   const char *deviceClass = nullptr)
{
    String topic = String(MQTT_DISCOVERY_PREFIX) + "/binary_sensor/" + componentId + "/" + objectId + "/config";
    String payload = "{";
    payload += "\"name\":\"";
    payload += name;
    payload += "\",\"unique_id\":\"";
    payload += componentId;
    payload += "_";
    payload += objectId;
    payload += "\",\"state_topic\":\"";
    payload += stateTopic;
    payload += "\",\"payload_on\":\"ON\",\"payload_off\":\"OFF\",\"availability_topic\":\"";
    payload += availabilityTopic;
    payload += "\"";
    if (deviceClass)
    {
        payload += ",\"device_class\":\"";
        payload += deviceClass;
        payload += "\"";
    }
    payload += ",";
    payload += deviceJson(deviceIdentifier, deviceName, deviceIdentifier == "clodagh_base" ? "Heltec" : "LILYGO",
                          deviceIdentifier == "clodagh_base" ? "WiFi LoRa 32 V3" : "T-Echo Lite");
    payload += "}";
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
}

void mqttPublishDeviceTrackerConfig(uint8_t slot)
{
    const String componentId = trackerDeviceId(slot);
    const String displayName = trackerName(slot);
    String topic = String(MQTT_DISCOVERY_PREFIX) + "/device_tracker/" + componentId + "/location/config";
    String payload = "{";
    payload += "\"name\":\"Location\",";
    payload += "\"unique_id\":\"" + componentId + "_location\",";
    payload += "\"source_type\":\"gps\",";
    payload += "\"json_attributes_topic\":\"";
    payload += trackerTopic(slot, "location");
    payload += "\",\"availability_topic\":\"";
    payload += trackerTopic(slot, "availability");
    payload += "\",\"payload_available\":\"online\",\"payload_not_available\":\"offline\",";
    payload += deviceJson(componentId, displayName, "LILYGO", "T-Echo Lite");
    payload += "}";
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
}

void mqttPublishModeSelectConfig(uint8_t slot)
{
    const String componentId = trackerDeviceId(slot);
    const String displayName = trackerName(slot);
    String topic = String(MQTT_DISCOVERY_PREFIX) + "/select/" + componentId + "/mode_command/config";
    String payload = "{";
    payload += "\"name\":\"Mode Command\",";
    payload += "\"unique_id\":\"" + componentId + "_mode_command\",";
    payload += "\"command_topic\":\"" + trackerTopic(slot, "mode/set") + "\",";
    payload += "\"state_topic\":\"" + trackerTopic(slot, "mode") + "\",";
    payload += "\"availability_topic\":\"" + trackerTopic(slot, "availability") + "\",";
    payload += "\"payload_available\":\"online\",\"payload_not_available\":\"offline\",";
    payload += "\"options\":[\"HOME\",\"MONITORING\",\"ACTIVITY\",\"SEARCH\",\"EMERGENCY\"],";
    payload += deviceJson(componentId, displayName, "LILYGO", "T-Echo Lite");
    payload += "}";
    mqttClient.publish(topic.c_str(), payload.c_str(), true);
}

void publishHomeAssistantDiscovery()
{
    const String baseId = "clodagh_base";
    const String baseAvailability = mqttTopic("availability");
    mqttPublishBinarySensorConfig(baseId, "receiving_packets", "Receiving Packets", mqttTopic("receiving_packets"),
                                  baseAvailability, baseId, "Clodagh Base Station", "connectivity");
    mqttPublishSensorConfig(baseId, "received_packets", "Received Packets", mqttTopic("received_packets"),
                            baseAvailability, baseId, "Clodagh Base Station", nullptr, nullptr, "total_increasing");
    mqttPublishSensorConfig(baseId, "last_packet_age", "Last Packet Age", mqttTopic("last_packet_age"),
                            baseAvailability, baseId, "Clodagh Base Station", "duration", "s", "measurement");
    mqttPublishSensorConfig(baseId, "ip_address", "IP Address", mqttTopic("ip_address"),
                            baseAvailability, baseId, "Clodagh Base Station");

    for (uint8_t slot = 1; slot <= 2; slot++)
    {
        const String componentId = trackerDeviceId(slot);
        const String displayName = trackerName(slot);
        const String availability = trackerTopic(slot, "availability");
        mqttPublishSensorConfig(componentId, "dog_name", "Dog Name", trackerTopic(slot, "dog_name"),
                                availability, componentId, displayName);
        mqttPublishSensorConfig(componentId, "device_id", "Device ID", trackerTopic(slot, "device_id"),
                                availability, componentId, displayName);
        mqttPublishSensorConfig(componentId, "packet_counter", "Packet Counter", trackerTopic(slot, "packet_counter"),
                                availability, componentId, displayName);
        mqttPublishSensorConfig(componentId, "battery_mv", "Battery Voltage", trackerTopic(slot, "battery_mv"),
                                availability, componentId, displayName, "voltage", "mV", "measurement");
        mqttPublishSensorConfig(componentId, "battery_percent", "Battery", trackerTopic(slot, "battery_percent"),
                                availability, componentId, displayName, "battery", "%", "measurement");
        mqttPublishSensorConfig(componentId, "gps_satellites", "GPS Satellites", trackerTopic(slot, "gps_satellites"),
                                availability, componentId, displayName, nullptr, nullptr, "measurement");
        mqttPublishSensorConfig(componentId, "gps_hdop", "GPS HDOP", trackerTopic(slot, "gps_hdop"),
                                availability, componentId, displayName, nullptr, nullptr, "measurement");
        mqttPublishSensorConfig(componentId, "gps_age", "GPS Age", trackerTopic(slot, "gps_age"),
                                availability, componentId, displayName, "duration", "ms", "measurement");
        mqttPublishSensorConfig(componentId, "latitude", "Latitude", trackerTopic(slot, "latitude"),
                                availability, componentId, displayName);
        mqttPublishSensorConfig(componentId, "longitude", "Longitude", trackerTopic(slot, "longitude"),
                                availability, componentId, displayName);
        mqttPublishSensorConfig(componentId, "rssi", "LoRa RSSI", trackerTopic(slot, "rssi"),
                                availability, componentId, displayName, "signal_strength", "dBm", "measurement");
        mqttPublishSensorConfig(componentId, "snr", "LoRa SNR", trackerTopic(slot, "snr"),
                                availability, componentId, displayName, nullptr, "dB", "measurement");
        mqttPublishSensorConfig(componentId, "last_seen", "Last Seen", trackerTopic(slot, "last_seen"),
                                availability, componentId, displayName, "timestamp");
        mqttPublishSensorConfig(componentId, "mode", "Mode", trackerTopic(slot, "mode"),
                                availability, componentId, displayName);
        mqttPublishSensorConfig(componentId, "interval", "Update Interval", trackerTopic(slot, "interval"),
                                availability, componentId, displayName, nullptr, "s", "measurement");
        mqttPublishBinarySensorConfig(componentId, "gps_fix", "GPS Fix", trackerTopic(slot, "gps_fix"),
                                      availability, componentId, displayName);
        mqttPublishBinarySensorConfig(componentId, "radio_ready", "Radio Ready", trackerTopic(slot, "radio_ready"),
                                      availability, componentId, displayName);
        mqttPublishDeviceTrackerConfig(slot);
        mqttPublishModeSelectConfig(slot);
    }
    mqttDiscoveryPublished = true;
    Serial.println("Clean Home Assistant MQTT discovery published");
}

void publishPacketToMqtt(const CollarPacket &packet)
{
    if (!mqttClient.connected())
    {
        return;
    }

    const uint8_t slot = trackerSlotForPacket(packet);
    mqttPublishString(mqttTopic("receiving_packets"), "ON", true);
    mqttPublishString(mqttTopic("received_packets"), String(receivedPackets), true);
    mqttPublishString(mqttTopic("last_packet_age"), "0", true);
    mqttPublishString(mqttTopic("ip_address"), ipText, true);

    mqttPublishString(trackerTopic(slot, "availability"), "online", true);
    mqttPublishString(trackerTopic(slot, "dog_name"), packet.dogName, true);
    mqttPublishString(trackerTopic(slot, "device_id"), packet.deviceId, true);
    mqttPublishString(trackerTopic(slot, "packet_counter"), String(packet.counter), true);
    const bool accurateGps = packetHasAccurateGps(packet);
    mqttPublishString(trackerTopic(slot, "gps_fix"), accurateGps ? "ON" : "OFF", true);
    mqttPublishString(trackerTopic(slot, "latitude"), packet.latitude, true);
    mqttPublishString(trackerTopic(slot, "longitude"), packet.longitude, true);
    mqttPublishString(trackerTopic(slot, "battery_mv"), String(packet.batteryMv), true);
    mqttPublishString(trackerTopic(slot, "battery_percent"), String(packet.batteryPercent), true);
    mqttPublishString(trackerTopic(slot, "mode"), packet.trackerMode, true);
    mqttPublishString(trackerTopic(slot, "interval"), String(packet.intervalS), true);
    mqttPublishString(trackerTopic(slot, "radio_ready"), packet.trackerRadioReady ? "ON" : "OFF", true);
    mqttPublishString(trackerTopic(slot, "gps_age"), String(packet.gpsAgeMs), true);
    mqttPublishString(trackerTopic(slot, "gps_satellites"), String(packet.gpsSatellites), true);
    mqttPublishString(trackerTopic(slot, "gps_hdop"), packet.gpsHdop, true);
    mqttPublishString(trackerTopic(slot, "rssi"), String(packet.rssi), true);
    mqttPublishString(trackerTopic(slot, "snr"), String(packet.snr, 1), true);
    const String lastSeen = currentIsoTimestamp();
    mqttPublishString(trackerTopic(slot, "last_seen"), lastSeen, true);
    if (accurateGps)
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
        mqttPublishString(trackerTopic(slot, "location"), locationJson, true);
    }

    String json = "{";
    json += "\"dog_name\":\"" + jsonEscape(packet.dogName) + "\",";
    json += "\"device_id\":\"" + jsonEscape(packet.deviceId) + "\",";
    json += "\"event\":\"" + jsonEscape(packet.event) + "\",";
    json += "\"counter\":" + String(packet.counter) + ",";
    json += "\"gps_fix\":";
    json += accurateGps ? "true" : "false";
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
    mqttPublishString(trackerTopic(slot, "state"), json, true);
}

void publishBaseStatusToMqtt()
{
    if (!mqttClient.connected())
    {
        return;
    }

    const uint32_t ageS = latestPacket.valid ? (millis() - latestPacket.receivedAtMs) / 1000 : 999999;
    mqttPublishString(mqttTopic("availability"), "online", true);
    mqttPublishString(mqttTopic("ip_address"), ipText, true);
    mqttPublishString(mqttTopic("received_packets"), String(receivedPackets), true);
    mqttPublishString(mqttTopic("last_packet_age"), String(ageS), true);
    mqttPublishString(mqttTopic("receiving_packets"), latestPacket.valid && ageS < 180 ? "ON" : "OFF", true);
}

void handleMqttMessage(char *topic, byte *payload, unsigned int length)
{
    String topicText = topic;
    String value;
    value.reserve(length);
    for (unsigned int i = 0; i < length; i++)
    {
        value += (char)payload[i];
    }
    value.trim();
    value.toUpperCase();

    uint8_t slot = 0;
    if (topicText == trackerTopic(1, "mode/set"))
    {
        slot = 1;
    }
    else if (topicText == trackerTopic(2, "mode/set"))
    {
        slot = 2;
    }

    if (slot == 0 || !validModeName(value))
    {
        Serial.print("MQTT command ignored topic=");
        Serial.print(topicText);
        Serial.print(" value=");
        Serial.println(value);
        return;
    }

    pendingTrackerMode[slot - 1] = value;
    pendingCommandCounter[slot - 1] = ++commandCounter;
    mqttPublishString(trackerTopic(slot, "pending_mode"), value, true);
    Serial.print("Queued mode command for tracker ");
    Serial.print(slot);
    Serial.print(": ");
    Serial.println(value);
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
            publishBaseStatusToMqtt();
            publishHomeAssistantDiscovery();
            mqttClient.subscribe(trackerTopic(1, "mode/set").c_str());
            mqttClient.subscribe(trackerTopic(2, "mode/set").c_str());
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
        publishBaseStatusToMqtt();
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
    display.print("Clodagh Base");

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
    pinMode(VEXT_ENABLE, OUTPUT);
    digitalWrite(VEXT_ENABLE, LOW);
    delay(100);

    pinMode(OLED_RST, OUTPUT);
    digitalWrite(OLED_RST, LOW);
    delay(20);
    digitalWrite(OLED_RST, HIGH);
    delay(20);

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
    loraSpi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
    lastRadioState = radio.begin(LORA_FREQUENCY_MHZ, LORA_BANDWIDTH_KHZ,
                                 LORA_SPREADING_FACTOR, LORA_CODING_RATE,
                                 LORA_SYNC_WORD, 10, LORA_PREAMBLE_LENGTH,
                                 LORA_TCXO_VOLTAGE);
    if (lastRadioState != RADIOLIB_ERR_NONE)
    {
        Serial.print("SX1262 init failed: ");
        Serial.println(lastRadioState);
        return;
    }

    radio.setCRC(false);
    Serial.println("SX1262 listening for collar packets");
}

void sendPendingCommandForPacket(const CollarPacket &packet)
{
    const uint8_t slot = trackerSlotForPacket(packet);
    if (slot < 1 || slot > 2 || pendingTrackerMode[slot - 1].length() == 0)
    {
        return;
    }

    String command = buildCommandPacket(slot);
    Serial.print("LoRa TX command: ");
    Serial.println(command);
    const int state = radio.transmit(command);
    Serial.print("LoRa command TX state=");
    Serial.println(state);
    if (state == RADIOLIB_ERR_NONE)
    {
        mqttPublishString(trackerTopic(slot, "last_command"), pendingTrackerMode[slot - 1], true);
        pendingTrackerMode[slot - 1] = "";
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println();
    Serial.println("Heltec WiFi LoRa 32 V3 Clodagh base station");

    startDisplay();
    drawOled();
    startWiFi();
    startWebServer();
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setCallback(handleMqttMessage);
    mqttClient.setBufferSize(1024);
    startRadio();
    drawOled();
}

void loop()
{
    server.handleClient();
    handleWiFi();
    handleMqtt();

    if (lastRadioState == RADIOLIB_ERR_NONE)
    {
        String packet;
        const int state = radio.receive(packet, 250);
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
                sendPendingCommandForPacket(latestPacket);
            }
            else
            {
                badPackets++;
                Serial.print("RX bad: ");
                Serial.println(packet);
            }
            drawOled();
        }
        else if (state != RADIOLIB_ERR_RX_TIMEOUT)
        {
            lastRadioState = state;
            Serial.print("LoRa receive error: ");
            Serial.println(state);
            drawOled();
            delay(1000);
            startRadio();
        }
    }

    static uint32_t lastOledRefresh = 0;
    if (millis() - lastOledRefresh > 5000)
    {
        lastOledRefresh = millis();
        drawOled();
    }
}
