#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <DNSServer.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RadioLib.h>

#if __has_include("../local_config.h")
#include "../local_config.h"
#endif

#ifndef REMOTE_AP_PASSWORD
#define REMOTE_AP_PASSWORD "CHANGE_ME"
#endif

// Portable receiver settings. The phone connects to this access point and
// opens http://192.168.4.1 to view the tracker dashboard.
static const char *AP_NAME = "Clodagh-Remote";
static const char *AP_PASSWORD = REMOTE_AP_PASSWORD;
static const byte DNS_PORT = 53;

// Collar LoRa settings. These must match the T-Echo Lite dog_tracker firmware.
static const float LORA_FREQUENCY_MHZ = 868.0;
static const float LORA_BANDWIDTH_KHZ = 125.0;
static const uint8_t LORA_SPREADING_FACTOR = 9;
static const uint8_t LORA_CODING_RATE = 6;
static const uint8_t LORA_SYNC_WORD = 0x12; // Standard private LoRa sync word; compatible with SX1262 and SX1276.
static const uint16_t LORA_PREAMBLE_LENGTH = 16;
static const float LORA_TCXO_VOLTAGE = 1.8;
static const uint32_t LORA_RECEIVE_TIMEOUT_MS = 2500;

// LilyGO T3-S3 radio pins from LilyGO LoRa Series T3-S3 hardware docs.
static const int LORA_SCK = 5;
static const int LORA_MISO = 3;
static const int LORA_MOSI = 6;
static const int LORA_CS = 7;
static const int LORA_RESET = 8;
static const int LORA_DIO0 = 9;
static const int LORA_DIO1 = 33;
static const int LORA_BUSY = 34;
static const int OLED_SDA = 18;
static const int OLED_SCL = 17;
static const int USER_LED = 37;

static const int SCREEN_WIDTH = 128;
static const int SCREEN_HEIGHT = 64;
static const int OLED_ADDRESS = 0x3C;
static const int OLED_RESET = -1;

// This unit appears to be the T3-S3 SX1276 868/915 MHz variant. LilyGO's
// examples use DIO0 on GPIO9 and DIO1 on GPIO33 for SX1276.
SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RESET, LORA_DIO1);

/*
// LilyGO's T3-S3 SX1262 examples use the default Arduino SPI object with
// explicit pins, so keep RadioLib on that same path for this board.
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RESET, LORA_BUSY);
*/
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
WebServer server(80);
DNSServer dnsServer;

struct TrackerPacket
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
    int16_t rssi = 0;
    float snr = 0.0;
    uint32_t receivedAtMs = 0;
};

TrackerPacket latestPacket;
uint32_t receivedPackets = 0;
uint32_t badPackets = 0;
int lastRadioState = RADIOLIB_ERR_NONE;
bool displayReady = false;
String lastBadPacket = "";
String lastBadReason = "NONE";
int16_t lastBadRssi = 0;
float lastBadSnr = 0.0;

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

bool parseTrackerPacket(const String &raw, TrackerPacket &out)
{
    out = TrackerPacket();
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
    const String crcText = raw.substring(star + 1, star + 5);
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
    out.valid = out.crcOk && out.dogName.length() > 0;
    return out.valid;
}

String ageText()
{
    if (!latestPacket.valid)
    {
        return "never";
    }
    const uint32_t ageS = (millis() - latestPacket.receivedAtMs) / 1000;
    return String(ageS) + "s ago";
}

String htmlEscape(const String &value)
{
    String escaped;
    escaped.reserve(value.length() + 8);
    for (size_t i = 0; i < value.length(); i++)
    {
        const char c = value[i];
        if (c == '&')
            escaped += "&amp;";
        else if (c == '<')
            escaped += "&lt;";
        else if (c == '>')
            escaped += "&gt;";
        else if (c == '"')
            escaped += "&quot;";
        else
            escaped += c;
    }
    return escaped;
}

String mapUrl()
{
    if (!latestPacket.valid || !latestPacket.gpsFix)
    {
        return "";
    }
    return String("https://maps.google.com/?q=") + latestPacket.latitude + "," + latestPacket.longitude;
}

void drawOled()
{
    if (!displayReady)
    {
        return;
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Clodagh Remote");
    display.print("AP: ");
    display.println(AP_NAME);
    display.println("IP: 192.168.4.1");
    display.drawLine(0, 27, 127, 27, SSD1306_WHITE);

    if (!latestPacket.valid)
    {
        display.setCursor(0, 33);
        display.println("Waiting for LoRa");
        display.print("Bad: ");
        display.print(badPackets);
        display.print(" Radio: ");
        display.println(lastRadioState);
        display.print("Reason: ");
        display.println(lastBadReason);
    }
    else
    {
        display.setCursor(0, 31);
        display.print("Pkt ");
        display.print(latestPacket.counter);
        display.print(" ");
        display.println(ageText());
        display.print("Bat ");
        display.print(latestPacket.batteryMv);
        display.print("mV ");
        display.print(latestPacket.batteryPercent);
        display.println("%");
        display.print("RSSI ");
        display.print(latestPacket.rssi);
        display.print(" SNR ");
        display.println(latestPacket.snr, 1);
        display.print("GPS ");
        display.println(latestPacket.gpsFix ? "FIX" : "NO FIX");
    }

    display.display();
}

void handleRoot()
{
    String html = F("<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
                    "<meta http-equiv='refresh' content='5'>"
                    "<title>Clodagh Remote</title>"
                    "<style>body{font-family:Arial,sans-serif;margin:0;background:#f4f6f8;color:#101820}"
                    "main{max-width:620px;margin:auto;padding:18px}.hero{background:#101820;color:white;padding:18px;border-radius:8px}"
                    "h1{margin:0;font-size:32px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:14px}"
                    ".card{background:white;border-radius:8px;padding:14px;box-shadow:0 1px 4px #0002}.label{font-size:12px;color:#657080}"
                    ".value{font-size:22px;font-weight:700;margin-top:4px;word-break:break-word}.wide{grid-column:1/-1}"
                    "a.btn{display:block;text-align:center;background:#0b63ce;color:white;text-decoration:none;padding:16px;border-radius:8px;font-size:20px;font-weight:700;margin-top:14px}"
                    ".map{height:220px;border-radius:8px;background:linear-gradient(135deg,#d8eadb,#d7e2f7);position:relative;overflow:hidden;margin-top:14px}"
                    ".pin{position:absolute;left:50%;top:50%;transform:translate(-50%,-100%);font-size:44px}.muted{color:#657080;font-size:14px}</style></head><body><main>");

    html += F("<section class='hero'><h1>Clodagh Remote</h1><div>");
    html += latestPacket.valid ? "Tracker heard " + ageText() : "Waiting for LoRa tracker packet";
    html += F("</div></section><section class='grid'>");

    html += F("<div class='card'><div class='label'>Packets</div><div class='value'>");
    html += String(receivedPackets);
    html += F("</div></div><div class='card'><div class='label'>Bad Packets</div><div class='value'>");
    html += String(badPackets);
    html += F("</div></div><div class='card'><div class='label'>Battery</div><div class='value'>");
    html += latestPacket.valid ? String(latestPacket.batteryMv) + " mV" : "--";
    html += F("</div></div><div class='card'><div class='label'>Signal</div><div class='value'>");
    html += latestPacket.valid ? String(latestPacket.rssi) + " dBm" : "--";
    html += F("</div></div><div class='card'><div class='label'>GPS</div><div class='value'>");
    html += latestPacket.valid ? (latestPacket.gpsFix ? "Fix" : "No fix") : "--";
    html += F("</div></div><div class='card'><div class='label'>SNR</div><div class='value'>");
    html += latestPacket.valid ? String(latestPacket.snr, 1) + " dB" : "--";
    html += F("</div></div><div class='card wide'><div class='label'>Location</div><div class='value'>");

    if (latestPacket.valid && latestPacket.gpsFix)
    {
        html += htmlEscape(latestPacket.latitude + ", " + latestPacket.longitude);
    }
    else
    {
        html += "No GPS fix yet";
    }
    html += F("</div>");

    if (latestPacket.valid && latestPacket.gpsFix)
    {
        html += F("<div class='map'><div class='pin'>&#128205;</div></div><a class='btn' href='");
        html += mapUrl();
        html += F("'>Open Map</a>");
    }
    else
    {
        html += F("<p class='muted'>The map button appears once the collar sends GPS coordinates.</p>");
    }

    html += F("</div><div class='card wide'><div class='label'>Raw Packet</div><div class='muted'>");
    html += latestPacket.valid ? htmlEscape(latestPacket.raw) : "None yet";
    html += F("</div></div><div class='card wide'><div class='label'>Last Bad Packet</div><div class='muted'>");
    if (lastBadPacket.length() > 0)
    {
        html += htmlEscape(lastBadReason);
        html += F(" | RSSI ");
        html += String(lastBadRssi);
        html += F(" dBm / SNR ");
        html += String(lastBadSnr, 1);
        html += F(" dB<br><code>");
        html += htmlEscape(printablePacket(lastBadPacket));
        html += F("</code>");
    }
    else
    {
        html += "None yet";
    }
    html += F("</div></div></section></main></body></html>");

    server.send(200, "text/html", html);
}

void handleCaptivePortalProbe()
{
    handleRoot();
}

void handleJson()
{
    String json = "{";
    json += "\"valid\":";
    json += latestPacket.valid ? "true" : "false";
    json += ",\"gps_fix\":";
    json += latestPacket.gpsFix ? "true" : "false";
    json += ",\"lat\":\"" + latestPacket.latitude + "\"";
    json += ",\"lon\":\"" + latestPacket.longitude + "\"";
    json += ",\"battery_mv\":";
    json += latestPacket.batteryMv;
    json += ",\"rssi\":";
    json += latestPacket.rssi;
    json += ",\"snr\":";
    json += String(latestPacket.snr, 1);
    json += ",\"packets\":";
    json += receivedPackets;
    json += ",\"bad_packets\":";
    json += badPackets;
    json += ",\"last_bad_reason\":\"";
    json += lastBadReason;
    json += "\",\"last_bad_packet\":\"";
    json += printablePacket(lastBadPacket);
    json += "\"";
    json += "}";
    server.send(200, "application/json", json);
}

void setupRadio()
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
        // Leave SX1276 CRC mode permissive when receiving SX1262 packets.
        // RadioLib still reports RF CRC mismatch via IRQ flags, while this
        // avoids rejecting valid packets if the SX1262 header is interpreted
        // differently by the older SX127x modem.
        lastRadioState = radio.setCRC(false);
    }

    Serial.print("SX1276 setup result=");
    Serial.println(lastRadioState);
}

void setup()
{
    Serial.begin(115200);
    delay(1500);
    Serial.println();
    Serial.println("LilyGO T3-S3 Clodagh portable LoRa receiver");

    Wire.begin(OLED_SDA, OLED_SCL);
    displayReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
    if (!displayReady)
    {
        Serial.println("OLED init failed");
    }

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_NAME, AP_PASSWORD);
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
    Serial.print("AP name: ");
    Serial.println(AP_NAME);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", handleRoot);
    server.on("/generate_204", handleCaptivePortalProbe);
    server.on("/hotspot-detect.html", handleCaptivePortalProbe);
    server.on("/ncsi.txt", handleCaptivePortalProbe);
    server.on("/connecttest.txt", handleCaptivePortalProbe);
    server.on("/json", handleJson);
    server.onNotFound(handleRoot);
    server.begin();

    setupRadio();
    drawOled();
}

void loop()
{
    dnsServer.processNextRequest();
    server.handleClient();

    String received;
    // A tracker packet is long enough at SF9/BW125 that a very short receive
    // timeout can abort the packet before it finishes. Keep the receiver open
    // for a complete packet window, then service the web/captive portal again.
    const int state = radio.receive(received, LORA_RECEIVE_TIMEOUT_MS);
    if (state == RADIOLIB_ERR_NONE)
    {
        TrackerPacket parsed;
        if (parseTrackerPacket(received, parsed))
        {
            latestPacket = parsed;
            receivedPackets++;
            digitalWrite(USER_LED, !digitalRead(USER_LED));
            Serial.print("LoRa RX OK: ");
            Serial.println(received);
        }
        else
        {
            badPackets++;
            lastBadPacket = received;
            lastBadReason = badPacketReason(received);
            lastBadRssi = (int16_t)radio.getRSSI();
            lastBadSnr = radio.getSNR();
            Serial.print("LoRa RX bad packet: ");
            Serial.print(lastBadReason);
            Serial.print(" RSSI=");
            Serial.print(lastBadRssi);
            Serial.print(" SNR=");
            Serial.print(lastBadSnr, 1);
            Serial.print(" DATA=");
            Serial.println(printablePacket(received));
        }
        drawOled();
    }
    else if (state != RADIOLIB_ERR_RX_TIMEOUT)
    {
        lastRadioState = state;
        Serial.print("LoRa RX error=");
        Serial.println(state);
        drawOled();
    }

    static uint32_t lastDisplayRefresh = 0;
    if (millis() - lastDisplayRefresh > 5000)
    {
        lastDisplayRefresh = millis();
        drawOled();
    }
}
