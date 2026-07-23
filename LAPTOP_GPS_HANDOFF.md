# Laptop GPS Handoff

This project is the LilyGO T-Echo Lite dog tracker and LoRa base station firmware.

## Project Path

Open this folder on the outside laptop:

```text
C:\Users\Barry\Documents\Codex\2026-06-14\you-are-working-on-a-lilygo\T-Echo-Lite-main\T-Echo-Lite-main
```

If the folder is copied to a different laptop path, open the copied `T-Echo-Lite-main\T-Echo-Lite-main` folder.

## Current Hardware

- Tracker: LilyGO T-Echo Lite nRF52840
- Tracker firmware env: `dog_tracker`
- Tracker serial speed: `115200`
- Tracker normal USB usually appeared as `COM3`
- Tracker DFU port usually appeared as `COM4`
- Main replacement base: LilyGO T3-S3 SX1276, env `t3s3_main_base`
- Old Heltec base: `heltec_base_station`, IP `192.168.0.90`
- T3-S3 base IP: `192.168.0.83`
- MQTT broker/user/password: stored locally in `src/local_config.h` and not tracked by Git

## Current Tracker Firmware State

File:

```text
src\dog_tracker\dog_tracker.cpp
```

Important current settings:

```cpp
DOG_NAME = "Clodagh"
TRACKER_MODE = "GPS_BATTERY_TEST"
LOCATION_SEND_INTERVAL_MS = 60000
GPS_FRESH_FIX_MAX_AGE_MS = 30000
GPS_RAW_NMEA_DEBUG = true
GPS_RAW_NMEA_WINDOW_MS = 120000
```

The tracker now treats GPS as valid only if the location age is under 30 seconds. This prevents stale GPS coordinates from being shown as live in Home Assistant.

For the first 120 seconds after tracker boot, it prints raw GPS NMEA lines to serial.

## What We Are Diagnosing

The GPS module is powered and sending NMEA data, but it reports zero satellites.

Observed raw GPS output:

```text
$GNGGA,,,,,,0,00,25.5,,,,,,*64
$GNGLL,,,,,,V,M*79
$GNGSA,A,1,,,,,,,,,,,,,25.5,25.5,25.5,1*01
$GNGSA,A,1,,,,,,,,,,,,,25.5,25.5,25.5,2*02
$GPGSV,1,1,00,0*65
$GLGSV,1,1,00,0*79
$GNRMC,,V,,,,,,,,,,M,V*34
$GNVTG,,,,,,,,,M*2D
$GNZDA,,,,,,*56
$GPTXT,01,01,01,ANTENNA OK*35
```

Meaning:

- GPS UART and power are working.
- GPS antenna status reports `ANTENNA OK`.
- GPS sees `0` GPS satellites and `0` GLONASS satellites.
- No valid time/date/location is being produced.

The goal on the outside laptop is to place the tracker/GPS module in full sky view and check whether `$GPGSV` / `$GLGSV` changes from `00` satellites to real satellites.

Good signs would look like:

```text
$GPGSV,3,1,10,...
$GLGSV,2,1,07,...
GPS debug: fix=YES ...
```

Bad sign, still no satellites:

```text
$GPGSV,1,1,00,0*65
$GLGSV,1,1,00,0*79
GPS debug: fix=NO ... sats=0 ...
```

## Laptop Test Steps

1. Plug the tracker into the outside laptop.
2. Put the GPS module/antenna side facing the sky.
3. Keep it away from the battery, LoRa antenna, laptop body, metal, and walls if possible.
4. Open PowerShell in this project folder.
5. List serial ports:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\python.exe" -m serial.tools.list_ports -v
```

If that Python path does not exist on the laptop, try:

```powershell
python -m serial.tools.list_ports -v
```

6. Open serial monitor at 115200, replacing `COMx` with the tracker port:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" device monitor --port COMx --baud 115200
```

Alternative if PlatformIO is not available:

```powershell
python -m serial.tools.miniterm COMx 115200
```

7. Reboot the tracker so raw NMEA appears for the first 120 seconds.

If using PlatformIO:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e dog_tracker -t upload --upload-port COMx
```

If normal upload fails, the T-Echo Lite usually needs DFU mode:

```powershell
python - <<'PY'
import serial, time
ser = serial.Serial("COMx", 1200)
ser.dtr = False
ser.rts = False
time.sleep(0.2)
ser.close()
time.sleep(3)
PY
```

Then list ports again and upload to the DFU port.

## What To Report Back

Copy/paste 20-40 lines containing:

```text
GPS raw: $GPGSV...
GPS raw: $GLGSV...
GPS raw: $GNRMC...
GPS raw: $GNGGA...
GPS raw: $GPTXT...
GPS debug: fix=...
```

Especially report whether these still show `00`:

```text
$GPGSV,1,1,00
$GLGSV,1,1,00
```

## Home Assistant MQTT Fields

The firmware publishes:

```text
clodagh_tracker/gps_fix
clodagh_tracker/gps_age
clodagh_tracker/gps_satellites
clodagh_tracker/gps_hdop
clodagh_tracker/gps_chars
clodagh_tracker/latitude
clodagh_tracker/longitude
clodagh_tracker/location
```

`clodagh_tracker/location` is intended for Home Assistant map/device_tracker use and is only published when GPS fix is fresh.

## Known Current Conclusion

If raw NMEA still shows `GPGSV ... 00` and `GLGSV ... 00` outside with clear sky, the likely fault is physical GPS reception:

- GPS antenna orientation
- GPS module shielded by battery/case/board
- damaged or poor GPS module/antenna
- module mounted wrong way up

It is probably not a code pin/power problem because the GPS module continuously outputs valid NMEA sentences and reports `ANTENNA OK`.
