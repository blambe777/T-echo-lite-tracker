# LilyGO T-Echo Lite Dog Tracker

This project adds a PlatformIO `dog_tracker` environment for the LilyGO T-Echo Lite nRF52840. Hardware pins are taken from `libraries/private_library/t_echo_lite_config.h` and the implementation follows the repository examples under `examples/T-Echo-Lite`.

## Build

From this folder:

```powershell
C:\Users\Barry\.platformio\penv\Scripts\platformio.exe run -e dog_tracker
```

In VS Code, open this repository folder, wait for PlatformIO to load, then select **Project Tasks > dog_tracker > General > Build**.

The checked examples can be built with:

```powershell
C:\Users\Barry\.platformio\penv\Scripts\platformio.exe run -e serial_test
C:\Users\Barry\.platformio\penv\Scripts\platformio.exe run -e example_gps
C:\Users\Barry\.platformio\penv\Scripts\platformio.exe run -e example_display
C:\Users\Barry\.platformio\penv\Scripts\platformio.exe run -e example_sx126x_pingpong
C:\Users\Barry\.platformio\penv\Scripts\platformio.exe run -e example_battery_measurement
C:\Users\Barry\.platformio\penv\Scripts\platformio.exe run -e example_ble_uart
```

If PlatformIO appears to build the wrong source after switching environments, run:

```powershell
C:\Users\Barry\.platformio\penv\Scripts\platformio.exe run -t clean
```

Each environment also has an explicit `build_src_filter` in `platformio.ini` to reduce stale `.pio` source selection problems.

## Upload

The T-Echo Lite was detected on this machine as `COM3`:

```powershell
C:\Users\Barry\.platformio\penv\Scripts\platformio.exe run -e dog_tracker -t upload --upload-port COM3
```

In VS Code, select **Project Tasks > dog_tracker > General > Upload**.

Upload was attempted from this machine, but `nrfutil` reported `No data received on serial port`, which means the board was visible on USB but was not responding in DFU/bootloader mode. If you see the same error, put the board into its documented nRF52840 DFU/bootloader mode, then run upload again. PlatformIO's error text says to use the DFU/BOOT and RESET method for the board and confirms baud rate 115200 with flow control off.

## Serial Monitor

```powershell
C:\Users\Barry\.platformio\penv\Scripts\platformio.exe device monitor --port COM3 --baud 115200
```

In VS Code, select **Project Tasks > dog_tracker > Monitor** or use the PlatformIO serial monitor button. The firmware prints startup state, device ID, LoRa frequency, packet contents, and LoRa transmit results.

## Change Dog Name

Edit `src/dog_tracker/dog_tracker.cpp`:

```cpp
static const char DOG_NAME[] = "Clodagh";
```

This value is used in BLE advertising and in LoRa packets.

## Change Phone Number On Display

Edit `src/dog_tracker/dog_tracker.cpp`:

```cpp
static const char DISPLAY_PHONE[] = "+353 838383474";
```

## Change LoRa Frequency

Edit `src/dog_tracker/dog_tracker.cpp`:

```cpp
static const float LORA_FREQUENCY_MHZ = 868.0;
```

The default is EU 868 MHz. Check local radio rules before changing frequency or power.

## Packet Format

Packets are ASCII and transmitted over LoRa:

```text
DT1,<device_id>,<dog>,<counter>,<event>,<fix>,<lat>,<lon>,<alt_m>,<battery_mv>*<crc16>
```

Fields:

- `DT1`: packet version
- `device_id`: nRF52840 unique ID from the Bluefruit helper
- `dog`: BLE/packet dog name, default `Clodagh`
- `counter`: incrementing packet counter
- `event`: `PERIODIC` or `EMERGENCY`
- `fix`: `1` when GPS location is valid, otherwise `0`
- `lat`, `lon`: decimal degrees, or zero when no fix exists
- `alt_m`: altitude in meters, or zero when no fix exists
- `battery_mv`: measured battery voltage in millivolts
- `crc16`: CRC-16/CCITT-FALSE of the bytes before `*`

## Known Limitations

- Upload from this machine did not complete because the connected board did not answer `nrfutil` DFU traffic; enter bootloader/DFU mode and retry.
- GPS fix quality depends on antenna view of sky. The firmware still sends packets with `fix=0` when no valid fix is available.
- LoRa receive/base-station firmware is not included here; this firmware only transmits tracker packets.
- Battery percentage is a simple voltage estimate based on the repository battery measurement example, not a fuel-gauge reading.
- The original `examples/T-Echo-Lite/Pet_Tracker` contains hard-coded pins that do not match `t_echo_lite_config.h`; this firmware does not use those pins.
- GPS UART/wake/1PPS pins are overridden in `src/dog_tracker/dog_tracker.cpp` from the LilyGO T-Echo-Lite pinmap image because `Chars` stayed at 0 with the `t_echo_lite_config.h` GPS mapping.
