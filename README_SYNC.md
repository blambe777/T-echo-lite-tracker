# Syncing Between PCs

This project is designed to sync through GitHub:

```text
Main PC <-> GitHub <-> Laptop
```

Do not sync PlatformIO build output. The `.pio` folder is ignored because it is
machine-specific and can cause stale cache/build problems when moving between
the T-Echo Lite tracker and ESP32 base station environments.

## Private Local Settings

Wi-Fi, MQTT, OTA, and remote hotspot passwords are kept out of Git.

1. Copy `src/local_config.example.h` to `src/local_config.h`.
2. Edit `src/local_config.h` with the local Wi-Fi/MQTT details.
3. Keep `src/local_config.h` local on each machine.

Barry's current local settings are already in `src/local_config.h` on the main
PC and should be copied manually to any trusted laptop used for uploads/tests.

## Normal Workflow

Before changing code on a machine:

```powershell
git pull
```

After changing and testing code:

```powershell
git status
git add .
git commit -m "Describe the change"
git push
```

Then on the other machine:

```powershell
git pull
```

Build/upload from PlatformIO on whichever machine has the device plugged in.
