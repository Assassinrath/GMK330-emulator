# GMK330 emulator

[Download the latest firmware release](https://github.com/GerritPost/GMK330-emulator/releases/latest)

## Diagram
<img width="300" alt="GMK330 Emulator webpage" src="https://github.com/user-attachments/assets/a9951ab7-61fb-45b7-aec6-81745c2e638b" />

## Diagram
<img width="1200" alt="GMK330 Diagram" src="https://github.com/user-attachments/assets/e45c2356-e3d5-4d22-9df2-c6ebd140b4f4" />


Firmware for a LilyGo T-CAN485 that reads a HomeWizard P1, Shelly 3EM,
Shelly Pro 3EM, or Shelly EM Mini Gen4 over Wi-Fi and emulates a GoodWe GMK330
three-phase grid meter over Modbus RTU. It was developed for a GoodWe ET-series
inverter and tested on the `GW15K-ET-20`.

Features include:

- GoodWe meter discovery and binding responses
- A web dashboard for control, diagnostics, firmware updates, and CSV logs
- A fail-safe that stops meter responses after repeated grid-meter read failures

## Hardware

- LilyGo T-CAN485 (ESP32-WROOM, MAX13487E auto-direction RS485)
- A supported grid meter reachable on the same Wi-Fi network
- GoodWe inverter with the external-meter RS485 connection
- Optional FAT32 microSD card for CSV logging

RS485 uses TX GPIO 22 and RX GPIO 21 at 9600 baud, 8N1. The onboard microSD
interface uses MISO 2, MOSI 15, SCLK 14, and CS 13.

## Configuration

1. Copy `include/secrets.example.h` to `include/secrets.h`.
2. Enter the Wi-Fi SSID/password, choose strong web-interface credentials, and
	enter the 16-character GMK330 identity observed during meter binding.
3. Review `P1_IP_DEFAULT` and `INVERTER_IP_DEFAULT` near the top of `src/main.cpp`.
	Both addresses and the grid-meter type can be changed later under
	**Configure settings** on the dashboard.
4. Review phase rotation and single-phase splitting options for the installation.

Do not commit `include/secrets.h`; it contains private credentials and is ignored
by Git.

## Installation

The first installation must be performed over USB to install the OTA partition
layout and save the configured credentials. Install
[PlatformIO](https://platformio.org/), connect the T-CAN485, and run:

```sh
pio run -e lilygo-t-can485
pio run -e lilygo-t-can485 -t upload
pio device monitor -b 115200
```

### Firmware updates

After the first USB installation, update over Wi-Fi through the dashboard:

1. Download `GMK330Emulator_vX.Y.Z_LilygoT-CAN485.ota.bin` from the latest
	[GitHub Release](https://github.com/GerritPost/GMK330-emulator/releases/latest).
2. Open **Firmware update** on the dashboard.
3. Select the downloaded `.ota.bin` and install it.

Do not upload a factory binary. Settings are preserved, and the controller
restarts automatically after a valid update. Perform updates only on a trusted
local network. Reapply `Meter1` as `External` in the GoodWe configuration after
the restart.

## Operation

After startup, open `http://GMK330emulator.local/` or the board's DHCP address
and log in with the configured credentials. The dashboard provides controller
settings, live status and graphs, diagnostics, firmware updates, and CSV logging.

The grid-meter selector supports:

- **HomeWizard P1**
- **Shelly 3EM (experimental)**
- **Shelly Pro 3EM (experimental)**
- **Shelly EM Mini Gen4 (experimental)**

Meter APIs must be reachable without authentication. The emulator does not
automatically switch to another meter if the selected source becomes unavailable.

The EM Mini Gen4 is single-phase. Select it only when that device measures the
complete grid total; the firmware distributes that total evenly over the three
emulated phases. One EM Mini installed on only one phase cannot provide an
accurate three-phase grid total.

Read-only inverter diagnostics are enabled by default and can be configured or
disabled under **Configure settings**. Import and export energy counters are
retained across restarts.

> **Important:** After every LilyGo reboot or power cycle, activate `Meter1` and
> set it to `External` again in the GoodWe configuration. The inverter will not
> poll the emulator until this setting is reapplied.

### Status LED

The onboard RGB LED reports communication state:

| LED behavior | Meaning |
| --- | --- |
| Slow blue breathing | The inverter is not polling the emulator. |
| Off between flashes | The inverter is actively polling the emulator over RS485. |
| Brief green flash | The latest grid-meter read succeeded. |
| Brief red flash | The latest grid-meter read failed. |

The dashboard shows the separate inverter diagnostics connection status.

### Controller defaults

| Setting | Default | Allowed range |
| --- | ---: | ---: |
| Normal gain | `0.33` | `0.00-1.00` |
| Step gain | `0.25` | `0.00-0.33` |
| Crossing brake gain | `0.15` | `0.00-0.33` |
| Step threshold | `500 W` | `50-5000 W` |
| Hold time | `100 ms` | `0-15000 ms` |
| Ramp time | `1000 ms` | `1000-60000 ms` |
| Integral freeze | `30000 ms` | `0-120000 ms` |
| Target import | `0 W` | `-500-500 W` |

Normal gain values above `0.33` may reduce stability and cause oscillation.
These defaults are based on one installation and may not be safe elsewhere.

## Safety

This firmware participates in inverter power control and is experimental. Verify
meter polarity, phase mapping, inverter meter-loss behavior, and export limiting
under supervision before unattended use. Keep the original meter and a recovery
method available. The software is provided without warranty under the MIT license.
