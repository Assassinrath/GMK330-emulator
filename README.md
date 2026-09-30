# GMK330 emulator

Firmware for a LilyGo T-CAN485 that reads a HomeWizard P1 meter over Wi-Fi and
emulates a GoodWe GMK330 three-phase grid meter over Modbus RTU. It was developed
for a GoodWe ET-series inverter configured with an external meter.

The emulator also provides:

- GoodWe meter discovery and binding responses
- A web interface for live control tuning and diagnostics
- Read-only inverter diagnostics over Modbus TCP
- CSV logging to the onboard microSD card
- Downloadable current and historical CSV logs
- A fail-safe that stops meter responses after repeated P1 read failures

## Hardware

- LilyGo T-CAN485 (ESP32-WROOM, MAX13487E auto-direction RS485)
- HomeWizard P1 meter reachable on the same Wi-Fi network
- GoodWe inverter with the external-meter RS485 connection
- Optional FAT32 microSD card for CSV logging

RS485 uses TX GPIO 22 and RX GPIO 21 at 9600 baud, 8N1. The onboard microSD
interface uses MISO 2, MOSI 15, SCLK 14, and CS 13.

## Configuration

1. Copy `include/secrets.example.h` to `include/secrets.h`.
2. Enter the Wi-Fi SSID/password, choose strong web-interface credentials, and
	enter the 16-character GMK330 identity observed during meter binding.
3. Review `P1_IP_DEFAULT` and `INVERTER_IP_DEFAULT` near the top of `src/main.cpp`.
	Both addresses can be changed later from the dashboard.
4. Review phase rotation and single-phase splitting options for the installation.

`include/secrets.h` is ignored by Git. Do not commit real credentials.

## Build and upload

Install [PlatformIO](https://platformio.org/), connect the T-CAN485, then run:

```sh
pio run -e lilygo-t-can485
pio run -e lilygo-t-can485 -t upload
pio device monitor -b 115200
```

If multiple serial devices are connected, add `--upload-port` to the upload
command or set a local port in PlatformIO.

### Web interface development

The dashboard source is [data/index.html](data/index.html). A normal PlatformIO
build runs `scripts/embed_web.py`, compresses the page with gzip, and embeds it
in the firmware. No filesystem image or separate web-interface upload is needed.
Edit the HTML file and run the usual build/upload commands to deploy UI changes.

### Firmware updates over Wi-Fi

The firmware uses two application partitions so updates can be uploaded from the
dashboard. Because installing this partition layout rewrites the flash partition
table, deploy the OTA-enabled firmware once over USB with the normal PlatformIO
upload command. Existing settings remain in the unchanged NVS partition, but the
GoodWe `Meter1` external-meter setting must still be reapplied after the restart.

For later updates:

1. Build with `pio run -e lilygo-t-can485`.
2. Open **Firmware update** on the dashboard.
3. Select `.pio/build/lilygo-t-can485/firmware.bin` and install it.

Do not select `firmware.factory.bin` for a dashboard update. Meter responses,
P1 polling, inverter diagnostics, and SD logging pause while the image is being
written and resume only if the update fails. A validated image restarts the
controller automatically. The upload endpoint uses the dashboard credentials,
but HTTP does not encrypt them or the firmware; perform updates only on a trusted
local network.

## Operation

After startup, open `http://GMK330emulator.local/` or the board's DHCP address and log
in with the configured web credentials. The page exposes hybrid-controller
settings, current controller state, a rolling grid-power graph, diagnostics,
pause/resume controls, and CSV downloads. Saved settings are retained in ESP32
Preferences.

Inverter Modbus TCP diagnostics are enabled by default. Use **Configure Inverter
Modbus** to change the inverter IP or disable these read-only diagnostics. This
setting does not disable the RS485 meter emulator. The status section at the
bottom of the dashboard shows the inverter's grid, PV, AC, battery, backup, and
load readings when the connection is available. Battery status is decoded from
the inverter's mode register and reports Charging, Discharging, Standby, Waiting
to charge, or No battery together with the current power magnitude.

The GMK330 cumulative-energy block is calculated from the raw P1 phase powers
and retained in ESP32 Preferences. New installations start at zero and preserve
their accumulated import/export values across restarts. Values are saved every
15 minutes to limit flash wear. Two unidentified auxiliary counters in the
captured GMK330 block are returned as zero.

> **Important:** After every LilyGo reboot or power cycle, activate `Meter1` and
> set it to `External` again in the GoodWe configuration. The inverter will not
> poll the emulator until this setting is reapplied.

### Status LED

The onboard RGB LED reports communication state:

| LED behavior | Meaning |
| --- | --- |
| Slow blue breathing | The inverter has not sent a valid RS485 request recently. After every LilyGo reboot, activate `Meter1` and set it to `External` again in the GoodWe configuration so the inverter polls the emulator. |
| Off between flashes | The inverter is actively polling the emulator over RS485. |
| Brief green flash | The latest Grid Meter (P1) read succeeded. |
| Brief red flash | The latest Grid Meter (P1) read failed. Repeated failures eventually activate the meter-response fail-safe. |

Green and red Grid Meter flashes temporarily override the blue breathing effect.
The dashboard's third indicator pulses green when the separate inverter Modbus
TCP diagnostics connection is healthy, pulses red when enabled but unreachable,
and remains gray when disabled.

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

Normal gain values above `0.33` are available for testing but reduce stability
and may cause oscillation. The web interface shows a warning above `0.33` and
an extreme warning above `0.66`. Step and Crossing brake gains retain a hard
firmware maximum of `0.33`.

These defaults are based on one tested installation and must not be assumed
safe for a different inverter, battery, meter placement, or phase arrangement.

## Safety

This firmware participates in inverter power control and is experimental. Verify
meter polarity, phase mapping, inverter meter-loss behavior, and export limiting
under supervision before unattended use. Keep the original meter and a recovery
method available. The software is provided without warranty under the MIT license.
