# GMK330 emulator

[Download the latest firmware release](https://github.com/GerritPost/GMK330-emulator/releases/latest)

Firmware for a LilyGo T-CAN485 that reads a HomeWizard P1, Shelly 3EM,
Shelly Pro 3EM, or Shelly EM Mini Gen4 over Wi-Fi and emulates a GoodWe GMK330
three-phase grid meter over Modbus RTU. It was developed for a GoodWe ET-series
inverter configured with an external meter.

The emulator also provides:

- GoodWe meter discovery and binding responses
- A web interface for live control tuning and diagnostics
- Read-only inverter diagnostics over Modbus TCP
- CSV logging to the onboard microSD card
- Downloadable current and historical CSV logs
- A fail-safe that stops meter responses after repeated grid-meter read failures

## Hardware

- LilyGo T-CAN485 (ESP32-WROOM, MAX13487E auto-direction RS485)
- HomeWizard P1, Shelly 3EM, or Shelly Pro 3EM reachable on the same Wi-Fi network
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

`include/secrets.h` is ignored by Git. Do not commit real credentials.

## First-time setup and USB installation

The first installation must be performed over USB. This installs the required
dual-slot OTA partition layout and a firmware build containing the installation's
Wi-Fi and dashboard credentials.

Complete the [Configuration](#configuration) steps, install
[PlatformIO](https://platformio.org/), connect the T-CAN485 over USB, then run:

```sh
pio run -e lilygo-t-can485
pio run -e lilygo-t-can485 -t upload
pio device monitor -b 115200
```

If multiple serial devices are connected, add `--upload-port` to the upload
command or set a local port in PlatformIO.

After this one-time USB setup, later firmware versions can be installed through
the dashboard without reconnecting USB, as described below.

### Web interface development

The dashboard source is [data/index.html](data/index.html). A normal PlatformIO
build runs `scripts/embed_web.py`, compresses the page with gzip, and embeds it
in the firmware. No filesystem image or separate web-interface upload is needed.
Edit the HTML file and run the usual build/upload commands to deploy UI changes.

### Firmware updates over Wi-Fi

After the first-time USB installation, firmware updates can be uploaded through
the dashboard. Existing settings remain in the unchanged NVS partition, but the
GoodWe `Meter1` external-meter setting must still be reapplied after the restart.

For later updates:

1. Download `GMK330Emulator_vX.Y.Z_LilygoT-CAN485.ota.bin` from the latest
	GitHub Release, or build locally with `pio run -e lilygo-t-can485`.
2. Open **Firmware update** on the dashboard.
3. Select the downloaded `.ota.bin`, or the local
	`.pio/build/lilygo-t-can485/firmware.bin`, and install it.

Do not select `firmware.factory.bin` for a dashboard update. Meter responses,
P1 polling, inverter diagnostics, and SD logging pause while the image is being
written and resume only if the update fails. A validated image restarts the
controller automatically. The upload endpoint uses the dashboard credentials,
but HTTP does not encrypt them or the firmware; perform updates only on a trusted
local network.

### GitHub releases

Version tags automatically create a GitHub Release through
`.github/workflows/release.yml`. Each release contains:

- `GMK330Emulator_vX.Y.Z_LilygoT-CAN485.ota.bin` for dashboard updates
- `GMK330Emulator_vX.Y.Z_SHA256SUMS.txt` for download verification

The automated build deliberately uses `include/secrets.example.h`; it never
contains the maintainer's private credentials. The customized first USB build
stores the installation's Wi-Fi and dashboard credentials in NVS. Later release
OTA images reuse those saved credentials instead of their build-time placeholders.
Do not publish a locally built binary containing real credentials.

To publish a release, update `FIRMWARE_VERSION` in `src/main.cpp`, commit and push
the finished changes, then create and push a matching tag:

```sh
git tag v1.0.4
git push origin v1.0.4
```

The workflow rejects tags that do not exactly match the source version. Release
notes are generated automatically from the commits and pull requests since the
previous tag. The repository must be public if downloads should work without a
GitHub login.

## Operation

After startup, open `http://GMK330emulator.local/` or the board's DHCP address and log
in with the configured web credentials. The page exposes hybrid-controller
settings, current controller state, a rolling grid-power graph, diagnostics,
pause/resume controls, a **Configure settings** panel for the grid meter and
inverter diagnostics, a separate **Firmware update** panel, and a **Debug menu**
for CSV logging, log downloads, live JSON, serial logs, and raw registers. Saved
settings are retained in ESP32 Preferences.

The grid-meter selector supports:

- **HomeWizard P1** via `/api/v1/data`
- **Shelly 3EM (experimental)** via the Gen1 `/status` API
- **Shelly Pro 3EM (experimental)** via `/rpc/EM.GetStatus?id=0`
- **Shelly EM Mini Gen4 (experimental)** via `/rpc/Shelly.GetStatus` and its
	`pm1:0` component

All sources are normalized to positive import and negative export before entering
the existing controller. Switching source invalidates the cached reading and the
emulator withholds meter responses until the newly selected source returns valid
data. It does not automatically fail over to another configured meter. Meter
APIs must be reachable without authentication; Shelly's optional SHA-256 Digest
authentication is not implemented.

The EM Mini Gen4 is single-phase. Select it only when that device measures the
complete grid total; the firmware distributes that total evenly over the three
emulated phases. One EM Mini installed on only one phase cannot provide an
accurate three-phase grid total.

P1 acquisition uses adaptive polling. After changed measurements are detected,
the firmware waits 800 ms and then requests data every 100 ms until the next
change appears. This normally detects each one-second HomeWizard refresh within
about 100 ms without continuously making ten requests per second. The API has no
source timestamp, so identical consecutive measurements cannot be recognized as
a new refresh; probing continues at 100 ms until a value changes.

Inverter Modbus TCP diagnostics are enabled by default. Use **Configure settings**
to change the inverter IP or disable these read-only diagnostics. This
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
