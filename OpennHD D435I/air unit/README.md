# Air Unit — Q6A OpenHD Configuration (D435I)

Everything needed to reproduce the working air-unit setup on a Radxa Dragon Q6A board, running an Intel RealSense D435I as the camera. Files are laid out here exactly as they live on the actual board — copy each into the matching path shown below.

Full write-up of every problem hit and fixed while getting this working: see the `Q6A Camera Link` doc (exported separately as `q6a_camera_link.html`).

## Not included, on purpose

**`/usr/local/share/openhd/txrx.key`** — the shared encryption key between this air unit and its ground unit. This is a secret, not a config file, so it's deliberately left out of this repo. It must be generated (or copied from the working ground unit) separately and placed at that exact path — the air and ground units must use the **identical** key file for the radio link to work at all.

## File map

| File in this repo | Installs to | What it does |
|---|---|---|
| `ohd-cam/ohd-q6a-cam.sh` | `/home/radxa/ohd-cam/ohd-q6a-cam.sh` | Captures the D435I's colour camera, hardware-encodes it, sends it to OpenHD over RTP on `127.0.0.1:5500`. Auto-detects the camera's USB device node every time it starts. |
| `ohd-cam/ohd-q6a-cam.service` | `/etc/systemd/system/ohd-q6a-cam.service` (enable with `systemctl enable --now ohd-q6a-cam.service`) | Runs the script above at boot, restarts it if it crashes. |
| `ohd-cam/ohd-q6a-cam-imx415.sh.bak` | reference only, not installed | The original capture script for this board's earlier IMX415 CSI camera. Kept in case that camera is ever reconnected instead of the D435I. |
| `systemd/openhd-air.service` | `/etc/systemd/system/openhd-air.service` | Runs the main OpenHD program in air mode (`openhd -a`). On this particular board it's kept **disabled** by choice — the air unit is started manually with `sudo openhd -a` instead — but the unit file is here so it can be re-enabled (`systemctl enable openhd-air.service`) any time. |
| `systemd/openhd-air.service.d/regdomain.conf` | `/etc/systemd/system/openhd-air.service.d/regdomain.conf` | Systemd drop-in that runs `iw reg set IN` before every start of `openhd-air.service`, so the Wi‑Fi adapter's regulatory region is always set correctly. Fixes a real bug where the 5GHz band was blocked because no country code had ever been set. |
| `boot-openhd/hardware.config` | `/boot/openhd/hardware.config` | Tells OpenHD explicitly which Wi‑Fi adapter is the wifibroadcast radio link (`wlx20e15d3dc150`), instead of relying on autodetection. |
| `boot-openhd/air.txt` | `/boot/openhd/air.txt` | Empty marker file. Its mere presence tells OpenHD "this device is the air unit." |
| `openhd-share/debug.txt` | `/usr/local/share/openhd/debug.txt` | Empty gate file. Without it present, this OpenHD build silently ignores `hardware.config` entirely. |
| `openhd-share/interface/wifibroadcast_settings.json` | `/usr/local/share/openhd/interface/wifibroadcast_settings.json` | Radio link settings: frequency, MCS index, channel width, FEC, TX power. |
| `openhd-share/interface/networking_settings.json` | `/usr/local/share/openhd/interface/networking_settings.json` | Ethernet/network-forwarding settings (unused on this board, left at defaults). |
| `openhd-share/video/air_camera_generic.json` | `/usr/local/share/openhd/video/air_camera_generic.json` | Declares the primary camera as an "external" camera (type 2) — i.e. video is fed in externally via RTP, not captured by OpenHD itself. |
| `openhd-share/video/EXTERNAL_0.json` | `/usr/local/share/openhd/video/EXTERNAL_0.json` | Settings for that external camera slot: resolution/fps label, bitrate, keyframe interval, etc. |
| `openhd-share/telemetry/air_settings.json` | `/usr/local/share/openhd/telemetry/air_settings.json` | Flight-controller serial port, baud rate, and related telemetry settings. Currently points at `/dev/ttyACM0` — see the FC connection note below for why. |

## Flight controller (PX4) connection

**Use the Pixhawk's own USB port, plugged into a USB port on the Radxa — not the 40-pin GPIO header.** This board's GPIO header UART (physically labelled `UART6`, exposed in Linux as `/dev/ttyHS1`) was tested directly wired to the FC's TELEM2 port and never produced valid MAVLink data, even after confirming the wiring was crossed correctly, the baud rate matched (`SER_TEL2_BAUD` = 115200), and after fixing the FC's `MAV_2_CONFIG` parameter and rebooting it. The most likely explanation is a voltage mismatch: PX4 TELEM ports are 3.3V TTL, while this board's GPIO header is commonly 1.8V logic.

The Pixhawk's own USB port sidesteps this entirely (verified with 2100+ correctly-parsed MAVLink messages in a 6 second test, including a working `HEARTBEAT`): plug it into any USB port on the Radxa, it shows up as `/dev/ttyACM0`, and that's what `air_settings.json` above is configured for.

## Known open issues

- **Wi‑Fi adapter frequency bug**: the `rtl88x2bu_ohd` driver (TP-Link Archer T3U Plus) never actually switches to the frequency configured in `wifibroadcast_settings.json`, always staying on channel 1 (2412MHz) regardless of what's requested. The regulatory-domain fix above is still necessary and correct, but does not fix this separate issue. Both the air and ground units currently just operate on channel 1 as a workaround. See the full write-up for details.
- **D435I USB fragility**: the camera's USB3 connection has been observed to drop and re-enumerate on a different `/dev/videoN` node when the board is physically handled (e.g. while wiring other peripherals nearby). The capture script auto-detects the node each start, so a restart of `ohd-q6a-cam.service` recovers it — but the cable itself is worth checking/reseating if video ever stops unexpectedly.
