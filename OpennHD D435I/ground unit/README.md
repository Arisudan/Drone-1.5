# Ground Unit — OpenHD Configuration (D435I link)

Everything needed to reproduce the working ground-unit setup that receives the video/telemetry link from the [air unit](../air%20unit/README.md) (Radxa Dragon Q6A + Intel RealSense D435I). Files are laid out here exactly as they live on the actual ground machine — copy each into the matching path shown below.

The ground unit itself has no camera and does no SLAM — its job is to run OpenHD in ground mode, receive the wifibroadcast video/telemetry link from the air unit, and hand telemetry off to a GCS app (QGroundControl / QOpenHD) and video to a display.

## Not included, on purpose

**`/usr/local/share/openhd/txrx.key`** — the shared encryption key between this ground unit and its air unit. This is a secret, not a config file, so it's deliberately left out of this repo, same as on the air-unit side. It must be generated once (or copied from the air unit's key) and placed at this exact path on **both** units — air and ground must use the **identical** key file for the radio link to work at all.

Also excluded, as it's runtime state rather than setup config:
- `/usr/local/share/openhd/unit.id` — auto-generated unique ID for this unit, not needed to reproduce the setup.
- `/usr/local/share/openhd/telemetry/*.ohd` — recorded flight-telemetry logs from past sessions, not configuration.
- `/usr/local/share/openhd/recording.txt` — just a local path (`/home/openhd/Videos/`) where OpenHD saves recorded video; adjust to taste, no need to copy verbatim.

## How ground vs. air mode is chosen

Unlike the air unit — which is marked by the presence of `/boot/openhd/air.txt` — this machine is a ground unit simply because **that file is absent**. There is no separate "ground.txt" marker required; OpenHD defaults to ground mode whenever `air.txt` is not present at `/boot/openhd/air.txt`.

## File map

| File in this repo | Installs to | What it does |
|---|---|---|
| `boot-openhd/hardware.config` | `/boot/openhd/hardware.config` | Tells OpenHD explicitly which network interfaces to use: `WIFI_WB_LINK_CARDS` for the wifibroadcast radio (must match the same physical radio type/band as the air unit's link card), and `WIFI_WIFI_HOTSPOT_CARD` for the local access point OpenHD raises so the GCS app can connect. Autodetection is disabled so the correct adapters are always picked. |
| `openhd-share/debug.txt` | `/usr/local/share/openhd/debug.txt` | Empty gate file. Without it present, this OpenHD build silently ignores `hardware.config` entirely (same requirement as on the air unit). |
| `openhd-share/interface/wifibroadcast_settings.json` | `/usr/local/share/openhd/interface/wifibroadcast_settings.json` | Radio link settings: frequency, MCS index, channel width, FEC, TX power. **Must match the air unit's radio parameters** (frequency in particular) for the two ends to find each other. |
| `openhd-share/interface/networking_settings.json` | `/usr/local/share/openhd/interface/networking_settings.json` | Ethernet/WiFi-hotspot/WiFi-client operating modes for how the ground unit exposes itself to the GCS laptop/phone. Left at defaults here — adjust if you want the ground unit to join an existing WiFi network instead of hosting its own hotspot. |
| `openhd-share/telemetry/ground_settings.json` | `/usr/local/share/openhd/telemetry/ground_settings.json` | Ground-side telemetry routing: baud rate/serial settings if a flight controller is also plugged directly into the ground unit (for RC-over-joystick or a wired FC link), and UART priority ordering between FC, OpenHD, and RC. |
| `systemd/openhd.service` | `/etc/systemd/system/openhd.service` | Runs the main OpenHD program (`/usr/local/bin/openhd`) as a background service, auto-restarting on crash. Unlike the air unit (which uses a mode-specific `openhd-air.service` and is started manually), the ground unit runs the single generic `openhd.service`, which auto-detects ground mode from the absence of `air.txt`. Enable with `systemctl enable --now openhd.service`. |

## Setting up a new ground unit from scratch

1. **Flash/install OpenHD** on the ground machine (Raspberry Pi, x86 laptop/mini-PC, etc.) following the official [OpenHD installation docs](https://www.openhdfpv.org/).
2. **Confirm it's in ground mode**: make sure `/boot/openhd/air.txt` does **not** exist on this machine. If it does, delete it — its presence is what would flip this unit into air mode.
3. **Copy the config files** from this folder to the paths listed in the table above (all as `root`, since `/boot/openhd` and `/usr/local/share/openhd` are root-owned):
   ```bash
   sudo cp "boot-openhd/hardware.config" /boot/openhd/hardware.config
   sudo mkdir -p /usr/local/share/openhd/interface /usr/local/share/openhd/telemetry
   sudo cp "openhd-share/debug.txt" /usr/local/share/openhd/debug.txt
   sudo cp "openhd-share/interface/networking_settings.json" /usr/local/share/openhd/interface/networking_settings.json
   sudo cp "openhd-share/interface/wifibroadcast_settings.json" /usr/local/share/openhd/interface/wifibroadcast_settings.json
   sudo cp "openhd-share/telemetry/ground_settings.json" /usr/local/share/openhd/telemetry/ground_settings.json
   sudo cp "systemd/openhd.service" /etc/systemd/system/openhd.service
   ```
4. **Edit `hardware.config`** to reference the actual network interface names on this machine (`ip -br link` to list them) — the wifibroadcast radio adapter must be the same model/chipset (and same regulatory-domain fix, see the air unit's `regdomain.conf`) as the one used on the air unit.
5. **Generate or copy the shared `txrx.key`** and place it at `/usr/local/share/openhd/txrx.key` on this machine — copy the exact same file used on the air unit (see "Not included, on purpose" above). Without a matching key on both ends, the link will not establish at all.
6. **Reload and enable the service**:
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable --now openhd.service
   ```
7. **Point a GCS app at it**: install [QGroundControl](https://qgroundcontrol.com/) and/or QOpenHD on this machine (or a laptop/phone connected to its hotspot) — telemetry and video should appear automatically once the air unit powers up and the radio link locks.
8. **Verify the link**: `systemctl status openhd.service` should show it running with no crash-loop, and QGroundControl should show "GCS connection regained" with live attitude/battery data, plus a live video feed from the air unit's D435I stream once both units are within range and on the same key/frequency.

## Known open issue (shared with the air unit)

The wifibroadcast radio adapters on both ends have a confirmed driver bug (`rtl88x2bu_ohd` driver): they never actually switch to the frequency configured in `wifibroadcast_settings.json`, always staying on channel 1 (2412MHz) regardless of what's requested. Both the air and ground units currently just operate on channel 1 as a workaround — see the air unit's README for the full write-up.
