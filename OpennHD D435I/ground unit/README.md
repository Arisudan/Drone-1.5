# Ground Unit — OpenHD Configuration (D435I link)

Everything needed to reproduce the working ground-unit setup that receives the video/telemetry link from the [air unit](../air%20unit/README.md) (Radxa Dragon Q6A + Intel RealSense D435I). Files are laid out here exactly as they live on the actual ground machine — copy each into the matching path shown below.

The ground unit itself has no camera and does no SLAM — its job is to run OpenHD in ground mode, receive the wifibroadcast video/telemetry link from the air unit, and hand telemetry off to a GCS app (QGroundControl / QOpenHD) and video to a display.

## Not included, on purpose

**`/usr/local/share/openhd/txrx.key`** — the shared encryption key between this ground unit and its air unit. This is a secret, not a config file, so it's deliberately left out of this repo, same as on the air-unit side. It must be generated once (or copied from the air unit's key) and placed at this exact path on **both** units — air and ground must use the **identical** key file for the radio link to work at all.

Also excluded, as it's runtime state rather than setup config:
- `/usr/local/share/openhd/unit.id` — auto-generated unique ID for this unit, not needed to reproduce the setup.
- `/usr/local/share/openhd/telemetry/*.ohd` — recorded flight-telemetry logs from past sessions, not configuration.
- `/usr/local/share/openhd/recording.txt` — just a local path (`/home/openhd/Videos/`) where OpenHD saves recorded video; adjust to taste, no need to copy verbatim.
- The full `rtl88x2bu` driver source tree (~60MB) — not committed here. See **Wi-Fi radio driver** below for where to get it and the one-line patch needed instead.

## How ground vs. air mode is chosen

Unlike the air unit — which is marked by the presence of `/boot/openhd/air.txt` — this machine is a ground unit simply because **that file is absent**. There is no separate "ground.txt" marker required; OpenHD defaults to ground mode whenever `air.txt` is not present at `/boot/openhd/air.txt`.

## File map

| File in this repo | Installs to | What it does |
|---|---|---|
| `boot-openhd/hardware.config` | `/boot/openhd/hardware.config` | Tells OpenHD explicitly which network interfaces to use: `WIFI_WB_LINK_CARDS` for the wifibroadcast radio (must match the same physical radio type/band as the air unit's link card), and `WIFI_WIFI_HOTSPOT_CARD` for the local access point OpenHD raises so the GCS app can connect. Autodetection is disabled so the correct adapters are always picked. |
| `openhd-share/debug.txt` | `/usr/local/share/openhd/debug.txt` | Empty gate file. Without it present, this OpenHD build silently ignores `hardware.config` entirely (same requirement as on the air unit). |
| `openhd-share/interface/wifibroadcast_settings.json` | `/usr/local/share/openhd/interface/wifibroadcast_settings.json` | Radio link settings: frequency, MCS index, channel width, FEC, TX power. **Must match the air unit's radio parameters** (frequency in particular) for the two ends to find each other. Currently set to 5200MHz (channel 40, UNII-1 band) — moved off the original 5745MHz/channel 149 because that channel overlaps this site's office Wi-Fi APs and also happened to trip a driver DFS bug (see below); channel 40 is never DFS-gated in any regulatory domain and wasn't in use by anything else nearby. |
| `openhd-share/interface/networking_settings.json` | `/usr/local/share/openhd/interface/networking_settings.json` | Ethernet/WiFi-hotspot/WiFi-client operating modes for how the ground unit exposes itself to the GCS laptop/phone. Left at defaults here — adjust if you want the ground unit to join an existing WiFi network instead of hosting its own hotspot. |
| `openhd-share/telemetry/ground_settings.json` | `/usr/local/share/openhd/telemetry/ground_settings.json` | Ground-side telemetry routing: baud rate/serial settings if a flight controller is also plugged directly into the ground unit (for RC-over-joystick or a wired FC link), and UART priority ordering between FC, OpenHD, and RC. |
| `systemd/openhd.service` | `/etc/systemd/system/openhd.service` | Runs the main OpenHD program (`/usr/local/bin/openhd`) as a background service, auto-restarting on crash. Unlike the air unit (which uses a mode-specific `openhd-air.service`), the ground unit runs the single generic `openhd.service`, which auto-detects ground mode from the absence of `air.txt`. Enable with `systemctl enable --now openhd.service`. |
| `networkmanager/99-unmanage-wifibroadcast.conf` | `/etc/NetworkManager/conf.d/99-unmanage-wifibroadcast.conf` | Tells NetworkManager to never manage any `wlx*`-named interface (the udev naming convention for USB Wi-Fi adapters identified by MAC, which is what this TP-Link wifibroadcast dongle gets named). Without this, NetworkManager briefly grabs the interface and starts `wpa_supplicant` on it every time the driver is (re)loaded, before OpenHD claims it — this isn't the main cause of the driver crash described below, but it's an unnecessary source of interference and worth excluding regardless. Reload with `sudo systemctl reload NetworkManager` after installing. |
| `driver/hal8822b_fw.c.patch` | Applied to the driver source before building — see **Wi-Fi radio driver** below | One-line source fix required to build OpenHD's own `rtl88x2bu` driver fork on newer kernels. Not a config file installed onto the running system; a patch applied to the driver source tree before `dkms build`. |

## Wi-Fi radio driver — must be OpenHD's own fork, not a generic community driver

The wifibroadcast radio card (TP-Link Archer T3U Plus / AC1300, RTL8812BU chipset) **will not be recognized as a wifibroadcast card at all** if it's running the generic `rtl88x2bu` driver (e.g. from `morrownr/8812bu` or similar community forks). OpenHD's card-discovery code specifically checks the loaded kernel module's name and only accepts `rtl88x2bu_ohd` / `88x2bu_ohd` — anything else and OpenHD falls back to treating the card as a plain Wi-Fi hotspot adapter instead of the wifibroadcast link.

Build the correct driver:

```bash
git clone https://github.com/OpenHD/rtl88x2bu /usr/src/rtl88x2bu-5.13.1-git
```

**On recent kernels (this machine: `7.0.0-31-generic`), the driver as cloned fails to build** with undefined-symbol errors (`array_mp_8822b_fw_nic`, `array_length_mp_8822b_fw_nic`) at the modpost/link stage. Root cause: `hal/rtl8822b/hal8822b_fw.c` wraps its own `#include "drv_types.h"` inside an `#ifdef CONFIG_RTL8822B` guard, but that same header is what supplies the fallback `#define CONFIG_RTL8822B` when the build system doesn't pass it in as a compiler flag (which it doesn't, for this particular chip family's Makefile fragment on this build) — so the file's body gets skipped entirely and the firmware-array symbols it defines never exist. Fix by moving the include above the guard — apply the included patch before building:

```bash
patch -p1 -d /usr/src/rtl88x2bu-5.13.1-git < "driver/hal8822b_fw.c.patch"
```

Then build and install via DKMS:

```bash
sudo dkms add -m rtl88x2bu -v 5.13.1-git
sudo dkms build -m rtl88x2bu -v 5.13.1-git -k $(uname -r)
sudo dkms install -m rtl88x2bu -v 5.13.1-git -k $(uname -r)
sudo modprobe 88x2bu_ohd
```

Confirm it bound correctly: `lsusb -t` should show `Driver=rtl88x2bu_ohd` on the TP-Link device, and `lsmod | grep 88x2bu` should show `88x2bu_ohd` loaded.

### About the channel/frequency "not switching" — resolved, was a false alarm

An earlier note here claimed the driver never actually switches to the configured frequency, always staying on channel 1 (2412MHz). **That's not accurate** — it was a misreading of a deliberate feature. This chipset's OpenHD driver fork implements a regulatory-domain workaround for 5GHz channels: it writes the *real* desired channel to a driver sysfs parameter (`/sys/module/88x2bu_ohd/parameters/openhd_override_channel`) and then tells the kernel a harmless "dummy" frequency (which is what `iw dev` displays and what caused the original confusion) while the hardware silently tunes to the real overridden channel. To verify the *actual* channel in use, don't trust `iw dev` — check:

```bash
cat /sys/module/88x2bu_ohd/parameters/openhd_override_channel
```

and cross-reference against `sudo journalctl -k | grep openhd_override_channel`, which logs a `RTW: WARN OpenHD: using openhd_override_channel` line at the kernel level whenever the override is actually applied. Both confirmed the card was genuinely on the configured channel the whole time.

## Setting up a new ground unit from scratch

1. **Flash/install OpenHD** on the ground machine (Raspberry Pi, x86 laptop/mini-PC, etc.) following the official [OpenHD installation docs](https://www.openhdfpv.org/), or build from source — see **Building OpenHD from source** below if you need to match a specific air-unit version.
2. **Confirm it's in ground mode**: make sure `/boot/openhd/air.txt` does **not** exist on this machine. If it does, delete it — its presence is what would flip this unit into air mode.
3. **Build and install the Wi-Fi driver** — see **Wi-Fi radio driver** above. Skipping this step is the most common reason the wifibroadcast card silently gets treated as a hotspot-only device instead of the actual video/telemetry link.
4. **Copy the config files** from this folder to the paths listed in the table above (all as `root`, since `/boot/openhd`, `/usr/local/share/openhd`, and `/etc` are root-owned):
   ```bash
   sudo cp "boot-openhd/hardware.config" /boot/openhd/hardware.config
   sudo mkdir -p /usr/local/share/openhd/interface /usr/local/share/openhd/telemetry
   sudo cp "openhd-share/debug.txt" /usr/local/share/openhd/debug.txt
   sudo cp "openhd-share/interface/networking_settings.json" /usr/local/share/openhd/interface/networking_settings.json
   sudo cp "openhd-share/interface/wifibroadcast_settings.json" /usr/local/share/openhd/interface/wifibroadcast_settings.json
   sudo cp "openhd-share/telemetry/ground_settings.json" /usr/local/share/openhd/telemetry/ground_settings.json
   sudo cp "systemd/openhd.service" /etc/systemd/system/openhd.service
   sudo cp "networkmanager/99-unmanage-wifibroadcast.conf" /etc/NetworkManager/conf.d/99-unmanage-wifibroadcast.conf
   sudo systemctl reload NetworkManager
   ```
5. **Edit `hardware.config`** to reference the actual network interface names on this machine (`ip -br link` to list them) — the wifibroadcast radio adapter must be the same model/chipset as the one used on the air unit.
6. **Generate or copy the shared `txrx.key`** and place it at `/usr/local/share/openhd/txrx.key` on this machine — copy the exact same file used on the air unit (see "Not included, on purpose" above). Without a matching key on both ends, the link will not establish at all.
7. **Reload and enable the service**:
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable --now openhd.service
   ```
8. **Point a GCS app at it**: install [QGroundControl](https://qgroundcontrol.com/) and/or QOpenHD on this machine (or a laptop/phone connected to its hotspot) — telemetry and video should appear automatically once the air unit powers up and the radio link locks.
9. **Verify the link**: `systemctl status openhd.service` should show it running with no crash-loop, and QGroundControl/QOpenHD should show a live link with attitude/battery data plus video from the air unit's D435I stream once both units are within range and on the same key/frequency.

## Building OpenHD from source (to match a specific air-unit version)

If the air unit is running a build from a particular git ref rather than a release package, the ground unit's OpenHD version should match — mismatched builds can behave inconsistently even when the wire protocol is nominally compatible. This setup currently runs OpenHD built from the **`2.6-evo`** branch of [OpenHD/OpenHD](https://github.com/OpenHD/OpenHD) (17 commits past the `2.6.4` tag), to match the air unit reporting itself as `2.6.4-evo`:

```bash
git clone https://github.com/OpenHD/OpenHD.git
cd OpenHD
git checkout -B 2.6-evo origin/2.6-evo
git submodule update --init --recursive
cmake OpenHD/
make -j$(nproc)
sudo cp openhd /usr/local/bin/openhd
```

## Operational gotcha: never run `openhd` manually while the service is active

Running `sudo openhd` (or `sudo openhd -a` / `-g`) by hand in a terminal **while `openhd.service` is already running in the background** results in two `openhd` processes fighting for control of the same wifi card and camera/video ports simultaneously. In practice this caused a real, reproducible kernel-level crash in the radio driver's disconnect/DFS-decision path (`rtw_dfs_rd_en_decision`, `disconnect_hdl`) and split incoming/outgoing UDP video traffic across two competing sockets — symptoms that looked exactly like a flaky radio link or a bad camera pipeline, but were actually caused by having two OpenHD processes active on the same hardware at once. If you need to manually inspect or restart OpenHD, use the service instead:
```bash
systemctl status openhd.service
sudo systemctl restart openhd.service
journalctl -u openhd.service -f
```
Also note: running plain `openhd` (no `-a`/`-g` flag) does **not** ask which mode to run in — it checks for marker files and **silently defaults to ground mode** if neither is found. Always pass the explicit flag if running by hand at all.
