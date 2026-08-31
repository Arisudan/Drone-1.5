# Pixhawk 6X + D435i + Radxa Dragon Q6A — Companion Computer Telemetry Relay

Progress log for setting up a Pixhawk 6X flight controller and Intel RealSense D435i depth camera on Ubuntu, then relaying live telemetry from a Radxa Dragon Q6A companion computer to a laptop over WiFi — from scratch, as a beginner.

**Goal:** `Pixhawk 6X ---USB---> Radxa Dragon Q6A ---WiFi---> Laptop (QGroundControl)`, with the D435i also connected to the Radxa, all lag-free.

---

## Hardware

| Component | Role |
|---|---|
| Pixhawk 6X | Flight controller (PX4/ArduPilot) |
| Intel RealSense D435i | Depth + RGB camera with onboard IMU |
| Radxa Dragon Q6A (Qualcomm QCS6490, Ubuntu 24.04) | Companion computer / telemetry relay |
| Laptop (Ubuntu) | Ground control station |

## Software used

- **QGroundControl** — graphical ground control station (GCS), reads MAVLink telemetry
- **librealsense / realsense-viewer** — Intel's SDK for the D435i
- **mavlink-router** — lightweight daemon that relays MAVLink between a serial/USB port and UDP/TCP endpoints

---

## Progress Log

### Day 1 — Aug 31, 2026

#### ✅ Step 1: Direct Pixhawk 6X → Laptop telemetry
- Connected Pixhawk 6X to laptop via USB-C.
- Hit a `Permission denied` error on `/dev/ttyACM0` in QGroundControl — root cause was the user account not being in the `dialout` group (or a group change not yet applied to the active login session).
- Fix:
  ```bash
  sudo usermod -aG dialout "$(id -un)"
  # then fully log out and log back in (not just close/reopen terminal)
  ```
- Verified fix by checking `groups` output included `dialout`.
- QGroundControl connected successfully — confirmed via "GCS connection regained" message and live attitude/battery data.
- Expected (not actual) warnings seen, since no GPS/RC connected yet: `No valid position estimate`, `No manual control input`. Both are normal without GPS/RC hardware.

**Screenshot — QGroundControl connected, live status:**

![QGroundControl connected showing vehicle messages and status](images/qgc-connected-status.png)

#### ✅ Step 2: D435i camera → Laptop
- Connected D435i to a USB 3.0 port.
- librealsense already installed; `realsense-viewer` opens and streams color + depth successfully.
- *(Note: Intel's apt package repository for librealsense was found to be unreliable at time of writing — building from source is the more dependable install path if needed again.)*

#### ✅ Step 3: MAVProxy installed as a CLI alternative to QGroundControl
- Hit Ubuntu's `externally-managed-environment` pip restriction (PEP 668, common on Ubuntu 24.04+).
- Fix:
  ```bash
  python3 -m pip install mavproxy pymavlink --user --upgrade --break-system-packages
  ```

#### ✅ Step 4: Architecture upgrade — Radxa Dragon Q6A as companion computer
New data path: `Pixhawk (USB) → Radxa Dragon Q6A → WiFi → Laptop`.

- **Physical connections on the Radxa:**
  - D435i → the single USB 3.1 port (needs USB3 bandwidth)
  - Pixhawk 6X → a USB 2.0 port (telemetry is low-bandwidth, USB2 is enough)
- Repeated the `dialout` group fix on the Radxa itself.
- Installed **mavlink-router** from source:
  ```bash
  sudo apt install -y git meson ninja-build pkg-config gcc g++ systemd libsystemd-dev
  git clone https://github.com/mavlink-router/mavlink-router.git
  cd mavlink-router
  git submodule update --init --recursive
  meson setup build .
  ninja -C build
  sudo ninja -C build install
  ```
- Config (`/etc/mavlink-router/main.conf`) relays the Pixhawk's USB serial to the laptop's IP over UDP:
  ```ini
  [General]
  TcpServerPort=5760
  ReportStats=true
  MavlinkDialect=auto

  [UartEndpoint pixhawk]
  Device=/dev/ttyACM0
  Baud=115200

  [UdpEndpoint laptop]
  Mode=normal
  Address=<laptop_ip>
  Port=14550
  ```
- Ran manually to verify:
  ```bash
  mavlink-routerd -c /etc/mavlink-router/main.conf
  ```
  Output confirmed a clean startup: UART opened on `/dev/ttyACM0`, UDP client opened toward the laptop's IP on port 14550, no errors.
- **Result: QGroundControl on the laptop auto-connected over WiFi** (port 14550 is QGC's default autoconnect UDP port) — full relay chain working end-to-end.

#### ✅ Step 5: Latency verification (in progress)
Rather than assume the WiFi relay is "flawless," ran actual measurements:

1. **MAVLink message rates** (QGroundControl → Analyze Tools → MAVLink Inspector):
   - `ATTITUDE`: **92.0 Hz** (well above the typical 10–20Hz default — strong signal the relay is keeping up)
   - `HIGHRES_IMU`: 46.0 Hz, `LOCAL_POSITION_NED` / `ODOMETRY`: 27.6 Hz — all steady, no dropouts

   ![MAVLink Inspector showing ATTITUDE at 92.0Hz with live roll/pitch/yaw data](images/mavlink-inspector-attitude-92hz.png)

2. **Raw WiFi latency** (`ping -c 30 <radxa_ip>` from the laptop):
   - `rtt min/avg/max/mdev = 7.642/63.268/729.604/158.832 ms`
   - 0% packet loss, but **two spikes of 730ms and 574ms** pulled the average up and produced high jitter (`mdev`) — indicating the connection is mostly good but has occasional stalls.

3. **Root cause investigation:**
   - Checked `iwconfig` on the Radxa: **`Power Management: on`** on `wlan0`, signal `-65 dBm`, link quality `45/70`.
   - WiFi power-saving is the leading suspect for the latency spikes (it periodically sleeps the radio to save power, delaying packet delivery).
   - **Fix applied on Radxa:**
     ```bash
     sudo iw dev wlan0 set power_save off
     ```
   - Same check pending on the laptop's WiFi adapter.
   - Re-test (ping + MAVLink rates) planned after power-save is disabled on both ends, to confirm the spikes are gone.

---

## Current Status

| Pipeline | Status |
|---|---|
| Pixhawk 6X → Laptop (direct USB) | ✅ Working |
| D435i → Laptop (direct USB) | ✅ Working |
| MAVProxy on laptop | ✅ Installed |
| Pixhawk 6X → Radxa → Laptop (WiFi relay via mavlink-router) | ✅ Working, latency being tuned |
| D435i → Radxa (companion computer) | ✅ Connected, not yet streamed off-board |

## Known Issues / In Progress

- [ ] Confirm WiFi power-save disabled on **both** Radxa and laptop, then re-run latency test
- [ ] Improve WiFi signal quality (currently -65 dBm / 45-70 link quality on the Radxa)
- [ ] Make `mavlink-router` auto-start on Radxa boot via `systemd`
- [ ] Decide on approach for streaming D435i data off the Radxa (not yet started)

## Next Steps

- Re-test latency after WiFi power-save fix; document before/after numbers
- Set up `mavlink-router` as a persistent `systemd` service
- Evaluate whether to bring D435i data (e.g. via ROS2 + realsense-ros) into the same companion-computer pipeline

---

*This is a living log — updated as the project progresses.*
