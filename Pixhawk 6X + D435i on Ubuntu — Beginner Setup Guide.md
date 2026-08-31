# Pixhawk 6X + D435i on Ubuntu — Beginner Setup Guide

**Goal:** See live Pixhawk flight telemetry AND live D435i camera video/depth on your Ubuntu laptop. These are two completely separate pipelines — the camera does not need to go through the Pixhawk at all for monitoring.

Do each step in order. Don't skip ahead — most failures later happen because a permission or package step earlier was missed.

---

## PART 1 — Pixhawk 6X Telemetry to Laptop

### Step 1: Check your Ubuntu version
Open a terminal (Ctrl+Alt+T) and run:
```bash
lsb_release -a
```
Note the "Release" number (e.g. 22.04, 24.04). This matters later — QGroundControl's ready-made app currently only officially supports **Ubuntu 24.04 or 26.04**. If you're on 22.04, note this now (see the box at the end of Step 4).

### Step 2: Connect the Pixhawk 6X
Plug the Pixhawk 6X into your laptop using a USB-C cable (into the Pixhawk's USB port, not TELEM1/2 — those are for radio telemetry, not for this).

Check Ubuntu sees it:
```bash
ls /dev/tty*
```
Run this once with the Pixhawk unplugged, then again after plugging it in — a new entry like `/dev/ttyACM0` appearing confirms it's detected. You can also run:
```bash
dmesg | tail -n 20
```
right after plugging in — you should see kernel messages mentioning a new USB serial/ACM device.

### Step 3: Give yourself permission to use the serial port
Linux blocks normal users from serial devices by default. Fix this:
```bash
sudo usermod -aG dialout "$(id -un)"
```
`dialout` is the Linux group allowed to talk to serial ports. This won't take effect until you get a fresh login session:
```bash
```
**Log out of Ubuntu completely and log back in** (a terminal restart is not enough).

Optional but recommended — Ubuntu's ModemManager service sometimes grabs the port before your software can:
```bash
sudo systemctl mask --now ModemManager.service
```

### Step 4: Install QGroundControl (the app that shows telemetry)
Install a few required libraries first:
```bash
sudo apt update
sudo apt install -y libfuse2 libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0
```
Download QGroundControl (this is an AppImage — a single executable file, no installer needed):
```bash
cd ~/Downloads
wget https://d176tv9ibo4jno.cloudfront.net/builds/master/QGroundControl-x86_64.AppImage
chmod +x QGroundControl-x86_64.AppImage
```
Run it:
```bash
./QGroundControl-x86_64.AppImage
```

> **If you're on Ubuntu 22.04:** the official AppImage won't run. Easiest workaround — use an older QGroundControl release from https://github.com/mavlink/qgroundcontrol/releases (look for a 4.x AppImage build), or upgrade Ubuntu, or use `MAVProxy` (a lighter, terminal-only alternative) instead: `pip3 install mavproxy` then `mavproxy.py --master=/dev/ttyACM0`.

### Step 5: Confirm telemetry is working
With the Pixhawk plugged in and QGroundControl open, it should auto-connect within a few seconds — you'll see a vehicle icon, artificial horizon, battery %, and GPS status appear. If nothing connects, go to the **Application Settings → Comm Links** area inside QGroundControl and manually add a Serial link pointing to `/dev/ttyACM0` (or whatever device name you saw in Step 2), baud rate `57600`.

**Part 1 done — you now have live Pixhawk telemetry on your laptop.**

---

## PART 2 — D435i Camera to Laptop

### Step 6: Connect the camera
Plug the D435i into a **USB 3.0 port** (usually colored blue inside, or labeled "SS"). The D435i needs USB3 speed — USB2 ports often won't provide enough bandwidth for depth+color streams.

Verify Ubuntu sees it:
```bash
lsusb | grep Intel
```
You should see a line mentioning Intel Corp / RealSense.

### Step 7: Install librealsense (Intel's camera SDK)
> **Note:** Intel's own apt package repository (`librealsense.intel.com`) has been unreliable recently (DNS/signing issues reported on their GitHub). The **build-from-source** method below is currently the more dependable path, so that's what this guide uses.

Install build dependencies:
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
sudo apt install -y git wget cmake build-essential
sudo apt install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
```
Download the source code:
```bash
cd ~
git clone https://github.com/realsenseai/librealsense.git
cd librealsense
```
Set up camera permissions (udev rules) — **unplug the D435i before running this**:
```bash
./scripts/setup_udev_rules.sh
```
Patch and build the kernel driver (for Ubuntu 20/22/24 with a recent kernel):
```bash
./scripts/patch-realsense-ubuntu-lts-hwe.sh
```
Check it worked:
```bash
sudo dmesg | tail -n 50
```
You should see a line about a new `uvcvideo` driver being registered.

### Step 8: Build the SDK itself
```bash
mkdir build && cd build
cmake ../ -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=Release
make -j$(($(nproc)-1))
sudo make install
```
This step can take 10–30 minutes depending on your laptop — that's normal.

### Step 9: View the live camera feed
Plug the D435i back in, then run:
```bash
realsense-viewer
```
A window opens showing color video and a colorized depth map, plus IMU data (accelerometer/gyroscope) if you enable the "Motion Module" toggle inside the app.

**Part 2 done — you now have live D435i video on your laptop, completely independent of the Pixhawk.**

---

## Troubleshooting quick reference

| Symptom | Likely cause | Fix |
|---|---|---|
| QGC never connects | Permission or ModemManager issue | Re-check Step 3, confirm you fully logged out/in |
| `/dev/ttyACM0` doesn't appear | Bad cable or port | Try a different USB-C cable/port; some cables are power-only |
| `realsense-viewer` shows no camera | Not on USB3, or driver patch failed | Try a different port; re-run Step 7's patch script; check `dmesg` |
| librealsense build fails on `openssl` | Missing dependency | `sudo apt install libssl-dev` and retry `cmake`/`make` |
| librealsense build fails on `fastrtps` | Optional DDS feature can't build | Re-run cmake with `-DBUILD_WITH_DDS=OFF` |

---

## What you'll have at the end
- QGroundControl open in one window showing live Pixhawk flight data.
- `realsense-viewer` open in another window showing live D435i color + depth.
- No wiring or software link between the two is needed for this monitoring-only setup.
