# Phase 2 — Pixhawk → Radxa Dragon Q6A → Laptop (WiFi telemetry relay)

**Architecture:**
```
Pixhawk 6X --USB--> Radxa Dragon Q6A --WiFi--> Laptop (QGroundControl)
D435i      --USB3-->      (same board, viewed/processed locally or later streamed)
```

The Radxa Dragon Q6A runs Ubuntu 24.04, so most commands look identical to what you did on your laptop. The key new piece of software is **mavlink-router** — a lightweight relay daemon built exactly for this job: it reads MAVLink off a serial/USB port and re-sends it over UDP to another machine, with very low overhead and low latency. This is the standard tool companion computers use (used across PX4/ArduPilot ecosystems) — much better suited here than MAVProxy or QGroundControl itself.

---

## PART 1 — Hardware connections on the Radxa

The Dragon Q6A has **one USB 3.1 port and three USB 2.0 ports**. Bandwidth matters here:

- **D435i → the single USB 3.1 port.** It needs USB3 speed for full-res color+depth. Don't put it on a USB2 port or streams will be limited/laggy.
- **Pixhawk 6X → any USB 2.0 port.** Telemetry is tiny (a few KB/s), USB2 is more than enough.

## PART 2 — Get the Radxa and laptop talking on WiFi

1. On the Radxa (via its own screen/keyboard, or SSH if already networked), connect it to the **same WiFi network** your laptop is on — ideally the **5GHz band** of your router, since it has less interference and lower latency than 2.4GHz.
2. Find the Radxa's IP address:
```bash
hostname -I
```
Note this down (e.g. `192.168.1.45`).
3. Find your laptop's IP address (on the laptop):
```bash
hostname -I
```
Note this down too (e.g. `192.168.1.30`).
4. (Optional but very helpful) Set up SSH so you can control the Radxa from your laptop's terminal instead of needing a separate monitor/keyboard for it:
```bash
# On the Radxa:
sudo apt install -y openssh-server
sudo systemctl enable --now ssh
```
Then from your laptop:
```bash
ssh <radxa_username>@<radxa_ip>
```
All the remaining Radxa commands below can be run through this SSH session.

## PART 3 — Serial permissions on the Radxa (same fix as before)

```bash
sudo usermod -aG dialout "$(id -un)"
```
Log out and back in (or reboot the Radxa) for this to apply — exactly like on your laptop earlier.

Plug in the Pixhawk and confirm it's seen:
```bash
ls /dev/tty*
```
Look for `/dev/ttyACM0` (or similar).

## PART 4 — Install mavlink-router on the Radxa

Install build tools:
```bash
sudo apt update
sudo apt install -y git meson ninja-build pkg-config gcc g++ systemd libsystemd-dev
```
Clone the repo and pull in its dependencies:
```bash
cd ~
git clone https://github.com/mavlink-router/mavlink-router.git
cd mavlink-router
git submodule update --init --recursive
```
Build and install:
```bash
meson setup build .
ninja -C build
sudo ninja -C build install
```

## PART 5 — Configure the relay (Pixhawk USB → your laptop's IP)

Create the config folder and file:
```bash
sudo mkdir -p /etc/mavlink-router
sudo nano /etc/mavlink-router/main.conf
```
Paste this in (replace `192.168.1.30` with **your laptop's actual IP** from Part 2):
```ini
[General]
TcpServerPort=5760
ReportStats=false
MavlinkDialect=auto

[UartEndpoint pixhawk]
Device=/dev/ttyACM0
Baud=115200

[UdpEndpoint laptop]
Mode=normal
Address=192.168.1.30
Port=14550
```
Save and exit (Ctrl+O, Enter, Ctrl+X in nano).

**Why port 14550:** QGroundControl automatically listens for incoming MAVLink on UDP port 14550 by default — so once mavlink-router sends data there, QGroundControl on your laptop should pick it up **with no manual configuration needed** on the laptop side.

## PART 6 — Run it

Test it manually first (easier to see errors):
```bash
mavlink-routerd -c /etc/mavlink-router/main.conf
```
Leave this running, then open QGroundControl on your laptop as usual. It should auto-connect within a few seconds, exactly like the direct-USB test you already did — except now the Pixhawk is physically connected to the Radxa, not your laptop.

Once you confirm it works, make it start automatically on every Radxa boot:
```bash
sudo systemctl enable mavlink-router
sudo systemctl start mavlink-router
sudo systemctl status mavlink-router
```

## PART 7 — Keeping it lag-free

| Do this | Why |
|---|---|
| Use 5GHz WiFi, not 2.4GHz | Less interference, lower latency |
| Keep Radxa and laptop reasonably close to the router (or each other, if using direct WiFi) | WiFi latency rises sharply with weak signal |
| Avoid heavy WiFi traffic on the same network while testing (large downloads, video calls) | Shared airtime directly adds latency/jitter |
| Consider the Radxa's WiFi in **Access Point mode**, with your laptop connecting directly to it | Removes the router as a hop entirely — the most reliable low-latency option once you're comfortable with the basics |
| Don't run both mavlink-router and QGroundControl's own USB connection to the same Pixhawk at once | They'll fight over the same serial port |

---

## Quick troubleshooting

- **QGroundControl doesn't connect:** double check the laptop's IP hasn't changed (WiFi DHCP can reassign it) — re-run `hostname -I` on the laptop and update `main.conf` if needed.
- **`mavlink-routerd` errors on startup about the config:** re-check indentation/brackets in `main.conf` — it's picky about exact section syntax.
- **Connects then drops repeatedly:** usually WiFi signal strength — check `iwconfig` or your router's connected-devices signal indicator on the Radxa.
