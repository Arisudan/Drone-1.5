# Link range test (`link_range_test.py`)

A walk-away test for the camera-feed Wi-Fi link. It answers one question:

> **How far can I carry the laptop from the Radxa/router before the live feed loses a packet or stalls?**

Usage is in [`guide.md`](../guide.md#10-range-test-how-far-does-the-live-feed-run-without-packet-loss).
This page is the reference: how it measures, how every number is defined, and what it cannot tell you.

## Files

| File | Runs on | Purpose |
|---|---|---|
| `scripts/diagnostics/link_range_test.py` | laptop | The engine (`Session`), parsers, analysis, report writer, terminal flow, and the entry point. |
| `scripts/diagnostics/link_range_gui.py` | laptop | The window: setup → walking → result. Opened when you run with no arguments. |
| `scripts/diagnostics/link_range_walk.py` | laptop | Continuous-walk mode: per-second points, the seconds-and-dBm verdict, its report, the terminal stream. |
| `scripts/diagnostics/link_probe_server.py` | Radxa | Tiny UDP echo helper for the packet-loss probe. Standalone; copied to `/tmp` and started over SSH, exits when idle. |
| `tests/test_link_range.py`, `tests/test_link_range_walk.py`, `tests/test_link_range_gui.py` | — | Hermetic tests (loopback only; the window tests run on a private Xvfb display and are skipped without it). |

## Continuous-walk mode (`--walk`, or the first option in the window)

Instead of stopping at measured marks, you walk away at your own pace and the test streams one line of
values per second. **No distance is recorded or estimated** — the answer is in seconds and dBm, the two things
the laptop actually knows.

**How a second is judged.** Each second, the 2 s ending then is analysed with the same code and strict rules
as a hold: any ping or UDP loss beyond the allowed tolerance, any video reconnect, or a gap of more than 0.5 s
between frames makes it **LOSS**. A window with no loss figure at all is **unknown**, never clean. (2 s rather
than 1 s so a single reply still in flight at the window edge cannot look like loss; loss itself is counted from
sequence numbers, as in a hold.)

**The verdict**

| Item | Meaning |
|---|---|
| loss-free for *N* s | verified loss-free seconds before the first LOSS second |
| down to *X* dBm | weakest signal reached during that clean run |
| first loss at *t* s (*Y* dBm) | when it first broke, at what signal, and why |
| by signal level | for each 5 dB band: seconds spent, loss-free seconds, % clean |
| "signal alone did not decide it" | printed when loss happened more than 3 dB above the weakest clean signal — look for walls, corners, interference |

Outputs in the same folder layout: `report.html` / `report.md` (time-axis charts: signal, video fps, longest
gap, data lost), `walk.json` (every second), `summary.json`, `samples.csv`.

Terminal: `--walk` (Enter stops), `--walk-seconds N` (stop by itself), `--auto` (start without waiting for Enter).
`--demo --walk` writes a synthetic report. Limits: a walk is a single pass, so it shows where it first broke but
does not average like a 15 s hold; moving through a room is also noisier than standing still.

## Setup assumed

The Radxa and the router stay together; **only the laptop moves**. The one radio link that changes is the
laptop's Wi-Fi to the router, so that is where signal strength and retries are read. Everything else (ping, the
camera stream, UDP) is measured end to end to the Radxa. The laptop must be on the **same Wi-Fi network as the
Radxa** — the window shows which network it is on and warns if the Radxa doesn't answer.

## Running it

```
python3 link_range_test.py                      # window (asks for the Radxa IP)
python3 link_range_test.py --cli --radxa 172.16.100.182 --udp --start-helper radxa@172.16.100.182
python3 link_range_test.py --demo               # synthetic data -> a sample report, no network
```

Useful options: `--step` (metres between marks, 5), `--hold` / `--baseline-hold` (seconds standing still, 15),
`--loss-tolerance` (percent still counted as loss-free, 0), `--udp-mbps` (probe rate, 1), `--no-video`,
`--video-url`, `--control-ssh user@host`, `--auto-marks 0,5,10 --walk-time 20` (no keypresses; timers instead),
`--out DIR`. The window remembers the last IP and settings in `~/.link_range_test.json`.

## What is measured

| Layer | Measurement | Source |
|---|---|---|
| Wi-Fi radio | signal (dBm), link speed, TX retries and failures per second | `iw dev <if> link` / `station dump` on the laptop (`/proc/net/wireless` as a fallback) |
| Network | ping round-trip (avg / p95 / max), jitter, % lost — small (56 B) and video-sized (1200 B) | `ping -i 0.2` |
| Camera feed | frames/s, Mbit/s, longest gap between frames (stall), reconnects | the real MJPEG stream, parsed by counting JPEG start/end markers |
| Packet loss | UDP packets lost **up** (laptop→Radxa) and **down** (Radxa→laptop) | numbered probe datagrams echoed by the Radxa helper |
| Control | the Radxa's own Wi-Fi signal before and after | `ssh … iw dev … link` (optional) |

It sends only test traffic. It sends no MAVLink command and changes nothing on the vehicle; the helper writes only
to `/tmp` on the Radxa and removes itself afterwards.

## How distance is known

Indoors there is no GPS, so **you provide it**: stand at a measured spot (tape, floor tiles), press the button, and
everything recorded while you stand there is tagged with that distance (a "hold"). Seconds spent walking between
marks are recorded in `samples.csv` as `walk` with no distance and are not part of any verdict.

A log-distance path-loss model, `signal(d) = signal_1m − 10·n·log10(d)`, is least-squares fitted to the holds at 1 m
or more (needs three distinct distances) so a distance can be *estimated* from signal alone afterwards. Indoors
this is rough — reflections and walls dominate — and the report says so. The 0 m baseline is excluded from the fit
(log of zero).

## The headline: "loss-free up to N m"

A hold is **loss-free** when *nothing* was lost:

- every measured loss figure (ping, large ping, UDP up, UDP down) is within the allowed tolerance (**0 % by default**),
- the video never reconnected, and
- the longest gap between video frames was no more than 0.5 s.

A hold with **no loss figure at all is "unknown"**, never loss-free — missing data must not read as a clean link.
**`loss_free_to`** is the furthest distance such that *every* hold up to it is loss-free; one lossy hold ends the
range even if a farther one happens to be clean. Alongside it:

| Range | Meaning |
|---|---|
| `loss_free_to` | headline: nothing lost, no stall, at every mark up to here |
| `good_to` | every hold up to here has verdict **Good** (a small loss is tolerated) |
| `marginal_to` | no **Poor** verdict up to here |

### Verdicts (per hold)

| Reading | Good if | Poor if |
|---|---|---|
| signal | ≥ −70 dBm | < −80 dBm |
| ping / UDP loss | ≤ 2 % | > 10 % |
| longest video stall | ≤ 0.5 s | > 2 s |
| video frames/s vs the 0 m baseline | ≥ 80 % | < 50 % |
| video reconnects | none | (any → at least Marginal) |

Between the two is **Marginal**; the worst reading wins and the reasons are listed. The limits are constants
(`THRESHOLDS`) at the top of `link_range_test.py`.

## How loss is counted (why it can report an honest zero)

Comparing replies received with `duration ÷ interval` can never give a strict 0 %: a reply still in flight when a
hold ends makes a perfect link look one or two packets short. So loss is counted from **sequence numbers**:

- **Ping:** packets missing *inside* the received sequence span are lost, plus a dead stretch at the start or end if it
  lasted more than 1 s. A hold with no reply at all is 100 %.
- **UDP:** each echo carries its own sequence number `s` and the helper's running count `c` of probes received. Between
  the first and last echo in a hold:
  `up lost = (s₂ − s₁) − (c₂ − c₁)` (sent minus received) and
  `down lost = (c₂ − c₁) − (echoes between)` (received minus echoed back). Exact, with nothing in flight at the edges.
  Silence longer than 1 s at either end is added to both directions (when the link goes quiet the direction cannot
  be told apart). No echo at all = 100 % both ways.

Counters are read **across a reset**: the Wi-Fi card's counters restart when it roams to another access point, so a
plain last-minus-first goes negative; a drop is treated as a restart.

## Output (`--out`, default `~/link_range_results/<timestamp>/`)

| File | Contents |
|---|---|
| `report.html` | the readable result: headline, findings, per-distance table, "why not Good", four charts, run details |
| `report.md` | the same table and findings as Markdown |
| `holds.json` | every statistic for every hold |
| `summary.json` | ranges, path-loss fit, thresholds, findings, run metadata |
| `samples.csv` | 1 Hz timeline (`hold`/`walk`, distance, signal, loss, fps, …) |

Charts are inline SVG (no matplotlib needed) in the station's restrained palette: grey/blue lines; amber and red
only for caution and poor.

## Limits — read these before trusting a number

- **Distance is as accurate as your marks.** The signal-based estimate is only a rough cross-check.
- **Not true glass-to-glass latency.** Ping and stream timing are measured from the laptop; encoder and camera delay
  are not observable from here.
- **The laptop's Wi-Fi is the variable.** If the *Radxa's* link also changes (it is wireless and moved, or the router
  roams it), the control reading before/after is what tells you.
- **UDP probe rate is low on purpose** (1 Mbit/s) so it does not disturb the video it shares the link with. To stress the
  link at video bitrate, use `--udp-mbps 8 --no-video`.
- **Strict zero is strict.** A single lost ping out of ~75 in a 15 s hold is 1.3 % and breaks "loss-free"; raise
  *Allowed loss* if one stray packet shouldn't end the range.
- **Not yet run against the real Radxa.** It was verified on this laptop's real Wi-Fi card, on loopback against
  stand-ins for the camera and the helper, and in the window on a virtual display. The over-SSH start of the helper
  and a real walk have not been run.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| "The Radxa does not answer" | The laptop is on a different Wi-Fi network (e.g. `HTIC_INCUBATION` instead of `HTIC_RND`: `nmcli connection up HTIC_RND`), the Radxa is off, or its IP changed between networks. |
| Packet loss reads "off" | The window and the report now say why, in plain words under the readouts, on the result screen and in the run details (`udp: off - …`): *not selected*; *SSH login failed – run ssh-copy-id radxa@<ip>*; *cannot reach <ip> over SSH (Radxa off, wrong IP, other Wi-Fi network)*; *python3 is not installed on the Radxa*; *the helper exited at once – see /tmp/link_probe.log*. Fix that, or untick the option (ping and the stream are still measured). |
| Camera feed "off" / 0 fps | `camera.sh` / the video streamer is not running on the Radxa; check `curl -I http://<ip>:8080/video`. |
| "Marginal" at 0 m | The laptop's signal is already below −70 dBm next to the router — worth knowing before walking anywhere. |
| No window opens | No display or `tkinter` (`sudo apt install python3-tk`); it falls back to asking for the IP in the terminal. |
