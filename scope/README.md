# smc3_scope: the page server

`smc3_scope.py` is a single file that uses only the Python standard library (3.8 or newer). It connects to the
board's telemetry port over WiFi, keeps the last two minutes of data, and serves the page. It also runs the test
signals, auto-tune and Check, Seat setup, moving the motors from the page (2.4), the watcher and the diagnostics. SimHub keeps the USB port; the page never needs it.

The 3D view's engine, three.js r160 (MIT licence, served at `/vendor/three/LICENSE.txt`), is built into the file,
so the page works on a network without internet access.

## Running it

Step by step for Windows, Mac, Linux, Raspberry Pi, Docker and Synology: [docs/INSTALL.md](../docs/INSTALL.md).

**Docker, ready-made image** (`ghcr.io/winreboot/smc3-scope`, built for every release; amd64, arm64, arm/v7):

```sh
docker compose -f compose.prebuilt.yaml up -d     # from this folder
```

**Docker, built here** (recommended for a NAS or home server if you change the code):

```sh
docker compose up -d --build        # from this folder
docker logs -f smc3-scope           # connects, events, every auto-tune and diagnostics step
```

**Directly:**

```sh
python3 smc3_scope.py --host 192.168.1.50 --bind 0.0.0.0 --web 8198
```

| Option | Environment variable | Default | Meaning |
|---|---|---|---|
| `--host` | `SMC3_HOST` | (none) | The board's address. You can also set it in the page; the page's setting wins after the first start. |
| `--port` | `SMC3_PORT` | 4210 | `TELEMETRY_PORT` in the firmware. |
| `--web` | `SMC3_WEB_PORT` | 8080 (compose: 8198) | Port of the page. If it's busy, the next free one is used. |
| `--bind` | `SMC3_BIND` | 127.0.0.1 (Docker: 0.0.0.0) | 127.0.0.1 = this machine only; 0.0.0.0 = the whole LAN (the board fetches firmware updates from the page, so updates need it). |
| `--data` | `SMC3_DATA_DIR` | `./smc3-scope-data` (Docker: `/data`) | Where everything below is kept. |

## The data folder

| File | What it holds |
|---|---|
| `config.json` | Settings made in the page: board address, theme, motors in use, motor names, what motor 3 does (traction loss or a lift), which way the rear goes and each lift goes for higher numbers (and which way each lift's corner sank in the last full check), auto-tune's "who's in the seat" and rider's weight, and the test-move speed auto-tune measured for each motor (Check uses it). |
| `seat.json` | The last Seat setup: where the seat is level (and the rear centred), how each motor tilts it, how far the rear slide turns it per count, the ends found and the limits saved. The 3D view uses it; the limits themselves live on the board. |
| `profiles.json` | Named profiles. |
| `backups.json` | Backups of the board settings: every Save to EEPROM, every bigger change, and manual ones. |
| `diagnostics/events.jsonl` | The event log. |
| `diagnostics/incidents/` | Problems the watcher noticed, each with a few seconds of telemetry. The newest 40 are kept. |
| `diagnostics/reports/` | Diagnostics files from the quick and full checks. The newest 20 are kept. |
| `updates/` | A newer page downloaded from GitHub (Settings, Updates, 2.7) and `state.json`: which file, and whether it started. |

To upgrade, press **Update page** in Settings, Updates - or rebuild with the new `smc3_scope.py`. Keep the data
folder, and everything carries over.

The LED matrix and motion sensor settings (firmware 1.8) and the seat set-up (firmware 1.9: level trim, travel
limits, end stops, tilt limits) are not in the data folder: they live on the board itself, so they stay with the rig.

## Firmware updates

Settings, Firmware update (firmware 1.9.2 on the board) keeps the chosen file in memory only. While an update runs
the board fetches it from `http://<this machine>:<web port>/ota/<number>.ota`; that address answers only during an
update and only for its number.

## Updates from GitHub

Twice a day (and on **Check now**) the page asks `https://api.github.com/repos/winreboot/smc3-r4/releases/latest`
what the newest release has: `smc3_scope_v<version>.py` and `SMC3_R4_v<version>.ino` among its files.

- **Update page** downloads the page file into `<data>/updates`, checks GitHub's SHA-256 of it (when the release
  lists one), its `VERSION` line and that it compiles, writes `updates/state.json` (`new`) and restarts - once
  nothing is moving the motors - with the same options. The file you started decides what runs: at every start it
  hands over to the update if the update is newer than itself and isn't `off` or `failed` (`trying`, one more start
  counted). Once the update serves it says `ok`; after 10 minutes, or once its own page has loaded in a browser, the
  start count is cleared.
- A start that finds `trying` (the last one didn't get as far as serving) or three counted starts (it serves, then
  stops - a crash loop under a restart policy) sets `failed`, and the file you started runs instead.
  `SMC3_NO_UPDATE=1` skips the hand-over altogether.
- Restarting: `os.execv` on Linux, macOS and in Docker (same process - PID 1 stays PID 1). On Windows the file you
  started stays as a parent: it runs the page as a child and starts the right one again when the child exits with
  code 75 (`SMC3_RESTART_SPAWN=1` does the same on Linux, for testing).
- **Go back to built-in** sets `off` and restarts. Firmware is only offered as a download link; the browser fetches
  it from GitHub.
- The three `/update/` POSTs need the header `X-SMC3: 1`, which the page sends and another website open in the same
  browser can't (without a CORS preflight this server doesn't answer).

`update_check` in `config.json` (the tick box) switches the twice-a-day check off; `update_repo` (only in
`config.json`) points a fork at its own releases. `SMC3_GITHUB_API` replaces `https://api.github.com` for testing
(`tools/smoke_test.py` runs a stand-in).

## Moving the motors from the page

Dragging a motor card's knob, or Return to center (2.4, firmware 1.9 on the board), makes this server move the
motors itself: 50 times a second it sends each motor's position, eased in and out and never faster than
200 counts/s (120 for Return to center), and every second it renews the board's hold on SimHub's position
commands (`[kH 30]`: 3 s). So if this server stops, SimHub has the seat back within 3 s; Resume SimHub gives it back
at once (`[kH 0]`). A dropped connection to the board, or anything else that moves the motors, ends it too.

## The USB cable (2.8)

With firmware 1.9.5 (1.9.6 recommended) the page can reach the board over its USB cable instead of WiFi, for calibration with SimHub
closed. A bridge on the PC with the cable connects out to the page (`POST /usb/link` on the web port, then the board's
bytes both ways), the page drops its WiFi connection and uses the cable until the bridge ends; it sends `[kU1]` every
second so the board streams over USB and leaves WiFi alone. Windows: the page serves a PowerShell bridge at
`/usb-bridge.ps1` with its own address in it (Settings, USB cable shows the line for Win+R). Anywhere else:
`smc3_scope.py --usb-bridge PORT --page http://<page>:<port>` (pyserial if installed; Linux and macOS work without).
From 2.8.1 the bridge keeps running when the page goes back to WiFi: it closes the port and asks the page every second
(`POST /usb/standby`) until the header's WiFi / USB switch asks for the cable again (`--standby` starts it that way).
Details in [docs/PROTOCOL.md](../docs/PROTOCOL.md#the-usb-bridge-page-28).

## Security

The page has no login, and it can move the rig and change its settings. Keep it on your home network; don't
forward its port to the internet. If you need it away from home, use a VPN such as WireGuard.
