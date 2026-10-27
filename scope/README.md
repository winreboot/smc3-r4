# smc3_scope: the page server

`smc3_scope.py` is a single file that uses only the Python standard library (3.8 or newer). It connects to the
board's telemetry port over WiFi, keeps the last two minutes of data, and serves the page. It also runs the test
signals, auto-tune, Seat setup, the watcher and the diagnostics. SimHub keeps the USB port; the page never needs it.

The 3D view's engine, three.js r160 (MIT licence, served at `/vendor/three/LICENSE.txt`), is built into the file,
so the page works on a network without internet access.

## Running it

**Docker** (recommended for a NAS or home server):

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
| `--bind` | `SMC3_BIND` | 127.0.0.1 (Docker: 0.0.0.0) | 127.0.0.1 = this machine only; 0.0.0.0 = the whole LAN. |
| `--data` | `SMC3_DATA_DIR` | `./smc3-scope-data` (Docker: `/data`) | Where everything below is kept. |

## The data folder

| File | What it holds |
|---|---|
| `config.json` | Settings made in the page: board address, theme, motors in use, motor names, what motor 3 does (traction loss or a lift) and which way the rear goes for higher numbers. |
| `seat.json` | The last Seat setup: where the seat is level (and the rear centred), how each motor tilts it, how far the rear slide turns it per count, the ends found and the limits saved. The 3D view uses it; the limits themselves live on the board. |
| `profiles.json` | Named profiles. |
| `backups.json` | Backups of the board settings: every Save to EEPROM, every bigger change, and manual ones. |
| `diagnostics/events.jsonl` | The event log. |
| `diagnostics/incidents/` | Problems the watcher noticed, each with a few seconds of telemetry. The newest 40 are kept. |
| `diagnostics/reports/` | Diagnostics files from the quick and full checks. The newest 20 are kept. |

To upgrade, rebuild with the new `smc3_scope.py`. Keep the data folder, and everything carries over.

The LED matrix and motion sensor settings (firmware 1.8) and the seat set-up (firmware 1.9: level trim, travel
limits, end stops, tilt limits) are not in the data folder: they live on the board itself, so they stay with the rig.

## Security

The page has no login, and it can move the rig and change its settings. Keep it on your home network; don't
forward its port to the internet. If you need it away from home, use a VPN such as WireGuard.
