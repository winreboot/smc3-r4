# smc3_scope: the page server

`smc3_scope.py` is a single file that uses only the Python standard library (3.8 or newer). It connects to the
board's telemetry port over WiFi, keeps the last two minutes of data, and serves the page. It also runs the test
signals, auto-tune, the watcher and the diagnostics. SimHub keeps the USB port; the page never needs it.

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
| `config.json` | Settings made in the page: board address, theme, motors in use, motor names. |
| `profiles.json` | Named profiles. |
| `backups.json` | Backups of the board settings: every Save to EEPROM, every bigger change, and manual ones. |
| `diagnostics/events.jsonl` | The event log. |
| `diagnostics/incidents/` | Problems the watcher noticed, each with a few seconds of telemetry. The newest 40 are kept. |
| `diagnostics/reports/` | Diagnostics files from the quick and full checks. The newest 20 are kept. |

To upgrade, rebuild with the new `smc3_scope.py`. Keep the data folder, and everything carries over.

## Security

The page has no login, and it can move the rig and change its settings. Keep it on your home network; don't
forward its port to the internet. If you need it away from home, use a VPN such as WireGuard.
