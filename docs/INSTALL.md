# Running the page, step by step

The page (`smc3_scope`) is **one Python file** that needs nothing but Python 3.8 or newer: no packages to install,
no database, no internet connection. Run it on whatever is on all the time near your rig:

| Where | How | Go to |
|---|---|---|
| Windows PC (the sim PC itself is fine) | Python + one file | [Windows](#windows) |
| Mac, Linux PC, Raspberry Pi | Python + one file | [macOS, Linux, Raspberry Pi](#macos-linux-raspberry-pi) |
| NAS, home server, any PC with Docker | the ready-made Docker image | [Docker: ready-made image](#docker-ready-made-image) |
| Synology NAS, without a command line | Container Manager | [Synology Container Manager](#synology-container-manager) |
| Docker, built from this repository | `docker compose --build` | [Docker: build it yourself](#docker-build-it-yourself) |

SimHub keeps the board's USB port as before; the page talks to the board over WiFi, so both run at the same time.

Once it runs, the page keeps itself up to date: **Settings, Updates** says when GitHub has a newer page or firmware,
and **Update page** installs the new page with one click, whichever way you run it ([Updates](#updates)).

## Before you start

1. **The board runs SMC3_R4 firmware and is on your WiFi.** Flashing is in [firmware/README.md](../firmware/README.md)
   (WiFi name and password go into the sketch; the board only does 2.4 GHz).
2. **Note the board's address.** It scrolls across the LED matrix after power-up (for example `192.168.1.50`).
   Reserve it in your router (DHCP reservation), so it stays the same - the board calls itself `smc3-r4` there.
3. The computer that runs the page must be on the **same network** as the board.

You can also skip the address: the page's **Settings, Find board on network** looks for it.

---

## Windows

1. **Install Python.** Download it from <https://www.python.org/downloads/> (3.12 or newer is fine). In the
   installer tick **Add python.exe to PATH**, then **Install Now**.
2. **Make a folder** for the page, for example `C:\smc3-scope`.
3. **Download the page** from the [latest release](https://github.com/winreboot/smc3-r4/releases/latest): the file
   `smc3_scope_v<version>.py` (under Assets). Save it into that folder.
4. **Start it.** Open the folder in Explorer, click the address bar, type `cmd` and press Enter. In the window that
   opens, type (use the name of the file you downloaded):

   ```bat
   py smc3_scope_v2.8.py --bind 0.0.0.0 --web 8198 --host 192.168.1.50
   ```

   `--host` is your board's address (you can leave it out and set it in the page). Keep this window open: closing
   it stops the page.
5. **Allow it through the firewall.** Windows asks the first time: allow **Private networks**. That lets your phone
   open the page, and lets the board fetch firmware updates from it.
6. **Open the page:** <http://localhost:8198> on this PC, or `http://<this PC's address>:8198` from a phone or
   tablet. If it doesn't connect by itself: **Settings**, type the board's address in **Board address**, press
   **Save and connect** (or **Find board on network**).

Settings, profiles, backups and diagnostics are kept in the folder `smc3-scope-data` next to the file.

**Start it with Windows** (optional): make a file `smc3-scope.bat` in the same folder containing

```bat
@echo off
cd /d "%~dp0"
start "smc3-scope" /min py smc3_scope_v2.8.py --bind 0.0.0.0 --web 8198
```

press Win+R, type `shell:startup`, Enter, and put a shortcut to `smc3-scope.bat` in the folder that opens.

**Update:** from the page - Settings, Updates, **Update page** ([Updates](#updates)). Or by hand: download
the new `smc3_scope_v<version>.py` into the same folder, close the old window and start the new file (change the name
in the `.bat` too). Your settings stay - they're in `smc3-scope-data`.

---

## macOS, Linux, Raspberry Pi

1. **Python 3.8 or newer.** Linux and Raspberry Pi OS have it (`python3 --version`). On a Mac, install it from
   <https://www.python.org/downloads/> or with Homebrew (`brew install python`).
2. **Download and start** (use the current version number from the
   [latest release](https://github.com/winreboot/smc3-r4/releases/latest)):

   ```sh
   mkdir -p ~/smc3-scope && cd ~/smc3-scope
   curl -LO https://github.com/winreboot/smc3-r4/releases/latest/download/smc3_scope_v2.8.py
   python3 smc3_scope_v2.8.py --bind 0.0.0.0 --web 8198 --host 192.168.1.50
   ```

3. **Open** `http://<this computer's address>:8198` (or <http://localhost:8198> on the same machine). A Mac asks
   once whether Python may accept incoming connections: allow it.

**Run it in the background, starting at boot (Linux, Raspberry Pi).** Create `/etc/systemd/system/smc3-scope.service`
(replace `pi` with your user name and the path with yours):

```ini
[Unit]
Description=SMC3 R4 page (smc3_scope)
After=network-online.target
Wants=network-online.target

[Service]
User=pi
WorkingDirectory=/home/pi/smc3-scope
ExecStart=/usr/bin/python3 /home/pi/smc3-scope/smc3_scope_v2.8.py --bind 0.0.0.0 --web 8198
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

then:

```sh
sudo systemctl daemon-reload
sudo systemctl enable --now smc3-scope
systemctl status smc3-scope        # should say "active (running)"
journalctl -u smc3-scope -f        # its log, Ctrl+C to leave
```

**Update:** from the page - Settings, Updates, **Update page** ([Updates](#updates)); systemd keeps running the same
service. Or by hand: download the new file, change the name in `ExecStart`, then `sudo systemctl daemon-reload && sudo
systemctl restart smc3-scope`.

---

## Docker: ready-made image

Every release is built as an image for PCs and NAS (amd64), Raspberry Pi 4/5 and ARM NAS (arm64) and older 32-bit
Raspberry Pi OS (arm/v7): `ghcr.io/winreboot/smc3-scope`. Tags: `latest`, the version (`2.7`, `2.6.2`) and the
minor version (`2.6`).

**One command:**

```sh
docker run -d --name smc3-scope --restart unless-stopped -p 8198:8198 -e SMC3_WEB_PORT=8198 -e SMC3_HOST=192.168.1.50 -v smc3-scope-data:/data ghcr.io/winreboot/smc3-scope:latest
```

Open `http://<that machine's address>:8198`. Keep the two port numbers in `-p` and `SMC3_WEB_PORT` the same: the
page tells the board which port to fetch firmware updates from.

**Or with Docker Compose:** download [scope/compose.prebuilt.yaml](../scope/compose.prebuilt.yaml), put the board's
address into `SMC3_HOST` (or leave it empty and set it in the page), then in the same folder:

```sh
docker compose -f compose.prebuilt.yaml up -d
```

Settings, profiles, backups and diagnostics are kept in `./data` next to the compose file (with `docker run`, in the
Docker volume `smc3-scope-data`).

**Update:** from the page - Settings, Updates, **Update page** ([Updates](#updates)). Or pull the new image:

```sh
docker compose -f compose.prebuilt.yaml pull && docker compose -f compose.prebuilt.yaml up -d
```

or, with `docker run`: `docker pull ghcr.io/winreboot/smc3-scope:latest`, `docker rm -f smc3-scope`, then the same
`docker run` command again. Your data stays in the volume.

---

## Synology Container Manager

DSM 7.2 or newer, no SSH needed:

1. **File Station:** in the `docker` shared folder make two folders, `smc3-scope` and `smc3-scope-data`.
2. **Container Manager, Project, Create.** Project name `smc3-scope`, path `/docker/smc3-scope`, source
   **Create docker-compose.yml**, and paste:

   ```yaml
   services:
     smc3-scope:
       image: ghcr.io/winreboot/smc3-scope:latest
       container_name: smc3-scope
       restart: unless-stopped
       environment:
         SMC3_HOST: "192.168.1.50"     # your board's address
         SMC3_PORT: "4210"
         SMC3_WEB_PORT: "8198"
       ports:
         - "8198:8198"
       volumes:
         - /volume1/docker/smc3-scope-data:/data
   ```

3. **Next, Done.** It downloads the image and starts it. Open `http://<NAS address>:8198`.

**Update:** from the page - Settings, Updates, **Update page** ([Updates](#updates)). Or the new image: Container
Manager, **Project**, right-click `smc3-scope`, **Stop**; when it has stopped, right-click it
again and **Build** - that pulls the newest image and starts it. (Or **Image**, which marks images with an update,
and **Update**.)

With SSH, the [ready-made image](#docker-ready-made-image) commands work on a Synology as they are (with `sudo`).

---

## Docker: build it yourself

```sh
git clone https://github.com/winreboot/smc3-r4.git
cd smc3-r4/scope
docker compose up -d --build
```

Data is kept in `scope/data`; to keep it elsewhere:
`SMC3_DATA=/volume1/docker/smc3-scope-data docker compose up -d --build`. Update from the page, or with `git pull`
and the same `docker compose up -d --build`.

---

## Updates

From page 2.7 on, the page asks GitHub twice a day (the first time 20 s after it starts) what the newest release
has, and **Settings, Updates** shows it - the header says **Update available** too. **Check now** asks at once; the
tick box switches the twice-a-day check off. Nothing is installed by itself.

- **Update page** downloads the new `smc3_scope_v<version>.py` from the release, checks it (GitHub's SHA-256 of the
  file, its version line, and that Python can read it) and restarts into it - a few seconds; the tab reloads by
  itself. Nothing on the board changes, SimHub keeps running, and settings, profiles and backups stay. If the page
  holds the seat (Return to center, a dragged knob) it gives it back to SimHub first - the motors stay where they
  are - and a test signal stops. It waits while auto-tune, the full check, Seat setup or a firmware update runs: they
  have the board on temporary settings until they finish.
- **Download firmware** only downloads the `.ino`: open it in the Arduino IDE and upload it over USB (SimHub closed),
  or **Sketch, Export Compiled Binary** and install the `.bin` in **Settings, Firmware update**, over WiFi. Firmware
  is never installed by itself.
- **Go back to built-in** returns to the page you started and stays there.

**Where the update lives.** The new page goes into the data folder, in `updates/` (`smc3-scope-data/updates`, in
Docker `/data/updates`); the file you started - in Docker the image's - stays as it was. At every start the file
you started hands over to the update when the update is newer. So:

- an update survives a restart, a reboot and a re-created container;
- a newer image (or a newer file you start yourself) simply wins over an older update;
- if an update doesn't get as far as serving the page, or stops three times in a row within 10 minutes of starting,
  the next start runs the file you started again (Docker's `restart: unless-stopped` and the systemd service above
  start it again by themselves) and the card says why;
- on Linux, macOS and in Docker the process restarts in place (same process); on Windows the file you started stays
  as a small parent that runs the page, so the window, Ctrl+C and a service wrapper still see one program.

**If an update misbehaves** and the page can't be used to go back: start the page with the environment variable
`SMC3_NO_UPDATE=1` (Docker: `-e SMC3_NO_UPDATE=1`, or under `environment:` in the compose file), or delete the
`updates` folder in the data folder, and start it again. The file you started runs, with your settings.

It needs this machine to reach `api.github.com` and GitHub's download servers. Updates come from
`winreboot/smc3-r4`; a fork can set `"update_repo": "owner/name"` in `config.json` (the page itself can't change it).

## USB cable for calibration

From page 2.8 with firmware 1.9.5 on the board, the page can talk to the board over its USB cable instead of WiFi -
for auto-tune, the full check and Seat setup. The board's WiFi module now and then holds the board up for a second
or more; on the cable nothing waits. Only with SimHub closed: one program at a time can have the USB port.

1. Close SimHub.
2. On the PC the board is plugged into, open the page in a browser, **Settings, USB cable**, press **Copy**, then
   press **Win+R**, paste and press Enter. The line looks like this (with your page's address):

       powershell -NoProfile -ExecutionPolicy Bypass -Command "irm http://192.168.1.20:8198/usb-bridge.ps1 | iex"

   A window opens: the bridge finds the board's COM port (it asks if there are several) and links it to the page.
   It uses Windows PowerShell - nothing to install - and connects out to the page, so no firewall question.
3. The page switches over by itself: the header says **USB cable**. Calibrate as usual.
4. Close the bridge window (or press **Back to WiFi** in the page). The board and the page go back to WiFi by
   themselves. Start SimHub again.

The page can be anywhere on your network (the NAS, a Raspberry Pi): only the bridge runs on the PC with the cable.
**Download the bridge (.ps1)** on the same card gives you the script as a file to keep on that PC (right-click, Run
with PowerShell - it asks for the page's address). On macOS and Linux, or Windows with Python and pyserial, the page
file is the bridge too:

    python3 smc3_scope_v2.8.py --usb-bridge /dev/ttyACM0 --page http://192.168.1.20:8198

**Settings, USB cable** can also turn bridges away (WiFi only). Firmware updates from the page still go over WiFi.

---

## Options

Every option works on the command line or as an environment variable (Docker):

| Command line | Environment | Default | What it is |
|---|---|---|---|
| `--host` | `SMC3_HOST` | empty | The board's address. Empty: set it in the page, or let the page find it. |
| `--port` | `SMC3_PORT` | 4210 | The board's telemetry port (`TELEMETRY_PORT` in the sketch). |
| `--web` | `SMC3_WEB_PORT` | 8080 | The page's port. The examples here use 8198. |
| `--bind` | `SMC3_BIND` | 127.0.0.1 (Docker image: 0.0.0.0) | `0.0.0.0` = reachable from other devices, and the board can fetch firmware updates. `127.0.0.1` = this computer only. |
| `--data` | `SMC3_DATA_DIR` | `smc3-scope-data` next to the file (Docker image: `/data`) | Where settings, profiles, backups and diagnostics are kept. |
| `--usb-bridge PORT` | | | Run as the USB bridge instead of the page: link the page (`--page http://...`) to the board on this serial port (2.8). |

Settings you change in the page (board address, theme, motors in use, motor names ...) are saved there too. The
board address and port from the command line (or `SMC3_HOST` / `SMC3_PORT`) are used the first time, and again
whenever you change them there; otherwise what you set in the page stays.

## When something doesn't work

| What you see | What to check |
|---|---|
| The page doesn't open | Is the window / container running? The right port (8198 in these examples)? From another device: `--bind 0.0.0.0`, and the firewall allows it. |
| "Board not connected" | The board's address in **Settings** (it scrolls across the LED matrix at power-up). Same network as the page? Try **Find board on network**; set **Search network** (e.g. `192.168.0`) if your network isn't `192.168.1.x`. |
| The LED matrix shows `ERROR E9 NO WIFI` | WiFi name and password in the sketch; the board only does 2.4 GHz, WPA2. |
| Port already in use | The page then takes the next free port and says which in its window; or choose one, e.g. `--web 8199` (Docker: `-p 8199:8199 -e SMC3_WEB_PORT=8199`). |
| `docker pull` asks for a login | The package isn't public yet - the repository owner makes it public once (see below). |
| Firmware update from the page can't start | The page must be reachable from the board: `--bind 0.0.0.0`, firewall open, and in Docker the same port number on both sides. The board's WiFi module needs firmware 0.4.0 or newer. |
| Auto-tune says "WiFi hiccup during that move - doing it again", or stops with "The WiFi link to the board kept stalling" | The board's WiFi drops out for a second or more now and then. Put the board closer to the router (or an access point), keep the 2.4 GHz channel clear, and update the board's WiFi module (Arduino IDE, Tools, Firmware Updater). A few repeated moves are harmless. |
| USB bridge: "COM5 is in use or missing" | Close SimHub (and SMC3Utils, the Arduino IDE's Serial Monitor) - only one program can have the port. Check the board is plugged into this PC. |
| USB bridge: "needs firmware 1.9.5 or newer" | The board answers on the cable but its firmware can't send the page's data over USB. Install SMC3_R4 1.9.5 (Settings, Firmware update, or the Arduino IDE), then run the bridge again. |
| USB bridge: "Can't reach the page" | Use the page's plain `http://` address as in your browser (not https or a reverse proxy). The page must listen on the network (`--bind 0.0.0.0`; the Docker image does). |
| USB bridge: "Waiting: Auto-tune is running ..." | The bridge links as soon as auto-tune, the full check, Seat setup or a firmware update has finished - switching mid-way would spoil it. |
| Updates: "Couldn't reach GitHub" | This machine (in Docker: the container) needs internet access to `api.github.com` and GitHub's download servers. The rest of the page works without it. |
| `python` / `py` not found (Windows) | Re-run the Python installer and tick **Add python.exe to PATH**, or use the full path to `python.exe`. |

## Try it without a rig

The simulator speaks the same protocol as the board:

```sh
git clone https://github.com/winreboot/smc3-r4.git && cd smc3-r4
python3 tools/simboard.py 4210 --tuned &
python3 scope/smc3_scope.py --host 127.0.0.1 --port 4210
# open http://localhost:8080
```

On Windows use `py` instead of `python3` and start the two lines in two windows (without the `&`).

---

### For the repository owner: the first image

The image is built by the `docker-image` workflow (Actions tab) whenever a release tag (`v...`) is pushed. After
the very first build, check that the package is **public**: your GitHub profile, **Packages**, `smc3-scope`,
**Package settings**, at the bottom **Change visibility**, **Public**. Once public, `docker pull` works for
everyone without a login. The workflow can also be started by hand: Actions, docker-image, **Run workflow**.
