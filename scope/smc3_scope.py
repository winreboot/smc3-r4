#!/usr/bin/env python3
"""
smc3_scope v1.8.1 - live scope, tuner and auto-tuner for the SMC3_R4 motion controller (Arduino UNO R4 WiFi).

Connects to the board's TCP telemetry port, keeps a rolling buffer of samples and serves a web page with live
charts, a parameter editor (sends ordinary SMC3 commands back to the board), test signals and a per-motor
auto-tune.  SimHub keeps the USB port; this runs alongside it.

    python3 smc3_scope.py --host 192.168.1.50 --bind 0.0.0.0 --web 8198

Every option can also come from the environment (Docker): SMC3_HOST, SMC3_PORT, SMC3_WEB_PORT, SMC3_BIND,
SMC3_DATA_DIR.  Settings changed in the page (board address, theme, auto-find) are saved to
<data dir>/config.json and survive restarts and upgrades.  Standard-library Python 3.8+ only.

v1.8.1: single-sample pot glitches no longer distort step measurements (full check, auto-tune); wider
      test-signal motor menu; a browser closing mid-request no longer prints a traceback.
v1.8: motor names (Settings): call the motors by where they sit (Left, Right, Rear, ... or your own words);
      the names show on the cards, graphs, PID table, menus, auto-tune and diagnostics, next to the number.
v1.7: Diagnostics (Settings section): a quick check (no movement) and a full check that moves each motor a
      little - direction, wiring swaps, drive needed, step response, and what you saw - plus an event log and a
      watcher that notices problems while you drive (pot glitches, a motor moving away from its target or
      stalling, hunting, cutoff trips, board restarts) and keeps a few seconds of data around each. Everything
      goes into one file to hand to whoever helps you. Firmware 1.7 adds reset cause, SimHub USB counters, pot
      glitches at 4 kHz, why a motor is off and an EEPROM copy.
v1.6: EEPROM backups kept on the NAS (every save and every bigger change) with one-click switch back,
      recommended starting points per sim type (circuit, rally, flight, space, trucks), selected-motor
      highlight across the page, live-graph expand/collapse, motors-in-use setting (1-3), target smoothing
      and max speed as settings (firmware 1.6), favicon.
v1.5: new dashboard look (sidebar sections: Dashboard, Tuning, Motors, Live Graphs, Test & Tools, Profiles,
      Auto-Tune, EEPROM, Settings, Help), motor cards, quick controls incl. Disable Motors (firmware 1.5),
      test-signal presets and offset, named profiles stored next to config.json, EEPROM backup download /
      restore, Day / Dark / Auto theme.
v1.4: settings dialog (board address, port, find-board scan, auto-find), day/night theme, hover help on every
      setting, per-motor auto-tune; Kp/Ki/Kd shown in SMC3Utils units (raw numbers, e.g. Kp 300);
      fixed a thread race that made the page flash "not reachable".
"""

VERSION = "1.8.1"

import argparse
import base64
import collections
import json
import math
import os
import re
import socket
import statistics
import struct
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse


def log(msg):
    print(time.strftime("%Y-%m-%d %H:%M:%S"), msg, flush=True)


# --------------------------------------------------------------------------------------------------------------
# Persistent settings
# --------------------------------------------------------------------------------------------------------------

class Config:
    DEFAULTS = {"host": "", "port": 4210, "theme": "dark", "auto_find": True, "scan_subnet": "",
                "motors": 3, "disable_unused": True, "motor_names": ["", "", ""]}

    def __init__(self, data_dir, env_host, env_port):
        self.lock = threading.Lock()
        self.path = os.path.join(data_dir, "config.json")
        self.writable = True
        self.data = dict(self.DEFAULTS)
        saved = {}
        try:
            with open(self.path) as f:
                saved = json.load(f)
        except (OSError, ValueError):
            pass
        self.data.update({k: v for k, v in saved.items() if k in self.DEFAULTS or k.startswith("env_")})
        # A value set in compose/command line wins once when it changes; after that, the page's setting wins.
        changed = False
        if env_host and env_host != saved.get("env_host_seen"):
            self.data["host"] = env_host
            self.data["env_host_seen"] = env_host
            changed = True
        if env_port and env_port != saved.get("env_port_seen"):
            self.data["port"] = int(env_port)
            self.data["env_port_seen"] = env_port
            changed = True
        if changed or not saved:
            self.save()

    def get(self, key):
        with self.lock:
            return self.data.get(key, self.DEFAULTS.get(key))

    def public(self):
        with self.lock:
            d = {k: self.data.get(k, v) for k, v in self.DEFAULTS.items()}
        d["writable"] = self.writable
        d["path"] = self.path
        return d

    def update(self, **kw):
        with self.lock:
            for k, v in kw.items():
                if k in self.DEFAULTS:
                    self.data[k] = v
        self.save()

    def save(self):
        with self.lock:
            data = dict(self.data)
        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            tmp = self.path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(data, f, indent=2)
            os.replace(tmp, self.path)
            self.writable = True
        except OSError as e:
            self.writable = False
            log(f"could not save settings to {self.path}: {e}")


# --------------------------------------------------------------------------------------------------------------
# Telemetry stream from the board
# --------------------------------------------------------------------------------------------------------------

SAMPLE_FMT = "<I" + "hhhh" * 3 + "BBhhhH"
SAMPLE_BYTES = struct.calcsize(SAMPLE_FMT)      # 38
assert SAMPLE_BYTES == 38

# Read codes -> (name, kind). kind "w" = 16-bit word, "p" = two bytes.
PARAM_CODES = {
    "D": ("Kp1", "w"), "E": ("Kp2", "w"), "F": ("Kp3", "w"),
    "G": ("Ki1", "w"), "H": ("Ki2", "w"), "I": ("Ki3", "w"),
    "J": ("Kd1", "w"), "K": ("Kd2", "w"), "L": ("Kd3", "w"),
    "M": ("Ks1", "w"), "N": ("Ks2", "w"), "O": ("Ks3", "w"),
    "P": ("PWM1", "p"), "Q": ("PWM2", "p"), "R": ("PWM3", "p"),          # (PWMmin, PWMmax)
    "S": ("LIM1", "p"), "T": ("LIM2", "p"), "U": ("LIM3", "p"),          # (cutoff limit, input clip)
    "V": ("DZ1", "p"), "W": ("DZ2", "p"), "X": ("DZ3", "p"),             # (dead zone, PWMrev)
    "Y": ("STATUS", "p"), "~": ("PWMFREQ", "p"), "$": ("ISRUS", "w"),
    "!": ("FW", "p"),                                                    # (flags, version*10) - firmware 1.4+
    "f": ("FILT", "p"),                                                  # (smoothing shift, max speed/100) - 1.6+
}


class Board:
    def __init__(self, cfg):
        self.cfg = cfg
        self.sock = None
        self.connected = False
        self.lock = threading.Lock()
        self.slock = threading.Lock()
        self.samples = collections.deque(maxlen=12000)   # 2 minutes at 100 Hz
        self.seq = 0
        self.params = {}
        self.last_rx = 0.0
        self.disconnected_since = time.time()
        self.current_host = None
        self.fw_cache = None
        self.on_connect = None
        self.events = None                   # EventLog (diagnostics)
        self.diag = None                     # last firmware 1.7 diagnostics frame, decoded
        self.diag_seq = 0
        self.last_dis_cmd = [0.0, 0.0, 0.0]  # when this page last switched each motor off

    def start(self):
        """Start connecting - called once everything that reacts to a connection is set up."""
        threading.Thread(target=self._reader, daemon=True).start()

    def _ev(self, kind, msg, **kw):
        if self.events:
            self.events.add(kind, msg, **kw)

    # ---- outgoing ----
    def send(self, data: bytes):
        with self.lock:
            if self.sock:
                try:
                    self.sock.sendall(data)
                    return True
                except OSError:
                    pass
        return False

    def cmd_word(self, code, value):
        value = max(0, min(65535, int(value)))
        return self.send(b"[" + code.encode() + bytes([value >> 8, value & 0xFF]) + b"]")

    def cmd_pair(self, code, a, b):
        a = max(0, min(255, int(a))); b = max(0, min(255, int(b)))
        return self.send(b"[" + code.encode() + bytes([a, b]) + b"]")

    def cmd_text(self, text3):
        if text3 == "dis":
            self.last_dis_cmd = [time.time()] * 3
        elif len(text3) == 3 and text3[:2] == "di" and text3[2] in "123":
            self.last_dis_cmd[int(text3[2]) - 1] = time.time()
        return self.send(b"[" + text3.encode()[:3] + b"]")

    def read_diag(self, timeout=1.5):
        """Firmware 1.7: ask for one diagnostics frame and wait for it."""
        if (self.fw_cache or [0, 0])[1] < 17 or not self.connected:
            return None
        seq0 = self.diag_seq
        self.send(b"[rdd]")
        end = time.time() + timeout
        while time.time() < end:
            if self.diag_seq != seq0:
                return dict(self.diag)
            time.sleep(0.02)
        return None

    def ping(self, timeout=1.5):
        """Round trip of one read command in ms (firmware 1.4+), None if no answer."""
        self.params.pop("FW", None)
        t0 = time.time()
        self.send(b"[rd!]")
        while time.time() - t0 < timeout:
            if "FW" in self.params:
                return round((time.time() - t0) * 1000)
            time.sleep(0.005)
        return None

    def read_params(self, timeout=2.0):
        for name, _ in PARAM_CODES.values():
            self.params.pop(name, None)
        for code in PARAM_CODES:
            self.send(b"[rd" + code.encode() + b"]")
            time.sleep(0.03)
        deadline = time.time() + timeout
        expected = len(PARAM_CODES) - 2          # "!" (1.4+) and "f" (1.6+) are missing on older firmware
        while time.time() < deadline and len(self.params) < len(PARAM_CODES):
            if len(self.params) >= expected and time.time() > deadline - timeout + 1.0:
                break
            time.sleep(0.05)
        return dict(self.params)

    def reconnect(self):
        self._drop()

    # ---- incoming ----
    def _reader(self):
        buf = b""
        while True:
            if not self.sock:
                host, port = self.cfg.get("host"), int(self.cfg.get("port"))
                if not host:
                    time.sleep(1)
                    continue
                try:
                    s = socket.create_connection((host, port), timeout=5)
                    s.settimeout(1.0)
                    with self.lock:
                        self.sock = s
                    self.connected = True
                    self.current_host = host
                    self.last_rx = time.time()
                    buf = b""
                    log(f"connected to board {host}:{port}")
                    self._ev("link", f"Connected to the board at {host}:{port}.", quiet=True)
                    if self.on_connect:
                        threading.Thread(target=self.on_connect, daemon=True).start()
                except OSError:
                    self.connected = False
                    time.sleep(2)
                    continue
            try:
                chunk = self.sock.recv(4096)
                if not chunk:
                    raise OSError("closed")
                buf += chunk
                self.last_rx = time.time()
            except socket.timeout:
                if time.time() - self.last_rx > 5:
                    log("board stopped sending - reconnecting")
                    self._ev("link", "The board stopped sending data for 5 s - reconnecting.", quiet=True)
                    self._drop()
                continue
            except (OSError, AttributeError):
                self._drop()
                continue
            buf = self._parse(buf)

    def _drop(self):
        with self.lock:
            try:
                if self.sock:
                    self.sock.close()
            except OSError:
                pass
            was = self.sock is not None
            self.sock = None
        if was:
            self.disconnected_since = time.time()
            log("disconnected from board")
            self._ev("link", "Disconnected from the board.", quiet=True)
        self.connected = False

    def _parse(self, buf):
        while True:
            if len(buf) < 5:
                return buf
            if buf[:4] == b"SM3T":
                if len(buf) < 6:
                    return buf
                count = buf[5]
                need = 6 + count * SAMPLE_BYTES
                if len(buf) < need:
                    return buf
                for i in range(count):
                    self._add_sample(struct.unpack_from(SAMPLE_FMT, buf, 6 + i * SAMPLE_BYTES))
                buf = buf[need:]
            elif buf[:4] == b"SM3D":                       # firmware 1.7 diagnostics frame
                if len(buf) < 6:
                    return buf
                need = 6 + buf[5]
                if len(buf) < need:
                    return buf
                d = decode_diag(buf[6:need])
                if d:
                    self.diag = d
                    self.diag_seq += 1
                buf = buf[need:]
            elif buf[0:1] == b"[" and buf[4:5] == b"]":
                self._add_reply(chr(buf[1]), buf[2], buf[3])
                buf = buf[5:]
            else:
                buf = buf[1:]          # resync

    def _add_sample(self, u):
        motors = []
        for m in range(3):
            t, g, f, p = u[1 + m * 4: 5 + m * 4]
            motors.append({"t": t, "g": g / 16.0, "f": f / 16.0, "p": p / 16.0})
        status, power, roll, pitch, yaw, isr = u[13:19]
        with self.slock:
            self.seq += 1
            self.samples.append({
                "seq": self.seq, "ms": u[0], "m": motors,
                "dis": [bool(status & 1), bool(status & 2), bool(status & 4)],
                "div": status >> 4, "low": power == 9,
                "roll": roll / 100.0, "pitch": pitch / 100.0, "yaw": yaw / 100.0, "isr": isr,
            })

    def _add_reply(self, code, a, b):
        if code not in PARAM_CODES:
            return
        name, kind = PARAM_CODES[code]
        self.params[name] = (a * 256 + b) if kind == "w" else [a, b]
        if name == "FW":
            self.fw_cache = [a, b]

    def latest(self):
        with self.slock:
            return self.samples[-1] if self.samples else None

    def since(self, seq, limit=None):
        with self.slock:
            snap = list(self.samples)
        out = [s for s in snap if s["seq"] > seq]
        return out[-limit:] if limit else out


# --------------------------------------------------------------------------------------------------------------
# Finding the board on the network
# --------------------------------------------------------------------------------------------------------------

def subnet_of(host, override=""):
    if override:
        return override.strip().rstrip(".")
    try:
        ip = socket.gethostbyname(host) if host else ""
    except OSError:
        ip = ""
    parts = ip.split(".")
    if len(parts) == 4 and all(p.isdigit() for p in parts):
        return ".".join(parts[:3])
    return "192.168.1"


def scan_for_boards(prefix, port, skip=()):
    """Try every address in prefix.1-254 for an SMC3 telemetry stream. Returns the addresses that answer."""
    def probe(h):
        if h in skip:
            return None
        try:
            s = socket.create_connection((h, port), timeout=0.4)
        except OSError:
            return None
        try:
            s.settimeout(0.5)
            data = b""
            end = time.time() + 4.0
            while time.time() < end and b"SM3T" not in data:
                try:
                    chunk = s.recv(2048)
                    if not chunk:
                        break
                    data += chunk
                except socket.timeout:
                    pass
            return h if b"SM3T" in data else None
        finally:
            s.close()

    hosts = [f"{prefix}.{i}" for i in range(1, 255)]
    with ThreadPoolExecutor(max_workers=64) as ex:
        return [h for h in ex.map(probe, hosts) if h]


class AutoFinder:
    """If the board stays unreachable for a minute, look for it on the subnet (then every 10 minutes)."""
    def __init__(self, board, cfg):
        self.board, self.cfg = board, cfg
        self.last_scan = 0.0
        self.scanning = False
        threading.Thread(target=self._loop, daemon=True).start()

    def scan_now(self, subnet=""):
        if self.scanning:
            return None
        self.scanning = True
        try:
            prefix = subnet_of(self.cfg.get("host"), subnet or self.cfg.get("scan_subnet"))
            skip = (self.board.current_host,) if self.board.connected else ()
            found = scan_for_boards(prefix, int(self.cfg.get("port")), skip)
            self.last_scan = time.time()
            return prefix, found
        finally:
            self.scanning = False

    def _loop(self):
        while True:
            time.sleep(5)
            b = self.board
            if (not self.cfg.get("auto_find") or b.connected or self.scanning
                    or time.time() - b.disconnected_since < 60 or time.time() - self.last_scan < 600):
                continue
            r = self.scan_now()
            if not r:
                continue
            prefix, found = r
            if found and found[0] != self.cfg.get("host"):
                log(f"auto-find: board found at {found[0]} (was {self.cfg.get('host')}) - switching")
                self.cfg.update(host=found[0])
                b.reconnect()
            else:
                log(f"auto-find: no board answering on {prefix}.x")


# --------------------------------------------------------------------------------------------------------------
# Test-signal generator (sends [Axx]/[Bxx]/[Cxx] at 50 Hz). Stop SimHub motion while using it.
# --------------------------------------------------------------------------------------------------------------

class Generator:
    def __init__(self, board):
        self.board = board
        self.mode = "off"
        self.motor = 0
        self.amp = 200
        self.period = 2.0
        self.value = 512
        self.centre = 512
        threading.Thread(target=self._run, daemon=True).start()

    def set(self, **kw):
        for k, v in kw.items():
            if hasattr(self, k):
                setattr(self, k, v)

    def _run(self):
        t0 = time.time()
        while True:
            time.sleep(0.02)
            if self.mode == "off":
                continue
            t = time.time() - t0
            per = max(0.05, self.period)
            if self.mode == "sine":
                v = self.centre + self.amp * math.sin(2 * math.pi * t / per)
            elif self.mode == "step":
                v = self.centre + (self.amp if int(t / per) % 2 else -self.amp)
            elif self.mode == "triangle":
                ph = (t / per) % 1.0
                v = self.centre + self.amp * (4 * ph - 1 if ph < 0.5 else 3 - 4 * ph)
            else:                                   # manual
                v = self.value
            self.board.cmd_word("ABC"[self.motor], int(max(0, min(1023, v))))


# --------------------------------------------------------------------------------------------------------------
# Auto-tune
# --------------------------------------------------------------------------------------------------------------

class TuneAbort(Exception):
    def __init__(self, msg, kind="benign"):        # benign | unsafe | refuse
        super().__init__(msg)
        self.kind = kind


# style -> (overshoot allowed %, cap as share of the highest stable Kp, how close to the best settle time is "enough")
STYLES = {"gentle": (5.0, 0.85, 1.30), "normal": (10.0, 0.90, 1.15), "firm": (15.0, 0.95, 1.05)}
SSE_FLOOR = 12.0             # never pick a Kp that leaves more than this many counts of error on big moves
PID_NOISE_MAX = 6.0          # PWM std of the PID output while holding - above this it buzzes
KP_START, KP_STEP, KP_MAX = 40, 1.3, 2000
OS_STOP = 30.0
KI_STEPS = [5, 10, 20, 40, 70]
KD_STEPS = [5, 10, 20, 40]


def _mean(xs):
    return sum(xs) / len(xs) if xs else 0.0


def _std(xs):
    return statistics.pstdev(xs) if len(xs) > 1 else 0.0


def analyze_step(samples, m, t_old, pwmmin, pwmmax, window_ms):
    """Step-response metrics for motor m from telemetry samples that contain a target change away from t_old."""
    idx = next((i for i, s in enumerate(samples) if s["m"][m]["t"] != t_old), None)
    if idx is None:
        return None
    t_new = samples[idx]["m"][m]["t"]
    f0 = samples[idx - 1]["m"][m]["f"] if idx > 0 else samples[idx]["m"][m]["f"]
    t0 = samples[idx]["ms"]
    post = [s for s in samples[idx:] if 0 <= s["ms"] - t0 <= window_ms and s["m"][m]["t"] == t_new]
    size = abs(t_new - f0)
    if len(post) < 10:
        return None
    if size < 8:     # the seat was already (nearly) there - nothing to measure except where it rests
        fb_tail = [s["m"][m]["f"] for s in post[-30:]]
        return {"size": size, "rise": 0.0, "reached": True, "os": 0.0, "settle": 0.0, "settled": True,
                "sse": abs(t_new - _mean(fb_tail)), "osc": 0, "fb_std": _std(fb_tail),
                "pid_std": _std([s["m"][m]["p"] for s in post[-30:]]), "sat": 0.0, "wrong": False}
    sgn = 1 if t_new >= f0 else -1
    ys = [((s["m"][m]["f"] - f0) * sgn / size, s["ms"] - t0, s) for s in post]
    t10 = next((t for y, t, _ in ys if y >= 0.1), None)
    t90 = next((t for y, t, _ in ys if y >= 0.9), None)          # 90% of the way to the TARGET
    span = ys[-1][1]
    tail = [s for y, t, s in ys if t >= span - 300]
    fb_tail = [s["m"][m]["f"] for s in tail]
    final = _mean(fb_tail)
    sse = (t_new - final) * sgn                                    # leftover error = Ki's job
    # Dynamics are judged against where the seat actually comes to rest, so a few counts of leftover
    # error (sag) don't masquerade as slow settling or overshoot.
    yf = (final - f0) * sgn / size
    tol = max(3.0, 0.03 * size)
    peak = max(y for y, _, _ in ys)
    os_pct = max(0.0, (peak - yf) * 100.0)
    last_out = None
    for y, t, _ in ys:
        if abs(y - yf) * size > tol:
            last_out = t
    settle = last_out or 0
    settled = (span - settle) >= 200
    t_near = next((t for y, t, _ in ys if y >= 0.9 * yf), None) if yf > 0.2 else None
    crossings, state = 0, 0
    for y, t, _ in ys:
        if t_near is None or t < t_near:
            continue
        e = (yf - y) * size
        st = 1 if e > tol else (-1 if e < -tol else 0)
        if st == 0:
            continue
        if state and st != state:
            crossings += 1
        state = st
    early = [s for y, t, s in ys if t <= 500]
    wrong = any((s["m"][m]["f"] - f0) * sgn < -15 for s in early)
    rising = [s for y, t, s in ys if t90 is None or t <= t90]
    sat = _mean([1.0 if abs(s["m"][m]["p"]) + pwmmin >= pwmmax else 0.0 for s in rising])
    return {
        "size": size, "rise": (t90 - t10) if (t10 is not None and t90 is not None) else None,
        "reached": t90 is not None, "os": os_pct, "settle": settle, "settled": settled,
        "sse": abs(sse), "osc": crossings, "fb_std": _std(fb_tail),
        "pid_std": _std([s["m"][m]["p"] for s in tail]), "sat": sat, "wrong": wrong,
    }


def worst(a, b):
    return {
        "size": min(a["size"], b["size"]),
        "rise": None if (a["rise"] is None or b["rise"] is None) else max(a["rise"], b["rise"]),
        "reached": a["reached"] and b["reached"], "os": max(a["os"], b["os"]),
        "settle": max(a["settle"], b["settle"]), "settled": a["settled"] and b["settled"],
        "sse": max(a["sse"], b["sse"]), "osc": max(a["osc"], b["osc"]),
        "fb_std": max(a["fb_std"], b["fb_std"]), "pid_std": max(a["pid_std"], b["pid_std"]),
        "sat": max(a["sat"], b["sat"]), "wrong": a["wrong"] or b["wrong"],
    }


def fmt(r):
    rise = f"{r['rise']:.0f} ms" if r["rise"] is not None else "never got to 90%"
    return (f"rise {rise}, overshoot {r['os']:.0f}%, settle {r['settle']:.0f} ms, "
            f"off by {r['sse']:.1f}, ringing {r['osc']}, hold noise {r['fb_std']:.2f}, pid noise {r['pid_std']:.1f}")


class Tuner:
    def __init__(self, board, gen):
        self.board, self.gen = board, gen
        self.state = "idle"            # idle | running | done | aborted | error
        self.stage = ""
        self.progress = 0
        self.lines = []
        self.result = None
        self.motor = None
        self.original = None
        self.abort_flag = False
        self.applied = False
        self.recent = collections.deque(maxlen=400)
        self.hist = collections.deque(maxlen=200)
        self.cur = 512
        self.last_cmd = 0.0
        self.chk_seq = 0
        self.interf = 0
        self.rev = False
        self.t_start = 0.0
        self.runaway_limit = 400
        self.clip = 100
        self.events = None

    # ---- public ----
    def running(self):
        return self.state == "running"

    def status(self):
        return {"state": self.state, "motor": None if self.motor is None else self.motor + 1,
                "stage": self.stage, "progress": self.progress, "log": self.lines[-300:],
                "result": self.result, "can_revert": bool(self.original) and not self.running()}

    def start(self, motor, amp, style, do_friction, do_ki, do_kd):
        if self.running():
            return False, "An auto-tune is already running."
        if not self.board.connected:
            return False, "The board is not connected."
        self.motor = motor
        self.state, self.stage, self.progress = "running", "Starting", 0
        self.lines, self.result, self.original, self.abort_flag = [], None, None, False
        self.applied = False
        threading.Thread(target=self._thread, args=(motor, amp, style, do_friction, do_ki, do_kd),
                         daemon=True).start()
        return True, "started"

    def abort(self):
        self.abort_flag = True

    def revert(self):
        if self.running() or not self.original:
            return False
        self._apply(self.original)
        self.say(f"Reverted motor {self.motor + 1} to the settings it had before auto-tune.")
        return True

    # ---- helpers ----
    def say(self, msg):
        self.lines.append(time.strftime("%H:%M:%S ") + msg)
        log(f"[tune M{(self.motor or 0) + 1}] {msg}")

    def _apply(self, p):
        b, m = self.board, self.motor
        b.cmd_word("DEF"[m], p["kp"])
        b.cmd_word("GHI"[m], p["ki"])
        b.cmd_word("JKL"[m], p["kd"])
        b.cmd_word("MNO"[m], max(1, p["ks"]))
        b.cmd_pair("PQR"[m], p["pwmmin"], p["pwmmax"])
        b.cmd_pair("VWX"[m], p["dz"], p["pwmrev"])
        self.applied = True
        time.sleep(0.25)

    def _get_params(self):
        pr = self.board.read_params()
        n = self.motor + 1
        need = [f"Kp{n}", f"Ki{n}", f"Kd{n}", f"Ks{n}", f"PWM{n}", f"DZ{n}", f"LIM{n}"]
        if any(k not in pr for k in need):
            raise TuneAbort(f"Couldn't read motor {n}'s settings from the board. Try again.", "refuse")
        p = {"kp": pr[f"Kp{n}"], "ki": pr[f"Ki{n}"], "kd": pr[f"Kd{n}"], "ks": pr[f"Ks{n}"],
             "pwmmin": pr[f"PWM{n}"][0], "pwmmax": pr[f"PWM{n}"][1],
             "dz": pr[f"DZ{n}"][0], "pwmrev": pr[f"DZ{n}"][1],
             "lim": pr[f"LIM{n}"][0], "clip": pr[f"LIM{n}"][1]}
        return p, pr.get("FW")

    def _cmd_value(self, v_int):
        return 1023 - v_int if self.rev else v_int

    def set_target(self, v_int):
        v_int = int(max(self.clip, min(1023 - self.clip, round(v_int))))
        self.board.cmd_word("ABC"[self.motor], self._cmd_value(v_int))
        self.cur = v_int
        self.last_cmd = time.time()
        self.recent.append((self.last_cmd, v_int))

    def _fb(self):
        s = self.board.latest()
        return s["m"][self.motor]["f"] if s else 512.0

    def _t(self):
        s = self.board.latest()
        return s["m"][self.motor]["t"] if s else self.cur

    def wait(self, sec):
        end = time.time() + sec
        while True:
            self._check()
            if time.time() - self.last_cmd > 0.8:          # keep-alive: holds full power on firmware 1.4
                self.set_target(self.cur)
            if time.time() >= end:
                return
            time.sleep(0.05)

    def _check(self):
        m, n = self.motor, self.motor + 1
        if self.abort_flag:
            raise TuneAbort("Stopped by you.")
        if not self.board.connected:
            raise TuneAbort("Lost the connection to the board.")
        new = self.board.since(self.chk_seq)
        if not new:
            return
        self.chk_seq = new[-1]["seq"]
        now = time.time()
        allowed = {v for t, v in self.recent if now - t < 2.0} | {self.cur}
        lo_ok, hi_ok = self.clip - 25, 1023 - self.clip + 25
        for s in new:
            mm = s["m"][m]
            if s["dis"][m]:
                raise TuneAbort(f"Motor {n} was shut off by its safety cutoff (feedback went past Max limit).",
                                "unsafe")
            if s["low"] and now - self.t_start > 4:
                raise TuneAbort("The board is still in low-power mode although this page is sending targets. "
                                "Auto-tune needs firmware 1.4 on the board.", "refuse")
            if mm["t"] in allowed:
                self.interf = 0
            else:
                self.interf += 1
                if self.interf >= 6:
                    raise TuneAbort(f"Something else is moving motor {n} (SimHub?). Stop SimHub's motion "
                                    f"output and start again.")
            if abs(mm["t"] - mm["f"]) > self.runaway_limit:
                raise TuneAbort(f"Motor {n} ran away from its target - most likely the motor or the pot is wired "
                                f"backwards (or the pot slipped on its shaft).", "unsafe")
            if mm["f"] < lo_ok or mm["f"] > hi_ok:
                raise TuneAbort(f"Motor {n} went outside the Clip window ({mm['f']:.0f}).")
            self.hist.append((s["ms"], mm["f"], mm["t"] - mm["f"], mm["p"]))
        # wrong-direction watch: pushing towards the target for 250 ms but moving away from it
        if len(self.hist) > 2:
            t_end = self.hist[-1][0]
            win = [h for h in self.hist if t_end - h[0] <= 300]
            if win and t_end - win[0][0] >= 250:
                es = [h[2] for h in win]
                if all(abs(e) > 30 for e in es) and (all(e > 0 for e in es) or all(e < 0 for e in es)):
                    sg = 1 if es[0] > 0 else -1
                    pushing = all(h[3] * sg > 15 for h in win)
                    moved = (win[-1][1] - win[0][1]) * sg
                    if pushing and moved < -10:
                        raise TuneAbort(f"Motor {n} moves AWAY from where it is driven. The motor or the pot is "
                                        f"wired backwards.", "unsafe")

    def _ramp_to(self, x, speed=400.0):
        start = self.cur
        steps = max(1, int(abs(x - start) / (speed * 0.02)))
        for i in range(1, steps + 1):
            self.set_target(start + (x - start) * i / steps)
            self.wait(0.02)
        self.set_target(x)

    def _settle(self, x, timeout, tol=None):
        tol = tol if tol is not None else 4.0
        end = time.time() + timeout
        while time.time() < end:
            self.wait(0.1)
            if abs(self._fb() - x) <= tol:
                self.wait(0.2)
                return True
        return False

    def _step(self, x, hold, p):
        t_old = self._t()
        seq0 = self.board.seq
        self.set_target(x)
        self.wait(hold)
        self.wait(0.15)                       # let the last telemetry burst arrive
        r = analyze_step(despike(self.board.since(seq0), self.motor), self.motor, t_old, p["pwmmin"], p["pwmmax"],
                         hold * 1000 - 100)
        if r is None:
            raise TuneAbort("Couldn't see the step in the telemetry (WiFi too slow?). Try again.")
        if r["wrong"]:
            raise TuneAbort(f"Motor {self.motor + 1} moved the wrong way. The motor or the pot is wired "
                            f"backwards.", "unsafe")
        return r

    def _step_pair(self, hi, lo, p, hold=2.0):
        a = self._step(hi, hold, p)
        b = self._step(lo, hold, p)
        return worst(a, b)

    def _safe_stop(self):
        p = dict(self.original or {})
        n = self.motor + 1
        self.board.cmd_pair("PQR"[self.motor], 0, 0)
        self.board.cmd_pair("VWX"[self.motor], p.get("dz", 0), 0)
        self.say(f"SAFETY: motor {n}'s drive is set to zero (PWMmax 0, PWMrev 0) so it can't run away. This is "
                 f"NOT saved - power-cycling the board brings back the stored settings. Fix the cause first, "
                 f"then press Revert (or Enable motors if the cutoff tripped).")

    def _restore_and_hold(self):
        if self.original:
            self._apply(self.original)
        try:
            self.set_target(self._fb())
        except Exception:
            pass

    # ---- the procedure ----
    def _thread(self, *a):
        self.gen.set(mode="off")
        try:
            self._run(*a)
            self.state = "done"
        except TuneAbort as e:
            self.say(("Cannot start: " if e.kind == "refuse" else "Stopped: ") + str(e))
            if e.kind == "unsafe":
                self._safe_stop()
            elif self.original and self.applied:
                self._restore_and_hold()
                self.say("Previous settings restored; the motor holds where it is.")
            self.state = "aborted"
        except Exception as e:                                  # a bug - leave the rig as it was
            self.say(f"Internal error: {e!r}")
            if self.original:
                self._restore_and_hold()
            self.state = "error"
        self.stage = ""
        if self.events:
            n = (self.motor or 0) + 1
            if self.state == "done" and self.result:
                a, b = self.result["before"], self.result["after"]
                self.events.add("tune", f"Auto-tune of motor {n} finished: " + ", ".join(
                    f"{k} {a[k]}->{b[k]}" for k in a if a[k] != b[k]) + " (not saved yet).", motor=n,
                    before=a, after=b)
            else:
                last = next((l[9:] for l in reversed(self.lines) if "Stopped" in l or "Cannot" in l or "error" in l),
                            self.state)
                self.events.add("tune", f"Auto-tune of motor {n} {self.state}: {last}", motor=n)

    def _run(self, m, amp, style, do_friction, do_ki, do_kd):
        b, n = self.board, m + 1
        os_target, margin, knee_tol = STYLES.get(style, STYLES["normal"])
        self.say(f"Auto-tune motor {n}, style '{style}', travel ±{amp} counts.")

        self.stage, self.progress = "Reading settings", 2
        orig, fw = self._get_params()
        self.original = dict(orig)
        if not fw or fw[1] < 14:
            raise TuneAbort("auto-tune needs firmware 1.4 or newer on the board (flash the newest SMC3_R4.ino).",
                            "refuse")
        self.rev = (m == 0 and bool(fw[0] & 1))
        s = b.latest()
        if not s:
            raise TuneAbort("No telemetry from the board yet.", "refuse")
        if s["dis"][m]:
            raise TuneAbort(f"motor {n} is shut off - press 'Enable motors' first.", "refuse")
        self.clip = orig["clip"]
        room = 512 - orig["clip"] - 60
        if room < 40:
            raise TuneAbort(f"the Clip window (Clip {orig['clip']}) is too narrow to test in.", "refuse")
        if orig["pwmmax"] < 30:
            raise TuneAbort(f"PWMmax is {orig['pwmmax']} - set it to at least 60 first.", "refuse")
        A = int(max(40, min(amp, room)))
        if A < amp:
            self.say(f"Travel limited to ±{A} by the Clip window.")
        c, lo, hi = 512, 512 - A, 512 + A
        self.runaway_limit = 2 * A + 80

        self.cur = s["m"][m]["t"]
        self.recent.clear(); self.hist.clear()
        self.recent.append((time.time(), self.cur))
        self.chk_seq, self.interf, self.t_start = b.seq, 0, time.time()

        p = dict(orig)
        p.update(kp=80, ki=0, kd=0, ks=1, dz=max(1, orig["dz"]))
        self._apply(p)

        # 1. centre and noise
        self.stage, self.progress = "Moving to centre", 5
        self._ramp_to(c)
        self._settle(c, 4.0, tol=max(4.0, p["dz"] + 2))
        if abs(self._fb() - c) > 40:
            raise TuneAbort(f"motor {n} didn't follow the target (feedback {self._fb():.0f}, wanted 512). "
                            f"Is motor power on?")
        if do_friction:
            self.stage, self.progress = "Measuring pot noise", 8
            self.wait(1.0)                       # let any ringing from the move die out first
            seq = b.seq
            self.wait(1.5)
            fs = [x["m"][m]["f"] for x in b.since(seq)]
            # noise = what's left after removing slow movement (5-sample moving average)
            resid = [fs[i] - _mean(fs[i - 2:i + 3]) for i in range(2, len(fs) - 2)]
            sigma = _std(resid) * 1.12
            p["dz"] = int(min(4, max(1, math.ceil(3 * sigma) + 1)))
            self.say(f"Pot noise at rest ±{sigma:.2f} counts -> dead zone {p['dz']}.")

            # 2. PWMmin: smallest drive that reliably moves the motor, per direction
            self.stage = "Finding the minimum drive (PWMmin)"
            p.update(kp=20)
            found = {1: None, -1: None}
            cap = int(min(80, p["pwmmax"] - 5))
            for pm in range(0, cap + 1, 2):
                self.progress = 10 + int(20 * pm / max(1, cap))
                p["pwmmin"] = pm
                self._apply(p)
                for d in (1, -1):
                    if found[d] is not None:
                        continue
                    ok = 0
                    for _ in range(2):
                        f0 = self._fb()
                        self.set_target(round(f0) + d * 12)
                        self.wait(0.6)
                        moved = (self._fb() - f0) * d
                        if moved <= -6:
                            raise TuneAbort(f"motor {n} moved the wrong way. The motor or the pot is wired "
                                            f"backwards.", "unsafe")
                        if moved >= 5:
                            ok += 1
                        else:
                            break
                    if ok == 2:
                        found[d] = pm
                        self.say(f"Moves reliably towards {'higher' if d > 0 else 'lower'} counts at PWM {pm}.")
                if found[1] is not None and found[-1] is not None:
                    break
            if found[1] is None and found[-1] is None:
                raise TuneAbort(f"motor {n} didn't move even with PWMmin {cap}. Check motor power and wiring.")
            if found[1] is None or found[-1] is None:
                good = 1 if found[1] is not None else -1
                raise TuneAbort(f"motor {n} moves towards {'higher' if good > 0 else 'lower'} counts but not the "
                                f"other way even at PWM {cap}: too weak for the load with PWMmax {p['pwmmax']}, "
                                f"or something binds.")
            p["pwmmin"] = min(found[1], found[-1])
            gap = abs(found[1] - found[-1])
            self.say(f"PWMmin {p['pwmmin']} (the easier direction; the harder one needs {max(found.values())}).")
            if gap > 15 and not do_ki:
                do_ki = True
                self.say("Large difference between directions (typical with a worm gear lifting your weight) - "
                         "the Ki step will run to make up for it.")
            self._ramp_to(c)
            self._settle(c, 3.0)

        # 3. Kp sweep
        self.stage, self.progress = "Sweeping Kp", 30
        p.update(ki=0, kd=0, ks=1, kp=KP_START)
        self._apply(p)
        self._ramp_to(lo)
        self._settle(lo, 3.0)
        results, over, kp = [], 0, float(KP_START)
        best_settle, best_sse, no_gain = float("inf"), float("inf"), 0
        while kp <= KP_MAX:
            self.progress = 30 + int(40 * math.log(kp / KP_START) / math.log(KP_MAX / KP_START))
            p["kp"] = int(round(kp))
            self._apply(p)
            self._settle(lo, 1.0)
            r = self._step_pair(hi, lo, p)
            results.append((p["kp"], r))
            self.say(f"Kp {p['kp']}: {fmt(r)}")
            if r["os"] > OS_STOP or r["osc"] >= 4 or r["fb_std"] > 2.5:
                self.say("Too lively - stopping the sweep here.")
                break
            if r["pid_std"] > PID_NOISE_MAX:
                self.say("Holding gets twitchy (the pot noise is being amplified) - stopping the sweep here.")
                break
            if r["os"] > os_target:
                over += 1
                if over >= 2:
                    break
            if r["reached"] and r["settled"]:
                # leftover error below dead zone + 3 counts can't be felt (and Ki handles it) - don't chase it
                sse_ok = p["dz"] + 3.0
                if r["settle"] < best_settle * 0.95 or (r["sse"] > sse_ok and r["sse"] < best_sse * 0.85):
                    no_gain = 0
                else:
                    no_gain += 1
                best_settle, best_sse = min(best_settle, r["settle"]), min(best_sse, r["sse"])
                if no_gain >= 3:
                    self.say("Higher Kp isn't improving anything any more - stopping the sweep.")
                    break
            kp *= KP_STEP
        good = [(k, r) for k, r in results if r["reached"] and r["os"] <= os_target and r["osc"] <= 2
                and r["fb_std"] <= 2.0 and r["pid_std"] <= PID_NOISE_MAX]
        notes = []
        if good:
            # sweet spot = the lowest Kp that is (nearly) as good as the best one; higher only adds harshness
            fast = min(r["settle"] for k, r in good if r["settled"]) if any(r["settled"] for k, r in good) else None
            tight = min(r["sse"] for k, r in good)
            knee = None
            if fast is not None:
                knee = next(((k, r) for k, r in good if r["settled"] and r["settle"] <= fast * knee_tol + 20
                             and r["sse"] <= max(tight * 1.5, p["dz"] + 3.0)), None)
            cap = max(KP_START, int(round(good[-1][0] * margin)))
            if knee and knee[0] <= cap:
                kp_best, r_best = knee
                self.say(f"Sweet spot: Kp {kp_best} - higher values didn't track noticeably better.")
            else:
                kp_best, r_best = good[-1][0] * margin, good[-1][1]
                self.say(f"Highest stable Kp {good[-1][0]} -> using {int(margin * 100)}% of it for a safety margin.")
        else:
            reached = [(k, r) for k, r in results if r["reached"]]
            if not reached:
                raise TuneAbort(f"motor {n} never reached the target within 2 s, even at Kp {results[-1][0]}. "
                                f"PWMmax {p['pwmmax']} is probably too low for this load.")
            kp_best, r_best = reached[0]
            notes.append(f"Even the gentlest Kp that reaches the target overshoots {r_best['os']:.0f}% - "
                         f"check for slack in the linkage between motor and pot.")
        if r_best["sat"] > 0.5:
            notes.append(f"Big moves run into PWMmax ({p['pwmmax']}) - if large movements feel slow, raise PWMmax "
                         f"by 10-20 and run auto-tune again.")
        if not good:
            kp_best = kp_best * margin
        # floor: whatever the style, don't settle on a Kp that leaves the seat far from where it is sent
        floor = next((k for k, r in results if r["reached"] and r["sse"] <= SSE_FLOOR), None)
        if floor is not None and kp_best < floor:
            self.say(f"Kp {int(kp_best)} would leave the seat more than {SSE_FLOOR:.0f} counts short - using Kp "
                     f"{floor}, the lowest that gets there.")
            kp_best = floor
        p["kp"] = max(KP_START, int(round(kp_best)))
        self.say(f"Kp {p['kp']}.")

        # 4. small movements (road-texture sized) must not ring
        self.stage, self.progress = "Checking small movements", 72
        self._apply(p)
        for _ in range(3):
            self._ramp_to(c)
            self._settle(c, 2.0)
            rs = self._step_pair(c + 15, c - 15, p, hold=1.5)
            self.say(f"Small ±15 steps at Kp {p['kp']}: {fmt(rs)}")
            if (rs["osc"] >= 2 or rs["os"] > 35 or rs["fb_std"] > 1.5) and p["kp"] > KP_START:
                p["kp"] = max(KP_START, int(p["kp"] * 0.8))
                self._apply(p)
                self.say(f"Small moves are too lively - Kp reduced to {p['kp']}.")
            else:
                break

        # 5. Ki, only if the motor settles short of the target (big or small moves)
        self.stage, self.progress = "Checking steady-state error (Ki)", 76

        def measure():
            self._ramp_to(lo)
            self._settle(lo, 2.0)
            big = self._step_pair(hi, lo, p, hold=2.5)
            self._ramp_to(c)
            self._settle(c, 2.0)
            small = self._step_pair(c + 15, c - 15, p, hold=1.5)
            return big, small

        r_base, r_small = measure()
        self.say(f"Kp {p['kp']}, big moves: {fmt(r_base)}")
        self.say(f"Kp {p['kp']}, small moves: {fmt(r_small)}")
        tol = p["dz"] + 1.5
        sse0 = max(r_base["sse"], r_small["sse"])
        if do_ki and sse0 > tol:
            best = None
            for i, ki in enumerate(KI_STEPS):
                self.progress = 77 + 2 * i
                p["ki"] = ki
                self._apply(p)
                big, small = measure()
                self.say(f"Ki {ki}: big {big['os']:.0f}% over / off by {big['sse']:.1f}, "
                         f"small {small['os']:.0f}% over / off by {small['sse']:.1f}")
                # judged against what was already there, so a springy linkage's normal ringing doesn't veto it
                if (big["os"] > r_base["os"] + 8 or big["osc"] > max(2, r_base["osc"] + 1)
                        or small["osc"] > max(2, r_small["osc"] + 1) or small["os"] > max(35.0, r_small["os"] + 15)):
                    self.say("Ki starts causing overshoot - stopping.")
                    break
                best = (ki, big, small)
                if max(big["sse"], small["sse"]) <= tol:
                    break
            if best and max(best[1]["sse"], best[2]["sse"]) < sse0 * 0.7:
                p["ki"], r_base, r_small = best
            else:
                p["ki"] = 0
            self.say(f"Ki {p['ki']}.")
        elif do_ki:
            p["ki"] = 0
            self.say(f"Settles within {sse0:.1f} counts on big and small moves - Ki not needed (0).")
        self._apply(p)

        # 6. Kd, only if there is overshoot to damp and it helps without adding noise
        self.stage, self.progress = "Trying damping (Kd)", 88
        if do_kd and r_base["os"] > 2:
            def score(r):
                return r["os"] + r["settle"] / 40.0
            self._ramp_to(lo)
            self._settle(lo, 2.0)
            best = None
            for i, kd in enumerate(KD_STEPS):
                self.progress = 88 + 2 * i
                p.update(ks=4, kd=kd)
                self._apply(p)
                r = self._step_pair(hi, lo, p)
                self.say(f"Kd {kd} (Ks 4): {fmt(r)}")
                if r["pid_std"] > r_base["pid_std"] * 1.5 + 3 or r["fb_std"] > 2.0 or r["osc"] >= 2:
                    self.say("Kd starts adding noise - stopping.")
                    break
                if best is None or score(r) < score(best[1]):
                    best = (kd, r)
            if best and score(best[1]) <= 0.85 * score(r_base):
                p.update(kd=best[0], ks=4)
                self.say(f"Kd {best[0]} with Ks 4 helps.")
            else:
                p.update(kd=0, ks=1)
                self.say("Kd doesn't help enough - keeping it at 0.")
            self._apply(p)
        elif do_kd:
            p.update(kd=0, ks=1)
            self._apply(p)
            self.say("Hardly any overshoot - Kd not needed (0).")

        # 7. verify and finish at centre
        self.stage, self.progress = "Verifying", 97
        r_final, r_fsmall = measure()
        self.say(f"Final, big moves: {fmt(r_final)}")
        self.say(f"Final, small moves: {fmt(r_fsmall)}")
        self._ramp_to(c)
        self._settle(c, 3.0)
        keys = ["kp", "ki", "kd", "ks", "pwmmin", "dz"]
        self.result = {"motor": n, "before": {k: orig[k] for k in keys}, "after": {k: p[k] for k in keys},
                       "final": r_final, "notes": notes}
        for note in notes:
            self.say("Note: " + note)
        self.say("Done. The new values are running now but NOT saved - try them in a game, then press "
                 "'Save to EEPROM' to keep them or 'Revert' to go back.")
        self.progress = 100



# --------------------------------------------------------------------------------------------------------------
# Applying a full parameter set (profiles, backup restore) and named profiles
# --------------------------------------------------------------------------------------------------------------

PROFILE_KEYS = [f"{k}{n}" for n in (1, 2, 3) for k in ("Kp", "Ki", "Kd", "Ks", "PWM", "LIM", "DZ")] + \
    ["STATUS", "PWMFREQ", "FILT"]


def _num(v, lo, hi):
    return int(max(lo, min(hi, int(v))))


def apply_params(board, pr):
    """Send a parameter set (same shape as Board.read_params()) to the board. Live only - not saved."""
    if not isinstance(pr, dict) or pr.get("Kp1") is None:
        raise ValueError("no motor settings in it")
    for m in range(3):
        n = m + 1
        for codes, key, lo, hi in (("DEF", "Kp", 0, 65535), ("GHI", "Ki", 0, 65535), ("JKL", "Kd", 0, 65535),
                                   ("MNO", "Ks", 1, 1000)):
            v = pr.get(f"{key}{n}")
            if v is not None:
                board.cmd_word(codes[m], _num(v, lo, hi))
        for codes, key in (("PQR", "PWM"), ("STU", "LIM"), ("VWX", "DZ")):
            v = pr.get(f"{key}{n}")
            if isinstance(v, (list, tuple)) and len(v) == 2:
                board.cmd_pair(codes[m], _num(v[0], 0, 255), _num(v[1], 0, 255))
    st = pr.get("STATUS")
    if isinstance(st, (list, tuple)) and st:
        board.cmd_pair("Z", _num(int(st[0]) >> 4, 1, 10), 0)
    fq = pr.get("PWMFREQ")
    if isinstance(fq, (list, tuple)) and len(fq) == 2:
        board.cmd_pair("~", _num(fq[0], 1, 100), _num(fq[1], 1, 100))
    ft = pr.get("FILT")                      # ignored by firmware older than 1.6
    if isinstance(ft, (list, tuple)) and len(ft) == 2:
        board.cmd_pair("f", _num(ft[0], 0, 8), _num(ft[1], 0, 255))


def summary(p):
    kp = "/".join(str(p.get(f"Kp{n}", "?")) for n in (1, 2, 3))
    ki = "/".join(str(p.get(f"Ki{n}", "?")) for n in (1, 2, 3))
    out = f"Kp {kp}, Ki {ki}"
    ft = p.get("FILT")
    if isinstance(ft, (list, tuple)) and len(ft) == 2:
        out += ", smoothing " + ("off" if not ft[0] else f"{(1 << ft[0]) / 4:g} ms")
        if ft[1]:
            out += f", max {ft[1] * 100}/s"
    return out


def canon(p):
    return {k: p[k] for k in PROFILE_KEYS if k in p}


# --------------------------------------------------------------------------------------------------------------
# Backups: every Save to EEPROM and every bigger change keeps a copy of the board settings on the NAS
# --------------------------------------------------------------------------------------------------------------

class Backups:
    KEEP = {"eeprom": 60, "auto": 40, "manual": 200}

    def __init__(self, data_dir):
        self.path = os.path.join(data_dir, "backups.json")
        self.lock = threading.Lock()
        try:
            with open(self.path) as f:
                self.items = json.load(f)
            if not isinstance(self.items, list):
                self.items = []
        except (OSError, ValueError):
            self.items = []

    def _write(self):
        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            tmp = self.path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(self.items, f, indent=1)
            os.replace(tmp, self.path)
            return True
        except OSError as e:
            log(f"could not save backups to {self.path}: {e}")
            return False

    def add(self, kind, label, params):
        p = canon(params)
        if p.get("Kp1") is None:
            return None
        with self.lock:
            # an automatic copy identical to the newest backup adds nothing
            if kind == "auto" and self.items and self.items[0]["params"] == p:
                return self.items[0]["id"]
            if kind == "eeprom":
                last = next((b for b in self.items if b["kind"] == "eeprom"), None)
                if last and last["params"] == p and last is self.items[0]:
                    last["when"] = time.strftime("%Y-%m-%d %H:%M:%S")
                    self._write()
                    return last["id"]
            bid = f"{time.time():.3f}".replace(".", "")
            self.items.insert(0, {"id": bid, "when": time.strftime("%Y-%m-%d %H:%M:%S"), "kind": kind,
                                  "label": label[:80], "params": p})
            for k, n in self.KEEP.items():               # retention per kind
                same = [b for b in self.items if b["kind"] == k]
                for old in same[n:]:
                    self.items.remove(old)
            self._write()
            return bid

    def list(self):
        with self.lock:
            return [{"id": b["id"], "when": b["when"], "kind": b["kind"], "label": b["label"],
                     "summary": summary(b["params"]), "params": b["params"]} for b in self.items]

    def get(self, bid):
        with self.lock:
            return next((dict(b) for b in self.items if b["id"] == bid), None)

    def delete(self, bid):
        with self.lock:
            self.items = [b for b in self.items if b["id"] != bid]
            return self._write()


# --------------------------------------------------------------------------------------------------------------
# Recommended starting points per kind of sim. They adjust the user's own (ideally auto-tuned) settings rather
# than imposing absolute numbers, because good absolute gains depend on the rig's motors and linkage.
# --------------------------------------------------------------------------------------------------------------

GENRES = [
    {"key": "circuit", "title": "Circuit racing", "short": "GT, formula, touring - crisp and detailed.",
     "blurb": "Tarmac racing: crisp, detailed motion so kerbs, braking and weight transfer come through. Uses your "
              "tuned gains as they are and keeps smoothing short.",
     "kp": 1.0, "ki_min": None, "kd_min": None, "pwmmax": 0, "dz": 0, "smooth": 5, "rate": 0,
     "simhub": ["Pitch and roll: a little smoothing, about 10-20%. Heave and road texture: little or none.",
                "Global Motion Smoothing low, about 10%.",
                "Washout isn't needed for circuit racing.",
                "The main effects are surge (braking and acceleration) to pitch and sway (cornering) to roll."]},
    {"key": "rally", "title": "Rally & off-road", "short": "Gravel, jumps, rough roads - sharp with damping.",
     "blurb": "Rough surfaces and jumps: a sharper response, the shortest smoothing for every bump, a little more "
              "speed for big hits and some damping so hard impacts don't ring.",
     "kp": 1.15, "ki_min": None, "kd_min": 10, "pwmmax": 15, "dz": 0, "smooth": 4, "rate": 0,
     "simhub": ["Heave and road texture up, with little smoothing - that's where rally detail lives.",
                "Keep the crash filter on so landings and impacts don't slam the rig.",
                "A soft limiter of 15-25% on heave cushions big landings instead of hitting the end of travel.",
                "Global Motion Smoothing 0-10%."]},
    {"key": "flight", "title": "Flight", "short": "Planes and helicopters - smooth, held attitudes.",
     "blurb": "Aircraft change attitude slowly and hold it for a long time: a softer response, long smoothing, a "
              "speed cap against sudden lurches and enough Ki to hold a banked or pitched seat without sagging.",
     "kp": 0.75, "ki_min": 15, "kd_min": None, "pwmmax": -15, "dz": 1, "smooth": 7, "rate": 1500,
     "simhub": ["Washout on pitch and roll (and yaw) is essential - aircraft hold attitudes longer than your "
                "travel allows.",
                "More smoothing on pitch and roll, about 30-50%.",
                "A soft limiter on pitch and roll lets sustained attitudes ease into the limit instead of stopping "
                "hard.",
                "Global Motion Smoothing about 20-30%."]},
    {"key": "space", "title": "Space & sci-fi", "short": "Elite, Star Citizen - floaty but responsive.",
     "blurb": "Spaceflight: sustained turns and thruster pushes, floaty but still responsive - between flight and "
              "racing.",
     "kp": 0.85, "ki_min": 10, "kd_min": None, "pwmmax": -10, "dz": 1, "smooth": 6, "rate": 2000,
     "simhub": ["Washout on pitch, roll and yaw, as for flight.",
                "Moderate smoothing, about 20-30%.",
                "Thruster surge and sway work well mapped to pitch and roll."]},
    {"key": "truck", "title": "Trucks & cruising", "short": "ETS2, ATS, relaxed driving - gentle and heavy.",
     "blurb": "Heavy vehicles and relaxed driving: gentle weight transfer and road feel without harshness, long "
              "smoothing and a speed cap.",
     "kp": 0.8, "ki_min": 5, "kd_min": None, "pwmmax": -10, "dz": 0, "smooth": 7, "rate": 1500,
     "simhub": ["Lower the surge gain - long braking and acceleration otherwise pin you at full pitch.",
                "Road texture low and smoothed; trucks should feel heavy.",
                "Global Motion Smoothing about 20%."]},
]


def genre_preview(base, key, fwnum):
    g = next((x for x in GENRES if x["key"] == key), None)
    if g is None:
        raise ValueError("unknown recommendation")
    base = canon(base)
    if base.get("Kp1") is None:
        raise ValueError("no settings to start from")
    p = json.loads(json.dumps(base))
    rows = {"Kp": [], "Ki": [], "Kd": [], "Ks": [], "PWMmax": [], "Dead zone": []}
    for n in (1, 2, 3):
        kp, ki, kd, ks = base[f"Kp{n}"], base[f"Ki{n}"], base[f"Kd{n}"], base[f"Ks{n}"]
        pmin, pmax = base[f"PWM{n}"]
        dz, rev = base[f"DZ{n}"]
        nkp = int(max(40, min(2000, round(kp * g["kp"]))))
        nki = max(ki, g["ki_min"]) if g["ki_min"] is not None else ki
        nki = int(min(100, nki))
        nkd, nks = kd, ks
        if g["kd_min"] is not None and kd < g["kd_min"]:
            nkd, nks = g["kd_min"], max(ks, 4)
        npmax = int(max(min(255, pmin + 20), min(255, pmax + g["pwmmax"])))
        ndz = int(max(1, min(8, dz + g["dz"])))
        p[f"Kp{n}"], p[f"Ki{n}"], p[f"Kd{n}"], p[f"Ks{n}"] = nkp, nki, nkd, nks
        p[f"PWM{n}"] = [pmin, npmax]
        p[f"DZ{n}"] = [ndz, rev]
        rows["Kp"].append([kp, nkp]); rows["Ki"].append([ki, nki]); rows["Kd"].append([kd, nkd])
        rows["Ks"].append([ks, nks]); rows["PWMmax"].append([pmax, npmax]); rows["Dead zone"].append([dz, ndz])
    notes = []
    if fwnum >= 16:
        old = base.get("FILT") or [6, 0]
        p["FILT"] = [g["smooth"], g["rate"] // 100]
        fmt_s = lambda v: "off" if not v else f"{(1 << v) / 4:g} ms"
        rows["Smoothing"] = [[fmt_s(old[0]), fmt_s(g["smooth"])]] * 3
        rows["Max speed /s"] = [[old[1] * 100 or "off", g["rate"] or "off"]] * 3
    else:
        p.pop("FILT", None)
        notes.append(f"With firmware 1.6 this would also set smoothing to {(1 << g['smooth']) / 4:g} ms"
                     + (f" and max speed to {g['rate']}/s" if g["rate"] else "") + " - flash it to include that.")
    if base.get("Kp1") == 100 and base.get("Ki1") == 40 and base.get("Kd1") == 40:
        notes.append("These look like the firmware's default settings. Run auto-tune first - the recommendation "
                     "adjusts from your tuned values, so a good baseline makes it much better.")
    out_rows = [{"name": k, "vals": v} for k, v in rows.items()]
    return {"ok": True, "title": g["title"], "blurb": g["blurb"], "simhub": g["simhub"], "notes": notes,
            "rows": out_rows, "params": p}



class Profiles:
    def __init__(self, data_dir):
        self.path = os.path.join(data_dir, "profiles.json")
        self.lock = threading.Lock()
        try:
            with open(self.path) as f:
                self.data = json.load(f)
            if not isinstance(self.data, dict):
                self.data = {}
        except (OSError, ValueError):
            self.data = {}

    def _write(self):
        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            tmp = self.path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(self.data, f, indent=2)
            os.replace(tmp, self.path)
            return True
        except OSError as e:
            log(f"could not save profiles to {self.path}: {e}")
            return False

    def list(self):
        with self.lock:
            out = []
            for name in sorted(self.data, key=str.lower):
                p = self.data[name].get("params", {})
                kp = "/".join(str(p.get(f"Kp{n}", "?")) for n in (1, 2, 3))
                out.append({"name": name, "saved": self.data[name].get("saved", ""), "summary": f"Kp {kp}"})
            return out

    def save(self, name, params):
        with self.lock:
            self.data[name] = {"saved": time.strftime("%Y-%m-%d %H:%M"),
                               "params": {k: params[k] for k in PROFILE_KEYS if k in params}}
            return self._write()

    def get(self, name):
        with self.lock:
            p = self.data.get(name)
            return dict(p["params"]) if p else None

    def delete(self, name):
        with self.lock:
            self.data.pop(name, None)
            return self._write()

# --------------------------------------------------------------------------------------------------------------
# Diagnostics (v1.7): an event log of everything that happens, a watcher that notices problems while you drive
# (keeping a few seconds of data around each one), a quick check (no movement) and a full check (moves each motor
# a little), and one self-describing file to hand to whoever helps you - or to Claude.
# --------------------------------------------------------------------------------------------------------------

DIAG_HEAD = struct.Struct("<I3BB4I2HIbBB")       # firmware 1.7 [rdd] frame, see SendDiag() in SMC3_R4.ino (1.7+)
DIAG_MOTOR = struct.Struct("<BHhHHHH")
OFF_REASONS = {0: "on", 1: "safety cutoff", 2: "switched off by command", 3: "not enabled since power-up"}
GLITCH_COUNTS = 60          # pot change within one 10 ms telemetry sample that no motor can make (6000 counts/s)
INC_KEEP = 40               # incident recordings kept on the NAS
NOTE_LIMIT = {"pot_glitch": 60, "pot_glitch_fw": 300, "hunting": 120, "wrong_direction": 20, "stall": 30,
              "motor_off": 0, "reboot": 0, "telemetry_gap": 120}
INC_TITLES = {"pot_glitch": "Pot reading jumped", "pot_glitch_fw": "Pot glitches (firmware count)",
              "hunting": "Swinging around its target", "wrong_direction": "Moved away from its target",
              "stall": "Pushed hard but not moving", "motor_off": "Motor switched off",
              "reboot": "Board restarted", "telemetry_gap": "No data from the board"}


def name_text(s, names):
    """'Motor 2 moves ...' -> 'Motor 2 (Right) moves ...' for motors the owner has named (Settings, Motor names)."""
    if not s or not names or not any(names):
        return s
    nm = lambda i: names[i - 1] if 1 <= i <= len(names) else ""
    tag = lambda i: f" ({nm(i)})" if nm(i) else ""
    s = re.sub(r"\b([Mm])otors ([123]) and ([123])\b(?! \()",
               lambda mo: f"{mo.group(1)}otors {mo.group(2)}{tag(int(mo.group(2)))} and {mo.group(3)}{tag(int(mo.group(3)))}", s)
    return re.sub(r"\b([Mm])otor ([123])\b(?! \()", lambda mo: mo.group(0) + tag(int(mo.group(2))), s)


def _n1(v):
    s = f"{v:.1f}"
    return s[:-2] if s.endswith(".0") else s


def _ago(sec):
    sec = int(max(0, sec))
    if sec < 90:
        return f"{sec} s"
    if sec < 5400:
        return f"{sec // 60} min"
    if sec < 172800:
        return f"{sec // 3600} h {sec % 3600 // 60} min"
    return f"{sec // 86400} days"


def decode_diag(pl):
    """Firmware 1.7 diagnostics frame -> dict (None if it is too short)."""
    need = DIAG_HEAD.size + 3 * DIAG_MOTOR.size + 1
    if len(pl) < need:
        return None
    (ms, r0, r1, r2, fl, usb_ok, usb_tgt, usb_bad, tcp_ok, since_usb, since_tgt, loop_us, rssi, build,
     fwn) = DIAG_HEAD.unpack_from(pl, 0)
    off = DIAG_HEAD.size
    motors = []
    for _ in range(3):
        why, trips, at, jumps, jmax, fmin, fmax = DIAG_MOTOR.unpack_from(pl, off)
        off += DIAG_MOTOR.size
        motors.append({"off_reason": OFF_REASONS.get(why, str(why)), "cutoff_trips": trips,
                       "cutoff_at": None if at < 0 else at, "pot_glitches": jumps,
                       "pot_biggest_step": round(jmax / 16.0, 1), "pot_min": round(fmin / 16.0, 1),
                       "pot_max": round(fmax / 16.0, 1)})
    n = pl[off]
    ee = list(pl[off + 1: off + 1 + n])
    return {"rx": time.time(), "uptime_ms": ms, "reset_flags": [r0, r1, r2], "cwsf_ok": bool(fl & 1),
            "monitor_mode": (fl >> 1) & 3, "usb_ok": usb_ok, "usb_targets": usb_tgt, "usb_bad_bytes": usb_bad,
            "wifi_cmds": tcp_ok, "ms_since_usb": since_usb, "ms_since_usb_target": since_tgt,
            "loop_max_us": loop_us, "rssi": rssi, "build": build, "fw": fwn, "motors": motors, "eeprom": ee}


def reset_cause(d):
    """What the reset-status registers say about the last start (firmware 1.7)."""
    r0, r1, r2 = d["reset_flags"]
    parts = []
    if r0 & 0x0E:
        parts.append("supply voltage monitor")
    if r1 & 0x03:
        parts.append("watchdog")
    if r1 & 0x04:
        parts.append("software reset (a new sketch was uploaded, or the WiFi/USB chip restarted the board)")
    cold = d["cwsf_ok"] and not (r2 & 1)
    if cold or (r0 & 1):
        head = "power-on (the supply was off or dipped low enough to restart it)"
    elif parts:
        head = parts.pop(0)
    else:
        head = "reset button, or restarted by the board's WiFi/USB chip"
    return head + (" + " + ", ".join(parts) if parts else ""), bool(cold or (r0 & 1))


def decode_eeprom(ee):
    """SMC3 v0.70 EEPROM layout (+ 1.6 smoothing) -> same shape as Board.read_params()."""
    if len(ee) < 56 or ee[0] != 114:
        return None
    w = lambda a: ee[a] * 256 + ee[a + 1]
    cut, clip, dz = [ee[1], ee[6], ee[31]], [ee[2], ee[7], ee[32]], [ee[5], ee[10], ee[35]]
    kp, ki, kd = [w(11), w(17), w(36)], [w(13), w(19), w(38)], [w(15), w(21), w(40)]
    pmin, pmax, prev, ks = [ee[23], ee[24], ee[42]], [ee[25], ee[26], ee[43]], [ee[27], ee[28], ee[29]], \
        [w(44), w(46), w(48)]
    p = {}
    for m in range(3):
        n = m + 1
        p[f"Kp{n}"], p[f"Ki{n}"], p[f"Kd{n}"], p[f"Ks{n}"] = kp[m], ki[m], kd[m], ks[m]
        p[f"PWM{n}"], p[f"LIM{n}"], p[f"DZ{n}"] = [pmin[m], pmax[m]], [cut[m], clip[m]], [dz[m], prev[m]]
    p["STATUS"] = [max(1, min(10, ee[50])) * 16, 0]
    p["PWMFREQ"] = [ee[51], ee[52]]
    if ee[55] == 0x5A:
        p["FILT"] = [min(8, ee[53]), ee[54]]
    return p


SETTING_NAMES = [("Kp", "Kp", None), ("Ki", "Ki", None), ("Kd", "Kd", None), ("Ks", "Ks", None),
                 ("PWMmin", "PWM", 0), ("PWMmax", "PWM", 1), ("Max limit (cutoff)", "LIM", 0), ("Clip", "LIM", 1),
                 ("Dead zone", "DZ", 0), ("PWMrev", "DZ", 1)]


def motor_settings(p, n):
    out = {}
    for name, key, idx in SETTING_NAMES:
        v = p.get(f"{key}{n}")
        out[name] = (v[idx] if isinstance(v, (list, tuple)) and len(v) == 2 else None) if idx is not None else v
    return out


def global_settings(p):
    st, fq, ft = p.get("STATUS"), p.get("PWMFREQ"), p.get("FILT")
    return {"Fpid divider": (st[0] >> 4) if st else None,
            "PWM kHz motors 1+2": fq[0] if fq else None, "PWM kHz motor 3": fq[1] if fq else None,
            "Target smoothing": (("off" if not ft[0] else f"{(1 << ft[0]) / 4:g} ms") if ft else None),
            "Max speed counts/s": ((ft[1] * 100 or "off") if ft else None)}


def settings_diff(run, stored, motors):
    out = []
    for n in motors:
        a, b = motor_settings(run, n), motor_settings(stored, n)
        for k in a:
            if a[k] is not None and b[k] is not None and a[k] != b[k]:
                out.append({"motor": n, "setting": k, "running": a[k], "saved": b[k]})
    ga, gb = global_settings(run), global_settings(stored)
    for k in ga:
        if ga[k] is not None and gb[k] is not None and ga[k] != gb[k]:
            out.append({"motor": None, "setting": k, "running": ga[k], "saved": gb[k]})
    return out


def trace_csv(samples, motor=None, motors=(1, 2, 3), every=1):
    """Telemetry as compact CSV. motor = the one of interest (all four values), others: feedback only."""
    if motor is None:
        main, others = list(motors), []
    else:
        main, others = [motor], [k for k in motors if k != motor]
    cols = ["ms"] + [f"{k}{n}" for n in main for k in ("t", "g", "f", "p")] + [f"f{n}" for n in others] + ["off", "low"]
    rows = []
    for i, s in enumerate(samples):
        if every > 1 and i % every:
            continue
        r = [str(s["ms"])]
        for n in main:
            mm = s["m"][n - 1]
            r += [str(mm["t"]), _n1(mm["g"]), _n1(mm["f"]), _n1(mm["p"])]
        r += [_n1(s["m"][n - 1]["f"]) for n in others]
        r.append("".join("1" if d else "0" for d in s["dis"][:3]))
        r.append("1" if s["low"] else "0")
        rows.append(",".join(r))
    return {"columns": ",".join(cols), "rate_hz": 100 // max(1, every), "rows": "\n".join(rows)}


def robust_sigma(xs):
    """1-sigma spread that a few wild values (glitches) can't inflate: 1.4826 x median absolute deviation."""
    if len(xs) < 5:
        return None
    med = statistics.median(xs)
    return 1.4826 * statistics.median(abs(x - med) for x in xs)


def despike(samples, m, lim=30.0):
    """The samples with single-sample pot glitches on motor m (a reading far from both neighbours, which agree with
    each other) replaced by the neighbours' average - one bad reading must not look like a 700% overshoot."""
    out = list(samples)
    for i in range(1, len(samples) - 1):
        a, b, c = samples[i - 1]["m"][m]["f"], samples[i]["m"][m]["f"], samples[i + 1]["m"][m]["f"]
        if abs(b - a) > lim and abs(b - c) > lim and abs(a - c) < lim:
            s = dict(samples[i])
            mm = list(s["m"])
            x = dict(mm[m])
            x["f"] = (a + c) / 2.0
            mm[m] = x
            s["m"] = mm
            out[i] = s
    return out


def rest_noise(fs):
    """Pot noise in counts (1 sigma) after removing slow movement (5-sample moving average), robust to glitches."""
    if len(fs) < 12:
        return None
    resid = [fs[i] - _mean(fs[i - 2:i + 3]) for i in range(2, len(fs) - 2)]
    return robust_sigma(resid) * 1.12


class EventLog:
    """Timeline of what happened - connects, restarts, settings changes, tests, problems noticed. Kept on the NAS
    (diagnostics/events.jsonl) so a diagnostics file can show what led up to a problem."""
    KEEP = 3000

    def __init__(self, data_dir):
        self.dir = os.path.join(data_dir, "diagnostics")
        self.path = os.path.join(self.dir, "events.jsonl")
        self.lock = threading.Lock()
        self.items = collections.deque(maxlen=self.KEEP)
        self.lines = 0
        try:
            with open(self.path) as f:
                for line in f:
                    try:
                        self.items.append(json.loads(line))
                        self.lines += 1
                    except ValueError:
                        pass
        except OSError:
            pass

    def add(self, kind, msg, motor=None, quiet=False, **data):
        e = {"t": time.strftime("%Y-%m-%d %H:%M:%S"), "ts": round(time.time(), 2), "kind": kind, "msg": msg}
        if motor is not None:
            e["motor"] = motor
        if data:
            e["data"] = data
        with self.lock:
            self.items.append(e)
            try:
                os.makedirs(self.dir, exist_ok=True)
                if self.lines >= 2 * self.KEEP:
                    tmp = self.path + ".tmp"
                    with open(tmp, "w") as f:
                        for x in self.items:
                            f.write(json.dumps(x) + "\n")
                    os.replace(tmp, self.path)
                    self.lines = len(self.items)
                else:
                    with open(self.path, "a") as f:
                        f.write(json.dumps(e) + "\n")
                    self.lines += 1
            except OSError:
                pass
        if not quiet:
            log(f"[event] {msg}")
        return e

    def recent(self, n=300, since=0.0, kinds=None):
        with self.lock:
            items = [x for x in self.items if x.get("ts", 0) >= since and (not kinds or x.get("kind") in kinds)]
        return items[-n:]


class Monitor:
    """Watches the telemetry all the time - also while you drive - and notices: pot glitches, a motor moving away
    from its target, a motor pushed hard that doesn't move, hunting at rest, motors switching off, board restarts
    and data gaps. Keeps a few seconds of data around each one (diagnostics/incidents) and per-minute statistics
    (how often each motor is at full power, how far behind SimHub it runs, ...)."""

    def __init__(self, board, gen, tuner, cfg, events, data_dir):
        self.board, self.gen, self.tuner, self.cfg, self.events = board, gen, tuner, cfg, events
        self.diag = None
        self.inc_dir = os.path.join(data_dir, "diagnostics", "incidents")
        self.lock = threading.Lock()
        self.incidents = []
        self._load_incidents()
        self.pending = []
        self.counts = collections.Counter()
        self.last_note = {}
        self.seq = 0
        self.prev = None
        self.win = [collections.deque(maxlen=260) for _ in range(3)]
        self.busy_until = [0.0] * 3
        self.started = time.time()
        self.sim_last = 0.0
        self.sim_on = False
        self.low = None
        self.dis = None
        self.last_ms = None
        self.minutes = collections.deque(maxlen=120)
        self.noise = [None] * 3
        self.pid_noise = [None] * 3
        self.noise_t = [0.0] * 3
        self.tick = 0
        self.prev_diag = None
        self.diag_hist = collections.deque(maxlen=400)
        self.last_diag_poll = 0.0
        self.want_cause = False
        self.usb_rate = None
        self.gaps = 0
        self.was_connected = False
        threading.Thread(target=self._loop, daemon=True).start()

    # ---- incidents on disk ----
    def _load_incidents(self):
        try:
            names = sorted((n for n in os.listdir(self.inc_dir) if n.endswith(".json")), reverse=True)
        except OSError:
            names = []
        for name in names[:INC_KEEP]:
            try:
                with open(os.path.join(self.inc_dir, name)) as f:
                    inc = json.load(f)
                inc.pop("trace", None)
                inc["file"] = name
                self.incidents.append(inc)
            except (OSError, ValueError):
                pass

    def incident_list(self, n=40):
        with self.lock:
            return [dict(x) for x in self.incidents[:n]]

    def incident_full(self, name):
        if "/" in name or ".." in name:
            return None
        try:
            with open(os.path.join(self.inc_dir, name)) as f:
                return json.load(f)
        except (OSError, ValueError):
            return None

    def clear_incidents(self):
        with self.lock:
            for x in self.incidents:
                try:
                    os.remove(os.path.join(self.inc_dir, x["file"]))
                except (OSError, KeyError):
                    pass
            self.incidents = []
            self.counts.clear()
        self.events.add("diag", "Recorded problems cleared from the page.", quiet=True)

    def summary(self):
        with self.lock:
            return {"count": len(self.incidents), "newest": self.incidents[0]["ts"] if self.incidents else 0}

    # ---- main loop ----
    def _loop(self):
        while True:
            time.sleep(0.2)
            try:
                self._step()
            except Exception as e:                       # the watcher must never die
                log(f"monitor: {e!r}")
                time.sleep(2)

    def _busy_any(self):
        return any(time.time() < t for t in self.busy_until)

    def _step(self):
        b, now = self.board, time.time()
        if b.connected != self.was_connected:
            self.was_connected = b.connected
            self.prev = None
            for w in self.win:
                w.clear()
        if not b.connected:
            return
        new = b.since(self.seq)
        if new:
            self.seq = new[-1]["seq"]
            for s in new:
                self._sample(s, now)
        self._finish_pending()
        fw = (b.fw_cache or [0, 0])[1]
        if fw >= 17 and (now - self.last_diag_poll > 10 or (self.want_cause and now - self.last_diag_poll > 2)):
            self.last_diag_poll = now
            self._poll_diag()

    def _bucket(self, now):
        key = int(now // 60)
        if not self.minutes or self.minutes[-1]["k"] != key:
            self.minutes.append({"k": key, "t": time.strftime("%Y-%m-%d %H:%M", time.localtime(key * 60)), "n": 0,
                                 "gaps": 0, "low": 0, "sim": 0,
                                 "m": [{"act": 0, "sat": 0, "esum": 0.0, "emax": 0.0, "hist": [0] * 9,
                                        "fmin": 9999.0, "fmax": -1.0, "tmin": 9999, "tmax": -1, "vmax": 0.0,
                                        "glitch": 0} for _ in range(3)]})
        return self.minutes[-1]

    def _busy(self, m, now):
        g, t, d = self.gen, self.tuner, self.diag
        if (g.mode != "off" and g.motor == m) or (t.running() and t.motor == m) or (d is not None and d.moving()):
            self.busy_until[m] = now + 3.0
        return now < self.busy_until[m]

    def _sample(self, s, now):
        ms = s["ms"]
        bk = self._bucket(now)
        if self.last_ms is not None:
            if ms + 2000 < self.last_ms:
                self._reboot(self.last_ms, ms, s)
            elif ms - self.last_ms > 300 and self.prev is not None:
                self.gaps += 1
                bk["gaps"] += 1
                if ms - self.last_ms > 1500:
                    self._note("telemetry_gap", None, s, f"No data from the board for {(ms - self.last_ms) / 1000:.1f} s "
                               f"(WiFi stalled, or the board's loop was blocked).", gap_ms=ms - self.last_ms)
        self.last_ms = ms
        bk["n"] += 1
        if s["low"]:
            bk["low"] += 1
        if self.low is None:
            self.low = s["low"]
        elif s["low"] != self.low:
            self.low = s["low"]
            self.events.add("power", "Low power: no position commands for 15 s, so the board runs at a quarter of "
                            "its drive." if s["low"] else "Full power: position commands are arriving again.",
                            quiet=True)
        nm = int(self.cfg.get("motors"))
        drive = [self._busy(m, now) for m in range(3)]
        if self.prev is not None:
            for m in range(nm):
                if not drive[m] and s["m"][m]["t"] != self.prev["m"][m]["t"]:
                    self.sim_last = now
        sim = now - self.sim_last < 3.0
        if sim:
            bk["sim"] += 1
        if sim != self.sim_on and (sim or now - self.sim_last > 6.0):
            self.sim_on = sim
            self.events.add("simhub", "Positions started arriving from outside this page (SimHub driving)." if sim else
                            "Positions from outside this page stopped (SimHub idle or closed).", quiet=True)
        if self.dis is None:
            self.dis = list(s["dis"])
        for m in range(3):
            if s["dis"][m] != self.dis[m]:
                self.dis[m] = s["dis"][m]
                self._dis_change(m, s, now, nm)
        self.tick += 1
        heavy = self.tick % 10 == 0
        for m in range(nm):
            self._motor(m, s, now, drive[m], sim, bk, heavy)
        self.prev = s

    def _reboot(self, before, after, s):
        self.events.add("board", f"The board restarted: it had been running {_ago(before / 1000)}, now "
                        f"{_ago(after / 1000)}.", uptime_before_s=round(before / 1000), uptime_after_s=round(after / 1000))
        self._note("reboot", None, s, f"The board restarted after running for {_ago(before / 1000)}.",
                   uptime_before_s=round(before / 1000))
        self.want_cause = True
        self.prev = None
        for w in self.win:
            w.clear()

    def _dis_change(self, m, s, now, nm):
        n = m + 1
        if not s["dis"][m]:
            self.events.add("motor", f"Motor {n} switched on.", motor=n, quiet=True)
            return
        if now - self.board.last_dis_cmd[m] < 3.0:
            self.events.add("motor", f"Motor {n} switched off from this page.", motor=n, quiet=True)
            return
        if m >= nm:
            self.events.add("motor", f"Motor {n} (not in use) switched off.", motor=n, quiet=True)
            return
        f = s["m"][m]["f"]
        lim = (self.board.params.get(f"LIM{n}") or [23, 100])[0]
        beyond = f < lim or f > 1023 - lim
        why = (f"its pot read {f:.0f}, past the safety cutoff ({lim} / {1023 - lim})" if beyond else
               f"pot {f:.0f}; most likely its safety cutoff tripped for a moment")
        self._note("motor_off", m, s, f"Motor {n} switched itself off - {why}.", pot=round(f, 1), cutoff=lim,
                   beyond=beyond)
        self.want_cause = True

    def _motor(self, m, s, now, busy, sim, bk, heavy):
        mm = s["m"][m]
        t, g, f, p = mm["t"], mm["g"], mm["f"], mm["p"]
        off = s["dis"][m]
        st = bk["m"][m]
        st["fmin"], st["fmax"] = min(st["fmin"], f), max(st["fmax"], f)
        st["tmin"], st["tmax"] = min(st["tmin"], t), max(st["tmax"], t)
        w = self.win[m]
        last = w[-1] if w else None
        w.append((s["ms"], t, g, f, p, off))
        if last and 0 < s["ms"] - last[0] <= 15:
            dv = abs(f - last[3])
            if dv >= GLITCH_COUNTS:
                st["glitch"] += 1
                self._note("pot_glitch", m, s, f"Motor {m + 1}'s pot reading jumped {dv:.0f} counts in "
                           f"{s['ms'] - last[0]} ms ({last[3]:.0f} -> {f:.0f}) - faster than any motor can move.",
                           jump=round(dv, 1), before=round(last[3], 1), after=round(f, 1))
            else:
                st["vmax"] = max(st["vmax"], dv * 1000.0 / (s["ms"] - last[0]))
        if off:
            return
        pr = self.board.params
        n = m + 1
        pwm = pr.get(f"PWM{n}")
        lim = pr.get(f"LIM{n}") or [23, 100]
        dz = (pr.get(f"DZ{n}") or [0, 0])[0]
        clip = lim[1]
        if sim and not busy:
            e = abs(g - f)
            st["act"] += 1
            st["esum"] += e
            st["emax"] = max(st["emax"], e)
            st["hist"][sum(1 for edge in (2, 5, 10, 20, 40, 80, 160, 320) if e >= edge)] += 1
            if pwm and abs(p) + pwm[0] >= pwm[1] - 1 and e > dz:
                st["sat"] += 1
        if busy or not heavy:
            return
        inside = lambda v: clip <= v <= 1023 - clip
        # moving away from the target although driven towards it (same rule as auto-tune's safety watch)
        win = [x for x in w if s["ms"] - x[0] <= 300]
        if len(win) >= 20 and win[-1][0] - win[0][0] >= 250 and not any(x[5] for x in win):
            es = [x[2] - x[3] for x in win]
            if all(abs(e) > 30 for e in es) and (all(e > 0 for e in es) or all(e < 0 for e in es)):
                sg = 1 if es[0] > 0 else -1
                moved = (win[-1][3] - win[0][3]) * sg
                if all(x[4] * sg > 15 for x in win) and moved < -10 and all(inside(x[3]) for x in win):
                    self._note("wrong_direction", m, s, f"Motor {n} moved AWAY from its target while it was being "
                               f"driven towards it ({-moved:.0f} counts in {win[-1][0] - win[0][0]} ms). The motor "
                               f"or its pot is wired backwards, or the pot slipped on its shaft.",
                               moved_away=round(-moved, 1))
        # pushed at full allowed power for a second without moving
        if pwm:
            win1 = [x for x in w if s["ms"] - x[0] <= 1000]
            if len(win1) >= 80 and win1[-1][0] - win1[0][0] >= 950 and not any(x[5] for x in win1):
                es = [x[2] - x[3] for x in win1]
                fs = [x[3] for x in win1]
                if (all(e > 40 for e in es) or all(e < -40 for e in es)) and \
                        all(abs(x[4]) + pwm[0] >= pwm[1] - 1 for x in win1) and max(fs) - min(fs) < 4 and \
                        all(inside(v) for v in fs):
                    self._note("stall", m, s, f"Motor {n} was pushed at full allowed power (PWMmax {pwm[1]}) for a "
                               f"second but didn't move (pot {fs[-1]:.0f}, goal {win1[-1][2]:.0f}).",
                               pot=round(fs[-1], 1), goal=round(win1[-1][2], 1), pwmmax=pwm[1])
        # at rest: hunting, and the noise estimate
        win2 = [x for x in w if s["ms"] - x[0] <= 2000]
        if len(win2) >= 150 and win2[-1][0] - win2[0][0] >= 1900 and not any(x[5] for x in win2):
            ts = [x[1] for x in win2]
            if max(ts) - min(ts) <= 1 and abs(win2[-1][2] - win2[-1][1]) < 2:
                fs = [x[3] for x in win2]
                mean = _mean(fs)
                nz_here = rest_noise(fs) or 0.0
                thr = max(3.0, dz + 1.5, 3.0 * nz_here)          # a noisy pot alone must not look like hunting
                crossings, state = 0, 0
                for v in fs:
                    st_ = 1 if v - mean > thr else (-1 if v - mean < -thr else 0)
                    if st_ and state and st_ != state:
                        crossings += 1
                    if st_:
                        state = st_
                if crossings >= 8:
                    self._note("hunting", m, s, f"Motor {n} kept swinging around its target at rest ({crossings} "
                               f"swings in 2 s, about ±{(max(fs) - min(fs)) / 2:.0f} counts) instead of holding.",
                               swings=crossings, amplitude=round((max(fs) - min(fs)) / 2, 1))
                elif now - self.noise_t[m] > 2.0:
                    self.noise_t[m] = now
                    nz = rest_noise(fs[-120:])
                    pn = robust_sigma([x[4] for x in win2[-120:]]) or 0.0
                    if nz is not None:
                        self.noise[m] = nz if self.noise[m] is None else 0.7 * self.noise[m] + 0.3 * nz
                        self.pid_noise[m] = pn if self.pid_noise[m] is None else 0.7 * self.pid_noise[m] + 0.3 * pn

    # ---- noticing ----
    def _note(self, kind, m, s, msg, **data):
        key = (kind, m)
        self.counts[key] += 1
        now = time.time()
        if now - self.last_note.get(key, 0.0) < NOTE_LIMIT.get(kind, 60):
            return
        self.last_note[key] = now
        inc = {"id": f"{int(now * 1000)}", "when": time.strftime("%Y-%m-%d %H:%M:%S"), "ts": round(now, 2),
               "kind": kind, "title": INC_TITLES.get(kind, kind), "motor": None if m is None else m + 1, "msg": msg,
               "data": data, "board_ms": s["ms"] if s else None, "seq": s["seq"] if s else self.board.seq}
        self.events.add("problem", msg, motor=inc["motor"], incident=kind)
        self.pending.append(inc)

    def _finish_pending(self):
        if not self.pending:
            return
        now, seq = time.time(), self.board.seq
        keep = []
        for inc in self.pending:
            if seq < inc["seq"] + 150 and now - inc["ts"] < 4.0:
                keep.append(inc)
                continue
            snap = [x for x in self.board.since(inc["seq"] - 400) if x["seq"] <= inc["seq"] + 150]
            nm = int(self.cfg.get("motors"))
            inc["trace"] = trace_csv(snap, inc["motor"], tuple(range(1, nm + 1)))
            self._store(inc)
        self.pending = keep

    def _store(self, inc):
        name = f"{inc['id']}_{inc['kind']}" + (f"_m{inc['motor']}" if inc["motor"] else "") + ".json"
        try:
            os.makedirs(self.inc_dir, exist_ok=True)
            with open(os.path.join(self.inc_dir, name), "w") as f:
                json.dump(inc, f)
        except OSError as e:
            log(f"could not save an incident recording: {e}")
            return
        meta = {k: v for k, v in inc.items() if k != "trace"}
        meta["file"] = name
        with self.lock:
            self.incidents.insert(0, meta)
            for old in self.incidents[INC_KEEP:]:
                try:
                    os.remove(os.path.join(self.inc_dir, old["file"]))
                except OSError:
                    pass
            del self.incidents[INC_KEEP:]

    # ---- firmware 1.7 counters ----
    def _poll_diag(self):
        d = self.board.read_diag()
        if not d:
            return
        prev, self.prev_diag = self.prev_diag, d
        self.diag_hist.append({k: d[k] for k in ("rx", "uptime_ms", "rssi", "loop_max_us", "usb_ok", "usb_targets",
                                                  "usb_bad_bytes")})
        nm = int(self.cfg.get("motors"))
        if self.want_cause:
            self.want_cause = False
            cause, _ = reset_cause(d)
            if d["uptime_ms"] < 600000:
                self.events.add("board", f"The board says it last started because of: {cause}.", cause=cause)
            for m in range(3):
                md = d["motors"][m]
                if md["off_reason"] == "safety cutoff" and md["cutoff_at"] is not None and m < nm:
                    self.events.add("motor", f"Motor {m + 1}: the board confirms its safety cutoff tripped at pot "
                                    f"{md['cutoff_at']} ({md['cutoff_trips']} trips since power-up).", motor=m + 1)
        if not prev or d["uptime_ms"] < prev["uptime_ms"]:
            return
        dt = max(0.001, (d["uptime_ms"] - prev["uptime_ms"]) / 1000.0)
        self.usb_rate = (d["usb_targets"] - prev["usb_targets"]) / dt
        s = self.board.latest()
        for m in range(nm):
            a, b = d["motors"][m], prev["motors"][m]
            k = a["pot_glitches"] - b["pot_glitches"]
            if k > 0 and (self.noise[m] or 0) < 2.5:          # a very noisy pot trips the counter too - that's noise
                self._note("pot_glitch_fw", m, s, f"Motor {m + 1}'s pot glitched {k} times in the last {dt:.0f} s "
                           f"(readings that jumped faster than any motor can move; biggest change "
                           f"{a['pot_biggest_step']:.0f} counts in 1/4 ms).", glitches=k, biggest=a["pot_biggest_step"])
            if a["cutoff_trips"] > b["cutoff_trips"]:
                self.events.add("motor", f"Motor {m + 1}: safety cutoff tripped at pot {a['cutoff_at']}.", motor=m + 1)
        bad = d["usb_bad_bytes"] - prev["usb_bad_bytes"]
        if bad > 20 and now_minus(self.last_note.get(("usb_bad", None), 0.0)) > 300:
            self.last_note[("usb_bad", None)] = time.time()
            self.events.add("simhub", f"{bad} bytes of unreadable data arrived on the USB port in {dt:.0f} s "
                            f"(another program on the COM port, or a wrong output format).", bad_bytes=bad)
        if d["loop_max_us"] > 150000 and now_minus(self.last_note.get(("loop", None), 0.0)) > 600:
            self.last_note[("loop", None)] = time.time()
            self.events.add("board", f"The board's main loop was blocked for {d['loop_max_us'] / 1000:.0f} ms "
                            f"(WiFi module) - SimHub's commands waited that long.", quiet=True)

    # ---- statistics ----
    def stats(self, minutes=60):
        cut = time.time() // 60 - minutes
        bks = [b for b in list(self.minutes) if b["k"] > cut]
        nm = int(self.cfg.get("motors"))
        tot = {"minutes_covered": len(bks), "samples": sum(b["n"] for b in bks), "gaps": sum(b["gaps"] for b in bks),
               "low_power_s": round(sum(b["low"] for b in bks) / 100.0, 1),
               "simhub_driving_s": round(sum(b["sim"] for b in bks) / 100.0, 1), "motors": []}
        edges = [2, 5, 10, 20, 40, 80, 160, 320, 1024]
        for m in range(nm):
            ms = [b["m"][m] for b in bks]
            act = sum(x["act"] for x in ms)
            hist = [sum(x["hist"][i] for x in ms) for i in range(9)]
            p95 = None
            if act:
                run, lim = 0, 0.95 * act
                for i, c in enumerate(hist):
                    run += c
                    if run >= lim:
                        p95 = edges[i]
                        break
            fmin = min((x["fmin"] for x in ms if x["fmax"] >= 0), default=None)
            fmax = max((x["fmax"] for x in ms if x["fmax"] >= 0), default=None)
            tot["motors"].append({
                "motor": m + 1, "driven_by_simhub_s": round(act / 100.0, 1),
                "at_full_allowed_power_pct": round(100.0 * sum(x["sat"] for x in ms) / act, 1) if act else None,
                "error_mean": round(sum(x["esum"] for x in ms) / act, 1) if act else None,
                "error_p95_below": p95, "error_max": round(max((x["emax"] for x in ms), default=0.0), 1),
                "pot_min": None if fmin is None else round(fmin, 1), "pot_max": None if fmax is None else round(fmax, 1),
                "target_min": min((x["tmin"] for x in ms if x["tmax"] >= 0), default=None),
                "target_max": max((x["tmax"] for x in ms if x["tmax"] >= 0), default=None),
                "speed_max_counts_s": round(max((x["vmax"] for x in ms), default=0.0)),
                "pot_glitches": sum(x["glitch"] for x in ms),
                "noise_at_rest": None if self.noise[m] is None else round(self.noise[m], 2),
                "pid_noise_at_rest": None if self.pid_noise[m] is None else round(self.pid_noise[m], 2)})
        return tot

    def minute_table(self, minutes=60):
        cut = time.time() // 60 - minutes
        nm = int(self.cfg.get("motors"))
        out = []
        for b in list(self.minutes):
            if b["k"] <= cut or not b["n"]:
                continue
            row = {"t": b["t"], "samples": b["n"], "gaps": b["gaps"], "low_s": round(b["low"] / 100.0, 1),
                   "simhub_s": round(b["sim"] / 100.0, 1)}
            for m in range(nm):
                x = b["m"][m]
                row[f"m{m + 1}"] = {"act_s": round(x["act"] / 100.0, 1),
                                    "sat_pct": round(100.0 * x["sat"] / x["act"], 1) if x["act"] else None,
                                    "err_mean": round(x["esum"] / x["act"], 1) if x["act"] else None,
                                    "err_max": round(x["emax"], 1), "pot": [round(x["fmin"], 1), round(x["fmax"], 1)]
                                    if x["fmax"] >= 0 else None, "glitches": x["glitch"]}
            out.append(row)
        return out

    def sim_active(self):
        return time.time() - self.sim_last < 3.0


def now_minus(t):
    return time.time() - t


class CheckStop(Exception):
    def __init__(self, code, msg, unsafe=False, **info):
        super().__init__(msg)
        self.code, self.unsafe, self.info = code, unsafe, info


class MotionCheck:
    """Moves one motor a little, gently, and watches it: which way it goes, how much drive it needs each way,
    whether another channel moves instead (swapped wiring), and how it answers a small step with its own
    settings. Other motors are switched off meanwhile (firmware 1.5+) so they hold still and can't fight."""

    def __init__(self, diag, board, m, fw, flags, params, noise):
        self.d, self.b, self.m, self.n = diag, board, m, m + 1
        self.fw, self.params = fw, params
        self.rev = (m == 0 and bool(flags & 1))
        n = self.n
        self.orig = {"kp": params[f"Kp{n}"], "ki": params[f"Ki{n}"], "kd": params[f"Kd{n}"], "ks": params[f"Ks{n}"],
                     "pwmmin": params[f"PWM{n}"][0], "pwmmax": params[f"PWM{n}"][1], "cut": params[f"LIM{n}"][0],
                     "clip": params[f"LIM{n}"][1], "dz": params[f"DZ{n}"][0], "pwmrev": params[f"DZ{n}"][1]}
        self.clip = self.orig["clip"]
        self.noise = noise
        self.cur = None
        self.recent = collections.deque(maxlen=300)
        self.last_cmd = 0.0
        self.chk_seq = board.seq
        self.interf = 0
        self.other_interf = 0
        self.base_other = {}
        self.cross_lim = {}
        self.expect_on = False
        self.watch = None
        self.step_watch = None
        self.hist = collections.deque(maxlen=200)
        self.runaway_limit = 300
        self.persist = collections.Counter()

    def _two(self, key, cond):
        """True when cond holds for 2 samples in a row - a single glitchy pot reading can't trigger a stop."""
        if cond:
            self.persist[key] += 1
            return self.persist[key] >= 2
        self.persist[key] = 0
        return False

    def wait_states(self, want, timeout=1.5):
        """Wait until the telemetry shows each motor in `want` (index -> switched off?) as wanted."""
        end = time.time() + timeout
        while time.time() < end:
            s = self.b.latest()
            if s and all(s["dis"][k] == v for k, v in want.items()):
                time.sleep(0.1)
                return True
            time.sleep(0.05)
        return False

    # ---- primitives ----
    def say(self, msg):
        self.d.say(f"M{self.n}: {msg}")

    def apply(self, kp, ki, kd, ks, pwmmin, pwmmax, dz, pwmrev):
        b, m = self.b, self.m
        b.cmd_word("DEF"[m], int(kp))
        b.cmd_word("GHI"[m], int(ki))
        b.cmd_word("JKL"[m], int(kd))
        b.cmd_word("MNO"[m], max(1, int(ks)))
        b.cmd_pair("PQR"[m], int(pwmmin), int(pwmmax))
        b.cmd_pair("VWX"[m], int(dz), int(pwmrev))
        time.sleep(0.25)

    def apply_orig(self):
        o = self.orig
        self.apply(o["kp"], o["ki"], o["kd"], o["ks"], o["pwmmin"], o["pwmmax"], o["dz"], o["pwmrev"])

    def set_target(self, v):
        v = int(max(self.clip, min(1023 - self.clip, round(v))))
        self.b.cmd_word("ABC"[self.m], 1023 - v if self.rev else v)
        self.cur = v
        self.last_cmd = time.time()
        self.recent.append((self.last_cmd, v))

    def fb(self):
        s = self.b.latest()
        return s["m"][self.m]["f"] if s else 512.0

    def wait(self, sec):
        end = time.time() + sec
        while True:
            self.check()
            if time.time() - self.last_cmd > 0.8 and self.cur is not None:
                self.set_target(self.cur)                 # keep-alive: full power over WiFi (firmware 1.4+)
            if time.time() >= end:
                return
            time.sleep(0.05)

    def check(self):
        m, n = self.m, self.n
        if self.d.abort_flag:
            raise CheckStop("stopped", "Stopped by you.")
        if not self.b.connected:
            raise CheckStop("link", "Lost the connection to the board.")
        new = self.b.since(self.chk_seq)
        if not new:
            return
        self.chk_seq = new[-1]["seq"]
        now = time.time()
        allowed = {v for t, v in self.recent if now - t < 2.0} | ({self.cur} if self.cur is not None else set())
        lo_ok, hi_ok = self.clip - 25, 1023 - self.clip + 25
        for s in new:
            mm = s["m"][m]
            f = mm["f"]
            if self.expect_on and s["dis"][m]:
                raise CheckStop("cutoff", f"Motor {n} was switched off by its safety cutoff (pot {f:.0f}).", True,
                                pot=round(f, 1))
            if self.cur is not None:
                if mm["t"] in allowed:
                    self.interf = 0
                else:
                    self.interf += 1
                    if self.interf >= 6:
                        raise CheckStop("simhub", "Something else is sending positions (SimHub?). Stop SimHub's motion "
                                        "output and run the check again.")
            moved_other = False
            for k, (f0, t0) in self.base_other.items():
                if s["m"][k]["t"] != t0:
                    moved_other = True
                d = s["m"][k]["f"] - f0
                if self._two(("x", k), abs(d) > self.cross_lim[k]):
                    raise CheckStop("crosstalk", f"While motor {n} was being driven, motor {k + 1}'s pot moved "
                                    f"{d:+.0f} counts instead.", True, other=k + 1, delta=round(d, 1))
            self.other_interf = self.other_interf + 1 if moved_other else 0
            if self.other_interf >= 6:
                raise CheckStop("simhub", "Something else is moving the other motors (SimHub?). Stop SimHub's motion "
                                "output and run the check again.")
            if self.watch is not None:
                f0, d = self.watch
                if self._two("rev", (f - f0) * d <= -8):
                    raise CheckStop("reversed", f"Motor {n} moved the WRONG way: driven towards "
                                    f"{'higher' if d > 0 else 'lower'} numbers, its pot went from {f0:.0f} to "
                                    f"{f:.0f}.", True, start=round(f0, 1), now=round(f, 1),
                                    driven="up" if d > 0 else "down")
            if self.step_watch is not None:
                tgt, size, t_start = self.step_watch
                if self._two("unst", now - t_start > 0.3 and abs(f - tgt) > max(100.0, 2.5 * size)):
                    raise CheckStop("unstable", f"Motor {n} swung out of control on its own settings: a {size:.0f}-count "
                                    f"step made it swing to {f:.0f} (target {tgt}), so the check stopped it.",
                                    peak=round(f, 1), target=tgt, size=size)
            if self._two("run", self.cur is not None and abs(mm["t"] - f) > self.runaway_limit):
                raise CheckStop("runaway", f"Motor {n} ran away from its target ({f:.0f} vs {mm['t']}).", True,
                                pot=round(f, 1))
            if self._two("clip", self.expect_on and (f < lo_ok or f > hi_ok)):
                raise CheckStop("clip", f"Motor {n} went outside its Clip window (pot {f:.0f}).", True, pot=round(f, 1))
            self.hist.append((s["ms"], f, mm["t"] - f, mm["p"]))
        if len(self.hist) > 2:
            t_end = self.hist[-1][0]
            win = [h for h in self.hist if t_end - h[0] <= 300]
            if win and t_end - win[0][0] >= 250:
                es = [h[2] for h in win]
                if all(abs(e) > 30 for e in es) and (all(e > 0 for e in es) or all(e < 0 for e in es)):
                    sg = 1 if es[0] > 0 else -1
                    if all(h[3] * sg > 15 for h in win) and (win[-1][1] - win[0][1]) * sg < -10:
                        raise CheckStop("reversed", f"Motor {n} moves AWAY from where it is driven.", True)

    def ramp_to(self, x, speed=250.0):
        start = self.cur if self.cur is not None else self.fb()
        steps = max(1, int(abs(x - start) / (speed * 0.02)))
        for i in range(1, steps + 1):
            self.set_target(start + (x - start) * i / steps)
            self.wait(0.02)
        self.set_target(x)

    def others_off(self, s):
        self.prev_dis = list(s["dis"])
        if self.fw >= 15:
            for k in range(3):
                if k != self.m and not s["dis"][k]:
                    self.b.cmd_text(f"di{k + 1}")
            self.wait_states({k: True for k in range(3) if k != self.m})
        s2 = self.b.latest() or s
        for k in range(3):
            if k != self.m:
                self.base_other[k] = (s2["m"][k]["f"], s2["m"][k]["t"])
                nz = self.noise[k] if self.noise[k] is not None else 0.5
                self.cross_lim[k] = max(12.0, 6 * nz)

    def others_back(self):
        self.base_other = {}
        if self.fw >= 15:
            back = [k for k in range(3) if k != self.m and not self.prev_dis[k]]
            for k in back:
                self.b.cmd_text(f"en{k + 1}")
            self.wait_states({k: False for k in back})

    def safe_stop(self):
        n = self.n
        if self.fw >= 15:
            self.b.cmd_text(f"di{n}")
            self.b.last_dis_cmd[self.m] = time.time()
            self.say(f"SAFETY: motor {n} switched off. Fix the cause, then press Enable Motors.")
        else:
            self.b.cmd_pair("PQR"[self.m], self.orig["pwmmin"], 0)
            self.b.cmd_pair("VWX"[self.m], self.orig["dz"], 0)
            self.say(f"SAFETY: motor {n}'s drive set to zero (PWMmax 0, not saved). Power-cycle the board after fixing "
                     f"the cause.")

    # ---- the procedure ----
    def run(self, ask):
        b, m, n, o = self.b, self.m, self.n, self.orig
        r = {"motor": n, "result": None, "message": "", "settings": dict(o)}
        time.sleep(0.3)                                  # let the telemetry catch up with the previous motor's check
        s = b.latest()
        if not s:
            r.update(result="no_data", message="No telemetry from the board.")
            return r
        f0, t0, was_off = s["m"][m]["f"], s["m"][m]["t"], s["dis"][m]
        r["start"] = {"pot": round(f0, 1), "target": t0, "was_off": was_off}
        seq_start = b.seq
        try:
            if f0 < 5 or f0 > 1018:
                r.update(result="pot_rail", message=f"Pot reads {f0:.0f} - at the very end of its range. Not driven.")
                return r
            if f0 < o["cut"] or f0 > 1023 - o["cut"]:
                r.update(result="beyond_cutoff", message=f"Pot reads {f0:.0f}, past the safety cutoff "
                         f"({o['cut']} / {1023 - o['cut']}), so the board won't let it run. Not driven.")
                return r
            self.others_off(s)
            self.d.motor = m
            hold = max(self.clip + 2, min(1023 - self.clip - 2, round(f0)))
            # 1. direction and the drive needed to move, each way - gentle settings, no recovery kick
            self.say("checking which way it moves and how much drive it needs")
            self.runaway_limit = 120
            lv0 = max(20, min(150, o["pwmmin"]))
            levels = sorted({lv0, 40, 60, 80, 100, 130, 160})
            self.apply(20, 0, 0, 1, levels[0], min(255, levels[0] + 30), max(1, o["dz"]), 0)
            self.set_target(hold)
            if was_off:
                b.cmd_text(f"en{n}")
                if not self.wait_states({m: False}, 1.2):
                    sf = self.fb()
                    r.update(result="wont_enable", message=f"Switched on, but the board switched it off again at once "
                             f"(pot {sf:.0f}, cutoff {o['cut']} / {1023 - o['cut']}).")
                    return self._finish(r, ok=False)
            time.sleep(0.2)
            self.chk_seq = b.seq                         # judge only what happens from now on
            self.expect_on = True
            self.wait(0.4)
            found = {}
            for d in (1, -1):
                for lvl in levels:
                    self.b.cmd_pair("PQR"[m], lvl, min(255, lvl + 30))
                    time.sleep(0.1)
                    fa = self.fb()
                    if not (self.clip + 30 <= fa + d * 25 <= 1023 - self.clip - 30):
                        tgt = fa - d * 40                     # too close to the end: back off first
                        self.watch = None
                        self.set_target(tgt)
                        self.wait(0.8)
                        fa = self.fb()
                    self.watch = (fa, d)
                    self.set_target(round(fa) + d * 25)
                    self.wait(0.7)
                    moved = (self.fb() - fa) * d
                    self.watch = None
                    self.set_target(self.fb())
                    self.wait(0.2)
                    if moved >= 6:
                        found[d] = lvl
                        self.say(f"moves {'up' if d > 0 else 'down'} (towards {'higher' if d > 0 else 'lower'} "
                                 f"numbers) at drive {lvl}")
                        break
            r["drive_to_move_up"], r["drive_to_move_down"] = found.get(1), found.get(-1)
            if not found:
                r.update(result="no_move", message=f"Didn't move either way even at drive {levels[-1]} "
                         f"({100 * levels[-1] // 255}% of the supply).")
                if ask:
                    ans = self.ask_show_drive(levels[-1])
                    r["seen"] = ans
                return self._finish(r, ok=False)
            if len(found) == 1:
                d_ok = next(iter(found))
                r.update(result="one_way", message=f"Moves {'up' if d_ok > 0 else 'down'} but not "
                         f"{'down' if d_ok > 0 else 'up'}, even at drive {levels[-1]}.")
                return self._finish(r, ok=False)
            # 2. a small step with its own settings
            self.say("small step with its own settings")
            self.apply_orig()
            self.runaway_limit = 200
            room_up = 1023 - self.clip - 20 - self.fb()
            a = 50 if room_up >= 50 else -50
            base = max(self.clip + 60, min(1023 - self.clip - 60, round(self.fb())))
            self.ramp_to(base)
            self.wait(1.0)
            steps = []
            for target in (base + a, base):
                t_old = self.cur
                seq0 = b.seq
                self.set_target(target)
                self.step_watch = (self.cur, abs(a), time.time())
                self.wait(1.6)
                self.step_watch = None
                self.wait(0.15)
                got = despike(b.since(seq0), m)
                res = analyze_step(got, m, t_old, o["pwmmin"], o["pwmmax"], 1500)
                vmax = 0.0
                for i in range(1, len(got)):
                    dt = got[i]["ms"] - got[i - 1]["ms"]
                    if 0 < dt <= 15:
                        vmax = max(vmax, abs(got[i]["m"][m]["f"] - got[i - 1]["m"][m]["f"]) * 1000.0 / dt)
                if res:
                    res = {k: (round(v, 1) if isinstance(v, float) else v) for k, v in res.items()}
                    res["speed_max"] = round(vmax)
                steps.append(res)
            r["step"] = {"size": abs(a), "first_direction": "up" if a > 0 else "down", "away": steps[0],
                         "back": steps[1]}
            # 3. what the person sees
            if ask:
                r["seen"] = self.ask_show_move(base)
            r.update(result="ok", message="Moves the right way in both directions.")
            return self._finish(r, ok=True)
        except CheckStop as e:
            r.update(result=e.code, message=str(e), info=e.info)
            self.say(f"stopped: {e}")
            self.step_watch = None
            if e.unsafe:
                self.safe_stop()
                r["left_off"] = True
            elif e.code == "unstable":                   # calm it with gentle settings before its own come back
                try:
                    self.apply(20, 0, 0, 1, o["pwmmin"], o["pwmmax"], max(1, o["dz"]), 0)
                    self.set_target(self.fb())
                    self.wait(1.2)
                except CheckStop:
                    pass
            return self._finish(r, ok=False, unsafe=e.unsafe)
        finally:
            self.d.motor = None
            got = [x for x in b.since(seq_start - 50)]
            r["trace"] = trace_csv(got, n, tuple(range(1, int(self.d.cfg.get("motors")) + 1)), every=2)

    def _finish(self, r, ok, unsafe=False):
        """Put everything back: settings, the other motors, and this one's on/off state and position."""
        m, n = self.m, self.n
        self.watch = None
        try:
            if not unsafe:
                self.apply_orig()
                start = r.get("start", {})
                if ok and start.get("target") is not None:
                    self.expect_on = True
                    try:
                        self.ramp_to(start["target"], speed=300)
                        self.wait(0.6)
                    except CheckStop:
                        pass
                else:
                    self.set_target(self.fb())
                if start.get("was_off") and self.fw >= 15:
                    self.b.cmd_text(f"di{n}")
                    self.b.last_dis_cmd[m] = time.time()
            elif self.fw >= 15:
                self.apply_orig()                       # the motor is off; leave its own settings in place
            self.expect_on = False
            if self.base_other:
                self.others_back()
        except Exception as e:                          # never leave others switched off because of a hiccup
            self.d.say(f"M{n}: problem while restoring ({e!r}) - press Enable Motors if a motor stays off.")
            try:
                self.others_back()
            except Exception:
                pass
        return r

    def ask_show_move(self, base):
        n = self.n
        ans = None
        for _ in range(3):
            self.d.set_prompt(self.m, f"Watch motor {n}'s corner of the seat: in a moment it moves towards HIGHER "
                              f"numbers (+60) and comes back.", [], info=True)
            self.wait(2.0)
            self.ramp_to(base + 60, speed=200)
            self.wait(1.0)
            self.ramp_to(base, speed=200)
            self.wait(0.3)
            ans = self.d.ask(self, f"Which way did motor {n}'s corner of the seat go when its number went UP?",
                             [["up", "It went UP"], ["down", "It went DOWN"], ["none", "I didn't see it move"],
                              ["unsure", "Not sure"], ["again", "Show me again"]], where=True)
            if ans["answer"] != "again":
                break
        return ans

    def ask_show_drive(self, lvl):
        n, m = self.n, self.m
        self.d.set_prompt(m, f"Watch and listen to motor {n}: in a moment it is driven at {100 * lvl // 255}% "
                          f"for a second each way.", [], info=True)
        self.wait(2.0)
        self.b.cmd_pair("PQR"[m], lvl, min(255, lvl + 30))
        f = self.fb()
        for d in (1, -1):
            self.watch = (f, d)
            self.set_target(round(f) + d * 60)
            self.wait(1.0)
            self.watch = None
            self.set_target(self.fb())
            self.wait(0.3)
            f = self.fb()
        return self.d.ask(self, f"What did motor {n} do just now?",
                          [["moved", "It moved"], ["hum", "Hummed or clicked, didn't move"],
                           ["nothing", "Nothing at all"], ["unsure", "Not sure"]], where=False)


ABOUT = {
    "what": "Diagnostics file from smc3_scope, the page server for a DIY 3-motor SMC3 motion seat: Arduino UNO R4 "
            "WiFi running SMC3_R4 firmware, IBT-2 / BTS7960 motor bridges, DC motors with feedback pots, driven by "
            "SimHub over USB. smc3_scope runs on a NAS and talks to the board over WiFi. Made to be read by a person "
            "or an AI (such as Claude) helping to find what is wrong.",
    "read_first": "'problem' is what the owner typed. 'summary' and 'findings' are the page's own conclusions; "
                  "everything after them is the measured data behind them. 'check' exists only for a full check "
                  "(each motor moved a little).",
    "units": {
        "positions": "0-1023 SMC3 counts (the pot reading scaled), 512 = middle of travel.",
        "t, g, f, p in traces": "t = target as commanded (SimHub or this page), g = goal (target after the firmware's "
                                "smoothing filter - what the PID chases), f = feedback (pot), p = PID output in PWM "
                                "units -255..255 before PWMmin is added; positive p drives towards higher counts.",
        "Kp, Ki, Kd": "SMC3Utils raw numbers (100 x the original gains). Ks = derivative sample spacing in PID cycles.",
        "PWMmin, PWMmax, PWMrev": "0-255 drive, 255 = 100% of the motor supply.",
        "Max limit (cutoff)": "counts from each end of the pot range where the safety cutoff switches the motor off.",
        "Clip": "counts from each end that bound the usable target window; PWMrev pushes back if the pot is outside.",
        "Dead zone": "counts; inside it the motor brakes instead of driving.",
        "ms": "board uptime in milliseconds",
    },
    "traces": "CSV text: header in 'columns', one row per sample at 'rate_hz'. off = three 0/1 flags (motors 1-3 "
              "switched off), low = 1 when the board runs at quarter power (no position commands for 15 s).",
    "wiring (firmware defaults)": "motor n: PWM on D9/D10/D11, bridge enable pins D2+D3 / D4+D5 / D6+D7, pots on "
                                  "A0/A1/A2. MODE2 = IBT-2 bridges.",
}


class Diagnostics:
    """Runs the quick or full check, keeps the reports on the NAS (diagnostics/reports) and lets the page answer
    the 'which way did it move?' questions."""
    KEEP = 20

    def __init__(self, board, gen, tuner, cfg, events, monitor, backups, profiles, data_dir):
        self.board, self.gen, self.tuner, self.cfg = board, gen, tuner, cfg
        self.events, self.monitor, self.backups, self.profiles = events, monitor, backups, profiles
        self.dir = os.path.join(data_dir, "diagnostics", "reports")
        self.data_dir = data_dir
        self.state = "idle"
        self.kind = None
        self.stage, self.progress = "", 0
        self.lines = []
        self.prompt = None
        self._answer = None
        self._ans_ev = threading.Event()
        self.abort_flag = False
        self.motor = None
        self.result = None
        self.started = time.time()

    def running(self):
        return self.state in ("running", "waiting")

    def moving(self):
        return self.running() and self.motor is not None

    def say(self, msg):
        self.lines.append(time.strftime("%H:%M:%S ") + msg)
        log(f"[diag] {msg}")

    def status(self):
        return {"state": self.state, "kind": self.kind, "stage": self.stage, "progress": self.progress,
                "log": self.lines[-200:], "prompt": self.prompt,
                "motor": None if self.motor is None else self.motor + 1, "result": self.result}

    def start(self, kind, problem, ask):
        if self.running():
            return False, "A check is already running."
        if self.tuner.running():
            return False, "Auto-tune is running - wait for it or stop it first."
        if kind == "full":
            if not self.board.connected:
                return False, "The board is not connected, so the full check can't move anything. Run the quick " \
                              "check - it still collects the history the page has recorded."
            if self.monitor.sim_active():
                return False, "Something (SimHub?) is moving the rig right now. Stop SimHub's motion output first."
        self.kind = kind
        self.state, self.stage, self.progress = "running", "Starting", 0
        self.lines, self.result, self.prompt, self.abort_flag, self.motor = [], None, None, False, None
        threading.Thread(target=self._thread, args=(kind, problem[:2000], ask), daemon=True).start()
        return True, "started"

    def abort(self):
        self.abort_flag = True
        self._answer = {"answer": "stop", "where": ""}
        self._ans_ev.set()

    def set_prompt(self, m, text, options, info=False, where=False):
        self.prompt = {"id": f"{time.time():.3f}", "motor": m + 1, "text": text, "options": options, "info": info,
                       "where": where}

    def ask(self, check, text, options, where=False, timeout=180):
        self.set_prompt(check.m, text, options, where=where)
        self._ans_ev.clear()
        self._answer = None
        self.state = "waiting"
        end = time.time() + timeout
        try:
            while not self._ans_ev.is_set() and time.time() < end:
                check.wait(0.2)                      # holds the motor and keeps watching it
        finally:
            self.state = "running" if not self.abort_flag else self.state
            self.prompt = None
        a = self._answer or {"answer": "no answer", "where": ""}
        if a["answer"] == "stop":
            raise CheckStop("stopped", "Stopped by you.")
        self.say(f"M{check.n}: answer '{a['answer']}'" + (f", {a['where']}" if a.get("where") else ""))
        return a

    def answer(self, pid, value, where):
        if self.prompt and self.prompt.get("id") == pid:
            self._answer = {"answer": str(value)[:20], "where": str(where or "")[:30]}
            self._ans_ev.set()
            return True
        return False

    # ---- running ----
    def _thread(self, kind, problem, ask):
        self.events.add("diag", f"{'Full' if kind == 'full' else 'Quick'} check started.")
        try:
            rep = self._collect(kind, problem, ask)
            self.stage, self.progress = "Working out what it means", 92
            findings = analyze(rep)
            names = list(self.cfg.get("motor_names") or ["", "", ""])
            for f in findings:
                for k in ("title", "detail", "fix"):
                    f[k] = name_text(f[k], names)
                if f["motor"]:
                    f["motor_name"] = names[f["motor"] - 1] or None
            for e in rep.get("events", []):
                e["msg"] = name_text(e.get("msg", ""), names)
            for x in rep.get("incidents", {}).get("list", []):
                x["msg"] = name_text(x.get("msg", ""), names)
            rep["findings"] = findings
            rep["summary"] = summary_text(rep, findings)
            name = self._save(rep)
            nfail = sum(1 for f in findings if f["level"] == "fail")
            nwarn = sum(1 for f in findings if f["level"] == "warn")
            self.result = {"name": name, "findings": findings, "summary": rep["summary"], "kind": kind,
                           "stopped": self.abort_flag}
            self.state, self.progress, self.stage = "done", 100, ""
            self.events.add("diag", f"{'Full' if kind == 'full' else 'Quick'} check finished: {nfail} problems, "
                            f"{nwarn} warnings. Saved as {name}.")
        except CheckStop as e:
            self.say(f"Stopped: {e}")
            self.state, self.stage = "aborted", ""
        except Exception as e:
            import traceback
            self.say(f"Internal error: {e!r}")
            log(traceback.format_exc())
            self.state, self.stage = "error", ""
        finally:
            self.motor = None
            self.prompt = None

    def _sleep(self, sec):
        end = time.time() + sec
        while time.time() < end and not self.abort_flag:     # Stop cuts it short; the report is still made
            time.sleep(0.05)

    # ---- reports on the NAS ----
    def _save(self, rep):
        first = ("about", "made", "kind", "problem", "summary", "findings")
        out = {"about": ABOUT}
        out.update({k: rep[k] for k in first[1:] if k in rep})
        out.update({k: v for k, v in rep.items() if k not in out and k != "made_ts"})
        name = f"smc3-diagnostics_{time.strftime('%Y-%m-%d_%H%M%S')}_{rep.get('kind', 'quick')}.json"
        try:
            os.makedirs(self.dir, exist_ok=True)
            with open(os.path.join(self.dir, name), "w") as f:
                json.dump(out, f, indent=1)
            for old in self.list_reports()[self.KEEP:]:
                os.remove(os.path.join(self.dir, old["name"]))
        except OSError as e:
            log(f"could not save the diagnostics report: {e}")
            self._unsaved = (name, out)                   # still downloadable from memory
        return name

    def list_reports(self):
        try:
            names = sorted((n for n in os.listdir(self.dir) if n.startswith("smc3-diagnostics_") and
                            n.endswith(".json")), reverse=True)
        except OSError:
            names = []
        out = []
        for n in names:
            try:
                size = os.path.getsize(os.path.join(self.dir, n))
            except OSError:
                continue
            parts = n[:-5].split("_")
            when = f"{parts[1]} {parts[2][:2]}:{parts[2][2:4]}:{parts[2][4:6]}" if len(parts) >= 3 else n
            out.append({"name": n, "when": when, "kind": parts[3] if len(parts) > 3 else "", "kb": round(size / 1024)})
        return out

    def report_bytes(self, name):
        if "/" in name or ".." in name or not name.endswith(".json"):
            return None
        un = getattr(self, "_unsaved", None)
        if un and un[0] == name:
            return json.dumps(un[1], indent=1).encode()
        try:
            with open(os.path.join(self.dir, name), "rb") as f:
                return f.read()
        except OSError:
            return None

    def delete_report(self, name):
        if "/" in name or ".." in name or not name.endswith(".json"):
            return False
        try:
            os.remove(os.path.join(self.dir, name))
            return True
        except OSError:
            return False

    def _collect(self, kind, problem, ask):
        b, cfg, mon = self.board, self.cfg, self.monitor
        nm = int(cfg.get("motors"))
        motors = list(range(1, nm + 1))
        rep = {"made": time.strftime("%Y-%m-%d %H:%M:%S"), "made_ts": round(time.time()), "kind": kind,
               "problem": problem}
        self.stage, self.progress = "Reading the board", 5
        self.say("Reading the board's settings and status.")
        connected = b.connected
        params = b.read_params() if connected else {}
        fw = b.fw_cache or [0, 0]                      # read_params() asks for it too
        latency = b.ping() if connected and fw[1] >= 14 else None
        dg = b.read_diag() if connected and fw[1] >= 17 else None
        s = b.latest() if connected else None
        board = {"connected": connected, "address": f"{cfg.get('host')}:{cfg.get('port')}",
                 "firmware": f"{fw[1] // 10}.{fw[1] % 10}" if fw[1] else ("1.3 or older" if params.get("Kp1") is not None else None),
                 "fw_num": fw[1], "reverse_motor1": bool(fw[0] & 1)}
        if not connected:
            board["disconnected_for"] = _ago(time.time() - b.disconnected_since)
        if s:
            board.update({"uptime": _ago(s["ms"] / 1000), "uptime_s": round(s["ms"] / 1000),
                          "isr_max_us": s["isr"], "pid_divider": s["div"],
                          "power": "low (no position commands for 15 s)" if s["low"] else "full",
                          "motors_off": [k + 1 for k in range(3) if s["dis"][k]],
                          "imu_roll_pitch": [s["roll"], s["pitch"]]})
        if latency is not None:
            board["command_round_trip_ms"] = latency
        if dg:
            cause, cold = reset_cause(dg)
            board.update({"last_start_cause": cause, "last_start_cold": cold, "reset_flags": dg["reset_flags"],
                          "wifi_rssi_dbm": dg["rssi"], "loop_max_ms": round(dg["loop_max_us"] / 1000.0, 1),
                          "build": {"bridge": "IBT-2 / BTS7960 (MODE2)" if dg["build"] & 2 else
                                    "PWM + direction pins (MODE1)", "reverse_motor1": bool(dg["build"] & 1),
                                    "imu": bool(dg["build"] & 4), "led_matrix": bool(dg["build"] & 8),
                                    "static_ip": bool(dg["build"] & 16)},
                          "usb": {"commands_ok": dg["usb_ok"], "position_commands": dg["usb_targets"],
                                  "bytes_thrown_away": dg["usb_bad_bytes"],
                                  "ms_since_last_command": dg["ms_since_usb"],
                                  "ms_since_last_position": dg["ms_since_usb_target"],
                                  "position_commands_per_s": None if mon.usb_rate is None else round(mon.usb_rate),
                                  "smc3utils_monitor_mode": dg["monitor_mode"]},
                          "wifi_commands_ok": dg["wifi_cmds"],
                          "motors": [dict(x, motor=k + 1) for k, x in enumerate(dg["motors"])]})
        rep["board"] = board
        names = list(cfg.get("motor_names") or ["", "", ""])
        rep["rig"] = {"motors_in_use": nm, "switch_off_unused_motors": bool(cfg.get("disable_unused")),
                      "motor_names": {f"motor {n}": (names[n - 1] or None) for n in motors}}
        st = {"running": {f"motor {n}": motor_settings(params, n) for n in motors} if params.get("Kp1") is not None
              else None, "running_global": global_settings(params) if params.get("Kp1") is not None else None,
              "raw": canon(params) if params else None}
        stored = decode_eeprom(dg["eeprom"]) if dg else None
        if stored:
            st["saved_in_eeprom"] = {f"motor {n}": motor_settings(stored, n) for n in motors}
            st["saved_global"] = global_settings(stored)
            st["not_saved"] = settings_diff(params, stored, motors) if params.get("Kp1") is not None else None
        elif dg:
            st["saved_in_eeprom"] = None
        last_save = next((x for x in self.backups.list() if x["kind"] == "eeprom"), None)
        if last_save:
            st["last_save_from_page"] = {"when": last_save["when"],
                                         "differs_from_running": settings_diff(params, last_save["params"], motors)
                                         if params.get("Kp1") is not None else None}
        rep["settings"] = st
        # listen for a few seconds
        self.stage, self.progress = "Listening to the motors", 12
        self.say("Listening to the motors for 3 seconds.")
        seq0 = b.seq
        t_listen = time.time()
        self._sleep(3.0)
        got = b.since(seq0)
        live = {"seconds": round(time.time() - t_listen, 1), "samples": len(got),
                "samples_per_s": round(len(got) / max(0.1, time.time() - t_listen)), "motors": []}
        for n in motors:
            m = n - 1
            if not got:
                break
            ts = [x["m"][m]["t"] for x in got]
            fs = [x["m"][m]["f"] for x in got]
            ps = [x["m"][m]["p"] for x in got]
            es = [x["m"][m]["g"] - x["m"][m]["f"] for x in got]
            still = max(ts) - min(ts) <= 1
            row = {"motor": n, "off": got[-1]["dis"][m], "pot": round(_mean(fs), 1), "target": ts[-1],
                   "target_still": still, "pot_range": [round(min(fs), 1), round(max(fs), 1)],
                   "error_mean": round(_mean(es), 1), "pid_mean": round(_mean(ps), 1)}
            if still and not got[-1]["dis"][m]:
                row["noise_at_rest"] = None if rest_noise(fs) is None else round(rest_noise(fs), 2)
                row["pid_noise_at_rest"] = round(robust_sigma(ps) or 0.0, 2)
            live["motors"].append(row)
        rep["live"] = live
        # full check: move each motor a little
        if kind == "full":
            rep["check"] = self._motion(params, fw, motors, ask)
        # history the page recorded
        self.stage, self.progress = "Collecting the history", 88
        rep["watch"] = {"page_running_since": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(mon.started)),
                        "last_60_min": mon.stats(60), "last_24_h_problem_counts": self._counts(),
                        "minutes": mon.minute_table(60)}
        incs = mon.incident_list(30)
        full = []
        for x in incs[:4]:
            inc = mon.incident_full(x["file"])
            if inc:
                full.append(inc)
        rep["incidents"] = {"recorded": len(incs), "list": incs, "recent_with_traces": full}
        rep["events"] = self.events.recent(250)
        snap = b.since(max(0, b.seq - 1000))
        rep["telemetry"] = {"last_10s": trace_csv(snap, None, tuple(motors)),
                            "last_2min_5hz": trace_csv(b.since(max(0, b.seq - 12000)), None, tuple(motors), every=20)}
        import platform
        rep["scope"] = {"version": VERSION, "python": platform.python_version(), "platform": platform.platform(),
                        "running_since": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(mon.started)),
                        "settings": {k: v for k, v in cfg.public().items() if k != "path"},
                        "data_folder_writable": cfg.writable, "test_signal": self.gen.mode,
                        "auto_tune": self.tuner.state}
        rep["profiles"] = [p["name"] for p in self.profiles.list()]
        rep["backups"] = [{k: x[k] for k in ("when", "kind", "label", "summary")} for x in self.backups.list()[:8]]
        tune_last = self.tuner.status()
        if tune_last.get("result") or tune_last.get("state") not in ("idle", None):
            rep["last_auto_tune"] = {"state": tune_last["state"], "motor": tune_last["motor"],
                                     "result": tune_last["result"], "log_tail": tune_last["log"][-25:]}
        return rep

    def _counts(self):
        since = time.time() - 86400
        c = collections.Counter()
        for e in self.events.recent(3000, since=since):
            if e.get("kind") == "problem":
                c[(e.get("data") or {}).get("incident", "?") + (f" m{e['motor']}" if e.get("motor") else "")] += 1
            elif e.get("kind") == "board" and "restarted" in e.get("msg", ""):
                c["board restarts"] += 1
            elif e.get("kind") == "link" and e.get("msg", "").startswith("Connected"):
                c["connections"] += 1
        return dict(c)

    def _motion(self, params, fw, motors, ask):
        b = self.board
        out = []
        if fw[1] < 14:
            self.say("Firmware older than 1.4 - the board would run at quarter power for page commands, so motors "
                     "are not moved.")
            return {"skipped": "firmware older than 1.4"}
        if params.get("Kp1") is None or any(params.get(f"LIM{n}") is None for n in motors):
            self.say("Couldn't read all settings - motors are not moved.")
            return {"skipped": "settings could not be read"}
        if self.monitor.sim_active():
            return {"skipped": "SimHub was moving the rig"}
        self.gen.set(mode="off")
        self.backups.add("auto", "Before the diagnostics full check", params)
        noise = list(self.monitor.noise)
        for i, n in enumerate(motors):
            if self.abort_flag:
                break
            self.stage = f"Checking motor {n}"
            self.progress = 15 + int(70 * i / len(motors))
            self.say(f"Motor {n}: start.")
            chk = MotionCheck(self, b, n - 1, fw[1], fw[0], params, noise)
            r = chk.run(ask)
            self.say(f"Motor {n}: {r['result']} - {r['message']}")
            out.append(r)
            if r["result"] in ("stopped", "link", "simhub"):
                break
            time.sleep(0.5)
        return out


# --------------------------------------------------------------------------------------------------------------
# Working out what the data means
# --------------------------------------------------------------------------------------------------------------

def analyze(rep):
    F = []

    def add(level, title, detail="", fix="", motor=None, code=""):
        F.append({"level": level, "motor": motor, "title": title, "detail": detail, "fix": fix, "code": code})

    board, st, live = rep.get("board", {}), rep.get("settings", {}), rep.get("live", {})
    nm = rep.get("rig", {}).get("motors_in_use", 3)
    motors = list(range(1, nm + 1))
    raw = st.get("raw") or {}
    watch = rep.get("watch", {})
    w60 = {x["motor"]: x for x in watch.get("last_60_min", {}).get("motors", [])}
    counts = watch.get("last_24_h_problem_counts", {})
    fwn = board.get("fw_num") or 0
    dgm = {x["motor"]: x for x in board.get("motors", [])} if board.get("motors") else {}
    livem = {x["motor"]: x for x in live.get("motors", [])}
    checks = {x["motor"]: x for x in rep.get("check", [])} if isinstance(rep.get("check"), list) else {}
    pins = {1: ("D9", "D2/D3", "A0"), 2: ("D10", "D4/D5", "A1"), 3: ("D11", "D6/D7", "A2")}

    # ---- connection and board ----
    if not board.get("connected"):
        add("fail", "The page can't reach the board",
            f"No connection to {board.get('address')} for {board.get('disconnected_for', 'a while')}. Everything below "
            f"is history the page recorded earlier.",
            "Check the board is powered and on WiFi: its address scrolls on the LED matrix at power-up (E9 = can't "
            "join the network, E0 = WiFi module not answering, E8 = no address from the router). Then check the "
            "address under Settings, or press Find board on network.", code="no_link")
    else:
        rtt = board.get("command_round_trip_ms")
        rate = live.get("samples_per_s")
        rssi = board.get("wifi_rssi_dbm")
        weak = (rssi is not None and rssi <= -75) or (rate is not None and rate < 70) or (rtt is not None and rtt > 400)
        if weak:
            add("warn", "Weak or slow WiFi link to the board",
                (f"Signal {rssi} dBm. " if rssi is not None else "") +
                f"Telemetry {rate}/s (expected 100), command round trip {rtt} ms.",
                "The UNO R4 only does 2.4 GHz. Move the router / access point closer or give the board a clear line "
                "of sight; avoid metal frames around the antenna (the small module near the USB socket). This affects "
                "the page only - SimHub uses USB.", code="wifi")
        elif rssi is not None:
            add("ok", f"WiFi signal {rssi} dBm, telemetry {rate}/s, command round trip {rtt} ms", code="wifi_ok")
    restarts = counts.get("board restarts", 0)
    if restarts:
        add("warn", f"The board restarted {restarts} time{'s' if restarts > 1 else ''} in the last 24 hours",
            "Restarts while driving usually mean the supply dips when the motors pull hard (the board and the motor "
            "bridges share a supply or a ground that sags)." + (f" The board says its last start was: "
                                                                 f"{board['last_start_cause']}." if board.get("last_start_cause") else ""),
            "Power the Arduino from USB or its own 5-12 V supply, not from the motor supply; use thick, short ground "
            "wires from each IBT-2 to the supply; check the motor supply holds its voltage under load. If the cause "
            "says 'reset button / restarted by the WiFi/USB chip', check the USB cable and that nothing re-opens the "
            "port.", code="restarts")
    elif board.get("last_start_cause"):
        add("info", f"Board running for {board.get('uptime')}; it last started because of: {board['last_start_cause']}",
            code="started")
    if board.get("connected") and fwn < 14:
        add("warn", f"Old firmware on the board ({board.get('firmware')})",
            "Commands from this page don't count as 'alive', so the board stays at quarter power for tests, auto-tune "
            "and the full check - and the page can't read most of what it needs to diagnose.",
            "Flash firmware 1.7 or newer (SMC3_R4.ino - put your WiFi name and password in first).", code="fw_old")
    elif fwn and fwn < 17:
        add("info", f"Firmware {board.get('firmware')} - flash 1.7 for complete diagnostics",
            "1.7 adds: why the board last restarted, SimHub's USB command counters, pot glitches seen at the full "
            "4 kHz loop rate, why each motor is off, and a copy of the EEPROM to spot unsaved settings.",
            "Flash firmware 1.7 or newer (SMC3_R4.ino - put your WiFi name and password in first).", code="fw")
    if board.get("isr_max_us") and board["isr_max_us"] > 200:
        add("warn", f"Control loop close to its time budget ({board['isr_max_us']} µs of 250)",
            "If it goes over, the motors get jerky.", "Set Fpid divider 2 (Tuning section) or turn the LED matrix off "
            "in the firmware.", code="isr")
    if board.get("loop_max_ms") and board["loop_max_ms"] > 100:
        add("info", f"The board's main loop was blocked for {board['loop_max_ms']:.0f} ms recently",
            "While blocked (usually waiting for the WiFi module) SimHub's commands queue up; a long pause can show as a "
            "hitch in the motion.", "Usually harmless. If you feel hitches while this page is open, close the page and "
            "check whether they go away; a weak WiFi signal makes it worse.", code="loop")
    b = board.get("build") or {}
    if b and "MODE1" in b.get("bridge", ""):
        add("warn", "Firmware is built for MODE1 bridges (PWM + two direction pins)",
            "Your rig uses IBT-2 / BTS7960 bridges, which need MODE2 - with the wrong mode the motors behave strangely "
            "(full speed one way, nothing the other).", "In SMC3_R4 set #define MODE2 and flash again.", code="mode")
    if board.get("reverse_motor1"):
        add("info", "Motor 1 is reversed in the firmware (REVERSE_MOTOR1)",
            "Its commands and readings are mirrored (1023 - value). Fine if intended.", code="rev1")
    # ---- SimHub / USB ----
    usb = board.get("usb") or {}
    if board.get("connected") and board.get("power", "").startswith("low"):
        since = usb.get("ms_since_last_position")
        if usb and (since is None or since >= 15000):
            add("info", "SimHub is not sending positions to the board right now",
                "No position command has come over USB for " + ("a long time" if since == 65535 else
                                                                f"{(since or 0) / 1000:.0f} s") +
                ", so the board runs at quarter power. Normal when SimHub's motion output is stopped.",
                "If SimHub IS running a game: in SimHub, Motion -> Output, the SMC3 device must be on this board's COM "
                "port and started. Only one program can use the COM port - close SMC3Utils and the Arduino serial "
                "monitor.", code="no_simhub")
        else:
            add("info", "Board is in low-power mode (no position commands for 15 s)",
                "Normal while SimHub's motion output is stopped.", code="low")
    if usb.get("bytes_thrown_away", 0) > 50:
        add("warn", f"{usb['bytes_thrown_away']} unreadable bytes arrived on the USB port since power-up",
            "Something sent data the SMC3 protocol doesn't understand.",
            "Make sure only SimHub uses the COM port (no SMC3Utils, Arduino serial monitor or other tools), and that "
            "SimHub's output for this board is the SMC3 type.", code="usb_bad")
    if usb.get("position_commands_per_s"):
        add("ok", f"SimHub sends about {usb['position_commands_per_s']} position commands per second", code="usb_rate")
    # ---- settings ----
    for n in motors:
        ms = (st.get("running") or {}).get(f"motor {n}")
        if not ms:
            continue
        pmin, pmax, kp = ms["PWMmin"], ms["PWMmax"], ms["Kp"]
        cut, clip, dz = ms["Max limit (cutoff)"], ms["Clip"], ms["Dead zone"]
        if pmax is not None and pmax < 30:
            add("fail", f"Motor {n} can't move: PWMmax is {pmax}",
                "Auto-tune sets PWMmax 0 when it stops for safety (motor moving the wrong way or running away); it's "
                "not saved, so a power cycle brings back the stored value.",
                f"Fix what auto-tune complained about, then set PWMmax back (e.g. 100) in the PID table and Apply M{n}.",
                n, "pwmmax0")
        elif pmin is not None and pmax is not None and pmin >= pmax:
            add("fail", f"Motor {n}: PWMmin ({pmin}) is not below PWMmax ({pmax})",
                "The drive is pinned at PWMmax for every correction - jerky, with no fine control.",
                "Typical: PWMmin 10-60, PWMmax around 100.", n, "pwmorder")
        if kp == 0:
            add("fail", f"Motor {n}: Kp is 0", "With no Kp there is no position control.", "Run auto-tune or set Kp "
                "150-300.", n, "kp0")
        if cut == 0:
            add("warn", f"Motor {n}: safety cutoff is off (Max limit 0)",
                "A broken pot wire or a reversed motor would drive it into its end stops.", "Set Max limit to about "
                "20-40.", n, "cut0")
        if cut is not None and clip is not None and clip <= cut:
            add("warn", f"Motor {n}: Clip ({clip}) is not inside the safety cutoff ({cut})",
                "SimHub can then command positions that trip the cutoff.", "Clip must be larger than Max limit, e.g. "
                "Max limit 23, Clip 100.", n, "clipcut")
        if dz is not None and dz > 8:
            add("info", f"Motor {n}: large dead zone ({dz})", "The seat stops up to that many counts short and moves in "
                "visible steps.", "2-4 is typical; auto-tune sets it from the pot noise.", n, "dz")
        if ms["Ki"] and ms["Ki"] > 150:
            add("info", f"Motor {n}: Ki {ms['Ki']} is high", "Can cause slow wobble and overshoot.", "Start at 0-40.",
                n, "ki")
        if ms["Kd"] and ms["Ks"] == 1:
            add("info", f"Motor {n}: Kd {ms['Kd']} with Ks 1 does very little",
                "At Ks 1 the speed reading is tiny, so the damping hardly acts.", "Use Ks 4-8 with Kd, or Kd 0.", n,
                "kdks")
    if raw.get("Kp1") == 100 and raw.get("Ki1") == 40 and raw.get("Kd1") == 40 and raw.get("Kp2") == 420:
        add("info", "These are the firmware's default settings",
            "Defaults are a safe start, not a tune for your motors and linkage.", "Run auto-tune on each motor, then "
            "Save to EEPROM.", code="defaults")
    g = st.get("running_global") or {}
    if g.get("Fpid divider") and g["Fpid divider"] > 1:
        add("info", f"Control loop runs at 4000 / {g['Fpid divider']} Hz (Fpid/{g['Fpid divider']})", "Only needed if "
            "the loop is overloaded.", code="fpid")
    for k in ("PWM kHz motors 1+2", "PWM kHz motor 3"):
        if g.get(k) and g[k] < 20:
            add("info", f"{k}: {g[k]} kHz - audible", "Below about 20 kHz the motors whine.", "25 kHz suits the "
                "IBT-2.", code="pwmf")
    ns = st.get("not_saved")
    if ns:
        lines = [f"{'Motor ' + str(x['motor']) + ' ' if x['motor'] else ''}{x['setting']}: running {x['running']}, "
                 f"saved {x['saved']}" for x in ns[:10]]
        add("warn", f"{len(ns)} setting{'s are' if len(ns) > 1 else ' is'} running but not saved to EEPROM",
            "; ".join(lines) + ("; ..." if len(ns) > 10 else "") + ". They are lost at power-off.",
            "If the rig feels right now, press Save to EEPROM. If it felt better after a restart, the saved values are "
            "the good ones - the EEPROM section has backups to switch to.", code="unsaved")
    elif ns == [] and st.get("saved_in_eeprom"):
        add("ok", "Running settings match what is saved in EEPROM", code="saved")
    # ---- motor state, pots, behaviour ----
    for n in motors:
        lm, dm, wm, ck = livem.get(n, {}), dgm.get(n, {}), w60.get(n, {}), checks.get(n)
        ms = (st.get("running") or {}).get(f"motor {n}") or {}
        cut = ms.get("Max limit (cutoff)") or 23
        pin = pins[n]
        if lm.get("off") and not (ck and ck.get("result") == "ok"):
            why = dm.get("off_reason")
            at = dm.get("cutoff_at")
            detail = {"safety cutoff": f"Its safety cutoff tripped" + (f" with the pot at {at}" if at is not None else "")
                      + f" (limits {cut} / {1023 - cut}).",
                      "switched off by command": "It was switched off by a command (Disable Motors, an unused motor, "
                                                 "or a safety stop by auto-tune / the full check).",
                      "not enabled since power-up": "It hasn't been enabled since power-up."}.get(why, "It is off.")
            add("fail", f"Motor {n} is switched off", detail,
                "Find out why first (pot slipped on its shaft, wired backwards, a mechanical end stop), then press "
                "Enable Motors. The LED matrix shows E" + str(n) + " while it is off.", n, "off")
        pot = lm.get("pot")
        if pot is not None and (pot < 5 or pot > 1018):
            add("fail", f"Motor {n}'s pot reads {pot:.0f} - the very end of its range",
                "A pot that is unplugged, has a broken wire or a dead wiper reads 0 or 1023.",
                f"Check the 3 pot wires: outer pins to 5 V and GND, wiper (middle) to {pin[2]}. Measure the wiper "
                f"voltage while turning it: it should change smoothly between 0 and 5 V.", n, "pot_rail")
        elif pot is not None and lm.get("target_still") and (pot < cut + 25 or pot > 1023 - cut - 25):
            add("warn", f"Motor {n}'s pot sits at {pot:.0f}, close to its safety cutoff",
                "The pot is near one end of its travel, so normal motion can trip the cutoff.",
                "Loosen the pot coupling and re-centre it so the motor's middle position reads about 512.", n, "pot_end")
        nz = lm.get("noise_at_rest") if lm.get("noise_at_rest") is not None else wm.get("noise_at_rest")
        gl_fw = (dm.get("pot_glitches") or 0) + counts.get(f"pot_glitch_fw m{n}", 0)
        gl_tel = (wm.get("pot_glitches") or 0) + counts.get(f"pot_glitch m{n}", 0)
        noisy = nz is not None and nz > 2.5
        extra = (f" The board also counted {gl_fw} single-cycle jumps of more than 16 counts - part of the same noise."
                 if gl_fw and noisy and not gl_tel else "")
        if nz is not None:
            if nz > 4:
                add("fail", f"Motor {n}'s pot is very noisy at rest (±{nz:.1f} counts)",
                    "Noise makes the motor buzz, hunt and feel gritty." + extra,
                    f"Use shielded or twisted pot wires, run them away from the motor wires, give the pots a good ground "
                    f"back to the Arduino GND, and add a 100 nF capacitor from {pin[2]} to GND. A worn pot also does "
                    f"this - replace it.", n, "noise")
            elif nz > 1.5:
                add("warn", f"Motor {n}'s pot is noisy at rest (±{nz:.1f} counts)", "More than the usual ±0.3-1." + extra,
                    f"Check the pot wiring and ground; a 100 nF capacitor from {pin[2]} to GND helps. Raise the dead zone "
                    f"to about {int(3 * nz) + 1} meanwhile.", n, "noise")
            else:
                add("ok", f"Motor {n}'s pot is quiet at rest (±{nz:.2f} counts)", code="noise_ok", motor=n)
        pn = lm.get("pid_noise_at_rest")
        if pn is not None and pn > 6:
            add("warn", f"Motor {n} buzzes while holding (PID output jitters ±{pn:.0f})",
                "The controller chases pot noise.", "Lower Kp or Kd, or raise the dead zone a little (auto-tune "
                "balances these).", n, "buzz")
        if gl_tel or (gl_fw and not noisy):
            lvl = "fail" if (gl_fw > 20 or gl_tel > 2) else "warn"
            add(lvl, f"Motor {n}'s pot reading jumps (glitches)",
                f"Readings that change faster than any motor can move: {gl_fw} counted by the board"
                + (f", {gl_tel} big jumps seen in the live data" if gl_tel else "") + ". This is the classic cause of "
                "sudden violent jerks: for a moment the controller thinks the motor is somewhere else and drives hard.",
                f"The pot wiper is losing contact: a worn or dirty pot, or a loose connector / crimp on the wire to "
                f"{pin[2]}. Re-seat the connectors; if it continues, replace the pot (a sealed or contactless one lasts "
                f"longer).", n, "glitch")
        c = counts.get(f"wrong_direction m{n}", 0)
        if c:
            add("fail", f"Motor {n} moved away from its target while driven ({c}x in 24 h)",
                "The drive and the pot disagree about direction: the motor wires or the pot's outer wires are swapped "
                "- or the pot slipped on its shaft.", f"Swap the two motor wires on its IBT-2 (M+ / M-). Then run the "
                "full check.", n, "wrong_dir_seen")
        c = counts.get(f"stall m{n}", 0)
        if c:
            add("warn", f"Motor {n} was pushed at full allowed power without moving ({c}x in 24 h)",
                "It hit something, the load is more than PWMmax allows, or the pot isn't turning with the motor.",
                "Look for a mechanical end stop or binding; raise PWMmax a little if it simply lacks power; check the "
                "pot coupling.", n, "stall_seen")
        c = counts.get(f"hunting m{n}", 0)
        if c:
            add("warn", f"Motor {n} kept swinging around its target at rest ({c}x in 24 h)",
                "Too much Kp (or Ki) for this motor, or too little dead zone for the pot noise.",
                "Run auto-tune, or lower Kp by about 20% and raise the dead zone by 1-2.", n, "hunting")
        c = counts.get(f"motor_off m{n}", 0)
        if c:
            add("warn", f"Motor {n} switched itself off {c}x in the last 24 hours",
                "Its safety cutoff tripped: the pot went past Max limit.", "Check the pot coupling (slipping?), the "
                "mechanical range vs. Clip, and for the motor being driven the wrong way.", n, "off_seen")
        sat = wm.get("at_full_allowed_power_pct")
        if sat is not None and wm.get("driven_by_simhub_s", 0) > 30:
            if sat > 10:
                add("warn", f"Motor {n} is at its power limit {sat:.0f}% of the time while driving",
                    "It can't keep up with what SimHub asks: big moves feel clipped and late.",
                    "Raise PWMmax in steps of 10-20, check the motor supply voltage under load, or reduce SimHub's "
                    "output for that axis.", n, "sat")
            elif sat > 3:
                add("info", f"Motor {n} reaches its power limit {sat:.0f}% of the time while driving",
                    "Normal for hard braking or crests; more than ~10% means it struggles.", code="sat", motor=n)
            p95 = wm.get("error_p95_below")
            if p95 is not None and p95 >= 80 and (sat or 0) < 3:
                add("info", f"Motor {n} often runs more than {p95 // 2} counts behind SimHub",
                    "It isn't at its power limit, so the settings are soft (low Kp, long smoothing or a speed cap).",
                    "Run auto-tune or try the Circuit racing recommendation.", n, "lag")
    # ---- full check ----
    if isinstance(rep.get("check"), dict) and rep["check"].get("skipped"):
        add("info", "Motors were not moved", "Reason: " + rep["check"]["skipped"] + ".", code="check_skipped")
    seen = {}
    for n, ck in checks.items():
        pin = pins[n]
        res, msg = ck.get("result"), ck.get("message", "")
        if res == "ok":
            up, dn = ck.get("drive_to_move_up"), ck.get("drive_to_move_down")
            stp = ck.get("step") or {}
            a, bk = stp.get("away") or {}, stp.get("back") or {}
            nums = f"Starts moving at drive {up} up / {dn} down"
            if a:
                nums += (f"; step of {stp.get('size')}: rise {a.get('rise')} ms, overshoot {a.get('os')}%, settles in "
                         f"{a.get('settle')} ms, ends {a.get('sse')} counts off, top speed {a.get('speed_max')}/s")
            add("ok", f"Motor {n} moves the right way in both directions", nums + ".", motor=n, code="check_ok")
            pmin = (ck.get("settings") or {}).get("pwmmin")
            need = min(x for x in (up, dn) if x is not None)
            if pmin is not None and pmin + 15 < need:
                add("info", f"Motor {n}: PWMmin {pmin} is below the {need} it needs to start moving",
                    "Small corrections get swallowed by friction - flat spots when the seat changes direction.",
                    f"Raise PWMmin to about {max(0, need - 10)} (auto-tune measures it precisely).", n, "pwmmin_low")
            if up and dn and max(up, dn) >= 2 * min(up, dn) and max(up, dn) - min(up, dn) >= 30:
                add("info", f"Motor {n} needs much more drive {'up' if up > dn else 'down'} ({max(up, dn)} vs "
                    f"{min(up, dn)})", "Usually the seat's weight on that corner. Fine unless it struggles.", motor=n,
                    code="asym")
            for name, r in (("away", a), ("back", bk)):
                if not r:
                    continue
                if not r.get("settled", True) and r.get("osc", 0) >= 3 and (r.get("fb_std", 0) > 10 or r.get("os", 0) > 50):
                    add("fail", f"Motor {n} keeps swinging after a small step (its own settings are unstable)",
                        f"A {stp.get('size')}-count step made it overshoot {r.get('os')}% and it hadn't settled 1.5 s "
                        f"later (swings ±{2 * r.get('fb_std', 0):.0f} counts). In a game this feels violent.",
                        "Set Ki to 0 and lower Kp by about 30%, then try a Square test - or simply run auto-tune on this "
                        "motor.", n, "unstable")
                    break
                if r.get("os", 0) > 25 or r.get("osc", 0) >= 3:
                    add("warn", f"Motor {n} overshoots or rings on a small step ({r.get('os')}% overshoot, "
                        f"{r.get('osc')} swings)", "Feels harsh and buzzy.", "Lower Kp about 20% or add damping "
                        "(Kd 5-20 with Ks 4); auto-tune finds the balance.", n, "overshoot")
                    break
                if r.get("sse", 0) > max(6, (ck.get("settings") or {}).get("dz", 0) + 3):
                    add("info", f"Motor {n} stops {r.get('sse')} counts short of the target",
                        "Friction or the seat's weight.", "Add a little Ki (5-20) or raise PWMmin.", n, "sse")
                    break
                if r.get("reached") is False:
                    add("warn", f"Motor {n} didn't reach a small step's target within 1.5 s", "Too weak for the load, "
                        "or PWMmax too low.", "Raise PWMmax / Kp, check the supply.", n, "slow")
                    break
        elif res == "reversed":
            add("fail", f"Motor {n} moves the WRONG way", msg + " With its pot it pushes away from where it should go, "
                "so in normal use it runs to the end until the safety cutoff stops it.",
                f"Swap the two motor wires on motor {n}'s IBT-2 (M+ and M-). Don't swap the pot as well - that would "
                f"reverse it again. (If the seat then moves the wrong way in games, invert that axis in SimHub.)"
                + (" The motor was switched off for safety - press Enable Motors after the fix." if ck.get("left_off")
                   else ""), n, "reversed")
        elif res == "crosstalk":
            o = (ck.get("info") or {}).get("other")
            if any(f["code"] == "crosstalk" and f["motor"] == o for f in F):
                continue                                    # the same pair, already reported from the other motor
            add("fail", f"Motors {n} and {o} are mixed up", msg + " The drive of one channel moves the other motor - "
                f"motor wires or pot wires are swapped between them.",
                f"Motor {n}'s IBT-2 (PWM {pin[0]}, enable {pin[1]}) must drive the motor whose pot goes to {pin[2]}. "
                f"Swap either the motor cables or the pot wires of motors {n} and {o} so each pot sits on the motor its "
                f"bridge drives. Motor {n} was switched off for safety.", n, "crosstalk")
        elif res == "no_move":
            seen_ans = (ck.get("seen") or {}).get("answer")
            if seen_ans == "moved":
                add("fail", f"Motor {n} turns, but its pot doesn't see it", msg + " You saw it move.",
                    f"The pot isn't turning with this motor: a slipped coupling / set screw, a broken pot wiper, or the "
                    f"pot belongs to another motor. Check the pot wiper wire to {pin[2]}.", n, "pot_not_following")
            elif seen_ans == "hum":
                add("fail", f"Motor {n} hums but doesn't move", msg,
                    "It gets power but can't turn: jammed mechanism or end stop, a seized gearbox, or far too little "
                    "supply voltage under load.", n, "jammed")
            else:
                add("fail", f"Motor {n} doesn't move", msg + (" You saw nothing move." if seen_ans == "nothing" else ""),
                    f"No drive reaches the motor: check the motor supply to motor {n}'s IBT-2 (and its fuse / switch), "
                    f"the R_EN/L_EN wires to {pin[1]}, the PWM wire to {pin[0]}, and the two motor wires. LED matrix "
                    f"E{n} means it is switched off.", n, "no_move")
        elif res == "unstable":
            add("fail", f"Motor {n} swings out of control on its own settings", msg + " In a game this feels violent "
                "and can hit the end stops.", "Set Ki to 0 and lower Kp by about 30%, then try a Square test - or run "
                "auto-tune on this motor (it starts from safe values).", n, "unstable")
        elif res == "wont_enable":
            add("fail", f"Motor {n} won't stay switched on", msg + " The board turns it off as soon as it is enabled.",
                f"Its pot reading is past the safety cutoff: the pot slipped on its shaft, is wired backwards, or the "
                f"Max limit is too tight. Re-centre the pot so the motor's middle position reads about 512.", n,
                "wont_enable")
        elif res == "one_way":
            add("warn", f"Motor {n} moves only one way", msg, "Something stops it in the other direction: a mechanical "
                "end stop, the seat's weight on a weak motor, or one half of its IBT-2 is damaged (one of the two "
                "chips).", n, "one_way")
        elif res in ("cutoff", "runaway", "clip"):
            add("fail", f"Motor {n} ran out of its safe range during the check", msg,
                "Most often the motor or pot is wired backwards, or the pot slipped. Check the pot coupling and the "
                "motor wires; the motor was switched off for safety.", n, res)
        elif res == "pot_rail":
            add("fail", f"Motor {n} not tested: its pot reads the end of its range", msg,
                f"Check the pot wiring (wiper to {pin[2]}).", n, "pot_rail")
        elif res == "beyond_cutoff":
            add("fail", f"Motor {n} not tested: its pot is past the safety cutoff", msg,
                "Turn the motor (or re-centre the pot) by hand into its normal range, then press Enable Motors.", n,
                "beyond_cutoff")
        elif res in ("stopped", "simhub", "link"):
            add("info", f"Check of motor {n} was interrupted", msg, code="interrupted", motor=n)
        sa = (ck.get("seen") or {}).get("answer")
        if sa in ("up", "down", "none"):
            seen[n] = (sa, (ck.get("seen") or {}).get("where", ""))
    if seen:
        dirs = {n: a for n, (a, _) in seen.items()}
        wh = ", ".join(f"motor {n} = {w}" for n, (_, w) in sorted(seen.items()) if w)
        ups = [n for n, a in dirs.items() if a == "up"]
        downs = [n for n, a in dirs.items() if a == "down"]
        nones = [n for n, a in dirs.items() if a == "none"]
        for n in nones:
            add("fail", f"Motor {n}: the pot moved but you saw nothing move",
                "Its pot reading changed, so something turned - but not the part of the seat you watched.",
                "The pot may be on a different motor, or the linkage is disconnected. Compare which motor actually "
                "turns.", n, "seen_none")
        if ups and downs:
            add("warn", "The motors don't all go the same way for higher numbers",
                f"Up: motor {', '.join(map(str, ups))}; down: motor {', '.join(map(str, downs))}. SimHub expects every "
                f"actuator to rise for higher numbers (or all to fall).",
                "Easiest: in SimHub invert the output of the odd one out. Or swap BOTH that motor's two wires and its "
                "pot's two outer wires (swapping only one makes it run away).", code="mixed_dirs")
        elif downs and not ups:
            add("info", "All motors go DOWN for higher numbers",
                "Fine if SimHub's outputs are set up for that; otherwise the whole motion is upside down.",
                "If pitch and roll feel inverted in games, invert the outputs in SimHub.", code="all_down")
        elif ups:
            add("ok", "Every motor checked goes UP for higher numbers", code="dirs_ok")
        if wh:
            add("info", "Where the motors are", wh + ".", code="where")
        given = rep.get("rig", {}).get("motor_names") or {}
        norm = lambda t: set(t.lower().replace("back", "rear").replace("-", " ").split())
        for n, (_, w) in sorted(seen.items()):
            nmv = given.get(f"motor {n}")
            if not (nmv and w):
                continue
            a_, b_ = norm(nmv), norm(w)
            clash = (("left" in a_ and "right" in b_) or ("right" in a_ and "left" in b_) or
                     ("front" in a_ and "rear" in b_) or ("rear" in a_ and "front" in b_))
            if clash:
                add("warn", f"Motor {n} is named '{nmv}', but you saw the {w} move",
                    "The name and what moved don't agree.",
                    "Either the name is wrong (fix it under Settings, Motor names), or the motor cables are on the wrong "
                    "channels - then SimHub's output for that corner drives a different corner. Check which bridge "
                    "(D9 / D10 / D11) drives which motor.", n, "name_clash")
    if not any(f["level"] in ("fail", "warn") for f in F):
        add("ok", "No problems found in the data", "If something still feels wrong, describe it in the box and run "
            "the full check - and send the file to whoever helps you.", code="none")
    order = {"fail": 0, "warn": 1, "info": 2, "ok": 3}
    F.sort(key=lambda f: (order[f["level"]], f["motor"] or 0))
    return F


def summary_text(rep, findings):
    b = rep.get("board", {})
    L = [f"SMC3 R4 diagnostics - {rep.get('made')} ({'full check' if rep.get('kind') == 'full' else 'quick check'}, "
         f"smc3_scope {VERSION})"]
    if rep.get("problem"):
        L.append(f"Problem described: {rep['problem']}")
    if b.get("connected"):
        L.append(f"Board {b.get('address')}, firmware {b.get('firmware')}, running {b.get('uptime', '?')}"
                 + (f", last start: {b['last_start_cause']}" if b.get("last_start_cause") else "")
                 + f", power {b.get('power', '?')}")
    else:
        L.append(f"Board {b.get('address')}: NOT CONNECTED ({b.get('disconnected_for', '')})")
    nms = rep.get("rig", {}).get("motor_names") or {}
    named = ", ".join(f"{k[6:]} = {v}" for k, v in nms.items() if v)
    L.append(f"Motors in use: {rep.get('rig', {}).get('motors_in_use')}" + (f" ({named})" if named else ""))
    names = {"fail": "PROBLEM", "warn": "WARNING", "info": "NOTE", "ok": "OK"}
    for lvl in ("fail", "warn", "info", "ok"):
        items = [f for f in findings if f["level"] == lvl]
        if not items:
            continue
        L.append("")
        for f in items:
            L.append(f"[{names[lvl]}] {f['title']}")
            if f["detail"] and lvl != "ok":
                L.append(f"    {f['detail']}")
            if f["fix"]:
                L.append(f"    What to do: {f['fix']}")
    run = (rep.get("settings") or {}).get("running") or {}
    if run:
        L.append("")
        L.append("Settings running now:")
        for k, v in run.items():
            L.append(f"  {k}: " + ", ".join(f"{a} {b}" for a, b in v.items()))
    return "\n".join(L)


# --------------------------------------------------------------------------------------------------------------
# Web page and artwork (seat logo, rig and motor pictures cropped from the dashboard design; Michroma font,
# SIL Open Font License 1.1, Copyright 2011 The Michroma Project Authors)
# --------------------------------------------------------------------------------------------------------------

ASSETS = {
    "seat.png": base64.b64decode(
        "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"),
    "rig.png": base64.b64decode(
        "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"),
    "motor.png": base64.b64decode(
        "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"),
    "michroma.woff2": base64.b64decode(
        "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"),
}


ASSETS["favicon.svg"] = base64.b64decode(
    "PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHZpZXdCb3g9IjAgMCA2NCA2NCI+CiA8cmVjdCB3aWR0aD0iNjQiIGhlaWdodD0iNjQiIHJ4PSIxNCIgZmlsbD0iIzBiMTQyMiIvPgogPHJlY3QgeD0iMTcuNSIgeT0iNi41IiB3aWR0aD0iMTEiIGhlaWdodD0iOCIgcng9IjMuNSIgdHJhbnNmb3JtPSJyb3RhdGUoMTAgMjMgMTAuNSkiIGZpbGw9IiMyYjg2ZmYiLz4KIDxwYXRoIGQ9Ik0xOC42IDE3LjRoOS4yYzEuNiAwIDIuNyAxLjQgMi41IDNMMjguNCAzNGgxMy44YzMuNCAwIDUuOCAyLjQgNS44IDUuNHYxLjhjMCAxLjUtMS4yIDIuOC0yLjggMi44SDE3LjRjLTIuOCAwLTQuOS0yLjUtNC41LTUuM2wzLjMtMTguOGMuMy0xLjQgMS4xLTIuNSAyLjQtMi41eiIgZmlsbD0iIzJiODZmZiIvPgogPHBhdGggZD0iTTQ3IDEzLjVsNS41IDMuNk00OC44IDguM2w3LjQgNC44IiBzdHJva2U9IiMyYjg2ZmYiIHN0cm9rZS13aWR0aD0iMi44IiBzdHJva2UtbGluZWNhcD0icm91bmQiIGZpbGw9Im5vbmUiIG9wYWNpdHk9Ii43Ii8+CiA8Y2lyY2xlIGN4PSIxOSIgY3k9IjUzLjUiIHI9IjQuNCIgZmlsbD0iIzJiODZmZiIvPjxjaXJjbGUgY3g9IjMyIiBjeT0iNTMuNSIgcj0iNC40IiBmaWxsPSIjZjc5NDFkIi8+PGNpcmNsZSBjeD0iNDUiIGN5PSI1My41IiByPSI0LjQiIGZpbGw9IiMyYmQ0NGYiLz4KPC9zdmc+Cg==")
ASSETS["favicon-32.png"] = base64.b64decode(
    "iVBORw0KGgoAAAANSUhEUgAAACAAAAAgCAYAAABzenr0AAAEhklEQVR4AbxWW0wcVRj+5szOshcWFmaLwLaAFIpaTFpL8ZJqQiJtStSH2j4YtbQaG2OiNSY+eAnR+oTG2qSptcTog0kbo4lgjaZCMVFD1TRaaww2Bl2KLbB0l+W2XHZmjucMZRjYmdnSQjfnn/nn+7/zf9/OnNk9BOZPsCzoC5W/6ZNvPeUPlQ+yoMsUg7wn7w2mYZY0DPjySrf7ROFPAWgSBGErIxWwWK5RwHvy3lyDa8011g3488u2CaL4NSMVzxVW6sw1uNacCYLckjxKhI+dBEPbD6H0xb9R3HgK+XVvQAxco0+XF96KbRDcgfT2hHyInNX5xCu6GgUIRemMWcSVW4Ls2x+GQAjccgVyNj6J8J4OkKycWYLDkWRlw1/1EPIfPAB38aYFTEEQir0u925CCL1nQWXRhSB5FyEAkbIgBkvS8MWANjGEeGcTtLF+BGufRaBmHwSX36BxbUIpHA2k4j0YPX/CmMST1OhlpIYu8NQ2iFeGXN8MSa5C/Pu3WY/j8BRtQE7NXmMO1ybsVpQaiFWiKYh3vI6LRzcj+tXziP/wDvpP7AC0lBXbwLTJGKb+O4PcmqcRvHc/ZvrPI9bxGsb++NTgcG1iXNkkxJOHrDV3QwpVQmVNpwd+g5RfDr42bKYY8ER3K+I/vgsXe1zB+/ZDmxwGfywGgSUZDaxqOISiXcfTovCxzwHRzVo4D+XKX4ifbsLIL8csiY4GKARkra61nOjyySCezG8Cn0xTE1DHLvE0LUgaYgL4a0dcbhMyn2rKDLudV+aB68ycDay6zbZtKhGxrS2l4GhACtkbUBK9S9Gx5ToacIeqbCembooBh0egDK/wHeA/wa5Ake0dUEZWeA24C9bbivNC4c5PUPZSj2Wsee5XyFubrf8F+WRT2K4Bt8MCNM23TEVPLgLVO5F3/8uWdTNoa0DMLjTzriv3rWvIOM/WwPTA7xknZyIQyZ+JAlsDkz3tSPx0BOr0WMYmVgQtNY3hrvesSgswWwOcleg6iL4jGxA5uHbJcfHwHRg728LbOIajAceZy1TMaEC6pRqBjY2QFr2WATfFrjtVPXhu9iNVeRHYIUNa5zHDljkzQActKwz0rq1H+PE2yHVNCD/xJTxlDzB0dpzcq+Doo5oen+1WZkF29G4JIHysAvILxQi3VMJTmw37Dx3ke8JzdgS+ozXXfBX1+mWlrKHa9CO5KUxRmE31mr8uqJ/nDr4t9nsGtic8RygEWwMzQ91zffSzMjK7qYgkBIxM6ZB+GJ8BYpN6ipkeU4FByoD93pFrE6pqH4DSBOOmjYkLJ5HsOQ11agTJfzox3t2qc1KqgObvCP6NQY+32kVwjBcnOhNIdo1CHVWQPDOK8fZhDqcH0+TaZCoRiWjQnkpngDW5hGjbPvS9fxeirc9AGx8waC0/i9h8WNLjo7OigavsG0df7UXfI92IvtILbWh+fRgkllDQPVybLUJgMtb7BVXVBkrpZVZb0cE1uFYyFmnjQroBniSHe79JqnQ9BQ4w0rcMi7K44XG1QZT35L25Bte6iuN/AAAA//+GgmVeAAAABklEQVQDAHodrBS+djwtAAAAAElFTkSuQmCC")
ASSETS["apple-touch-icon.png"] = base64.b64decode(
    "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")

PAGE = r"""<!doctype html>
<html data-theme="dark"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SMC3 R4 Motion Control</title>
<link rel="icon" href="/favicon.svg" type="image/svg+xml"><link rel="icon" href="/favicon-32.png" sizes="32x32" type="image/png"><link rel="apple-touch-icon" href="/apple-touch-icon.png">
<style>
@font-face{font-family:Michroma;src:url(/img/michroma.woff2) format("woff2");font-display:swap}
:root{
 /* chrome (header + sidebar): dark in both themes so the logo and rig artwork sit on their own background */
 --chrome:#0a1018; --chrome-2:#0f1822; --chrome-line:#1d2836; --chrome-text:#dfe7f1; --chrome-dim:#8794a6;
 --blue:#1f8fff; --blue-1:#2b7cf6; --blue-2:#0b4fc4; --orange:#f7941d; --orange-1:#f08a1c; --orange-2:#c85f05;
 --green:#2bd44f; --green-1:#2fae3e; --green-2:#17762a; --red:#e03b3b; --red-1:#d8383b; --red-2:#a3232a; --amber:#f5b301;
 --mono:ui-monospace,"Cascadia Mono",Consolas,"SFMono-Regular",Menlo,monospace;
 --sans:"Segoe UI",system-ui,-apple-system,Roboto,"Helvetica Neue",Arial,sans-serif;
}
html[data-theme=dark]{color-scheme:dark;
 --bg:#070c13; --card:#111a24; --card-2:#0d151e; --line:#223041; --line-2:#2c3b4f; --text:#e7edf5; --dim:#8a98aa; --faint:#5f6d80;
 --input:#060c12; --input-line:#283648; --chart:#03080e; --grid:#142030; --axis:#6f7d90;
 --c-t:#7f8da3; --c-g:#3d8bff; --c-f:#3ddc5a; --c-p:#ff8a1f; --tip:#0d1724; --tip-line:#2c3b4f; --tip-text:#e7edf5}
html[data-theme=light]{color-scheme:light;
 --bg:#eef2f7; --card:#ffffff; --card-2:#f5f8fb; --line:#d9e1ea; --line-2:#c7d2df; --text:#0f1b2d; --dim:#56677d; --faint:#8a99ab;
 --input:#f7f9fc; --input-line:#c9d4e1; --chart:#fbfcfe; --grid:#e6ebf1; --axis:#7c8a9c;
 --c-t:#9aa6b6; --c-g:#1f6fe5; --c-f:#14a046; --c-p:#e8780e; --tip:#ffffff; --tip-line:#c7d2df; --tip-text:#0f1b2d}
*{box-sizing:border-box}
html,body{margin:0;background:var(--bg);color:var(--text);font:14px/1.4 var(--sans)}
button,input,select{font:inherit;color:inherit}
:focus-visible{outline:2px solid var(--blue);outline-offset:2px}
svg.i{width:20px;height:20px;fill:none;stroke:currentColor;stroke-width:1.8;stroke-linecap:round;stroke-linejoin:round;flex:none}
[hidden]{display:none!important}

/* ---------- header ---------- */
.top{display:flex;align-items:center;gap:16px;padding:10px 16px 10px 8px;background:var(--chrome);color:var(--chrome-text);min-height:86px;flex-wrap:wrap}
.brand{display:flex;align-items:center;gap:12px;width:330px;flex:none}
.brand img{height:66px;width:auto;display:block}
.word{font-family:Michroma,"Eurostile Extended","Arial Black",var(--sans);font-size:31px;line-height:1;color:#f2f6fb;letter-spacing:.5px;-webkit-text-stroke:.7px #f2f6fb;white-space:nowrap}
.word span{color:#3d97ff;-webkit-text-stroke-color:#3d97ff}
.sub{margin-top:9px;font-size:12.5px;letter-spacing:1.6px;color:#c9d3df;white-space:nowrap}
.pill{flex:1;display:flex;align-items:center;min-width:280px;height:46px;padding:0 6px;border:1px solid var(--chrome-line);border-radius:10px;background:#0c141d;overflow:hidden}
.pill>span{padding:0 13px;font-size:13.5px;color:var(--chrome-dim);white-space:nowrap;border-left:1px solid var(--chrome-line)}
.pill>span:first-child{border-left:0;color:var(--chrome-text);font-weight:600;display:flex;align-items:center;gap:9px}
.pill .bad{color:#ff5757}.pill .ok{color:#4ade6a}
.dot{width:13px;height:13px;border-radius:50%;background:#7b8796;display:inline-block;flex:none}
.dot.on{background:#3ee35b;box-shadow:0 0 8px rgba(62,227,91,.55)}.dot.off{background:#ff4d4f}
.seg{display:flex;border:1px solid var(--chrome-line);border-radius:10px;overflow:hidden;background:#0c141d;height:46px}
.seg button{display:flex;align-items:center;gap:8px;padding:0 16px;border:0;background:transparent;color:var(--chrome-dim);cursor:pointer;border-left:1px solid var(--chrome-line)}
.seg button:first-child{border-left:0}
.seg button.on{background:linear-gradient(#1559cc,#0b45a8);color:#fff;box-shadow:inset 0 0 0 1px #3b82f6}
.hbtn{display:flex;align-items:center;gap:9px;height:46px;text-decoration:none;padding:0 18px;border:1px solid var(--chrome-line);border-radius:10px;background:#0c141d;color:var(--chrome-text);cursor:pointer}
.hbtn:hover,.seg button:hover:not(.on){background:#121d29;color:#fff}

/* ---------- frame ---------- */
.frame{display:flex;align-items:flex-start}
.side{width:196px;flex:none;background:var(--chrome);color:var(--chrome-text);padding:6px 8px 12px;align-self:stretch;min-height:calc(100vh - 86px);display:flex;flex-direction:column;gap:12px}
nav a{display:flex;align-items:center;gap:13px;padding:11px 13px;margin:2px 0;border-radius:8px;color:#c4cfdc;text-decoration:none;font-size:15px}
nav a:hover{background:#121c27;color:#fff}
nav a.on{background:linear-gradient(#0d5bd0,#0a46a6);color:#fff;box-shadow:inset 0 0 0 1px #2f78ea}
.rig{width:100%;height:auto;display:block;margin-top:8px;border-radius:8px}
.sys{background:var(--chrome-2);border:1px solid var(--chrome-line);border-radius:10px;padding:12px 12px 6px}
.sys h4{margin:0 0 8px;font-size:14px;display:flex;align-items:center;gap:9px}
.sys div{display:flex;align-items:center;gap:9px;font-size:13px;color:#c4cfdc;padding:4px 0}
.sys svg{width:18px;height:18px}
.st-ok{color:#3ee35b}.st-warn{color:var(--amber)}.st-bad{color:#ff5757}.st-off{color:#6b7787}
main{flex:1;min-width:0;padding:0 14px 16px 14px;display:grid;gap:14px;align-items:stretch}

/* ---------- views: one set of cards, laid out per section ---------- */
main[data-view=dashboard]{grid-template-columns:minmax(0,338fr) minmax(0,218fr) minmax(0,107fr) minmax(0,193fr) minmax(0,130fr) minmax(0,254fr);
 grid-template-areas:"motors motors motors motors motors quick" "graphs graphs graphs graphs pid pid" "test test presets presets tune tune"}
main[data-view=tuning]{grid-template-columns:minmax(360px,430px) minmax(0,1fr);grid-template-areas:"pid graphs" "global test" "quick presets"}
main[data-view=motors]{grid-template-columns:minmax(0,3fr) minmax(0,.78fr);grid-template-areas:"motors quick" "graphs graphs"}
main[data-view=graphs]{grid-template-columns:1fr;grid-template-areas:"graphs"}
main[data-view=tools]{grid-template-columns:minmax(0,1.75fr) minmax(0,1fr);grid-template-areas:"test presets" "graphs graphs"}
main[data-view=profiles]{grid-template-columns:minmax(0,1fr) minmax(360px,430px);grid-template-areas:"genres pid" "profiles pid"}
main[data-view=autotune]{grid-template-columns:minmax(360px,430px) minmax(0,1fr);grid-template-areas:"tune tunelog" "graphs graphs"}
main[data-view=eeprom]{grid-template-columns:minmax(0,1fr) minmax(360px,430px);grid-template-areas:"eeprom pid" "backups pid"}
main[data-view=settings]{grid-template-columns:repeat(2,minmax(0,1fr));grid-template-areas:"settings global" "settings mnames" "diag diag"}
main[data-view=help]{grid-template-columns:1fr;grid-template-areas:"help"}
main[data-view] > [data-views]{align-self:stretch}
.mrow{display:grid;grid-template-columns:repeat(var(--n,3),minmax(0,1fr));gap:14px;min-width:0}

/* ---------- cards ---------- */
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:13px 15px;min-width:0;position:relative}
.ct{display:flex;align-items:center;gap:10px;margin:0 0 11px;font-size:17px;font-weight:600}
.ct small{font-weight:400;color:var(--dim);font-size:14px}
.lbl{display:block;font-size:13px;color:var(--dim);margin-bottom:5px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.muted{color:var(--dim)}
input[type=number],input[type=text],select{background:var(--input);border:1px solid var(--input-line);border-radius:6px;height:36px;padding:0 9px;width:100%;min-width:0}
input[type=number]{font-family:var(--mono);font-size:14px}
select{padding-right:4px;cursor:pointer}
input:hover,select:hover{border-color:var(--line-2)}
label.chk{display:inline-flex;align-items:center;gap:8px;margin-right:16px;cursor:pointer;font-size:14px}
label.chk input{width:17px;height:17px;accent-color:var(--blue-1)}
.btn{display:inline-flex;align-items:center;justify-content:center;gap:8px;height:38px;text-decoration:none;padding:0 16px;border-radius:7px;border:1px solid transparent;cursor:pointer;color:#fff;font-weight:500;white-space:nowrap}
.btn:disabled{opacity:.45;cursor:not-allowed}
.b-blue{background:linear-gradient(#2b80f8,#0c5ce0);border-color:#3b8bff}
.b-red{background:linear-gradient(#e0403f,#b72a2d);border-color:#ec5a59}
.b-green{background:linear-gradient(#36b847,#22902f);border-color:#4bc95b}
.b-orange{background:linear-gradient(#f39221,#d86d06);border-color:#f8a646}
.b-grey{background:var(--card-2);border-color:var(--line-2);color:var(--text)}
html[data-theme=dark] .b-grey{background:linear-gradient(#2a3646,#202b38);border-color:#36465a;color:#dfe7f1}
.b-ghost{background:transparent;border-color:var(--line-2);color:var(--text)}
.btn:not(:disabled):hover{filter:brightness(1.12)}
.ibtn{width:38px;padding:0}

/* motor cards */
.motor{--mc:var(--blue)}
.motor[data-m="1"]{--mc:var(--orange)}.motor[data-m="2"]{--mc:var(--green)}
.mh{display:flex;align-items:center;gap:12px;margin-bottom:6px;padding-right:74px}
.badge{width:31px;height:31px;border-radius:50%;background:var(--mc);color:#fff;display:grid;place-items:center;font-weight:700;font-size:16px;flex:none}
.mh b{font-size:17px}
.state{margin-left:auto;display:flex;align-items:center;gap:7px;font-size:13.5px;color:var(--c-f)}
.state .dot{width:11px;height:11px;background:currentColor;box-shadow:0 0 6px currentColor}
.state.nodata .dot{box-shadow:none}
.state.off{color:var(--red)}.state.nodata{color:var(--faint)}
.mimg{position:absolute;right:12px;top:9px;width:54px;height:68px;object-fit:contain;border-radius:8px}
html[data-theme=light] .mimg{background:#111a24}
.mpos{display:flex;align-items:baseline;gap:12px;padding-right:74px}
.mpos span{color:var(--dim)}
.mpos b{margin-left:auto;font-size:24px;font-family:var(--mono);font-weight:600}
.track{position:relative;height:10px;border-radius:6px;background:var(--input);border:1px solid var(--input-line);margin:10px 0 4px}
.track .fill{position:absolute;left:0;top:0;bottom:0;border-radius:6px;background:linear-gradient(90deg,#0b55d6,#1f8fff)}
.track .fb{position:absolute;top:-4px;width:3px;height:16px;margin-left:-1.5px;background:var(--c-f);border-radius:2px}
.track .knob{position:absolute;top:50%;width:16px;height:16px;margin:-8px 0 0 -8px;border-radius:50%;background:#e9f6ff;border:3px solid #2bb3ff;box-shadow:0 0 6px rgba(43,179,255,.7)}
.scale{display:flex;justify-content:space-between;font-size:12.5px;color:var(--dim);margin-bottom:4px}
.rows div{display:flex;justify-content:space-between;padding:5px 2px;border-top:1px solid var(--line);font-size:14px}
.rows b{font-family:var(--mono);font-weight:600}

/* quick controls */
.qgrid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}
.qgrid .btn{min-width:0;padding:0 8px;white-space:normal;text-align:center;line-height:1.15}
.qgrid .btn:not(.tile){font-size:12.5px;white-space:nowrap;padding:0 4px}
.tile{height:72px;border-radius:7px;flex-direction:column;gap:6px;font-size:14px;white-space:normal;text-align:center;line-height:1.2;padding:0 8px}
.tile svg{width:22px;height:22px}
.qsub{font-size:15px;font-weight:600;margin:16px 0 9px}

/* graphs */
.gh{display:flex;align-items:center;gap:14px;flex-wrap:wrap;margin-bottom:4px}
.gh .ct{margin:0;margin-right:auto}
.leg{display:flex;gap:12px;flex-wrap:wrap;font-size:13px}
.leg span{display:flex;align-items:center;gap:6px}
.leg i{width:12px;height:12px;border-radius:2px;display:inline-block}
.gctl{display:flex;align-items:center;gap:8px}
.gctl select{width:66px;height:34px}
.gctl .btn{height:34px}
.chart{--mc:var(--blue);display:grid;grid-template-columns:minmax(0,1fr) 104px;border:1px solid color-mix(in srgb,var(--mc) 70%,transparent);border-radius:7px;margin-top:19px;position:relative;background:var(--chart)}
.chart[data-m="1"]{--mc:var(--orange)}.chart[data-m="2"]{--mc:var(--green)}
.chart .tab{position:absolute;left:-1px;top:-19px;background:var(--mc);color:#04121f;font-weight:700;font-size:13.5px;padding:1px 10px 2px;border-radius:5px 5px 0 0}
.chart canvas{width:100%;height:var(--ch,108px);display:block}
.ro{font-family:var(--mono);font-size:13px;padding:10px 10px 0 4px;display:grid;grid-template-columns:auto 1fr;gap:3px 8px;align-content:center}
.ro span{color:var(--dim)}.ro b{font-weight:500;text-align:right}

/* selected motor: one choice shared by charts, motor cards, PID columns, test signal and auto-tune */
.chart,.motor{cursor:pointer}
.chart.sel{border:2px solid var(--mc);box-shadow:0 0 0 3px color-mix(in srgb,var(--mc) 22%,transparent),0 0 18px color-mix(in srgb,var(--mc) 28%,transparent)}
.chart .tab em{font-style:normal;font-weight:600;margin-left:8px;padding:0 6px;border-radius:4px;background:rgba(0,0,0,.22);font-size:12px}
.chart .tab em:empty{display:none}
#charts.focus .chart:not(.sel){opacity:.5}
.motor.sel{border-color:var(--mc);box-shadow:0 0 0 2px color-mix(in srgb,var(--mc) 40%,transparent)}
.pid .c0{--mc:var(--blue)}.pid .c1{--mc:var(--orange)}.pid .c2{--mc:var(--green)}
.pid th.sel{box-shadow:0 0 0 2px var(--mc)}
.pid td.sel input{border-color:var(--mc)}

/* pid table */
.pid{width:calc(100% + 10px);table-layout:fixed;border-collapse:separate;border-spacing:5px 4px;margin:-4px -5px}
.pid col.l{width:84px}
.pid th{font-size:15px;padding:5px 0;border-radius:6px}
.pid th.h0{background:color-mix(in srgb,var(--blue) 28%,transparent);color:#5fb0ff}
.pid th.h1{background:color-mix(in srgb,var(--orange) 26%,transparent);color:#ffb24d}
.pid th.h2{background:color-mix(in srgb,var(--green) 22%,transparent);color:#56e872}
html[data-theme=light] .pid th.h0{color:#0b5fd6}html[data-theme=light] .pid th.h1{color:#b85a00}html[data-theme=light] .pid th.h2{color:#117a30}
.pid td:first-child{text-align:right;padding-right:6px;white-space:nowrap;font-size:14px;color:var(--text)}
.pid input{height:29px}
.pid .btn{width:100%;height:38px;padding:0 4px}

/* test / presets / tune */
.frow{display:grid;gap:10px;align-items:end}
.test .frow{grid-template-columns:minmax(92px,.95fr) minmax(0,1.45fr) minmax(0,.85fr) minmax(0,.7fr) minmax(0,.85fr) auto auto}
.presets{display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:8px}
.preset{height:70px;border:1px solid var(--line-2);border-radius:7px;background:var(--card-2);display:flex;flex-direction:column;align-items:center;justify-content:center;gap:7px;cursor:pointer;font-size:13px}
.preset:hover{border-color:var(--blue);color:var(--blue)}
.preset svg{width:26px;height:26px}
.tune .frow{grid-template-columns:minmax(0,.8fr) minmax(0,1fr) minmax(0,1.2fr);gap:8px}
.tune .ct,.test .ct,.presets-card .ct{margin-bottom:8px}
.tune .lbl,.test .lbl{font-size:12.5px;margin-bottom:3px}
.tune select,.test select,.test input{height:33px}
.test input[type=number]{-moz-appearance:textfield;appearance:textfield;padding:0 7px}
.test input[type=number]::-webkit-inner-spin-button,.test input[type=number]::-webkit-outer-spin-button{-webkit-appearance:none;margin:0}
.tune .chks{margin:8px 0 9px;display:flex;flex-wrap:wrap;gap:3px 0}
.tune .chks label{margin-right:9px;font-size:12px;gap:5px}
.tune .chks label:last-child{margin-right:0}
.tune .chks input{width:15px;height:15px}
.tune .acts{display:flex;gap:10px;align-items:center}
.tune .acts .b-green{flex:1;height:37px}
.tune .acts .btn{height:37px}
.bar{height:7px;background:var(--input);border:1px solid var(--input-line);border-radius:5px;overflow:hidden;margin:10px 0 6px}
.bar div{height:100%;width:0;background:linear-gradient(90deg,#0b55d6,#1f8fff);transition:width .4s}
@media (prefers-reduced-motion:reduce){.bar div{transition:none}}
pre.log{background:var(--chart);border:1px solid var(--line);border-radius:7px;height:260px;overflow:auto;font:12px/1.45 var(--mono);padding:8px 10px;margin:10px 0;white-space:pre-wrap}
table.res{border-collapse:collapse;margin:6px 0 10px}table.res td,table.res th{padding:4px 14px 4px 0;text-align:left}
td.chg{color:var(--blue);font-weight:700}

/* recommendations */
.genres{display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:10px;margin-top:12px}
.genre{border:1px solid var(--line-2);border-radius:9px;background:var(--card-2);padding:12px 11px;text-align:left;cursor:pointer;display:flex;flex-direction:column;gap:6px;min-height:124px;color:var(--text)}
.genre svg{width:28px;height:28px;color:var(--blue)}
.genre b{font-size:14px}.genre span{font-size:12.5px;color:var(--dim);line-height:1.35}
.genre:hover,.genre.on{border-color:var(--blue);box-shadow:inset 0 0 0 1px var(--blue)}
.pv{margin-top:14px;border-top:1px solid var(--line);padding-top:12px}
.pv h3{margin:0 0 4px;font-size:16px}
table.chg{border-collapse:collapse;font-size:13.5px;margin:8px 0}table.chg td,table.chg th{padding:3px 14px 3px 0;text-align:left}
table.chg td{font-family:var(--mono)}table.chg td:first-child{font-family:var(--sans)}
.up{color:var(--c-f)}.down{color:var(--c-p)}
ul.tips{margin:6px 0 10px;padding-left:18px}ul.tips li{color:var(--dim);margin:4px 0;max-width:82ch}

/* backups */
.blist{display:flex;flex-direction:column;gap:6px;margin-top:4px;max-height:640px;overflow:auto}
.brow{display:grid;grid-template-columns:132px minmax(0,1fr) auto;gap:12px;align-items:center;padding:9px 11px;border:1px solid var(--line);border-radius:8px;background:var(--card-2)}
.brow .when{font-family:var(--mono);font-size:12.5px;color:var(--dim)}
.brow .what{font-weight:600}.brow .sum{font-size:12.5px;color:var(--dim)}
.brow .acts{display:flex;gap:6px;flex-wrap:wrap}.brow .btn{height:31px;padding:0 10px;font-size:13px}
.kind{display:inline-block;font-size:11.5px;font-weight:500;padding:0 7px;border-radius:10px;margin-right:6px;border:1px solid var(--line-2);color:var(--dim)}
.kind.eeprom{border-color:var(--red);color:var(--red)}.kind.manual{border-color:var(--blue);color:var(--blue)}
.cur{color:var(--c-f);font-size:12px;font-weight:600;margin-left:6px}
#toast{position:fixed;right:18px;bottom:18px;max-width:380px;background:var(--tip);color:var(--tip-text);border:1px solid var(--tip-line);border-left:4px solid var(--c-f);border-radius:8px;padding:10px 14px;box-shadow:0 10px 26px rgba(0,0,0,.35);z-index:120;display:none}
#toast.err{border-left-color:var(--red)}

/* profiles / eeprom / settings / help */
.plist{display:flex;flex-direction:column;gap:8px;margin-top:14px}
.prow{display:flex;align-items:center;gap:12px;padding:10px 12px;border:1px solid var(--line);border-radius:8px;background:var(--card-2)}
.prow b{min-width:140px}.prow .muted{flex:1;font-size:13px}
.inline{display:flex;gap:10px;align-items:end}
.inline > div{flex:1}
.field{display:grid;grid-template-columns:150px 1fr;gap:10px;align-items:center;margin:8px 0}
.note{font-size:13.5px;color:var(--dim);max-width:70ch}
.help{max-width:none}
.help .cols{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:18px 28px}
.help h3{font-size:15.5px;margin:4px 0 6px}
.help p,.help li{color:var(--dim);max-width:72ch}
.help ol,.help ul{padding-left:20px;margin:6px 0}
.msg{margin-top:10px;font-size:13.5px}
.okc{color:var(--c-f)}.badc{color:var(--red)}

/* tooltips */
[data-tip]{cursor:help}
.pid tr[data-tip] td:first-child,.lbl[data-tip],label.chk[data-tip] span{text-decoration:underline dotted var(--faint);text-underline-offset:3px}
#tip{position:fixed;display:none;max-width:360px;background:var(--tip);color:var(--tip-text);border:1px solid var(--tip-line);border-radius:9px;padding:10px 12px;font-size:13px;line-height:1.45;box-shadow:0 10px 26px rgba(0,0,0,.35);z-index:100;pointer-events:none}
.modal{position:fixed;inset:0;background:rgba(2,6,12,.6);display:none;align-items:center;justify-content:center;z-index:50}
.modal .box{background:var(--card);border:1px solid var(--line-2);border-radius:12px;padding:18px 20px;width:min(560px,94vw)}
.modal h2{font-size:17px;margin:0 0 10px}.modal li{margin:5px 0}

/* motor names */
.nrow{display:grid;grid-template-columns:31px minmax(0,1fr) minmax(0,1fr);gap:10px;align-items:center;margin:8px 0}
.nrow .badge{background:var(--mc)}
.nrow[data-m="0"]{--mc:var(--blue)}.nrow[data-m="1"]{--mc:var(--orange)}.nrow[data-m="2"]{--mc:var(--green)}
.nrow input[hidden]{display:none}
.mtitle .num{color:var(--dim);font-weight:400;font-size:13px;margin-left:6px}
.pid th{white-space:nowrap;overflow:hidden;text-overflow:ellipsis;padding-left:4px;padding-right:4px}
.pid .btn .bl{overflow:hidden;text-overflow:ellipsis;white-space:nowrap;min-width:0}
.mh b{white-space:nowrap;overflow:hidden;text-overflow:ellipsis;min-width:0}.mh .state{flex:none}

/* diagnostics */
.dgrid{display:grid;grid-template-columns:minmax(0,1.55fr) minmax(280px,1fr);gap:20px}
textarea{background:var(--input);border:1px solid var(--input-line);border-radius:6px;padding:8px 9px;width:100%;resize:vertical;font:inherit;color:inherit;min-height:62px}
textarea:hover{border-color:var(--line-2)}
.dg-acts{display:flex;gap:10px;flex-wrap:wrap;align-items:center;margin:10px 0 4px}
.dg-acts label.chk{margin:0 0 0 4px;font-size:13px}
.dlive{font-family:var(--mono);font-size:13px;margin-top:4px;min-height:18px}
.dprompt{--mc:var(--blue);display:flex;gap:12px;align-items:flex-start;border:2px solid var(--mc);border-radius:9px;padding:12px 14px;margin:12px 0;background:color-mix(in srgb,var(--mc) 9%,var(--card))}
.dprompt[data-m="2"]{--mc:var(--orange)}.dprompt[data-m="3"]{--mc:var(--green)}
.dprompt .badge{background:var(--mc)}
.dprompt .q{font-size:15px;font-weight:600;margin:4px 0 9px}
.dprompt .opts{display:flex;gap:8px;flex-wrap:wrap}
.dprompt .opts .btn{height:34px}
.dprompt .where{margin-top:9px;display:flex;gap:8px;align-items:center;font-size:13px;color:var(--dim);flex-wrap:wrap}
.dprompt .where select{width:auto;height:32px}
.watching{display:inline-flex;align-items:center;gap:8px;color:var(--mc);font-weight:600;font-size:13px;text-transform:uppercase;letter-spacing:.5px}
.watching i{width:10px;height:10px;border-radius:50%;background:var(--mc);animation:pulse 1s infinite}
@keyframes pulse{50%{opacity:.25}}
@media (prefers-reduced-motion:reduce){.watching i{animation:none}}
.fsum{display:flex;gap:12px;align-items:baseline;flex-wrap:wrap;margin:14px 0 8px}
.fsum b{font-size:15.5px}
.facts{display:flex;gap:10px;flex-wrap:wrap;margin-bottom:10px}
.flist{display:flex;flex-direction:column;gap:7px}
.fitem{--fc:var(--blue);display:grid;grid-template-columns:24px minmax(0,1fr);gap:10px;padding:9px 11px;border:1px solid var(--line);border-left:4px solid var(--fc);border-radius:8px;background:var(--card-2)}
.fitem.fail{--fc:var(--red)}.fitem.warn{--fc:var(--amber)}.fitem.ok{--fc:var(--c-f)}
.fic{width:22px;height:22px;border-radius:50%;background:var(--fc);color:#fff;display:grid;place-items:center;font-weight:700;font-size:12.5px;font-style:normal}
.fitem.warn .fic{color:#1b1300}
.fitem b{font-size:14px}.fitem p{margin:3px 0 0;color:var(--dim);font-size:13px;max-width:92ch}
.fitem p.fix{color:var(--text)}
.mtag{display:inline-block;font-size:11.5px;font-weight:700;padding:0 6px;border-radius:4px;margin-right:7px;color:#04121f;background:var(--blue);vertical-align:1px}
.mtag.m2{background:var(--orange)}.mtag.m3{background:var(--green)}
details.oks summary,details.evd summary{cursor:pointer;color:var(--dim);margin:9px 0 4px;font-size:13.5px}
details.oks .flist{margin-top:6px}
.dside .qsub{display:flex;align-items:center;gap:8px;margin:2px 0 8px}
.dside .qsub small{font-weight:400;color:var(--dim);font-size:12.5px}
.dlist{display:flex;flex-direction:column;gap:5px;max-height:250px;overflow:auto;margin-bottom:14px}
.drow{display:grid;grid-template-columns:minmax(0,1fr) auto;gap:8px;align-items:center;padding:7px 9px;border:1px solid var(--line);border-radius:7px;background:var(--card-2);font-size:13px}
.drow .t{font-family:var(--mono);font-size:12px;color:var(--dim)}
.drow .m{color:var(--dim);font-size:12.5px;margin-top:2px}
.drow .acts{display:flex;gap:5px}.drow .btn{height:28px;padding:0 9px;font-size:12.5px}
.evlog{font:12px/1.5 var(--mono);max-height:320px;overflow:auto;background:var(--chart);border:1px solid var(--line);border-radius:7px;padding:6px 9px;margin-top:6px}
.evlog div{white-space:pre-wrap;padding:1px 0}.evlog span{color:var(--dim)}.evlog .problem{color:var(--amber)}
.nbadge{margin-left:auto;min-width:20px;height:20px;border-radius:10px;background:var(--amber);color:#1b1300;font-size:12px;font-weight:700;display:grid;place-items:center;padding:0 6px}
.linkbtn{background:none;border:0;color:var(--blue);cursor:pointer;font-size:12.5px;padding:0;margin-left:auto}
.sys div.clk{cursor:pointer}

/* ---------- smaller screens ---------- */
@media (max-width:1280px){
 main[data-view]{grid-template-columns:1fr 1fr!important;grid-template-areas:none!important}
 main[data-view] > [data-views]{grid-area:auto!important}
 main[data-view] > .wide{grid-column:1/-1}
 .genres{grid-template-columns:repeat(3,minmax(0,1fr))}
 .brand{width:auto}
}
@media (max-width:860px){
 .frame{flex-direction:column}
 .side{width:100%;min-height:0;padding:6px 10px}
 nav{display:flex;overflow-x:auto;gap:4px}
 nav a{white-space:nowrap;padding:9px 11px}
 .rig,.sys{display:none}
 .mrow{grid-template-columns:1fr}
 .genres{grid-template-columns:repeat(2,minmax(0,1fr))}
 .brow{grid-template-columns:1fr}
 .prow{flex-wrap:wrap}.prow b{min-width:0;flex-basis:100%}.prow .muted{flex-basis:100%}
 .inline{flex-wrap:wrap}.inline>div{flex-basis:100%}
 main{padding:12px}
 main[data-view]{grid-template-columns:1fr!important}
 .pill{order:3;flex-basis:100%;flex-wrap:wrap;height:auto;padding:6px 4px;row-gap:4px}
 .pill>span{border-left:0}
 .test .frow{grid-template-columns:1fr 1fr}
 .presets{grid-template-columns:repeat(3,1fr)}
 .chart{grid-template-columns:1fr}.ro{grid-template-columns:repeat(4,auto 1fr);padding:6px 8px}
 .field{grid-template-columns:1fr}
 .dgrid{grid-template-columns:1fr}
 .nrow{grid-template-columns:31px minmax(0,1fr)}.nrow input{grid-column:2}
 section[style*="grid-area:global"] .frow{grid-template-columns:1fr 1fr!important}
}
</style></head><body>
<div id="tip"></div>
<div id="toast" role="status"></div>

<header class="top">
 <div class="brand"><img src="/img/seat.png" alt=""><div><div class="word">SMC3 <span>R4</span></div><div class="sub">MOTION CONTROL DASHBOARD</div></div></div>
 <div class="pill" id="pill"><span><i class="dot"></i>Connecting…</span></div>
 <div class="seg" role="group" aria-label="Theme">
  <button data-theme-set="light" data-k="th_day"><svg class="i" viewBox="0 0 24 24"><circle cx="12" cy="12" r="4"/><path d="M12 2v2M12 20v2M2 12h2M20 12h2M4.9 4.9l1.4 1.4M17.7 17.7l1.4 1.4M4.9 19.1l1.4-1.4M17.7 6.3l1.4-1.4"/></svg>Day</button>
  <button data-theme-set="dark" data-k="th_dark"><svg class="i" viewBox="0 0 24 24"><path d="M21 12.8A9 9 0 1 1 11.2 3a7 7 0 0 0 9.8 9.8z"/></svg>Dark</button>
  <button data-theme-set="auto" data-k="th_auto"><svg class="i" viewBox="0 0 24 24"><circle cx="12" cy="12" r="9"/><path d="M12 7v5l3 2"/></svg>Auto</button>
 </div>
 <a class="hbtn" href="#settings" data-k="settingsBtn"><svg class="i" viewBox="0 0 24 24"><circle cx="12" cy="12" r="3"/><path d="M19.4 15a1.7 1.7 0 0 0 .3 1.8l.1.1a2 2 0 1 1-2.8 2.8l-.1-.1a1.7 1.7 0 0 0-1.8-.3 1.7 1.7 0 0 0-1 1.5V21a2 2 0 1 1-4 0v-.1a1.7 1.7 0 0 0-1.1-1.5 1.7 1.7 0 0 0-1.8.3l-.1.1a2 2 0 1 1-2.8-2.8l.1-.1a1.7 1.7 0 0 0 .3-1.8 1.7 1.7 0 0 0-1.5-1H3a2 2 0 1 1 0-4h.1a1.7 1.7 0 0 0 1.5-1.1 1.7 1.7 0 0 0-.3-1.8l-.1-.1a2 2 0 1 1 2.8-2.8l.1.1a1.7 1.7 0 0 0 1.8.3H9a1.7 1.7 0 0 0 1-1.5V3a2 2 0 1 1 4 0v.1a1.7 1.7 0 0 0 1 1.5 1.7 1.7 0 0 0 1.8-.3l.1-.1a2 2 0 1 1 2.8 2.8l-.1.1a1.7 1.7 0 0 0-.3 1.8V9a1.7 1.7 0 0 0 1.5 1H21a2 2 0 1 1 0 4h-.1a1.7 1.7 0 0 0-1.5 1z"/></svg>Settings</a>
</header>

<div class="frame">
<aside class="side">
 <nav id="nav">
  <a href="#dashboard"><svg class="i" viewBox="0 0 24 24"><path d="M3 11l9-7 9 7v9a1 1 0 0 1-1 1h-5v-6H9v6H4a1 1 0 0 1-1-1z"/></svg>Dashboard</a>
  <a href="#tuning"><svg class="i" viewBox="0 0 24 24"><path d="M4 6h9M17 6h3M4 12h3M11 12h9M4 18h11M19 18h1"/><circle cx="15" cy="6" r="2"/><circle cx="9" cy="12" r="2"/><circle cx="17" cy="18" r="2"/></svg>Tuning</a>
  <a href="#motors"><svg class="i" viewBox="0 0 24 24"><rect x="3" y="7" width="13" height="10" rx="2"/><path d="M16 12h5M7 7V5M12 7V5M7 11h5"/></svg>Motors</a>
  <a href="#graphs"><svg class="i" viewBox="0 0 24 24"><path d="M3 3v18h18"/><path d="M7 15l4-5 3 3 5-7"/></svg>Live Graphs</a>
  <a href="#tools"><svg class="i" viewBox="0 0 24 24"><path d="M14.7 6.3a4 4 0 0 0-5.4 5.4L3 18l3 3 6.3-6.3a4 4 0 0 0 5.4-5.4l-2.6 2.6-2.4-.6-.6-2.4z"/></svg>Test &amp; Tools</a>
  <a href="#profiles"><svg class="i" viewBox="0 0 24 24"><path d="M6 2h9l5 5v15H6z"/><path d="M14 2v6h6M9 13h8M9 17h6"/></svg>Profiles</a>
  <a href="#autotune"><svg class="i" viewBox="0 0 24 24"><path d="M4 20L15 9M13 4l1 2 2 1-2 1-1 2-1-2-2-1 2-1zM19 11l.7 1.3L21 13l-1.3.7L19 15l-.7-1.3L17 13l1.3-.7z"/></svg>Auto-Tune</a>
  <a href="#eeprom"><svg class="i" viewBox="0 0 24 24"><rect x="6" y="6" width="12" height="12" rx="1.5"/><path d="M9 2v4M15 2v4M9 18v4M15 18v4M2 9h4M2 15h4M18 9h4M18 15h4"/></svg>EEPROM</a>
  <a href="#settings"><svg class="i" viewBox="0 0 24 24"><circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3M4.9 4.9l2.1 2.1M17 17l2.1 2.1M4.9 19.1L7 17M17 7l2.1-2.1"/></svg>Settings<span class="nbadge" id="navbadge" data-k="navbadge" hidden></span></a>
  <a href="#help"><svg class="i" viewBox="0 0 24 24"><circle cx="12" cy="12" r="9.5"/><path d="M9.1 9a3 3 0 0 1 5.8 1c0 2-3 3-3 3M12 17h.01"/></svg>Help</a>
 </nav>
 <img class="rig" src="/img/rig.png" alt="">
 <div class="sys" id="sys">
  <h4><i class="dot" id="sysdot"></i>System Status</h4>
  <div data-k="sy_board"><svg class="i" viewBox="0 0 24 24"><rect x="3" y="6" width="18" height="12" rx="2"/><path d="M7 10h4M7 14h2"/></svg><span>Board</span></div>
  <div data-k="sy_motors"><svg class="i" viewBox="0 0 24 24"><rect x="4" y="7" width="12" height="10" rx="2"/><path d="M16 12h4"/></svg><span>Motors</span></div>
  <div data-k="sy_pid"><svg class="i" viewBox="0 0 24 24"><circle cx="12" cy="12" r="9"/><path d="M12 7v5l3 2"/></svg><span>PID loop</span></div>
  <div data-k="sy_power"><svg class="i" viewBox="0 0 24 24"><path d="M13 2L4 14h7l-1 8 9-12h-7z"/></svg><span>Power</span></div>
  <div data-k="sy_input"><svg class="i" viewBox="0 0 24 24"><path d="M3 12h4l3-8 4 16 3-8h4"/></svg><span>Input signal</span></div>
  <div data-k="sy_err"><svg class="i" viewBox="0 0 24 24"><path d="M12 3l8 3v6c0 5-3.5 8-8 9-4.5-1-8-4-8-9V6z"/><path d="M9 12l2 2 4-4"/></svg><span>Errors</span></div>
 </div>
</aside>

<main id="main" data-view="dashboard">

 <!-- motor cards -->
 <div class="mrow wide" id="mrow" style="grid-area:motors" data-views="dashboard motors">
  <section class="card motor" data-m="0"></section>
  <section class="card motor" data-m="1"></section>
  <section class="card motor" data-m="2"></section>
 </div>

 <section class="card" style="grid-area:quick" data-views="dashboard motors tuning">
  <div class="ct">Quick Controls</div>
  <div class="qgrid">
   <button class="btn b-blue tile" data-k="ena" onclick="txt('ena')"><svg class="i" viewBox="0 0 24 24"><path d="M7 4l13 8-13 8z" fill="currentColor"/></svg>Enable Motors</button>
   <button class="btn b-red tile" id="disBtn" data-k="dis" onclick="disableAll()"><svg class="i" viewBox="0 0 24 24"><rect x="6" y="6" width="12" height="12" rx="2" fill="currentColor"/></svg>Disable Motors</button>
  </div>
  <div class="qsub">Board</div>
  <div class="qgrid">
   <button class="btn b-blue" data-k="read" onclick="readParams()">Read from board</button>
   <button class="btn b-red" data-k="sav" onclick="saveEeprom()">Save to EEPROM</button>
  </div>
 </section>

 <!-- live graphs -->
 <section class="card wide" style="grid-area:graphs" data-views="dashboard motors graphs tuning tools autotune">
  <div class="gh">
   <div class="ct"><svg class="i" viewBox="0 0 24 24"><path d="M3 12h4l3-8 4 16 3-8h4"/></svg>Live Graphs</div>
   <div class="leg">
    <span data-k="lg_t"><i style="background:var(--c-t)"></i>Target</span>
    <span data-k="lg_g"><i style="background:var(--c-g)"></i>Goal</span>
    <span data-k="lg_p"><i style="background:var(--c-p)"></i>PID out</span>
    <span data-k="lg_f"><i style="background:var(--c-f)"></i>Feedback</span>
   </div>
   <div class="gctl">
    <span class="muted" data-k="win">Window</span><select id="win"><option value="2">2 s</option><option value="5" selected>5 s</option><option value="10">10 s</option><option value="30">30 s</option></select>
    <button class="btn b-grey" id="pauseBtn" data-k="pause" onclick="togglePause()"><svg class="i" viewBox="0 0 24 24"><path d="M8 5v14M16 5v14"/></svg>Pause</button>
    <button class="btn b-grey ibtn" id="expBtn" data-k="expand" onclick="toggleExpand()"><svg class="i" viewBox="0 0 24 24"><path d="M15 3h6v6M9 21H3v-6M21 3l-7 7M3 21l7-7"/></svg></button>
   </div>
  </div>
  <div id="charts"></div>
 </section>

 <!-- PID settings -->
 <section class="card wide" style="grid-area:pid" data-views="dashboard tuning profiles eeprom">
  <div class="ct"><svg class="i" viewBox="0 0 24 24"><rect x="4" y="3" width="16" height="18" rx="2"/><path d="M8 8h8M8 12h8M8 16h5"/></svg>PID Settings</div>
  <table class="pid" id="pidtable"></table>
 </section>

 <!-- loop & PWM -->
 <section class="card" style="grid-area:global" data-views="tuning settings">
  <div class="ct">Loop, PWM &amp; smoothing</div>
  <div class="frow" style="grid-template-columns:1fr 1fr 1fr auto">
   <div><span class="lbl" data-k="fpid">Fpid divider</span><input type="number" id="div" min="1" max="10" value="1"></div>
   <div><span class="lbl" data-k="pwmfreq">PWM kHz motors 1+2</span><input type="number" id="f1" value="25"></div>
   <div><span class="lbl" data-k="pwmfreq">PWM kHz motor 3</span><input type="number" id="f2" value="25"></div>
   <button class="btn b-blue ap" data-k="applyGlobal" onclick="applyGlobal()">Apply</button>
  </div>
  <div class="frow" style="grid-template-columns:1.3fr 1fr auto;margin-top:12px">
   <div><span class="lbl" data-k="smooth">Target smoothing</span><select id="fsm"><option value="0">Off</option><option value="4">4 ms - raw</option><option value="5">8 ms - crisp</option><option value="6">16 ms - balanced</option><option value="7">32 ms - soft</option><option value="8">64 ms - very soft</option></select></div>
   <div><span class="lbl" data-k="maxrate">Max speed (counts/s)</span><input type="number" id="frate" min="0" max="25500" step="100" value="0"></div>
   <button class="btn b-blue ap" id="fapply" data-k="applyFilt" onclick="applyFilt()">Apply</button>
  </div>
  <p class="note" id="fnote">Takes effect immediately; press Save to EEPROM to keep it after a power cycle.</p>
 </section>

 <!-- test signal -->
 <section class="card test" style="grid-area:test" data-views="dashboard tools tuning">
  <div class="ct"><svg class="i" viewBox="0 0 24 24"><rect x="3" y="5" width="18" height="14" rx="2"/><path d="M6 14l3-4 3 3 3-5 3 4"/></svg>Test Signal <small>(stop SimHub motion first)</small></div>
  <div class="frow">
   <div><span class="lbl" data-k="gmotor">Motor</span><select id="gm" class="msel" onchange="setSel(+this.value)"></select></div>
   <div><span class="lbl" data-k="gmode">Mode</span><select id="gmode"><option value="sine">Sine</option><option value="step">Square</option><option value="triangle">Triangle</option><option value="manual">Hold</option></select></div>
   <div><span class="lbl" data-k="gamp">Amplitude</span><input type="number" id="gamp" value="150"></div>
   <div><span class="lbl" data-k="gper">Period (s)</span><input type="number" id="gper" value="2" step="0.1"></div>
   <div><span class="lbl" data-k="goff">Offset</span><input type="number" id="goff" value="512"></div>
   <button class="btn b-blue ap" data-k="gstart" onclick="gen()">Start</button>
   <button class="btn b-grey" data-k="gstop" onclick="genStop()">Stop</button>
  </div>
  <div class="msg muted" id="gmsg"></div>
 </section>

 <!-- presets -->
 <section class="card presets-card" style="grid-area:presets" data-views="dashboard tools tuning">
  <div class="ct">Presets</div>
  <div class="presets">
   <button class="preset ap" data-k="p_sine" onclick="preset('sine')"><svg class="i" viewBox="0 0 24 24"><path d="M2 12c2.5-7 5.5-7 8 0s5.5 7 8 0c.9-2.5 2.2-4 4-4"/></svg>Sine</button>
   <button class="preset ap" data-k="p_square" onclick="preset('square')"><svg class="i" viewBox="0 0 24 24"><path d="M2 16h4V8h6v8h6V8h4"/></svg>Square</button>
   <button class="preset ap" data-k="p_tri" onclick="preset('triangle')"><svg class="i" viewBox="0 0 24 24"><path d="M2 16l5-8 5 8 5-8 5 8"/></svg>Triangle</button>
   <button class="preset ap" data-k="p_step" onclick="preset('step')"><svg class="i" viewBox="0 0 24 24"><path d="M2 17h8V7h12"/></svg>Step</button>
   <button class="preset ap" data-k="p_manual" onclick="preset('manual')"><svg class="i" viewBox="0 0 24 24"><path d="M12 5v14M5 12h14"/></svg>Manual</button>
  </div>
 </section>

 <!-- auto-tune -->
 <section class="card tune" style="grid-area:tune" data-views="dashboard autotune">
  <div class="ct" data-k="tune"><svg class="i" viewBox="0 0 24 24"><path d="M4 20L15 9M13 4l1 2 2 1-2 1-1 2-1-2-2-1 2-1zM19 11l.7 1.3L21 13l-1.3.7L19 15l-.7-1.3L17 13l1.3-.7z"/></svg>Auto-Tune</div>
  <div class="frow">
   <div><span class="lbl" data-k="tmotor">Motor</span><select id="tm" class="msel" onchange="setSel(+this.value)"></select></div>
   <div><span class="lbl" data-k="tamp">Travel ±</span><select id="tamp"><option>60</option><option>80</option><option selected>100</option><option>150</option><option>200</option></select></div>
   <div><span class="lbl" data-k="tstyle">Style</span><select id="tstyle"><option value="gentle">Gentle</option><option value="normal" selected>Normal</option><option value="firm">Firm</option></select></div>
  </div>
  <div class="chks">
   <label class="chk" data-k="tfr"><input type="checkbox" id="tfr" checked><span>PWMmin + dead zone</span></label>
   <label class="chk" data-k="tki"><input type="checkbox" id="tki" checked><span>Ki if needed</span></label>
   <label class="chk" data-k="tkd"><input type="checkbox" id="tkd" checked><span>Kd if it helps</span></label>
  </div>
  <div class="acts">
   <button class="btn b-green" id="tstart" data-k="tstart" onclick="tuneConfirm()">Start Auto-Tune</button>
   <button class="btn b-grey" id="tstop" data-k="tstop" onclick="post('/tune/abort')" disabled>Stop</button>
   <a class="btn b-grey ibtn" href="#autotune" data-k="tgear"><svg class="i" viewBox="0 0 24 24"><circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3M4.9 4.9l2.1 2.1M17 17l2.1 2.1M4.9 19.1L7 17M17 7l2.1-2.1"/></svg></a>
  </div>
  <div id="tmini" hidden><div class="bar"><div class="tbar"></div></div><div class="muted tstage"></div></div>
 </section>

 <section class="card" style="grid-area:tunelog" data-views="autotune">
  <div class="ct">Auto-tune progress</div>
  <div class="bar"><div class="tbar"></div></div>
  <div class="muted tstage">Idle - choose a motor and press Start Auto-Tune.</div>
  <pre class="log" id="tlog"></pre>
  <div id="tresult"></div>
 </section>

 <!-- recommendations -->
 <section class="card" style="grid-area:genres" data-views="profiles">
  <div class="ct"><svg class="i" viewBox="0 0 24 24"><path d="M4 20L15 9M13 4l1 2 2 1-2 1-1 2-1-2-2-1 2-1zM19 11l.7 1.3L21 13l-1.3.7L19 15l-.7-1.3L17 13l1.3-.7z"/></svg>Recommended starting points</div>
  <p class="note">Pick what you're about to play. Each recommendation starts from <b>your</b> settings - best after auto-tune - and adjusts how firm, damped, fast and smooth the rig feels for that kind of sim. You see every change before anything is applied.</p>
  <div class="inline" style="max-width:440px"><div><span class="lbl" data-k="g_base">Start from</span><select id="gbase"></select></div></div>
  <div class="genres" id="genres"></div>
  <div class="pv" id="gpv" hidden></div>
 </section>

 <!-- profiles -->
 <section class="card" style="grid-area:profiles" data-views="profiles">
  <div class="ct"><svg class="i" viewBox="0 0 24 24"><path d="M6 2h9l5 5v15H6z"/><path d="M14 2v6h6M9 13h8M9 17h6"/></svg>Profiles</div>
  <p class="note">A profile is a snapshot of every setting on the board - all three motors, the loop divider and PWM frequency. Keep one per car or game and switch in a click. Profiles are stored on the NAS.</p>
  <div class="inline">
   <div><span class="lbl" data-k="pr_name">Profile name</span><input type="text" id="prname" placeholder="e.g. ACC GT3 - firm" maxlength="40"></div>
   <button class="btn b-blue" data-k="pr_save" onclick="profileSave()">Save current settings</button>
  </div>
  <div class="plist" id="plist"></div>
  <div class="msg" id="prmsg"></div>
 </section>

 <!-- eeprom -->
 <section class="card" style="grid-area:eeprom" data-views="eeprom">
  <div class="ct"><svg class="i" viewBox="0 0 24 24"><rect x="6" y="6" width="12" height="12" rx="1.5"/><path d="M9 2v4M15 2v4M9 18v4M15 18v4M2 9h4M2 15h4M18 9h4M18 15h4"/></svg>EEPROM</div>
  <p class="note">Changes you apply from this page run straight away but are lost at power-off until you <b>Save to EEPROM</b>. Every save - and every bigger change (loading a profile or recommendation, auto-tune, switching backups) - keeps a backup of the settings on the NAS, so you can always switch back.</p>
  <div style="display:flex;gap:10px;flex-wrap:wrap;margin:12px 0">
   <button class="btn b-red" data-k="sav" onclick="saveEeprom()">Save to EEPROM</button>
   <button class="btn b-blue" data-k="read" onclick="readParams()">Read from board</button>
  </div>
  <div class="inline"><div><span class="lbl" data-k="bk_label">Name for a manual backup</span><input type="text" id="bklabel" maxlength="40" placeholder="e.g. before trying a firmer Kp"></div><button class="btn b-blue" data-k="bk_now" onclick="backupNow()">Back up now</button></div>
  <div style="display:flex;gap:10px;flex-wrap:wrap;margin-top:12px">
   <button class="btn b-grey" data-k="bk_dl" onclick="backupDownload()">Download current settings</button>
   <label class="btn b-grey" data-k="bk_up">Restore from file<input type="file" accept=".json" id="bkfile" hidden onchange="backupRestore(this)"></label>
  </div>
  <div class="msg" id="eemsg"></div>
 </section>
 <section class="card" style="grid-area:backups" data-views="eeprom">
  <div class="ct">Backups <small id="bkcount"></small></div>
  <div class="blist" id="blist"></div>
 </section>

 <!-- settings -->
 <section class="card" style="grid-area:settings" data-views="settings">
  <div class="ct">Connection &amp; appearance <small>smc3_scope v__VERSION__</small></div>
  <div class="field" data-k="s_host"><span>Board address</span><input type="text" id="s_host" placeholder="192.168.1.50"></div>
  <div class="field" data-k="s_port"><span>Board port</span><input type="number" id="s_port"></div>
  <div class="field" data-k="s_auto"><span>Auto-find</span><label class="chk"><input type="checkbox" id="s_auto"><span>Look for the board if its address changes</span></label></div>
  <div class="field" data-k="s_subnet"><span>Search network</span><input type="text" id="s_subnet" placeholder="auto (e.g. 192.168.1)"></div>
  <div class="field" data-k="s_motors"><span>Motors in use</span><select id="s_motors"><option value="1">1 motor</option><option value="2">2 motors</option><option value="3">3 motors</option></select></div>
  <div class="field" data-k="s_unused"><span>Unused motors</span><label class="chk"><input type="checkbox" id="s_unused"><span>Switch them off on the board too</span></label></div>
  <div class="field" data-k="theme"><span>Theme</span><select id="s_theme"><option value="dark">Dark</option><option value="light">Day</option><option value="auto">Auto (follow system)</option></select></div>
  <div style="display:flex;gap:10px;flex-wrap:wrap;margin-top:14px">
   <button class="btn b-blue" data-k="s_save" onclick="saveSettings()">Save and connect</button>
   <button class="btn b-grey" data-k="s_scan" id="scanBtn" onclick="scan()">Find board on network</button>
  </div>
  <div class="msg" id="s_msg"></div>
 </section>

 <!-- motor names -->
 <section class="card" style="grid-area:mnames" data-views="settings">
  <div class="ct"><svg class="i" viewBox="0 0 24 24"><path d="M20.6 13.4l-7.2 7.2a2 2 0 0 1-2.8 0L3 13V3h10l7.6 7.6a2 2 0 0 1 0 2.8z"/><circle cx="7.5" cy="7.5" r="1.5"/></svg>Motor names</div>
  <p class="note">Call each motor by where it sits on your rig. The names show on the motor cards, graphs, PID table, menus, auto-tune and diagnostics. The number stays next to them: the wiring, SimHub's outputs and the LED matrix (E1-E3) still go by number.</p>
  <div id="nrows"></div>
  <div style="display:flex;gap:10px;flex-wrap:wrap;align-items:center;margin-top:12px">
   <button class="btn b-blue" data-k="n_save" onclick="saveNames()">Save names</button>
   <span class="muted" style="font-size:13px">Not sure which is which? The Diagnostics full check asks where each motor is.</span>
  </div>
 </section>

 <!-- diagnostics -->
 <section class="card wide" id="diagcard" style="grid-area:diag" data-views="settings">
  <div class="ct"><svg class="i" viewBox="0 0 24 24"><path d="M3 12h4l2-5 4 10 2-5h6"/><circle cx="20" cy="12" r="1.2"/></svg>Diagnostics</div>
  <div class="dgrid">
   <div>
    <p class="note">Something not right - a motor going the wrong way, sudden jerks, a motor that won't move? Describe it, run a check, and save the file for whoever helps you (you can give it to Claude). It holds your settings, measurements, what the page noticed while you drove and the event log - no passwords.</p>
    <span class="lbl" data-k="dg_problem">What's wrong? (optional - goes into the file)</span>
    <textarea id="dg_problem" rows="3" maxlength="2000" placeholder="e.g. Motor 2 slams to the end as soon as SimHub starts"></textarea>
    <div class="dg-acts">
     <button class="btn b-blue" id="dg_quick" data-k="dg_quick" onclick="diagStart('quick')"><svg class="i" viewBox="0 0 24 24"><circle cx="11" cy="11" r="7"/><path d="M21 21l-5-5"/></svg>Quick check</button>
     <button class="btn b-orange" id="dg_full" data-k="dg_full" onclick="openModal('mDiag')"><svg class="i" viewBox="0 0 24 24"><path d="M12 3v18M5 10l7-7 7 7"/></svg>Full check (moves motors)</button>
     <button class="btn b-grey" id="dg_stop" data-k="dg_stop" onclick="post('/diag/abort')" hidden>Stop</button>
     <label class="chk" data-k="dg_ask"><input type="checkbox" id="dg_ask" checked><span>Ask me which way each motor moved</span></label>
    </div>
    <div id="dg_run" hidden><div class="bar"><div id="dg_bar"></div></div><div class="muted" id="dg_stage"></div><div class="dlive" id="dg_live"></div></div>
    <div id="dg_prompt" class="dprompt" hidden></div>
    <div id="dg_result"></div>
   </div>
   <div class="dside">
    <div class="qsub" data-k="dg_inc">Noticed while running <small id="dg_inc_n"></small><button class="linkbtn" data-k="dg_clear" onclick="clearIncidents()">Clear</button></div>
    <div id="dg_inc" class="dlist"></div>
    <div class="qsub" data-k="dg_reports">Saved reports</div>
    <div id="dg_reports" class="dlist"></div>
    <details class="evd" id="dg_evd" ontoggle="if(this.open)loadEvents()"><summary data-k="dg_events">Event log</summary><div id="dg_events" class="evlog"></div></details>
   </div>
  </div>
 </section>

 <!-- help -->
 <section class="card help" style="grid-area:help" data-views="help">
  <div class="ct">Help</div>
  <div class="cols">
   <div><h3>Every control explains itself</h3><p>Hold the mouse over any setting, button, legend entry or status item for a short note on what it does on the rig, what too low and too high feel like, and where to start.</p>
    <h3>Tuning by hand</h3><ol><li>Stop SimHub's motion output.</li><li>Set Ki 0, Kd 0, PWMmin 10-15, dead zone 2-4 and press Apply.</li><li>Run a Square test on one motor and raise Kp until the green line just stops overshooting the blue one.</li><li>Add a little Ki (5-20) only if the seat settles short of the target.</li><li>Save to EEPROM.</li></ol></div>
   <div><h3>Auto-tune</h3><p>Measures friction (PWMmin) and pot noise (dead zone), raises Kp until it stops tracking any better, checks small road-sized moves, then adds Ki and Kd only if they help. It stops by itself and restores the previous settings if SimHub moves the rig or anything looks wrong. Nothing is saved until you press Save to EEPROM.</p>
    <h3>Profiles and recommendations</h3><p>Save the settings that suit a car or game under a name and load them later. The recommendations (circuit, rally, flight, space, trucks) adjust your own tuned settings for that kind of sim and list the SimHub settings that matter most for it. Loading applies live; save to EEPROM to make it the power-on default.</p>
    <h3>Backups</h3><p>Every Save to EEPROM, and every bigger change, keeps a copy on the NAS (EEPROM section). Switch back to any of them live, or switch and save in one go.</p></div>
   <div><h3>LED matrix on the board</h3><ul><li>Boot: R4, then the IP address scrolls.</li><li>E0 WiFi module not answering, E9 can't join the network, E8 no address from the router yet.</li><li>E1 / E2 / E3: that motor is off - its safety cutoff tripped or you pressed Disable Motors. Enable Motors re-arms it.</li><li>Bottom row: commands arriving, timeout, motors on, loop overrun, IMU, WiFi, heartbeat.</li></ul>
    <h3>Motor names</h3><p>Settings, Motor names: call the motors Left, Right, Rear (or anything you like). The names appear everywhere in the page and in diagnostics files; the number stays alongside because the wiring, SimHub and the LED matrix use numbers.</p>
    <h3>Selected motor</h3><p>Click a motor card or chart, or choose a motor in Test Signal or Auto-Tune: the same motor is highlighted everywhere. While a test signal or auto-tune runs, its chart is marked and the others fade.</p>
    <h3>Diagnostics</h3><p>Settings section. <b>Quick check</b> reads the board and listens for a few seconds; <b>Full check</b> also moves each motor a little to see which way it goes, how much drive it needs, whether the wiring is swapped and how it answers a small step - and can ask you what you saw. Both make one file (saved on the NAS too) to download and give to whoever helps you. The page also watches while you drive and notes pot glitches, a motor moving the wrong way or stalling, hunting, cutoffs and board restarts.</p>
    <h3>Status bar</h3><p>LOW POWER means no position commands for 15 s, so the board runs at quarter drive until SimHub or this page sends targets again.</p></div>
  </div>
 </section>
</main>
</div>

<div class="modal" id="mTune"><div class="box">
 <h2>Before auto-tune on <span id="mt_n"></span></h2>
 <ul>
  <li>SimHub's motion output is <b>stopped</b> (or the game is closed). If SimHub moves the rig, the tune stops.</li>
  <li>The rig is clear of hands, feet, cables and pets. The motor makes repeated moves of ±<span id="mt_a"></span> counts.</li>
  <li>Your hand is near the <b>motor power switch</b>.</li>
  <li>For final values, <b>sit in the seat</b> in your normal position - your weight changes how the motors behave. A first run with the seat empty is a good idea.</li>
  <li>Takes about 2-3 minutes. Nothing is saved until you press <b>Save to EEPROM</b>; <b>Revert</b> undoes it.</li>
 </ul>
 <div style="display:flex;gap:10px;margin-top:12px"><button class="btn b-green" onclick="tuneStart()">Start auto-tune</button><button class="btn b-grey" onclick="closeModal('mTune')">Cancel</button></div>
</div></div>

<div class="modal" id="mDiag"><div class="box">
 <h2>Before the full check</h2>
 <ul>
  <li>SimHub's motion output is <b>stopped</b> (or the game is closed). If something else moves the rig, the check stops.</li>
  <li><b>Seat empty</b>, the rig clear of hands, feet, cables and pets, your hand near the <b>motor power switch</b>.</li>
  <li>One motor at a time moves a little - nudges of 25 counts, then a step of 50 and back. The other motors are switched off meanwhile and come back on afterwards.</li>
  <li>A motor that is off is switched on for its test. If one moves the wrong way or swaps with another, it is stopped at once and left off.</li>
  <li>About a minute (plus your answers). Your settings are put back afterwards; nothing is saved to EEPROM.</li>
 </ul>
 <div style="display:flex;gap:10px;margin-top:12px"><button class="btn b-orange" onclick="diagStart('full')">Start the full check</button><button class="btn b-grey" onclick="closeModal('mDiag')">Cancel</button></div>
</div></div>

<script>
const TIPS={
 kp:"<b>Kp - the spring.</b> How hard the motor pushes for each count it is away from where SimHub wants it (SMC3Utils units: 128 ≈ 1 PWM per count of error).<br><br><b>Too low:</b> lazy, lags behind the game, sags under your weight.<br><b>Too high:</b> overshoots, shakes or buzzes at rest, every bump feels harsh.<br><br><b>Start:</b> 150-300, then raise until the green line just stops overshooting the blue one in a step test. Auto-tune finds this for you.",
 ki:"<b>Ki - patience.</b> Slowly adds push while the seat sits a little off target - typically sagging under your weight on the lifting side - until it gets there. Ki 40 adds about 20 PWM per second for every count still missing. Resets inside the dead zone.<br><br><b>Too high:</b> slow wobble, overshoot after big moves, a floaty feel.<br><br><b>Start:</b> 0. Add 5-20 only after Kp is set, if the green line settles a few counts short of the blue one.",
 kd:"<b>Kd - the shock absorber.</b> Resists the motor's speed so fast moves stop without overshooting. Uses the measured speed (sampled every Ks cycles), so it doesn't kick when the target jumps.<br><br><b>Too high:</b> turns pot noise into a gritty, buzzing feel.<br><br><b>Start:</b> 0. If steps overshoot and lowering Kp makes it lazy, try Ks 4 with Kd 5-20.",
 ks:"<b>Ks - speed sample spacing for Kd.</b> How many control cycles (¼ ms each at Fpid/1) between the speed readings the damping uses.<br><br>Bigger Ks = a cleaner, larger speed reading, so the same Kd damps more, but a little later. At Ks 1 (SMC3 default) Kd barely does anything at normal speeds.<br><br><b>Start:</b> 1 when Kd is 0; 4-8 when you use Kd.",
 pwmmin:"<b>PWMmin - the nudge.</b> Added on top of the PID output whenever the motor needs to move, so small corrections overcome gearbox friction.<br><br><b>Too low:</b> small movements get swallowed; flat spots when the seat changes direction.<br><b>Too high:</b> a knock at every reversal and a hum while holding.<br><br><b>Start:</b> 10-15 (worm-geared wiper motors often need more, planetary gearboxes less). Auto-tune measures it.",
 pwmmax:"<b>PWMmax - speed and torque cap.</b> The most drive the motor can ever get, as a share of your motor supply (255 = 100%, 100 ≈ 40%). Also your main safety valve.<br><br><b>Too low:</b> big moves (crests, hard braking) feel slow and clipped.<br><b>Too high:</b> violent if anything goes wrong.<br><br><b>Start:</b> about 100; raise in steps of 10-20 only if large movements lag.",
 pwmrev:"<b>PWMrev - recovery push.</b> Drive used when the pot reads outside the Clip window (the seat overshot past its allowed travel), pushing it back inside. 0 = off.<br><br><b>Too high:</b> slams the seat back from the end of travel.<br><br><b>Start:</b> something moderate, about 100-150. It should rarely fire if Clip and the gains are right.",
 dz:"<b>Dead zone - the hold window</b>, in counts (0-1023 scale). When the pot is this close to the target the motor brakes instead of driving, and Ki's sum resets. Stops hunting and buzzing at rest.<br><br><b>Too big:</b> sloppy, the seat moves in visible steps.<br><br><b>Start:</b> 2-4. Auto-tune sets it from the measured pot noise.",
 lim:"<b>Max limit - safety cutoff</b>, in counts from each end of the pot range (23 = trips below 23 or above 1000).<br><br>If the pot ever reads beyond it - broken wire, slipped linkage, reversed motor - that motor shuts off and the matrix shows E1/E2/E3 until you press <b>Enable Motors</b>.<br><br>Must be smaller than Clip. Set it just outside the travel your mechanism can physically reach.",
 clip:"<b>Clip - usable travel window</b>, in counts from each end (100 = targets kept between 100 and 923). SimHub can never command the seat outside it, and PWMrev pushes back if it drifts past.<br><br>This is your range of motion: widen it carefully, watching for mechanical stops, to get more travel.",
 fpid:"<b>Fpid - control loop divider.</b> 1 = the PID runs 4000 times a second (most responsive); 2 = 2000.<br><br>Only raise it if <b>ISR max</b> climbs above ~200 µs. Changing it scales how strong Ki and Kd are, so re-check tuning afterwards.",
 pwmfreq:"<b>PWM switching frequency</b> for motors 1+2 and for motor 3.<br><br>25 kHz is above hearing so the motors don't whine; lower values whine audibly. The IBT-2's BTS7960 chips are rated up to 25 kHz - don't go higher.",
 apply:"Sends this motor's column to the board <b>immediately</b>. Takes effect live, but is lost at power-off until you <b>Save to EEPROM</b>.",
 applyGlobal:"Sends the loop divider and PWM frequencies to the board now (not saved until Save to EEPROM).",
 read:"Loads the values currently running on the board into the table.",
 ena:"Re-arms all motors - after a safety cutoff (E1/E2/E3 on the matrix) or after Disable Motors.",
 dis:"<b>Software stop.</b> All three motors stop driving at once and stay off until you press Enable Motors. Needs firmware 1.5.",
 sav:"Stores everything currently running on the board so it survives a power cycle, and keeps a backup of it on the NAS (EEPROM section).",
 gmotor:"Which motor the test signal drives. The other two hold their last position.",
 gmode:"<b>Sine</b> - smooth back and forth (tracking and lag).<br><b>Square</b> - instant jumps (overshoot and settling - the best test for Kp).<br><b>Triangle</b> - constant-speed moves (PWMmin flat spots at reversals).<br><b>Hold</b> - go to the Offset position and stay.",
 gamp:"How far either side of the Offset the test moves, in counts. 100-200 is plenty; stay inside the Clip window.",
 gper:"Seconds per full cycle. 2 s is a calm test; 0.5 s is closer to a bumpy road.",
 goff:"Centre of the test movement (0-1023, middle 512). Also the position used by Hold.",
 gstart:"Starts the test signal on the chosen motor (stop SimHub's motion output first).",
 gstop:"Stops sending test positions. The motor holds where it is.",
 p_sine:"Sine, ±150 around the Offset, 2 s cycle - a calm tracking test.",
 p_square:"Square, ±100, 2 s cycle - shows overshoot and settling. Best for tuning Kp.",
 p_tri:"Triangle, ±150, 3 s cycle - constant speed, shows PWMmin flat spots when direction changes.",
 p_step:"One step: moves to Offset + Amplitude and holds there.",
 p_manual:"Moves to the Offset and holds there.",
 tune:"<b>Auto-tune</b> finds settings for one motor by testing it: friction (PWMmin), pot noise (dead zone), then Kp from low to high until it stops tracking better, then Ki and Kd only if they help. Applied live; nothing is saved until you press Save to EEPROM. Needs firmware 1.4 or newer.",
 tmotor:"Motor to tune. Tune them one at a time.",
 tamp:"Size of the test moves, in counts either side of centre. Bigger moves give a more realistic tune but move you more. 80-150 is sensible; it is reduced automatically if the Clip window is narrow.",
 tstyle:"How the sweet spot is picked: the <b>lowest</b> Kp that tracks almost as well as the best one.<br><br><b>Gentle</b> - smoothest (within 30% of the best response, ~5% overshoot).<br><b>Normal</b> - balanced (15%, ~10%).<br><b>Firm</b> - snappiest, more road detail (5%, ~15%).<br><br>Whatever the style, it never leaves the seat more than 12 counts short.",
 tfr:"Measure the minimum drive that moves the motor (PWMmin) and the pot noise (dead zone). Leave on unless you want to keep your own values.",
 tki:"Add Ki only if the motor settles short of the target (usually lifting your weight).",
 tkd:"Try a little damping (Kd with Ks 4) and keep it only if it clearly reduces overshoot without adding noise.",
 tstart:"Shows a short checklist, then starts. Takes about 2-3 minutes.",
 tstop:"Stops the auto-tune at once and puts back the settings the motor had before.",
 tgear:"Opens the Auto-Tune section with the full log and results.",
 trevert:"Puts back the settings this motor had before the auto-tune.",
 pause:"Freezes the charts so you can look at a moment. Data keeps recording; press again to continue.",
 expand:"Opens the charts full width (Live Graphs section).",
 collapse:"Back to the layout you came from.",
 chartsel:"Click to select this motor - the test signal, auto-tune and PID column follow the selection.",
 smooth:"<b>Target smoothing</b> - how softly the motor eases into each new position SimHub sends (a low-pass filter on the target).<br><br><b>Off / 4 ms:</b> raw, every bump and kerb comes through - rally.<br><b>8-16 ms:</b> crisp but not jagged - circuit racing.<br><b>32-64 ms:</b> soft and floaty - flight, space, trucks.<br><br>Times are at Fpid/1 (double them at Fpid/2). Needs firmware 1.6.",
 maxrate:"<b>Max speed</b> - the fastest the target may move, in counts per second (full travel is about 820 counts). 0 = no limit.<br><br>A cap like 1500-2000 stops sudden lurches (crashes, game resets) and makes flight feel calmer. Needs firmware 1.6.",
 applyFilt:"Sends smoothing and max speed to the board now (not saved until Save to EEPROM).",
 g_base:"The settings the recommendation starts from: what's running on the board now, or one of your saved profiles.",
 genre:"Shows what this recommendation would change, and the SimHub settings that matter for it. Nothing is applied until you press Apply.",
 g_apply:"Sends these settings to the board now. A backup of the current settings is kept first (EEPROM section).",
 g_save:"Stores these settings as a profile without changing the board.",
 bk_label:"Optional name, so you recognise this backup later.",
 bk_now:"Reads the settings running on the board and keeps them as a backup on the NAS.",
 b_switch:"Sends this backup's settings to the board now (live). The current settings are backed up first.",
 b_switchsave:"Sends this backup to the board and saves it to EEPROM, so it is also the power-on setting.",
 b_dl:"Downloads this backup as a file.",
 b_del:"Removes this backup from the NAS.",
 s_motors:"How many motors your rig has. Unused motors are hidden from the cards, charts, PID table and motor menus.",
 s_unused:"Also switch the unused motors off on the board (as if Disable Motors was pressed for them), so an unconnected motor channel never drives. Needs firmware 1.5 or newer.",
 win:"How many seconds of history the charts show.",
 lg_t:"<b>Target</b> - the raw position SimHub (or the test signal) asks for.",
 lg_g:"<b>Goal</b> - the target after the firmware's smoothing filter. This is what the motor actually chases.",
 lg_f:"<b>Feedback</b> - where the pot says the motor really is. Good tuning = green sits on blue.",
 lg_p:"<b>PID output</b> before PWMmin is added. Middle line = 0; top/bottom = full ±255. Pinned at the edge means the motor is giving everything it is allowed.",
 mpos:"<b>Position</b> - where the motor is being sent (the smoothed goal, 0-1023). The green tick on the bar is where it really is.",
 mstate:"<b>Online</b> = enabled and reporting. <b>Off</b> = stopped by its safety cutoff or Disable Motors - press Enable Motors.",
 pr_name:"Any name you like - a car, a game, a feel. Saving with an existing name replaces it.",
 pr_save:"Reads every setting running on the board now and stores it on the NAS under this name.",
 pr_load:"Sends this profile's settings to the board now (live, not saved to EEPROM).",
 pr_del:"Removes this profile from the NAS. The board is not changed.",
 bk_dl:"Saves the settings running on the board as a file on this computer.",
 bk_up:"Loads a settings file and sends it to the board (live - press Save to EEPROM to keep it). The current settings are backed up first.",
 th_day:"Light work area for daytime.",th_dark:"Dark look for a dim room.",th_auto:"Follows your computer's light/dark setting.",
 theme:"Day, Dark, or follow your computer's setting. Saved on the NAS, so every browser gets the same.",
 settingsBtn:"Board address, port, auto-find and theme.",
 s_host:"IP address (or name) of the UNO R4. It scrolls across the LED matrix at boot.",
 s_port:"TCP port the board listens on - TELEMETRY_PORT in the firmware (4210).",
 s_auto:"If the board can't be reached for a minute, search the network for it (then every 10 minutes) and switch to the new address automatically.",
 s_subnet:"First three numbers of your network, e.g. 192.168.1. Leave empty to use the board address's network.",
 s_save:"Save these settings on the NAS and reconnect.",
 s_scan:"Search every address on the network for an SMC3 board (about 5 seconds).",
 st_conn:"Whether this page is receiving live data from the board.",
 st_fw:"Firmware version on the board. Auto-tune needs 1.4 or newer, Disable Motors 1.5.",
 st_isr:"Longest time the 4 kHz control interrupt took recently. Budget 250 µs; under ~200 is healthy.",
 st_div:"Control loop divider: PID runs at 4000 / this many times a second.",
 st_low:"No position commands for 15 s, so the board drops the drive to a quarter (like the original SMC3). Clears as soon as SimHub or this page sends targets.",
 st_full:"Position commands are arriving, so the motors run at full drive.",
 sy_board:"Live data from the board over WiFi.",sy_motors:"How many motors are enabled.",sy_pid:"The control loop is running and reporting.",
 sy_power:"Full drive, or low power after 15 s without position commands.",sy_input:"Whether the targets are changing - i.e. SimHub (or a test) is moving the rig.",
 sy_err:"Safety cutoffs, a slow control loop or a lost connection show up here - and problems the page noticed while you drove (click to see them).",
 dg_problem:"Describe what goes wrong, in your own words: which motor, when (starting SimHub, over kerbs, after a while...), what it does. It goes at the top of the file, so whoever reads it knows what to look for.",
 dg_quick:"<b>Quick check</b> - nothing moves. Reads every setting, the board's own counters (firmware 1.7), listens to the motors for 3 seconds, and adds what the page noticed while you drove. About 5 seconds; safe while SimHub runs.",
 dg_full:"<b>Full check</b> - moves each motor a little, one at a time: which way it goes, how much drive it needs each way, whether another motor moves instead (swapped wiring), and how it answers a small step. Stop SimHub's motion output first. About a minute.",
 dg_stop:"Stops the check at once. Motors are put back as they were; a report with what was measured so far is still made.",
 dg_ask:"During the full check, each motor nudges towards higher numbers and the page asks which way its corner of the seat went - the one thing sensors can't tell (is the motion upside down in games?).",
 dg_inc:"Problems the page noticed by itself while watching the live data - also while you drive. Each keeps a few seconds of data from before and after, and goes into the diagnostics file.",
 dg_clear:"Forgets the recorded problems (the event log keeps a line about each).",
 dg_reports:"Every check is saved on the NAS (the newest 20). Download one to send it, or view its results again.",
 dg_events:"Everything that happened, newest first: connections, restarts, settings changes, tests, problems.",
 dg_dl:"Saves the diagnostics file (.json) on this computer. Upload it to Claude or send it to whoever helps - it is self-describing.",
 dg_copy:"Copies a plain-text summary (findings, what to do, the settings) to paste into a chat or message.",
 navbadge:"Problems the page noticed while you drove - see Settings, Diagnostics.",
 n_name:"Where this motor sits - pick one, or Custom to type your own (up to 16 letters). Not named = shown as Motor 1, 2, 3.",
 n_save:"Saves the names on the NAS, so every browser shows them. Nothing changes on the board."
};
const ROWS=[['kp','Kp','Kp',1],['ki','Ki','Ki',1],['kd','Kd','Kd',1],['ks','Ks','Ks',1],['pwmmin','PWMmin','PWMmin'],['pwmmax','PWMmax','PWMmax'],['pwmrev','PWMrev','PWMrev'],['dz','Dead zone','DZ'],['lim','Max limit','LIM'],['clip','Clip','CLIP']];
const COLOR=['b-blue','b-orange','b-green'];
let lastSeq=0,data=[],params={},misses=0,tuneRunning=false,cfg={},paused=false,fwnum=0,lastMove=0,lastT=null,connected=false;
let NM=3,sel=0,prevView='dashboard',curView='dashboard',genMotor=null,tuneMotor=null,genres=[],gpv=null;
let MNAMES=['','',''];
const NAMEOPTS=['Left','Right','Rear','Front','Front left','Front right','Rear left','Rear right','Middle'];
let diagMotor=null,diagBusy=false,diagState='idle',diagRes=null,statusRes=null,dgPromptId=null,watch={count:0,newest:0},waitToast=0;
try{sel=+(localStorage.getItem('smc3_sel')||0)||0;}catch(e){}
const PKEYS=[1,2,3].flatMap(n=>['Kp','Ki','Kd','Ks','PWM','LIM','DZ'].map(k=>k+n)).concat(['STATUS','PWMFREQ','FILT']);
function toast(msg,err){const t=$('toast');t.innerHTML=msg;t.className=err?'err':'';t.style.display='block';clearTimeout(toast.h);toast.h=setTimeout(()=>t.style.display='none',err?7000:4000);}
const $=id=>document.getElementById(id);
// motor labels: the name the owner gave it (Settings, Motor names), always with the number nearby
const mname=m=>MNAMES[m]||'';
const mtitle=m=>mname(m)||'Motor '+(m+1);                                // motor card title (the badge shows the number)
const mtab=m=>mname(m)?(m+1)+' · '+mname(m):'Motor '+(m+1);               // chart tabs, menus, headings
const mshort=m=>mname(m)||'M'+(m+1);                                      // tight places: tags, buttons
const mfull=m=>mname(m)?`Motor ${m+1} (${mname(m)})`:`Motor ${m+1}`;      // sentences
function nameify(t){if(!t||!MNAMES.some(x=>x))return t;const tag=i=>MNAMES[i-1]?` (${MNAMES[i-1]})`:'';
 t=String(t).replace(/\b([Mm])otors ([123]) and ([123])\b(?! \()/g,(a,c,x,y)=>`${c}otors ${x}${tag(+x)} and ${y}${tag(+y)}`);
 return t.replace(/\b([Mm])otor ([123])\b(?! \()/g,(a,c,x)=>a+tag(+x));}
const escn=t=>esc(nameify(t));
const esc=s=>String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
async function post(url,obj){const r=await fetch(url,{method:'POST',body:JSON.stringify(obj||{})});return r.json()}
function txt(t){return post('/cmd',{text:t})}
function closeModal(id){$(id).style.display='none'}
function openModal(id){$(id).style.display='flex'}

// ---------- build repeated parts ----------
function buildMotorCards(){document.querySelectorAll('.motor').forEach(c=>{const m=+c.dataset.m;c.onclick=e=>{if(!e.target.closest('button,a,input,select'))setSel(m);};
 c.innerHTML=`<div class="mh"><span class="badge">${m+1}</span><b class="mtitle" id="mn${m}">Motor ${m+1}</b><span class="state nodata" data-k="mstate" id="ms${m}"><i class="dot"></i><span>No data</span></span></div>
 <img class="mimg" src="/img/motor.png" alt="">
 <div class="mpos" data-k="mpos"><span>Position</span><b id="mp${m}">-</b></div>
 <div class="track"><div class="fill" id="mf${m}"></div><div class="fb" id="mt${m}"></div><div class="knob" id="mk${m}"></div></div>
 <div class="scale"><span>0</span><span>1023</span></div>
 <div class="rows"><div data-k="lg_f"><span>Feedback (fb)</span><b id="mfb${m}">-</b></div><div><span>Error (err)</span><b id="me${m}">-</b></div><div data-k="lg_p"><span>PID Out</span><b id="mo${m}">-</b></div></div>`;});}
function buildCharts(){let h='';for(let m=0;m<3;m++)h+=`<div class="chart" data-m="${m}" data-k="chartsel" onclick="setSel(${m})"><span class="tab"><span id="ct${m}">Motor ${m+1}</span><em id="tag${m}"></em></span><canvas id="c${m}"></canvas><div class="ro"><span>goal</span><b id="rg${m}">-</b><span>fb</span><b id="rf${m}">-</b><span>err</span><b id="re${m}">-</b><span>pid</span><b id="rp${m}">-</b></div></div>`;$('charts').innerHTML=h;}
function buildPid(n){n=n||3;let h='<colgroup><col class="l">'+'<col>'.repeat(n)+'</colgroup><tr><th></th>'+[0,1,2].slice(0,n).map(m=>`<th class="h${m} c${m}" title="${esc(mfull(m))}">${mname(m)?(m+1)+' '+esc(mname(m)):'M'+(m+1)}</th>`).join('')+'</tr>';
 for(const [k,label,id] of ROWS){h+=`<tr data-k="${k}"><td>${label}</td>`;for(let m=1;m<=n;m++)h+=`<td class="c${m-1}"><input type="number" id="${id}${m}"></td>`;h+='</tr>';}
 h+='<tr><td></td>'+[1,2,3].slice(0,n).map(m=>`<td class="c${m-1}"><button class="btn ${COLOR[m-1]} ap" data-k="apply" onclick="apply(${m})" title="Apply ${esc(mfull(m-1))}"><span class="bl">Apply ${esc(mshort(m-1))}</span></button></td>`).join('')+'</tr>';
 $('pidtable').innerHTML=h;tipAttach($('pidtable'));}
buildMotorCards();buildCharts();

// ---------- tooltips ----------
function tipAttach(root){(root||document).querySelectorAll('[data-k]').forEach(el=>{const t=TIPS[el.dataset.k];if(t)el.dataset.tip=t;});}
const tipEl=$('tip');
function placeTip(e){const p=14;let x=e.clientX+p,y=e.clientY+p;const w=tipEl.offsetWidth,h=tipEl.offsetHeight;
 if(x+w>innerWidth-8)x=e.clientX-w-p;if(y+h>innerHeight-8)y=e.clientY-h-p;tipEl.style.left=Math.max(8,x)+'px';tipEl.style.top=Math.max(8,y)+'px';}
document.addEventListener('mouseover',e=>{const el=e.target.closest('[data-tip]');if(!el){tipEl.style.display='none';return;}
 tipEl.innerHTML=el.dataset.tip;tipEl.style.display='block';placeTip(e);});
document.addEventListener('mousemove',e=>{if(tipEl.style.display==='block')placeTip(e);});

// ---------- selected motor and motor count ----------
function setSel(m){m=Math.max(0,Math.min(NM-1,m|0));sel=m;try{localStorage.setItem('smc3_sel',m);}catch(e){}
 $('gm').value=m;if(!tuneRunning)$('tm').value=m;updateSelUI();}
function updateSelUI(){const busy=genMotor!=null||tuneMotor!=null||diagMotor!=null;
 for(let m=0;m<3;m++){const ch=document.querySelector(`.chart[data-m="${m}"]`);const on=m===sel;ch.classList.toggle('sel',on);
  $('tag'+m).textContent=tuneMotor===m?'Auto-tuning':(diagMotor===m?'Checking':(genMotor===m?'Test signal':(on?'Selected':'')));
  document.querySelector(`.motor[data-m="${m}"]`).classList.toggle('sel',on);}
 $('charts').classList.toggle('focus',busy);
 document.querySelectorAll('#pidtable th[class*=c],#pidtable td[class*=c]').forEach(c=>c.classList.toggle('sel',c.classList.contains('c'+sel)));}
function applyMotorCount(){NM=Math.max(1,Math.min(3,+cfg.motors||3));
 $('mrow').style.setProperty('--n',NM);
 document.querySelectorAll('.motor').forEach(c=>c.hidden=+c.dataset.m>=NM);
 document.querySelectorAll('.chart').forEach(c=>c.hidden=+c.dataset.m>=NM);
 const opts=[0,1,2].slice(0,NM).map(m=>`<option value="${m}">${esc(mname(m)?(m+1)+' '+mname(m):String(m+1))}</option>`).join('');
 for(let m=0;m<3;m++){$('mn'+m).innerHTML=esc(mtitle(m))+(mname(m)?`<span class="num">motor ${m+1}</span>`:'');$('ct'+m).textContent=mtab(m);}
 document.querySelectorAll('.msel').forEach(e=>e.innerHTML=opts);
 buildPid(NM);readParams();setSel(Math.min(sel,NM-1));chartHeight();}
function chartHeight(){const base={dashboard:108,graphs:230}[curView]||150;const h=NM===3?base:Math.round((base+21)*3/NM-21);
 $('charts').style.setProperty('--ch',h+'px');draw();}
function toggleExpand(){location.hash=curView==='graphs'?(prevView||'dashboard'):'graphs';}

// ---------- views ----------
const VIEWS=['dashboard','tuning','motors','graphs','tools','profiles','autotune','eeprom','settings','help'];
function showView(){let v=location.hash.slice(1);if(!VIEWS.includes(v))v='dashboard';
 if(v!=='graphs')prevView=v; curView=v;
 $('main').dataset.view=v;
 document.querySelectorAll('#main > [data-views]').forEach(c=>c.hidden=!c.dataset.views.split(' ').includes(v));
 document.querySelectorAll('#nav a').forEach(a=>a.classList.toggle('on',a.getAttribute('href')==='#'+v));
 const eb=$('expBtn');eb.dataset.k=v==='graphs'?'collapse':'expand';eb.dataset.tip=TIPS[eb.dataset.k];
 eb.innerHTML=v==='graphs'?'<svg class="i" viewBox="0 0 24 24"><path d="M4 14h6v6M20 10h-6V4M14 10l7-7M3 21l7-7"/></svg>':'<svg class="i" viewBox="0 0 24 24"><path d="M15 3h6v6M9 21H3v-6M21 3l-7 7M3 21l7-7"/></svg>';
 if(v==='profiles'){loadProfiles();loadGenres();} if(v==='settings'){fillSettings();diagLists();} if(v==='eeprom')loadBackups();
 chartHeight();}
window.addEventListener('hashchange',showView);

// ---------- theme ----------
const mq=matchMedia('(prefers-color-scheme: light)');
function applyTheme(t){const eff=t==='auto'?(mq.matches?'light':'dark'):t;document.documentElement.dataset.theme=eff;
 document.querySelectorAll('[data-theme-set]').forEach(b=>b.classList.toggle('on',b.dataset.themeSet===t));draw();}
mq.addEventListener('change',()=>applyTheme(cfg.theme||'dark'));
document.querySelectorAll('[data-theme-set]').forEach(b=>b.onclick=async()=>{cfg.theme=b.dataset.themeSet;applyTheme(cfg.theme);cfg=await post('/config',{theme:cfg.theme});});
async function loadConfig(){cfg=await (await fetch('/config')).json();MNAMES=(cfg.motor_names||['','','']).slice(0,3);applyTheme(cfg.theme||'dark');applyMotorCount();}

// ---------- settings ----------
function fillSettings(){$('s_host').value=cfg.host||'';$('s_port').value=cfg.port||4210;$('s_auto').checked=!!cfg.auto_find;$('s_subnet').value=cfg.scan_subnet||'';$('s_theme').value=cfg.theme||'dark';
 $('s_motors').value=cfg.motors||3;$('s_unused').checked=cfg.disable_unused!==false;fillNames();
 $('s_msg').innerHTML=cfg.writable===false?'<span class="badc">Settings can\'t be saved - '+esc(cfg.path)+' is not writable.</span>':'';}
async function saveSettings(){const before=cfg.motors;cfg=await post('/config',{host:$('s_host').value.trim(),port:+$('s_port').value||4210,auto_find:$('s_auto').checked,scan_subnet:$('s_subnet').value.trim(),theme:$('s_theme').value,motors:+$('s_motors').value,disable_unused:$('s_unused').checked});
 applyTheme(cfg.theme);if(cfg.motors!==before){applyMotorCount();fillNames();}$('s_msg').innerHTML='<span class="okc">Saved.</span> Board: '+esc(cfg.host)+', '+cfg.motors+' motor'+(cfg.motors>1?'s':'')+' in use.';}
async function scan(){$('scanBtn').disabled=true;$('s_msg').textContent='Searching the network…';
 try{const r=await post('/scan',{subnet:$('s_subnet').value.trim()});
  if(r.busy)$('s_msg').textContent='A search is already running.';
  else if(r.connected)$('s_msg').innerHTML='<span class="okc">Already connected to '+esc(r.current)+'.</span>'+(r.found.length?' Also found: '+r.found.map(esc).join(', '):'');
  else if(!r.found.length)$('s_msg').innerHTML='<span class="badc">No board answered on '+esc(r.subnet)+'.x</span> - is it powered and on WiFi? Its address scrolls on the LED matrix.';
  else $('s_msg').innerHTML='Found: '+r.found.map(h=>'<button class="btn b-blue" onclick="useHost(\''+esc(h)+'\')">Use '+esc(h)+'</button>').join(' ');
 }finally{$('scanBtn').disabled=false;}}
function useHost(h){$('s_host').value=h;saveSettings();}
function fillNames(){let h='';
 for(let m=0;m<3;m++){const v=MNAMES[m]||'',preset=!v||NAMEOPTS.includes(v);
  h+=`<div class="nrow" data-m="${m}"${m>=NM?' hidden':''}><span class="badge">${m+1}</span><select id="nsel${m}" data-k="n_name" onchange="nameSel(${m})"><option value="">Not named (Motor ${m+1})</option>`+
   NAMEOPTS.map(o=>`<option${o===v?' selected':''}>${o}</option>`).join('')+`<option value="*"${preset?'':' selected'}>Custom…</option></select>`+
   `<input type="text" id="ntxt${m}" maxlength="16" placeholder="e.g. Seat back" value="${preset?'':esc(v)}"${preset?' hidden':''}></div>`;}
 $('nrows').innerHTML=h;tipAttach($('nrows'));}
function nameSel(m){const c=$('nsel'+m).value==='*';$('ntxt'+m).hidden=!c;if(c)$('ntxt'+m).focus();}
async function saveNames(){const names=[0,1,2].map(m=>{const v=$('nsel'+m).value;return v==='*'?$('ntxt'+m).value.trim().slice(0,16):v;});
 cfg=await post('/config',{motor_names:names});MNAMES=(cfg.motor_names||names).slice(0,3);applyMotorCount();fillNames();
 const named=MNAMES.map((x,i)=>x?`${i+1} = ${esc(x)}`:'').filter(x=>x);toast(named.length?'Motor names saved: '+named.join(', ')+'.':'Motor names cleared - shown as Motor 1, 2, 3 again.');}

// ---------- live data ----------
async function poll(){
 try{const r=await fetch('/data?since='+lastSeq);const j=await r.json();
  if(j.samples.length){data.push(...j.samples);lastSeq=data[data.length-1].seq;}
  const keep=parseFloat($('win').value)*1000+500;
  if(data.length&&!paused){const tEnd=data[data.length-1].ms;data=data.filter(s=>tEnd-s.ms<=keep);}
  if(data.length>6000)data=data.slice(-6000);
  connected=j.connected; fwnum=j.fwnum||0;
  const gm2=j.gen&&j.gen.mode!=='off'?j.gen.motor:null;if(gm2!==genMotor){genMotor=gm2;updateSelUI();}
  if(j.diag){const dm=j.diag.motor;if(dm!==diagMotor){diagMotor=dm;if(dm!=null)setSel(dm);else updateSelUI();}
   const busy=dm!=null;if(busy!==diagBusy){diagBusy=busy;lockAp();}
   if(j.diag.prompt&&curView!=='settings'&&Date.now()-waitToast>15000){waitToast=Date.now();toast('The diagnostics check is waiting for your answer - <a href="#settings">Settings</a>.');}
   if((j.diag.state==='running'||j.diag.state==='waiting')&&diagState!==j.diag.state&&curView!=='settings')diagPoll(true);}
  if(j.watch){watch=j.watch;showBadge();}
  updateStatus(j); updateCards(); if(!paused)draw(); misses=0;
  const ls=data[data.length-1];$('dg_live').innerHTML=diagMotor!=null&&ls?`<span class="mtag m${diagMotor+1}">${esc(mshort(diagMotor))}</span>pot ${ls.m[diagMotor].f.toFixed(1)}  target ${ls.m[diagMotor].t}  drive ${ls.m[diagMotor].p.toFixed(0)}`:'';
 }catch(e){if(++misses>=5){$('pill').innerHTML='<span><i class="dot off"></i>Page server not reachable</span>';}}
 setTimeout(poll,100);}
function updateStatus(j){const s=data[data.length-1];const fresh_inc=watch.newest>seenInc();
 let h=connected?'<span data-k="st_conn"><i class="dot on"></i>Board Connected</span>':`<span data-k="st_conn"><i class="dot off"></i>Not connected <span class="muted" style="font-weight:400">(${esc(j.host||'no address set')})</span></span>`;
 if(j.fw)h+=`<span data-k="st_fw">fw v${esc(j.fw)}</span>`;
 if(s&&connected){h+=`<span data-k="st_isr">ISR max ${s.isr} µs</span><span data-k="st_div">Fpid/${s.div}</span>`+(s.low?'<span class="bad" data-k="st_low">LOW POWER (no position commands)</span>':'<span class="ok" data-k="st_full">Full power</span>');
  if(s.roll||s.pitch)h+=`<span>IMU ${s.roll.toFixed(1)}° / ${s.pitch.toFixed(1)}°</span>`;}
 if(j.scanning)h+='<span>searching for the board…</span>';
 if(tuneRunning)h+='<span class="ok">auto-tune running</span>';
 $('pill').innerHTML=h;tipAttach($('pill'));
 // system status
 const fresh=s&&connected; const on=s?s.dis.slice(0,NM).filter(d=>!d).length:0;
 if(s){const key=s.m.map(x=>x.t).join(',');if(lastT!==null&&key!==lastT)lastMove=Date.now();lastT=key;}
 const moving=Date.now()-lastMove<5000;
 const errs=[];if(!connected)errs.push('no connection');if(s&&s.dis.slice(0,NM).some(d=>d))errs.push('motor off');if(s&&s.isr>200)errs.push('slow loop');
 if(fresh_inc)errs.push(watch.count+' problem'+(watch.count>1?'s':'')+' noticed');
 const set=(k,cls,label)=>{const d=document.querySelector(`#sys [data-k=${k}]`);d.className=cls;d.querySelector('span').textContent=label;};
 set('sy_board',connected?'st-ok':'st-bad',connected?'Board connected':'Board not connected');
 set('sy_motors',!fresh?'st-off':(on===NM?'st-ok':'st-warn'),fresh?`${on} / ${NM} motor${NM>1?'s':''} online`:'Motors: no data');
 set('sy_pid',fresh?'st-ok':'st-off',fresh?'PID active':'PID: no data');
 set('sy_power',!fresh?'st-off':(s.low?'st-warn':'st-ok'),!fresh?'Power: no data':(s.low?'Low power':'Full power'));
 set('sy_input',!fresh?'st-off':(moving?'st-ok':'st-warn'),!fresh?'Input: no data':(moving?'Input signal moving':'Input signal idle'));
 set('sy_err',errs.length?(errs.length===1&&fresh_inc?'st-warn clk':'st-bad'+(fresh_inc?' clk':'')):'st-ok',errs.length?errs.join(', '):'No errors');
 const dot=$('sysdot');dot.className='dot '+(errs.length?'off':'on');
 $('disBtn').disabled=!(connected&&fwnum>=15);
 const fOk=connected&&fwnum>=16;['fsm','frate','fapply'].forEach(id=>$(id).disabled=!fOk||(id==='fapply'&&tuneRunning));
 $('fnote').textContent=fOk||!connected?'Takes effect immediately; press Save to EEPROM to keep it after a power cycle.':'Smoothing and max speed need firmware 1.6 on the board.';}
function updateCards(){const s=data[data.length-1];
 for(let m=0;m<3;m++){const st=$('ms'+m);
  if(!s||!connected){st.className='state nodata';st.lastElementChild.textContent='No data';continue;}
  const x=s.m[m];const off=s.dis[m];st.className='state'+(off?' off':'');st.lastElementChild.textContent=off?'Off':'Online';
  $('mp'+m).textContent=Math.round(x.g);$('mfb'+m).textContent=x.f.toFixed(1);$('me'+m).textContent=(x.g-x.f).toFixed(1);$('mo'+m).textContent=x.p.toFixed(1);
  const pc=v=>Math.max(0,Math.min(100,v/1023*100))+'%';$('mf'+m).style.width=pc(x.g);$('mk'+m).style.left=pc(x.g);$('mt'+m).style.left=pc(x.f);
  $('rg'+m).textContent=x.g.toFixed(1);$('rf'+m).textContent=x.f.toFixed(1);$('re'+m).textContent=(x.g-x.f).toFixed(1);$('rp'+m).textContent=x.p.toFixed(1);}}
function cssv(n){return getComputedStyle(document.documentElement).getPropertyValue(n).trim()}
function draw(){
 const win=parseFloat($('win').value)*1000;const col={t:cssv('--c-t'),g:cssv('--c-g'),f:cssv('--c-f'),p:cssv('--c-p'),grid:cssv('--grid'),axis:cssv('--axis')};
 const tEnd=data.length?data[data.length-1].ms:0,t0=tEnd-win;const dpr=window.devicePixelRatio||1;
 for(let m=0;m<3;m++){const cv=$('c'+m);if(!cv||!cv.offsetParent)continue;
  const W=cv.clientWidth,H=cv.clientHeight;cv.width=W*dpr;cv.height=H*dpr;const g=cv.getContext('2d');g.setTransform(dpr,0,0,dpr,0,0);g.clearRect(0,0,W,H);
  const L=74,B=20,T=10,PW=W-L-8,PH=H-B-T;
  g.font='11.5px '+getComputedStyle(document.body).fontFamily;g.textBaseline='middle';
  g.strokeStyle=col.grid;g.lineWidth=1;g.beginPath();for(let i=0;i<=4;i++){const y=T+PH*i/4;g.moveTo(L,y);g.lineTo(L+PW,y);}g.stroke();
  [['1023','255',0],['512','0',.5],['0','-255',1]].forEach(([a,b,f])=>{const y=T+PH*f;g.textAlign='right';g.fillStyle=col.axis;g.fillText(a,L-36,y);g.fillStyle=col.p;g.fillText(b,L-6,y);});
  g.textAlign='center';g.fillStyle=col.axis;const secs=win/1000,step=secs<=2?0.5:(secs<=5?1:(secs<=10?2:5));
  for(let s=0;s<=secs+1e-6;s+=step){const x=L+PW*(1-s/secs);g.fillText(s===0?'0s':'-'+(+s.toFixed(1))+'s',x,T+PH+B/2+2);}
  if(!data.length)continue;
  const X=t=>L+(t-t0)/win*PW,Y=v=>T+PH-(v/1023)*PH,YP=v=>T+PH/2-(v/255)*(PH/2);
  g.save();g.beginPath();g.rect(L,T-2,PW,PH+4);g.clip();
  const line=(key,color,yf,w)=>{g.strokeStyle=color;g.lineWidth=w;g.beginPath();let first=true;
   for(const s of data){if(s.ms<t0)continue;const x=X(s.ms),y=yf(s.m[m][key]);first?g.moveTo(x,y):g.lineTo(x,y);first=false;}g.stroke();};
  line('p',col.p,YP,1.4);line('t',col.t,Y,1.1);line('g',col.g,Y,1.8);line('f',col.f,Y,1.6);g.restore();}}
function togglePause(){paused=!paused;$('pauseBtn').innerHTML=paused?'<svg class="i" viewBox="0 0 24 24"><path d="M7 4l13 8-13 8z" fill="currentColor"/></svg>Resume':'<svg class="i" viewBox="0 0 24 24"><path d="M8 5v14M16 5v14"/></svg>Pause';}

// ---------- parameters ----------
const setv=(id,v)=>{const e=$(id);if(e)e.value=v;};
async function readParams(){const p=await post('/read');params=p;
 for(const m of [1,2,3]){if(!$('Kp'+m))continue;for(const k of ['Kp','Ki','Kd','Ks'])if(p[k+m]!=null)$(k+m).value=p[k+m];
  if(p['PWM'+m]){$('PWMmin'+m).value=p['PWM'+m][0];$('PWMmax'+m).value=p['PWM'+m][1];}
  if(p['LIM'+m]){$('LIM'+m).value=p['LIM'+m][0];$('CLIP'+m).value=p['LIM'+m][1];}
  if(p['DZ'+m]){$('DZ'+m).value=p['DZ'+m][0];$('PWMrev'+m).value=p['DZ'+m][1];}}
 if(p.STATUS)$('div').value=p.STATUS[0]>>4;if(p.PWMFREQ){$('f1').value=p.PWMFREQ[0];$('f2').value=p.PWMFREQ[1];}
 if(p.FILT){$('fsm').value=String(p.FILT[0]>=4||p.FILT[0]===0?p.FILT[0]:4);$('frate').value=p.FILT[1]*100;}
 return p;}
function apply(m){const i=m-1,w=(code,v)=>post('/cmd',{word:code,value:v}),pr=(code,a,b)=>post('/cmd',{pair:code,a:a,b:b});
 w('DEF'[i],Math.round($('Kp'+m).value));w('GHI'[i],Math.round($('Ki'+m).value));w('JKL'[i],Math.round($('Kd'+m).value));
 w('MNO'[i],Math.max(1,Math.round($('Ks'+m).value)));pr('PQR'[i],$('PWMmin'+m).value,$('PWMmax'+m).value);
 pr('STU'[i],$('LIM'+m).value,$('CLIP'+m).value);pr('VWX'[i],$('DZ'+m).value,$('PWMrev'+m).value);}
function applyGlobal(){post('/cmd',{pair:'Z',a:$('div').value,b:0});post('/cmd',{pair:'~',a:$('f1').value,b:$('f2').value});}
async function saveEeprom(){if(!confirm('Save all settings running on the board to its EEPROM?'))return;
 const r=await post('/eeprom/save');if(r.ok){toast('Saved to EEPROM. A backup of these settings is kept in the EEPROM section.');if(curView==='eeprom')loadBackups();}else toast(esc(r.error),true);}
function applyFilt(){post('/cmd',{pair:'f',a:+$('fsm').value,b:Math.round((+$('frate').value||0)/100)});toast('Smoothing and max speed sent to the board.');}
function disableAll(){txt('dis');}

// ---------- test signal ----------
async function gen(mode){const body={motor:+$('gm').value,mode:mode||$('gmode').value,amp:+$('gamp').value,period:+$('gper').value,centre:+$('goff').value,value:+$('goff').value};
 const r=await post('/gen',body);$('gmsg').innerHTML=r.ok?(body.mode==='off'?'Stopped.':'Running on '+esc(mfull(body.motor))+'.'):'<span class="badc">'+esc(r.error)+'</span>';}
function genStop(){gen('off');}
function preset(k){const P={sine:['sine',150,2],square:['step',100,2],triangle:['triangle',150,3]};
 if(P[k]){$('gmode').value=P[k][0];$('gamp').value=P[k][1];$('gper').value=P[k][2];gen();}
 else if(k==='step'){post('/gen',{motor:+$('gm').value,mode:'manual',value:+$('goff').value+ +$('gamp').value,centre:+$('goff').value}).then(r=>$('gmsg').innerHTML=r.ok?'Stepped to '+(+$('goff').value+ +$('gamp').value)+' and holding.':'<span class="badc">'+esc(r.error)+'</span>');}
 else if(k==='manual'){$('gmode').value='manual';gen();}}

// ---------- profiles ----------
async function loadProfiles(){const r=await (await fetch('/profiles')).json();
 const gb=$('gbase'),keep=gb.value;gb.innerHTML='<option value="">Your current settings (on the board)</option>'+r.map(p=>`<option value="${esc(p.name)}">Profile: ${esc(p.name)}</option>`).join('');if([...gb.options].some(o=>o.value===keep))gb.value=keep;
 $('plist').innerHTML=r.length?r.map(p=>`<div class="prow"><b>${esc(p.name)}</b><span class="muted">${esc(p.summary)}, saved ${esc(p.saved)}</span><button class="btn b-blue" data-k="pr_load" onclick="profileLoad(this.dataset.n)" data-n="${esc(p.name)}">Load</button><button class="btn b-grey" data-k="pr_del" onclick="profileDelete(this.dataset.n)" data-n="${esc(p.name)}">Delete</button></div>`).join(''):'<p class="muted">No profiles yet. Tune the rig the way you like it for one car or game, give it a name above and save.</p>';
 tipAttach($('plist'));}
async function profileSave(){const n=$('prname').value.trim();if(!n){$('prmsg').innerHTML='<span class="badc">Type a name for the profile first.</span>';return;}
 $('prmsg').textContent='Reading settings from the board…';const r=await post('/profiles/save',{name:n});
 $('prmsg').innerHTML=r.ok?'<span class="okc">Saved "'+esc(n)+'".</span>':'<span class="badc">'+esc(r.error)+'</span>';loadProfiles();}
async function profileLoad(n){const r=await post('/profiles/load',{name:n});if(r.ok)toast('Loaded "'+esc(n)+'". The previous settings were backed up first.');
 $('prmsg').innerHTML=r.ok?'<span class="okc">Loaded "'+esc(n)+'" - running now. Save to EEPROM to make it the power-on default.</span>':'<span class="badc">'+esc(r.error)+'</span>';setTimeout(readParams,600);}
async function profileDelete(n){if(!confirm('Delete profile "'+n+'"?'))return;await post('/profiles/delete',{name:n});loadProfiles();}

// ---------- recommendations ----------
const GICON={circuit:'<path d="M3 13l2-5a2 2 0 0 1 1.9-1.4h10.2A2 2 0 0 1 19 8l2 5v4a1 1 0 0 1-1 1h-1.5M3 13v4a1 1 0 0 0 1 1h1.5M3 13h18M8.5 18h7"/><circle cx="7" cy="18" r="1.8"/><circle cx="17" cy="18" r="1.8"/>',
 rally:'<path d="M2 20l6.5-11 4 6 2.5-4 7 9z"/><path d="M6.5 20c1-2 2-3 3.5-3"/>',
 flight:'<path d="M21 16v-2l-8-5V3.5a1.5 1.5 0 0 0-3 0V9l-8 5v2l8-2.5V19l-2 1.5V22l3.5-1 3.5 1v-1.5L13 19v-5.5z"/>',
 space:'<path d="M12 2c3 2 5 6 5 10l-2 4H9l-2-4c0-4 2-8 5-10z"/><circle cx="12" cy="9" r="1.8"/><path d="M9 16l-2 4 3-1M15 16l2 4-3-1"/>',
 truck:'<path d="M2 6h11v10H2zM13 9h4l4 4v3h-8z"/><circle cx="6" cy="17.5" r="1.8"/><circle cx="17" cy="17.5" r="1.8"/>'};
async function loadGenres(){if(!genres.length)genres=await (await fetch('/genres')).json();
 $('genres').innerHTML=genres.map(g=>`<button class="genre" data-g="${g.key}" data-k="genre" onclick="previewGenre('${g.key}')"><svg class="i" viewBox="0 0 24 24">${GICON[g.key]||''}</svg><b>${esc(g.title)}</b><span>${esc(g.short)}</span></button>`).join('');tipAttach($('genres'));}
async function previewGenre(k){document.querySelectorAll('.genre').forEach(b=>b.classList.toggle('on',b.dataset.g===k));
 const pv=$('gpv');pv.hidden=false;pv.innerHTML='<span class="muted">Working it out…</span>';
 const r=await post('/genres/preview',{genre:k,base:$('gbase').value});if(!r.ok){pv.innerHTML='<span class="badc">'+esc(r.error)+'</span>';return;}gpv=r;
 let t='<table class="chg"><tr><th></th>'+[0,1,2].slice(0,NM).map(m=>`<th>${esc(mshort(m))}</th>`).join('')+'</tr>';
 for(const row of r.rows){t+=`<tr><td>${esc(row.name)}</td>`+row.vals.slice(0,NM).map(([a,b])=>a===b?`<td>${esc(b)}</td>`:`<td><span class="muted">${esc(a)} → </span><b class="${(+b>+a)?'up':'down'}">${esc(b)}</b></td>`).join('')+'</tr>';}
 t+='</table>';
 pv.innerHTML=`<h3>${esc(r.title)}</h3><p class="note">${esc(r.blurb)}</p>${t}`+(r.notes.length?'<ul class="tips">'+r.notes.map(n=>`<li>${esc(n)}</li>`).join('')+'</ul>':'')+
  `<p class="note" style="margin-top:12px"><b>In SimHub</b> - most of the feel comes from there:</p><ul class="tips">${r.simhub.map(n=>`<li>${esc(n)}</li>`).join('')}</ul>
  <div class="inline" style="margin-top:12px"><div><span class="lbl">Profile name</span><input type="text" id="gname" maxlength="40" value="${esc(r.title)}"></div>
  <button class="btn b-blue ap" data-k="g_apply" onclick="applyGenre()">Apply now</button><button class="btn b-grey" data-k="g_save" onclick="saveGenre()">Save as profile</button></div>`;
 tipAttach(pv);}
async function applyGenre(){if(!gpv)return;const r=await post('/params/apply',{params:gpv.params,label:'Before applying the '+gpv.title+' recommendation'});
 if(r.ok){toast('Applied the '+esc(gpv.title)+' recommendation. Previous settings backed up. Save to EEPROM to keep it.');setTimeout(readParams,700);}else toast(esc(r.error),true);}
async function saveGenre(){if(!gpv)return;const n=$('gname').value.trim()||gpv.title;const r=await post('/profiles/save',{name:n,params:gpv.params});
 if(r.ok){toast('Saved as profile "'+esc(n)+'".');loadProfiles();}else toast(esc(r.error),true);}

// ---------- backups ----------
const same=(a,b)=>PKEYS.every(k=>JSON.stringify(a[k]??null)===JSON.stringify(b[k]??null));
async function loadBackups(){const [r]=await Promise.all([(await fetch('/backups')).json(),readParams()]);
 $('bkcount').textContent=r.length?`(${r.length})`:'';
 $('blist').innerHTML=r.length?r.map(b=>`<div class="brow"><span class="when">${esc(b.when)}</span><div><div class="what"><span class="kind ${b.kind}">${{eeprom:'EEPROM save',manual:'Manual',auto:'Automatic'}[b.kind]||b.kind}</span>${esc(b.label)}${same(b.params,params)?'<span class="cur">matches the board now</span>':''}</div><div class="sum">${esc(b.summary)}</div></div>
  <div class="acts"><button class="btn b-blue" data-k="b_switch" onclick="backupSwitch('${b.id}',false)">Switch to</button><button class="btn b-red" data-k="b_switchsave" onclick="backupSwitch('${b.id}',true)">Switch + save</button><button class="btn b-grey" data-k="b_dl" onclick="backupDl('${b.id}')">Download</button><button class="btn b-grey" data-k="b_del" onclick="backupDel('${b.id}')">Delete</button></div></div>`).join('')
  :'<p class="muted">No backups yet. One is kept automatically the next time you Save to EEPROM, or press Back up now.</p>';tipAttach($('blist'));}
async function backupNow(){const r=await post('/backups/now',{label:$('bklabel').value.trim()});if(r.ok){$('bklabel').value='';toast('Backup kept.');loadBackups();}else toast(esc(r.error),true);}
async function backupSwitch(id,save){if(!confirm(save?'Send this backup to the board AND save it to EEPROM?':'Send this backup to the board now (live, not saved)?'))return;
 const r=await post('/backups/restore',{id:id,save:save});if(r.ok){toast(save?'Switched and saved to EEPROM.':'Switched - running now. The previous settings were backed up first.');setTimeout(loadBackups,800);}else toast(esc(r.error),true);}
async function backupDel(id){if(!confirm('Delete this backup?'))return;await post('/backups/delete',{id:id});loadBackups();}
async function backupDl(id){const b=await (await fetch('/backups/get?id='+encodeURIComponent(id))).json();
 const blob=new Blob([JSON.stringify({smc3_backup:1,saved:b.when,label:b.label,params:b.params},null,2)],{type:'application/json'});
 const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='smc3-backup-'+b.when.replace(/[^0-9]/g,'').slice(0,12)+'.json';a.click();URL.revokeObjectURL(a.href);}

// ---------- backup file ----------
async function backupDownload(){const p=await readParams();const blob=new Blob([JSON.stringify({smc3_backup:1,saved:new Date().toISOString(),params:p},null,2)],{type:'application/json'});
 const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='smc3-backup-'+new Date().toISOString().slice(0,10)+'.json';a.click();URL.revokeObjectURL(a.href);
 $('eemsg').innerHTML='<span class="okc">Backup downloaded.</span>';}
async function backupRestore(inp){const f=inp.files[0];inp.value='';if(!f)return;
 try{const j=JSON.parse(await f.text());if(!j.params||j.params.Kp1==null)throw new Error('not an SMC3 backup file');
  if(!confirm('Send the settings from '+f.name+' to the board now?'))return;const r=await post('/params/apply',{params:j.params,label:'Before restoring from '+f.name});
  $('eemsg').innerHTML=r.ok?'<span class="okc">Restored from '+esc(f.name)+' - running now. Save to EEPROM to keep it.</span>':'<span class="badc">'+esc(r.error)+'</span>';setTimeout(readParams,600);
 }catch(e){$('eemsg').innerHTML='<span class="badc">That file can\'t be used: '+esc(e.message)+'.</span>';}}

// ---------- auto-tune ----------
function tuneConfirm(){$('mt_n').textContent=mfull(+$('tm').value);$('mt_a').textContent=$('tamp').value;openModal('mTune');}
async function tuneStart(){closeModal('mTune');const r=await post('/tune/start',{motor:+$('tm').value,amp:+$('tamp').value,style:$('tstyle').value,friction:$('tfr').checked,ki:$('tki').checked,kd:$('tkd').checked});if(!r.ok)alert(r.error);}
const NAMES={kp:'Kp',ki:'Ki',kd:'Kd',ks:'Ks',pwmmin:'PWMmin',dz:'Dead zone'};
async function tunePoll(){
 try{const s=await (await fetch('/tune/status')).json();tuneRunning=s.state==='running';
  const tm2=tuneRunning?s.motor-1:null;if(tm2!==tuneMotor){tuneMotor=tm2;if(tm2!=null)setSel(tm2);else updateSelUI();}
  lockAp();$('tstart').disabled=tuneRunning||diagBusy;$('tstop').disabled=!tuneRunning;
  document.querySelectorAll('.tbar').forEach(b=>b.style.width=(s.progress||0)+'%');
  const label={idle:'Idle - choose a motor and press Start Auto-Tune.',running:mfull(s.motor-1)+': '+s.stage,done:'Finished '+mfull(s.motor-1)+'.',aborted:'Stopped.',error:'Error - see the log.'}[s.state]||s.state;
  document.querySelectorAll('.tstage').forEach(e=>e.textContent=label);$('tmini').hidden=s.state==='idle';
  const lg=$('tlog'),atEnd=lg.scrollTop+lg.clientHeight>=lg.scrollHeight-5,t=nameify(s.log.join('\n'));if(lg.textContent!==t){lg.textContent=t;if(atEnd)lg.scrollTop=lg.scrollHeight;}
  let h='';if(s.result){const r=s.result;h='<table class="res"><tr><th>'+esc(mfull(r.motor-1))+'</th><th>before</th><th>after</th></tr>';
   for(const k of Object.keys(NAMES)){const a=r.before[k],b=r.after[k];h+=`<tr><td>${NAMES[k]}</td><td>${a}</td><td class="${a!==b?'chg':''}">${b}</td></tr>`;}h+='</table>';}
  if(s.can_revert)h+='<div style="display:flex;gap:10px"><button class="btn b-grey" data-k="trevert" onclick="tuneRevert()">Revert</button><button class="btn b-red" data-k="sav" onclick="saveEeprom()">Save to EEPROM</button></div>';
  if($('tresult').dataset.h!==h){$('tresult').innerHTML=h;$('tresult').dataset.h=h;tipAttach($('tresult'));}
  if(s.state==='done'&&!tunePoll.reread){tunePoll.reread=true;readParams();}if(s.state==='running')tunePoll.reread=false;
 }catch(e){}
 setTimeout(tunePoll,700);}
async function tuneRevert(){await post('/tune/revert');readParams();}

// ---------- diagnostics ----------
const FIC={fail:'✕',warn:'!',info:'i',ok:'✓'};
const WHERE=['','front left','front right','rear left','rear right','front','rear','left side','right side','middle'];
function lockAp(){document.querySelectorAll('.ap').forEach(b=>b.disabled=tuneRunning||diagBusy);}
function seenInc(){try{return +(localStorage.getItem('smc3_seen_inc')||0)}catch(e){return 0}}
function markSeen(){try{localStorage.setItem('smc3_seen_inc',String(watch.newest||0))}catch(e){}showBadge();}
function showBadge(){const b=$('navbadge'),n=watch.newest>seenInc()?watch.count:0;b.hidden=!n;b.textContent=n;}
document.querySelector('#sys [data-k=sy_err]').onclick=function(){if(this.classList.contains('clk')){location.hash='settings';setTimeout(()=>$('diagcard').scrollIntoView({behavior:'smooth'}),100);}};
async function diagStart(kind){closeModal('mDiag');
 const r=await post('/diag/start',{kind:kind,problem:$('dg_problem').value,ask:$('dg_ask').checked});
 if(!r.ok){toast(esc(r.error),true);return;}diagRes=null;$('dg_result').innerHTML='';diagState='running';diagPoll(true);}
async function diagAnswer(a){const w=$('dg_where');const id=dgPromptId;$('dg_prompt').querySelectorAll('button').forEach(b=>b.disabled=true);
 await post('/diag/answer',{id:id,answer:a,where:w?w.value:''});}
function renderPrompt(p){const box=$('dg_prompt');box.dataset.m=p.motor;
 let h=`<span class="badge">${p.motor}</span><div>`;
 if(p.info)h+=`<span class="watching"><i></i>Watch now</span><div class="q">${escn(p.text)}</div>`;
 else{h+=`<div class="q">${escn(p.text)}</div><div class="opts">`+p.options.map(([k,l])=>`<button class="btn ${k==='again'?'b-ghost':(k==='up'||k==='moved'?'b-blue':'b-grey')}" onclick="diagAnswer('${k}')">${esc(l)}</button>`).join('')+'</div>';
  if(p.where)h+=`<div class="where">Where on the seat is this motor?${mname(p.motor-1)?` <span>(you named it ${esc(mname(p.motor-1))})</span>`:''} <select id="dg_where">${WHERE.map(w=>`<option value="${w}">${w||'(not sure)'}</option>`).join('')}</select></div>`;}
 box.innerHTML=h+'</div>';}
function fItem(f){return `<div class="fitem ${f.level}"><i class="fic">${FIC[f.level]}</i><div><b>${f.motor?`<span class="mtag m${f.motor}">${esc(mshort(f.motor-1))}</span>`:''}${escn(f.title)}</b>`+
 (f.detail?`<p>${escn(f.detail)}</p>`:'')+(f.fix?`<p class="fix"><b>What to do:</b> ${escn(f.fix)}</p>`:'')+'</div></div>';}
function renderResult(res,title){const F=res.findings||[];const c=k=>F.filter(f=>f.level===k).length;
 const nf=c('fail'),nw=c('warn');
 let head=nf||nw?`${nf} problem${nf!==1?'s':''}, ${nw} warning${nw!==1?'s':''}`:'No problems found';
 let h=`<div class="fsum"><b class="${nf?'badc':(nw?'':'okc')}">${head}</b><span class="muted">${esc(title||((res.kind==='full'?'Full':'Quick')+' check'+(res.stopped?' (stopped early)':'')))} - ${esc(res.name)}</span></div>`;
 h+=`<div class="facts"><a class="btn b-blue" data-k="dg_dl" href="/diag/file?name=${encodeURIComponent(res.name)}&dl=1"><svg class="i" viewBox="0 0 24 24"><path d="M12 3v12M7 10l5 5 5-5M4 20h16"/></svg>Download file</a><button class="btn b-grey" data-k="dg_copy" onclick="copySummary()">Copy summary</button></div>`;
 const main=F.filter(f=>f.level!=='ok'),oks=F.filter(f=>f.level==='ok');
 h+='<div class="flist">'+main.map(fItem).join('')+'</div>';
 if(oks.length)h+=`<details class="oks"${main.length?'':' open'}><summary>${oks.length} thing${oks.length>1?'s':''} checked and fine</summary><div class="flist">${oks.map(fItem).join('')}</div></details>`;
 $('dg_result').innerHTML=h;tipAttach($('dg_result'));}
function copySummary(){const t=diagRes&&diagRes.summary;if(!t)return;
 const done=()=>toast('Summary copied - paste it into a chat or message.');
 if(navigator.clipboard&&window.isSecureContext){navigator.clipboard.writeText(t).then(done,()=>fallbackCopy(t,done));}else fallbackCopy(t,done);}
function fallbackCopy(t,done){const ta=document.createElement('textarea');ta.value=t;ta.style.position='fixed';ta.style.opacity='0';document.body.appendChild(ta);ta.select();
 let ok=false;try{ok=document.execCommand('copy');}catch(e){}document.body.removeChild(ta);ok?done():toast('Copy not allowed by the browser - use Download file instead.',true);}
async function diagPoll(once){
 try{const s=await (await fetch('/diag/status')).json();const was=diagState;diagState=s.state;
  const running=s.state==='running'||s.state==='waiting';
  $('dg_run').hidden=!running;$('dg_stop').hidden=!running;$('dg_quick').disabled=$('dg_full').disabled=running;
  if(running){$('dg_bar').style.width=(s.progress||0)+'%';const last=s.log.length?s.log[s.log.length-1].slice(9):'';
   $('dg_stage').textContent=nameify((s.kind==='full'?'Full':'Quick')+' check - '+(s.stage||'starting')+(last?': '+last:''));}
  const p=s.prompt,box=$('dg_prompt');
  if(p){if(p.id!==dgPromptId){dgPromptId=p.id;renderPrompt(p);}box.hidden=false;}else{box.hidden=true;dgPromptId=null;}
  if(s.result&&statusRes!==s.result.name&&!running){statusRes=s.result.name;diagRes=s.result;renderResult(s.result);diagLists();}
  if(!running&&(s.state==='aborted'||s.state==='error')&&was!==s.state)$('dg_result').innerHTML=`<p class="msg badc">${esc(s.log.length?s.log[s.log.length-1].slice(9):s.state)}</p>`;
 }catch(e){}
 if(!once)setTimeout(diagPoll,(curView==='settings'||diagState==='running'||diagState==='waiting')?600:2500);}
async function diagLists(){
 try{const [inc,rep]=await Promise.all([(await fetch('/diag/incidents')).json(),(await fetch('/diag/reports')).json()]);
  $('dg_inc_n').textContent=inc.length?`(${inc.length})`:'';
  $('dg_inc').innerHTML=inc.length?inc.map(x=>`<div class="drow"><div><span class="t">${esc(x.when.slice(5,16))}</span> ${x.motor?`<span class="mtag m${x.motor}">${esc(mshort(x.motor-1))}</span>`:''}<b>${esc(x.title)}</b><div class="m">${escn(x.msg)}</div></div></div>`).join('')
   :'<p class="muted" style="margin:0">Nothing noticed. The page keeps watching while you drive.</p>';
  $('dg_reports').innerHTML=rep.length?rep.map(x=>`<div class="drow"><div><span class="t">${esc(x.when)}</span> ${x.kind==='full'?'Full':'Quick'} check <span class="muted">${x.kb} kB</span></div><div class="acts"><button class="btn b-grey" onclick="viewReport('${esc(x.name)}')">View</button><a class="btn b-blue" href="/diag/file?name=${encodeURIComponent(x.name)}&dl=1">Download</a><button class="btn b-grey" onclick="delReport('${esc(x.name)}')">Delete</button></div></div>`).join('')
   :'<p class="muted" style="margin:0">No reports yet.</p>';
  if(curView==='settings')markSeen();
  if($('dg_evd').open)loadEvents();
 }catch(e){}}
async function viewReport(name){const r=await (await fetch('/diag/file?name='+encodeURIComponent(name))).json();
 diagRes={name:name,findings:r.findings||[],summary:r.summary||'',kind:r.kind};renderResult(diagRes,(r.kind==='full'?'Full':'Quick')+' check from '+r.made);
 if(r.problem)$('dg_result').insertAdjacentHTML('afterbegin',`<p class="note" style="margin:12px 0 0"><b>Problem described:</b> ${esc(r.problem)}</p>`);}
async function delReport(name){if(!confirm('Delete this report from the NAS?'))return;await post('/diag/delete',{name:name});diagLists();}
async function clearIncidents(){if(!confirm('Forget the problems the page has recorded?'))return;await post('/diag/incidents/clear');diagLists();}
async function loadEvents(){const ev=await (await fetch('/diag/events?n=200')).json();
 $('dg_events').innerHTML=ev.reverse().map(e=>`<div class="${e.kind==='problem'?'problem':''}"><span>${esc(e.t.slice(5))}</span> ${escn(e.msg)}</div>`).join('')||'<span>Empty.</span>';}

buildPid(3);tipAttach();showView();loadConfig();poll();tunePoll();diagPoll();
window.addEventListener('resize',draw);$('win').addEventListener('change',draw);
</script></body></html>
"""


WORD_NAMES = {c: (k, i + 1) for k, cs in (("Kp", "DEF"), ("Ki", "GHI"), ("Kd", "JKL"), ("Ks", "MNO"))
              for i, c in enumerate(cs)}
PAIR_NAMES = {c: (key, a, b, i + 1) for key, a, b, cs in (("PWM", "PWMmin", "PWMmax", "PQR"),
                                                         ("LIM", "Max limit", "Clip", "STU"),
                                                         ("DZ", "Dead zone", "PWMrev", "VWX")) for i, c in enumerate(cs)}
TEXT_NAMES = {"ena": "Enable Motors pressed.", "dis": "Disable Motors pressed.", "sav": "Saved to EEPROM.",
              "en1": "Motor 1 switched on.", "en2": "Motor 2 switched on.", "en3": "Motor 3 switched on.",
              "di1": "Motor 1 switched off.", "di2": "Motor 2 switched off.", "di3": "Motor 3 switched off."}


def describe_cmd(req, params):
    """A page command as a sentence for the event log (None for no change), and the params cache updated."""
    try:
        if "word" in req and str(req["word"]) in WORD_NAMES:
            name, n = WORD_NAMES[str(req["word"])]
            v, key = int(req.get("value", 0)), f"{name}{n}"
            old = params.get(key)
            params[key] = v
            return None if old == v else f"Motor {n}: {name} {old if old is not None else '?'} -> {v}"
        if "pair" in req:
            code, a, b = str(req["pair"]), int(req.get("a", 0)), int(req.get("b", 0))
            if code in PAIR_NAMES:
                key, na, nb, n = PAIR_NAMES[code]
                old = params.get(f"{key}{n}") or [None, None]
                params[f"{key}{n}"] = [a, b]
                ch = [f"{nm} {o if o is not None else '?'} -> {v}" for nm, o, v in ((na, old[0], a), (nb, old[1], b))
                      if o != v]
                return f"Motor {n}: " + ", ".join(ch) if ch else None
            if code == "Z":
                return f"Fpid divider set to {a}."
            if code == "~":
                return f"PWM frequency set to {a} kHz (motors 1+2) and {b} kHz (motor 3)."
            if code == "f":
                return f"Target smoothing set to {'off' if not a else f'{(1 << min(a, 8)) / 4:g} ms'}, max speed " \
                       f"{b * 100 or 'off'}."
        if "text" in req:
            return TEXT_NAMES.get(str(req["text"]))
    except (TypeError, ValueError):
        pass
    return None


class Handler(BaseHTTPRequestHandler):
    board: Board = None
    gen: Generator = None
    tuner: Tuner = None
    cfg: Config = None
    finder: AutoFinder = None
    profiles: Profiles = None
    backups: Backups = None
    events: EventLog = None
    monitor: Monitor = None
    diag: Diagnostics = None

    def log_message(self, *a):          # quiet
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):      # the browser went away mid-answer - nothing to do
            pass

    def handle(self):
        try:
            super().handle()
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _fw(self):
        fw = self.board.fw_cache
        return f"{fw[1] // 10}.{fw[1] % 10}" if fw else ("≤1.3" if self.board.params.get("Kp1") is not None else "")

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/":
            body = PAGE.replace("__VERSION__", VERSION).encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif u.path == "/data":
            q = parse_qs(u.query)
            try:
                since = int(q.get("since", ["0"])[0])
            except ValueError:
                since = 0
            if since > self.board.seq:      # scope restarted - page must start over
                since = 0
            self._json({"connected": self.board.connected, "host": self.cfg.get("host"),
                        "samples": self.board.since(since, limit=1500), "fw": self._fw(),
                        "fwnum": (self.board.fw_cache or [0, 0])[1],
                        "gen": {"mode": self.gen.mode, "motor": self.gen.motor},
                        "scanning": self.finder.scanning, "watch": self.monitor.summary(),
                        "diag": {"state": self.diag.state, "motor": None if self.diag.motor is None else self.diag.motor,
                                 "prompt": self.diag.prompt is not None}})
        elif u.path == "/config":
            self._json(self.cfg.public())
        elif (u.path.startswith("/img/") and u.path[5:] in ASSETS) or u.path[1:] in ("favicon.svg", "favicon-32.png",
                                                                                     "apple-touch-icon.png", "favicon.ico"):
            name = u.path[5:] if u.path.startswith("/img/") else ("favicon-32.png" if u.path == "/favicon.ico" else u.path[1:])
            body = ASSETS[name]
            self.send_response(200)
            self.send_header("Content-Type", "font/woff2" if name.endswith(".woff2") else
                             ("image/svg+xml" if name.endswith(".svg") else "image/png"))
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "max-age=86400")
            self.end_headers()
            self.wfile.write(body)
        elif u.path == "/profiles":
            self._json(self.profiles.list())
        elif u.path == "/backups":
            self._json([{k: v for k, v in b.items()} for b in self.backups.list()])
        elif u.path == "/backups/get":
            b = self.backups.get(parse_qs(u.query).get("id", [""])[0])
            self._json(b or {"error": "not found"}, 200 if b else 404)
        elif u.path == "/genres":
            self._json([{"key": g["key"], "title": g["title"], "short": g["short"]} for g in GENRES])
        elif u.path == "/tune/status":
            self._json(self.tuner.status())
        elif u.path == "/diag/status":
            st = self.diag.status()
            st["watch"] = self.monitor.summary()
            self._json(st)
        elif u.path == "/diag/reports":
            self._json(self.diag.list_reports())
        elif u.path == "/diag/file":
            q = parse_qs(u.query)
            name = q.get("name", [""])[0]
            body = self.diag.report_bytes(name)
            if body is None:
                self._json({"error": "not found"}, 404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            if q.get("dl"):
                self.send_header("Content-Disposition", f'attachment; filename="{name}"')
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        elif u.path == "/diag/incidents":
            self._json(self.monitor.incident_list(40))
        elif u.path == "/diag/incident":
            inc = self.monitor.incident_full(parse_qs(u.query).get("name", [""])[0])
            self._json(inc or {"error": "not found"}, 200 if inc else 404)
        elif u.path == "/diag/events":
            try:
                n = int(parse_qs(u.query).get("n", ["100"])[0])
            except ValueError:
                n = 100
            self._json(self.events.recent(max(1, min(1000, n))))
        else:
            self._json({"error": "not found"}, 404)

    def do_POST(self):
        n = int(self.headers.get("Content-Length", "0") or 0)
        try:
            req = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            req = {}
        p = self.path
        if p == "/cmd":
            ok = False
            if self.diag.moving() and ("word" in req or "pair" in req):
                self._json({"ok": False, "error": "The diagnostics check is moving the motors - wait or stop it."})
                return
            msg = describe_cmd(req, self.board.params)
            if "word" in req:
                ok = self.board.cmd_word(req["word"], req.get("value", 0))
            elif "pair" in req:
                ok = self.board.cmd_pair(req["pair"], req.get("a", 0), req.get("b", 0))
            elif "text" in req:
                ok = self.board.cmd_text(req["text"])
                if str(req["text"]) == "ena":
                    sync_unused(self.board, self.cfg)
            if msg and ok:
                self.events.add("setting", msg, quiet=True)
            self._json({"ok": ok})
        elif p == "/read":
            self._json(self.board.read_params())
        elif p == "/gen":
            if self.tuner.running():
                self._json({"ok": False, "error": "Auto-tune is running - stop it first."})
                return
            if self.diag.running() and self.diag.kind == "full":
                self._json({"ok": False, "error": "The diagnostics check is running - wait or stop it first."})
                return
            try:
                centre = float(req.get("centre", 512))
                was = self.gen.mode
                self.gen.set(motor=int(req.get("motor", 0)) % 3, mode=str(req.get("mode", "off")),
                             amp=float(req.get("amp", 200)), period=float(req.get("period", 2.0)),
                             value=float(req.get("value", centre)), centre=centre)
                if self.gen.mode != was:
                    self.events.add("test", f"Test signal {self.gen.mode} on motor {self.gen.motor + 1}" if
                                    self.gen.mode != "off" else "Test signal stopped.", quiet=True)
            except (TypeError, ValueError):
                self._json({"ok": False, "error": "Check the test-signal numbers."})
                return
            self._json({"ok": True})
        elif p == "/diag/start":
            kind = "full" if req.get("kind") == "full" else "quick"
            ok, msg = self.diag.start(kind, str(req.get("problem", "")).strip(), bool(req.get("ask", True)))
            self._json({"ok": ok, "error": None if ok else msg})
        elif p == "/diag/answer":
            self._json({"ok": self.diag.answer(str(req.get("id", "")), req.get("answer", ""), req.get("where", ""))})
        elif p == "/diag/abort":
            self.diag.abort()
            self._json({"ok": True})
        elif p == "/diag/delete":
            self._json({"ok": self.diag.delete_report(str(req.get("name", "")))})
        elif p == "/diag/incidents/clear":
            self.monitor.clear_incidents()
            self._json({"ok": True})
        elif p == "/config":
            old = (self.cfg.get("host"), self.cfg.get("port"))
            kw = {}
            if "host" in req:
                kw["host"] = str(req["host"]).strip()
            if "port" in req:
                try:
                    kw["port"] = int(req["port"])
                except (TypeError, ValueError):
                    pass
            if "theme" in req and req["theme"] in ("dark", "light", "auto"):
                kw["theme"] = req["theme"]
            if "auto_find" in req:
                kw["auto_find"] = bool(req["auto_find"])
            if "scan_subnet" in req:
                kw["scan_subnet"] = str(req["scan_subnet"]).strip()
            old_motors = int(self.cfg.get("motors"))
            if "motors" in req:
                try:
                    kw["motors"] = max(1, min(3, int(req["motors"])))
                except (TypeError, ValueError):
                    pass
            if "disable_unused" in req:
                kw["disable_unused"] = bool(req["disable_unused"])
            if isinstance(req.get("motor_names"), list):
                names = [re.sub(r"[\x00-\x1f<>\"]", "", str(x)).strip()[:16] for x in req["motor_names"][:3]]
                kw["motor_names"] = (names + ["", "", ""])[:3]
            before = {k: self.cfg.get(k) for k in kw}
            self.cfg.update(**kw)
            changed = [f"{k} {before[k]} -> {self.cfg.get(k)}" for k in kw if before[k] != self.cfg.get(k)]
            if changed:
                self.events.add("config", "Page settings changed: " + ", ".join(changed) + ".", quiet=True)
            new_motors = int(self.cfg.get("motors"))
            fw = (self.board.fw_cache or [0, 0])[1]
            if new_motors > old_motors and fw >= 15 and self.cfg.get("disable_unused"):
                for m in range(old_motors, new_motors):          # motors taken back into use
                    self.board.cmd_text(f"en{m + 1}")
            sync_unused(self.board, self.cfg)
            if (self.cfg.get("host"), self.cfg.get("port")) != old:
                log(f"board address changed to {self.cfg.get('host')}:{self.cfg.get('port')}")
                self.board.reconnect()
            self._json(self.cfg.public())
        elif p == "/scan":
            r = self.finder.scan_now(str(req.get("subnet", "")).strip())
            if r is None:
                self._json({"busy": True, "found": []})
                return
            prefix, found = r
            self._json({"busy": False, "subnet": prefix, "found": found, "connected": self.board.connected,
                        "current": self.board.current_host})
        elif p == "/tune/start":
            try:
                motor = int(req.get("motor", 0)) % 3
                amp = int(max(40, min(400, float(req.get("amp", 100)))))
            except (TypeError, ValueError):
                self._json({"ok": False, "error": "bad parameters"})
                return
            if self.diag.running():
                self._json({"ok": False, "error": "The diagnostics check is running - wait or stop it first."})
                return
            if not self.tuner.running() and self.board.connected:
                self.backups.add("auto", f"Before auto-tune of motor {motor + 1}", self.board.read_params())
            ok, msg = self.tuner.start(motor, amp, str(req.get("style", "normal")),
                                       bool(req.get("friction", True)), bool(req.get("ki", True)),
                                       bool(req.get("kd", True)))
            if ok:
                self.events.add("tune", f"Auto-tune of motor {motor + 1} started (style {req.get('style', 'normal')}, "
                                f"travel ±{amp}).", motor=motor + 1)
            self._json({"ok": ok, "error": None if ok else msg})
        elif p == "/eeprom/save":
            if not self.board.connected:
                self._json({"ok": False, "error": "The board is not connected."})
                return
            pr = self.board.read_params()
            if pr.get("Kp1") is None:
                self._json({"ok": False, "error": "Couldn't read the settings from the board, so nothing was saved. "
                                                  "Try again."})
                return
            self.backups.add("eeprom", "Saved to EEPROM", pr)
            self.board.cmd_text("sav")
            self.events.add("eeprom", "Saved to EEPROM: " + summary(pr) + ".", quiet=True)
            self._json({"ok": True})
        elif p == "/backups/now":
            if not self.board.connected:
                self._json({"ok": False, "error": "The board is not connected."})
                return
            pr = self.board.read_params()
            bid = self.backups.add("manual", str(req.get("label", "")).strip() or "Manual backup", pr)
            self._json({"ok": bid is not None, "error": None if bid else "Couldn't read the settings from the board."})
        elif p == "/backups/restore":
            b = self.backups.get(str(req.get("id", "")))
            if b is None:
                self._json({"ok": False, "error": "That backup doesn't exist any more."})
                return
            if not self.board.connected or self.tuner.running() or self.diag.moving():
                self._json({"ok": False, "error": "Board not connected, or auto-tune / the diagnostics check is running."})
                return
            self.backups.add("auto", f"Before switching to the backup from {b['when'][:16]}", self.board.read_params())
            apply_params(self.board, b["params"])
            self.events.add("backup", f"Switched to the backup from {b['when'][:16]} ({b['label']})"
                            + (" and saved it to EEPROM." if req.get("save") else "."), quiet=True)
            if req.get("save"):
                time.sleep(0.5)
                self.board.cmd_text("sav")
                self.backups.add("eeprom", f"Saved to EEPROM (switched to the backup from {b['when'][:16]})", b["params"])
            self._json({"ok": True})
        elif p == "/backups/delete":
            self._json({"ok": self.backups.delete(str(req.get("id", "")))})
        elif p == "/genres/preview":
            base_name = str(req.get("base", ""))
            try:
                if base_name:
                    base = self.profiles.get(base_name)
                    if base is None:
                        raise ValueError("that profile doesn't exist any more")
                else:
                    if not self.board.connected:
                        raise ValueError("the board is not connected - choose a saved profile to start from instead")
                    base = self.board.read_params()
                self._json(genre_preview(base, str(req.get("genre", "")), (self.board.fw_cache or [0, 0])[1]))
            except (ValueError, KeyError, TypeError) as e:
                self._json({"ok": False, "error": f"Can't work that out: {e}."})
        elif p == "/profiles/save":
            name = str(req.get("name", "")).strip()[:40]
            if not name:
                self._json({"ok": False, "error": "Type a name for the profile first."})
                return
            if isinstance(req.get("params"), dict) and req["params"].get("Kp1") is not None:
                ok = self.profiles.save(name, canon(req["params"]))
                self._json({"ok": ok, "error": None if ok else "Couldn't write profiles.json on the NAS."})
                return
            if not self.board.connected:
                self._json({"ok": False, "error": "The board is not connected, so there is nothing to save."})
                return
            pr = self.board.read_params()
            if pr.get("Kp1") is None or pr.get("Kp3") is None:
                self._json({"ok": False, "error": "Couldn't read all settings from the board. Try again."})
                return
            ok = self.profiles.save(name, pr)
            self._json({"ok": ok, "error": None if ok else "Couldn't write profiles.json on the NAS."})
        elif p == "/profiles/load":
            prof = self.profiles.get(str(req.get("name", "")))
            if prof is None:
                self._json({"ok": False, "error": "That profile doesn't exist any more."})
                return
            if not self.board.connected or self.tuner.running() or self.diag.moving():
                self._json({"ok": False, "error": "Board not connected, or auto-tune / the diagnostics check is running."})
                return
            self.backups.add("auto", f'Before loading profile "{req.get("name", "")}"', self.board.read_params())
            apply_params(self.board, prof)
            self.events.add("profile", f'Profile "{req.get("name", "")}" loaded: {summary(prof)}.', quiet=True)
            self._json({"ok": True})
        elif p == "/profiles/delete":
            self._json({"ok": self.profiles.delete(str(req.get("name", "")))})
        elif p == "/params/apply":
            if not self.board.connected or self.tuner.running() or self.diag.moving():
                self._json({"ok": False, "error": "Board not connected, or auto-tune / the diagnostics check is running."})
                return
            try:
                if not isinstance(req.get("params"), dict) or req["params"].get("Kp1") is None:
                    raise ValueError("no motor settings in it")
                self.backups.add("auto", str(req.get("label") or "Before applying settings")[:80], self.board.read_params())
                apply_params(self.board, req.get("params"))
                self.events.add("setting", f"Settings applied ({str(req.get('label') or 'from a file')[:80]}): "
                                f"{summary(req['params'])}.", quiet=True)
                self._json({"ok": True})
            except (TypeError, ValueError) as e:
                self._json({"ok": False, "error": f"That file can't be used ({e})."})
        elif p == "/tune/abort":
            self.tuner.abort()
            self._json({"ok": True})
        elif p == "/tune/revert":
            self._json({"ok": self.tuner.revert()})
        else:
            self._json({"error": "not found"}, 404)


def sync_unused(board, cfg):
    """Switch off motors beyond the number in use (firmware 1.5+ has the disable command)."""
    fw = (board.fw_cache or [0, 0])[1]
    if fw < 15 or not cfg.get("disable_unused") or not board.connected:
        return
    for m in range(int(cfg.get("motors")), 3):
        board.cmd_text(f"di{m + 1}")


def main():
    ap = argparse.ArgumentParser(description="SMC3 R4 live scope / tuner / auto-tuner")
    ap.add_argument("--host", default=os.environ.get("SMC3_HOST", ""), help="board IP address or hostname")
    ap.add_argument("--port", default=os.environ.get("SMC3_PORT", ""), help="board telemetry port (4210)")
    ap.add_argument("--web", type=int, default=int(os.environ.get("SMC3_WEB_PORT", "8080")),
                    help="web port for the page")
    ap.add_argument("--bind", default=os.environ.get("SMC3_BIND", "127.0.0.1"),
                    help="127.0.0.1 = this machine only (default); 0.0.0.0 = reachable from the LAN")
    default_data = "/data" if os.path.isdir("/data") else os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                                        "smc3-scope-data")
    ap.add_argument("--data", default=os.environ.get("SMC3_DATA_DIR", default_data),
                    help="folder for config.json (settings made in the page)")
    args = ap.parse_args()

    cfg = Config(args.data, args.host.strip(), str(args.port).strip())
    events = EventLog(args.data)
    board = Board(cfg)
    board.events = events
    gen = Generator(board)
    Handler.cfg, Handler.board, Handler.gen, Handler.events = cfg, board, gen, events
    Handler.tuner = Tuner(board, gen)
    Handler.tuner.events = events
    Handler.finder = AutoFinder(board, cfg)
    Handler.profiles = Profiles(args.data)
    Handler.backups = Backups(args.data)
    Handler.monitor = Monitor(board, gen, Handler.tuner, cfg, events, args.data)
    Handler.diag = Diagnostics(board, gen, Handler.tuner, cfg, events, Handler.monitor, Handler.backups,
                               Handler.profiles, args.data)
    Handler.monitor.diag = Handler.diag
    events.add("scope", f"smc3_scope v{VERSION} started (board {cfg.get('host') or 'not set'}:{cfg.get('port')}, "
               f"{cfg.get('motors')} motors in use).", quiet=True)

    def on_connect():
        # remember what the board runs when we first see it, and keep unused motors off
        time.sleep(1.5)
        pr = board.read_params()
        if pr.get("Kp1") is not None:
            Handler.backups.add("auto", "Settings found on the board when the page connected", pr)
            fw = (board.fw_cache or [0, 0])[1]
            s = board.latest()
            events.add("board", f"Board answered: firmware {f'{fw // 10}.{fw % 10}' if fw else '1.3 or older'}, "
                       f"running for {_ago(s['ms'] / 1000) if s else '?'}; settings {summary(pr)}.", quiet=True)
        sync_unused(board, cfg)
    board.on_connect = on_connect
    board.start()

    srv = None
    for port in range(args.web, args.web + 50):          # requested port, or the next free one after it
        try:
            srv = ThreadingHTTPServer((args.bind, port), Handler)
            break
        except OSError as e:
            print(f"port {port} busy ({e.strerror}), trying {port + 1}")
    if srv is None:
        raise SystemExit("no free port found")
    srv.daemon_threads = True
    where = "localhost" if args.bind == "127.0.0.1" else "<this machine's IP>"
    log(f"smc3_scope v{VERSION}   board {cfg.get('host') or '(not set)'}:{cfg.get('port')}   "
        f"page http://{where}:{port}   settings {cfg.path}")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
