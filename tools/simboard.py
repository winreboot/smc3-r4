"""
Simulated SMC3_R4 board (firmware 1.9, or 1.3-1.8 behaviour with --fw) - try smc3_scope without a rig, and
test its tuning and diagnostics against known faults.

  python3 tools/simboard.py 4210 --tuned            then:  python3 scope/smc3_scope.py --host 127.0.0.1

Same TCP protocol, telemetry and control law as the firmware; worm-geared motor model; plus fault injection:

  --reverse M          motor M's drive is reversed (motor wires swapped)
  --swap-motors A,B    channel A's bridge drives physical motor B and vice versa
  --swap-pots A,B      feedback channel A reads physical pot B and vice versa
  --noisy M            pot noise of 5 counts on motor M
  --glitch M           pot wiper drops out (reads ~0 for 1 ms) every ~1.5 s
  --dead M             no power reaches motor M
  --slip M             pot of motor M doesn't turn with it (slipped coupling)
  --oneway M           motor M can only move up (one half of the bridge dead)
  --simhub             a fake SimHub drives all three motors over "USB"
  --simhub-after SEC   ... starting SEC seconds after the first client connects
                       (or send [zs1] / [zs0] over TCP to switch it on / off)
  --reboot-at SEC      board restarts SEC seconds after start (cold start, connection dropped)
  --pwmmax0 M          motor M starts with PWMmax 0 (as after an auto-tune safety stop)
  --hunt M             motor M starts with Kp 1600 (hunts at rest)
  --fw N               13, 14, 15, 16, 17, 18 or 19 (default 19)

Rig layout (page 2.1):
  --layout tl          default: motors 1 and 2 lift the front corners (roll from their difference, pitch from their
                       average, 0.03 degrees per count, the rear rides on a pivot) and motor 3 slides the rear left
                       and right - traction loss, the seat turning about a pivot at the front, 0.02 degrees per
                       count, higher numbers = rear to the RIGHT
  --layout lifts       three lifts: roll from motors 1 and 2, pitch from their average against motor 3
  --tl-left            (tl) higher numbers move the rear to the LEFT instead

Motion sensor (firmware 1.8): a WT901C on the seat (mounted 1.3/-0.8 degrees off level):
  --imu HZ             sensor fitted, sending at HZ (10 = factory setting, 100 after "set to 100 Hz")
  --imu-no-tx          the board's TX isn't wired to the sensor, so "set to 100 Hz" does nothing
  --imu-lost-at SEC    the sensor goes quiet SEC seconds after start
  --flex               bouncy seat: the frame flexes (4 Hz, little damping) instead of following stiffly
  --slop M:COUNTS      play in motor M's linkage: the seat doesn't follow until the motor moves COUNTS
  --heavy M            heavy load on motor M: lifting needs much more drive, the more the higher it is
  --sensor-on          start with the sensor switched on in the board's settings

Seat set-up (firmware 1.9):
  --seat-tilt R:P      the seat isn't level with every motor at 512: R degrees roll, P degrees pitch
  --stops M:LO:HI      motor M hits a mechanical stop at pot LO and HI (default 30 and 993 for every motor)
  --seat-setup         start with a saved seat set-up (level trims, travel limits, end stops, tilt limits)

Soft start (firmware 1.9.1): after power-up and Enable Motors each motor creeps to its position (60 counts/s):
  --start M:POT        motor M's pot reads POT at power-up (the seat rode down while the rig was off); the board then
                       heads for its centre (512 + level trim), as the real one does
  --old19              firmware 1.9 without the soft start (diagnostics frame version 3)

Firmware update over WiFi (firmware 1.9.2, the default): [uP=] [uT=] [uI=] [uJ=] [uGO] [uOK] [rdu]; the board fetches
http://<page>:<port>/ota/<token>.ota, decodes and checks it like the UNO R4 WiFi's WiFi module (magic, LZSS, CRC),
"programs" itself - it drops the connection, stays away ~4 s and comes back as the version in the file (1.9.2+ with
updates, 1.9.1 soft start only, 1.9 neither), with the soft start - and remembers its WiFi like 1.9.2 does.
  --fw191              firmware 1.9.1 (soft start, no updates from the page; diagnostics frame version 4)
  --wifi-fw X.Y.Z      the WiFi module's own firmware (default 0.5.2; 0.4.x = no progress while downloading,
                       older = refused)
  --ota-fail WHAT      crc = the file arrives damaged, flash = the WiFi module can't program the board
"""
import argparse, math, random, re, socket, struct, threading, time, urllib.request, zlib

ap = argparse.ArgumentParser()
ap.add_argument("port", type=int)
ap.add_argument("--bind", default="127.0.0.1")
ap.add_argument("--reverse", type=int, default=0)
ap.add_argument("--swap-motors", default="")
ap.add_argument("--swap-pots", default="")
ap.add_argument("--noisy", type=int, default=0)
ap.add_argument("--glitch", type=int, default=0)
ap.add_argument("--dead", type=int, default=0)
ap.add_argument("--slip", type=int, default=0)
ap.add_argument("--oneway", type=int, default=0)
ap.add_argument("--simhub", action="store_true")
ap.add_argument("--simhub-after", type=float, default=0)
ap.add_argument("--reboot-at", type=float, default=0)
ap.add_argument("--pwmmax0", type=int, default=0)
ap.add_argument("--hunt", type=int, default=0)
ap.add_argument("--fw", type=int, default=19)
ap.add_argument("--imu", type=float, default=0)
ap.add_argument("--imu-no-tx", action="store_true")
ap.add_argument("--imu-lost-at", type=float, default=0)
ap.add_argument("--flex", action="store_true")
ap.add_argument("--slop", default="")
ap.add_argument("--heavy", type=int, default=0)
ap.add_argument("--sensor-on", action="store_true")
ap.add_argument("--seat-tilt", default="")
ap.add_argument("--stops", action="append", default=[])
ap.add_argument("--seat-setup", action="store_true")
ap.add_argument("--layout", choices=["tl", "lifts"], default="tl")
ap.add_argument("--start", action="append", default=[])
ap.add_argument("--old19", action="store_true")
ap.add_argument("--fw191", action="store_true")
ap.add_argument("--wifi-fw", default="0.5.2")
ap.add_argument("--ota-fail", choices=["crc", "flash"], default=None)
ap.add_argument("--tl-left", action="store_true")
ap.add_argument("--tuned", action="store_true", help="start with sane settings (Kp 250, Ki 0, Kd 0, PWMmin 30, dead zone 2)")
args = ap.parse_args()

PID_HZ, SUB = 4000, 40
DT = 1.0 / PID_HZ
VMAX, TAU, TLAG = 2200.0, 0.09, 0.006
FRIC_UP, FRIC_DN = 0.13, 0.05
FULL = 4080
drvmap, potmap = [0, 1, 2], [0, 1, 2]
if args.swap_motors:
    a, b = [int(x) - 1 for x in args.swap_motors.split(",")]
    drvmap[a], drvmap[b] = b, a
if args.swap_pots:
    a, b = [int(x) - 1 for x in args.swap_pots.split(",")]
    potmap[a], potmap[b] = b, a
SLOP = [0.0, 0.0, 0.0]
if args.slop:
    m_, c_ = args.slop.split(":")
    SLOP[int(m_) - 1] = float(c_)
DEG_PER_COUNT = 0.03
YAW_PER_COUNT = 0.02                          # --layout tl: degrees the seat turns per count of motor 3
TL_DIR = -1 if args.tl_left else 1            # +1: higher numbers move the rear to the right
SENSOR_OFFSET = (1.3, -0.8)                   # the sensor isn't mounted perfectly level
SEAT_TILT = tuple(float(x) for x in args.seat_tilt.split(":")) if args.seat_tilt else (0.0, 0.0)
STOPS = [[30.0, 993.0] for _ in range(3)]     # mechanical ends of each motor's travel (pot counts)
START = {}                                    # --start: where the pots are at power-up
for st_ in args.start:
    m_, pot_ = st_.split(":")
    START[int(m_) - 1] = float(pot_)
FW = {"ver": ("1.9" if args.old19 else ("1.9.1" if args.fw191 else "1.9.2")) if args.fw >= 19 else f"{args.fw // 10}.{args.fw % 10}"}


def _vt(v):
    try:
        return tuple(int(x) for x in v.split("."))
    except ValueError:
        return (0,)


def soft_fw():                                # firmware 1.9.1: soft start
    return args.fw >= 19 and _vt(FW["ver"]) >= (1, 9, 1)


def ota_fw():                                 # firmware 1.9.2: updates from the page, WiFi memory
    return args.fw >= 19 and _vt(FW["ver"]) >= (1, 9, 2)


WIFI_FW = tuple(int(x) for x in args.wifi_fw.split("."))
OTA = {"state": 0, "code": 0, "done": 0, "total": 0, "progress": False, "seq": 0, "port": 0, "token": 0, "ip": [0, 0],
       "url": "", "blocked": False, "placeholder": False}
OFFLINE_UNTIL = [0.0]                         # "programming": the board doesn't answer until then
SOFT = {"rate": 60}                           # counts/s (0 = off), EEPROM 105-107
for st_ in args.stops:
    m_, lo_, hi_ = st_.split(":")
    STOPS[int(m_) - 1] = [float(lo_), float(hi_)]
BAUDS = [9600, 115200, 230400, 57600, 38400, 19200, 4800]


class Phys:                                   # the motor + linkage + pot
    def __init__(self, i):
        self.i = i
        self.x = START.get(i, 512.0 + random.uniform(-30, 30)); self.v = 0.0; self.dl = 0.0; self.drive = 0.0; self.brake = True
        self.pot_x = self.x                       # where the pot thinks it is (differs only with --slip)
        self.link = self.x                        # where the seat's end of the linkage is (differs with --slop)


class Seat:                                   # the seat on the three linkages, and the sensor on it
    def __init__(self):
        self.roll = self.pitch = 0.0; self.vr = self.vp = 0.0
        self.yaw = 0.0; self.vy = 0.0; self.vy_prev = 0.0  # tl: the seat's turn, + = rear to the right (nose left)
        self.rate = args.imu; self.baud = 9600 if args.imu <= 20 else 115200
        self.last_out = 0.0; self.out = None      # last sensor reading (raw roll, raw pitch, yaw)
        self.packets = 0; self.bad = 0; self.found_at = None
        self.all = True                           # factory: acceleration, gyro and angles; [iS 0] = angles only
        self.acc = (0, 0, 2048); self.gyro = (0, 0, 0); self.acc_t = 0.0; self.acc_n = 0; self.peak = 0
        self.vr_prev = 0.0; self.vp_prev = 0.0


SEAT = Seat()


class Chan:                                   # the firmware's per-motor state
    def __init__(self, i):
        self.i = i
        self.target = 512; self.goal_acc = 512 << 12; self.goal_q = 512 << 4
        self.kp, self.ki, self.kd, self.ks = 100, 40, 40, 1
        self.pwmmin, self.pwmmax, self.pwmrev, self.dz = 50, 100, 200, 0
        self.cut, self.clip = 23, 100
        self.cum = 0; self.ilim = 0; self.lastpos = 512 << 4; self.dterm = 0; self.kdcnt = 0
        self.disabled = True; self.reason = 3; self.fb_q = 512 << 4; self.pid_q = 0
        self.trips = 0; self.trip_at = -1; self.jumps = 0; self.jmax = 0; self.fmin = 16383; self.fmax = 0
        self.trim, self.tlo, self.thi, self.elo, self.ehi = 0, 0, 1023, 0, 1023     # 1.9 seat set-up
        self.soft, self.soft_ticks, self.soft_out, self.soft_since = False, 0, 0, 0.0   # 1.9.1 soft start
        self.update_ilim()

    def window(self):                         # target limits in force: SMC3 clip and the travel limits (1.9)
        lo, hi = max(self.clip, self.tlo), min(1023 - self.clip, self.thi)
        return (self.clip, 1023 - self.clip) if hi < lo + 8 else (lo, hi)

    def cut_window(self):                     # cutoff in force: SMC3 Max limit and the end stops (1.9)
        lo, hi = max(self.cut, self.elo), min(1023 - self.cut, self.ehi)
        return (self.cut, 1023 - self.cut) if hi < lo + 8 else (lo, hi)

    def set_target(self, v):                  # SetTarget(): level trim, then the window (1.9)
        if args.fw >= 19:
            v += self.trim
        lo, hi = self.window()
        self.target = int(max(lo, min(hi, v)))

    def update_ilim(self):
        self.ilim = ((FULL << 13) // self.ki) if self.ki > 0 else 0


lock = threading.Lock()
TICKS = [0]
SIMHUB = [False]                          # switched with [zs1] / [zs0] from any client (simulator only)
HOLD = [0.0]                              # 1.9 [kH n]: SimHub's positions are ignored until then
EXTRAS_ANY = [False]
VREG = [0]                                # 1.9 [w hi lo]
def soft_start_from(c):
    """1.9.1: creep from where the motor is to its target (EnableMotor / power-up)."""
    lo, hi = c.cut_window()
    fb = c.fb_q
    c.soft_out = (lo * 16 - fb) if fb < lo * 16 else ((fb - hi * 16) if fb > hi * 16 else 0)
    c.soft_ticks, c.soft = 0, True
    c.goal_acc = fb << 8; c.goal_q = fb
    c.cum = 0; c.lastpos = fb; c.dterm = 0
    c.soft_since = time.time()


def any_soft():
    return any(c.soft and not c.disabled for c in C)


def enable(c):
    if c.disabled and soft_fw() and SOFT["rate"] > 0:
        soft_start_from(c)
    c.disabled, c.reason = False, 0


TILT4 = [0, 0, 0, 0]                      # 1.9 tilt limits x10: roll +, roll -, pitch +, pitch -
state = {}


eeprom = None
P = [Phys(i) for i in range(3)]


def boot(cold=True):
    """Power-up: params from 'EEPROM', motors enabled, counters reset."""
    global C, power_scale, last_target_t, t0, fshift, frate, usb, eeprom, reset_flags, board
    C = [Chan(i) for i in range(3)]
    fshift, frate = 6, 0
    # 1.8 board settings (EEPROM 56-65), sensor and matrix state
    board = {"view": 0, "lflags": 3, "sflags": 1 if args.sensor_on else 0, "tilt": 0, "lv_r": 0, "lv_p": 0,
             "baud_idx": 0, "tilt_hits": 0, "tilt_trips": 0, "grace": time.time() + 1.5, "last_pos": time.time(),
             "setup_until": 0.0, "scan_until": 0.0}
    first = eeprom is None
    if first:                                 # very first power-up: defaults (+ --hunt) become the saved values
        if args.tuned:
            for c in C:
                c.kp, c.ki, c.kd, c.pwmmin, c.dz = 250, 0, 0, 30, 2
                c.update_ilim()
        if args.hunt:
            h = C[args.hunt - 1]
            h.kp, h.ki, h.kd, h.dz, h.pwmmin = 2600, 0, 0, 0, 90
        eeprom = bytearray(128)
        save_eeprom()
        if args.seat_setup and args.fw >= 19:   # a seat set-up made earlier (Seat setup in the page)
            for c, (tr, lo, hi) in zip(C, ((18, 140, 880), (-24, 150, 870), (6, 160, 860))):
                c.trim, c.tlo, c.thi, c.elo, c.ehi = tr, lo, hi, lo - 15, hi + 15
            TILT4[:] = [130, 130, 110, 110]
            save_seat()
        if args.pwmmax0:                      # running-only, like auto-tune's safety stop
            C[args.pwmmax0 - 1].pwmmax = 0
    else:
        load_eeprom()
    if eeprom[56] == 0xA6:
        board.update(view=eeprom[57], lflags=eeprom[58], sflags=eeprom[59] & 15, tilt=eeprom[60],
                     lv_r=struct.unpack(">h", bytes(eeprom[61:63]))[0], lv_p=struct.unpack(">h", bytes(eeprom[63:65]))[0],
                     baud_idx=eeprom[65] % 7)
    if args.fw >= 19:
        load_seat()
    if soft_fw() and eeprom[105] == 0xB5:
        SOFT["rate"] = struct.unpack(">h", bytes(eeprom[106:108]))[0]
    SEAT.found_at = None; SEAT.packets = 0
    for c in C:
        p = P[potmap[c.i]]
        c.fb_q = int(p.pot_x * 16); c.lastpos = c.fb_q
        lo, hi = c.window()
        c.goal_acc = (max(lo, min(hi, int(p.pot_x))) << 12); c.goal_q = c.goal_acc >> 8
        if args.fw >= 19 and (any(ch.trim for ch in C) or START):
            c.set_target(512)                 # 1.9 with a seat set-up: the centre - the level seat
        else:                                 # (simplified: the real board also heads for 512 without one)
            c.target = max(lo, min(hi, int(p.pot_x)))
        c.disabled, c.reason = True, 3
        enable(c)                             # 1.9.1: creeps from where it is
    power_scale = 7
    last_target_t = 0.0
    t0 = time.time()
    usb = {"ok": 0, "tgt": 0, "bad": 0, "tcp": 0, "last": 0.0, "last_tgt": 0.0}
    reset_flags = [1, 0, 0] if cold else [0, 0, 1]


def save_eeprom():
    e = eeprom
    e[0] = 114
    for i, (cut, clip, dz) in enumerate([(1, 2, 5), (6, 7, 10), (31, 32, 35)]):
        e[cut], e[clip], e[dz] = C[i].cut, C[i].clip, C[i].dz
    for i, (kp, ki, kd) in enumerate([(11, 13, 15), (17, 19, 21), (36, 38, 40)]):
        for a, v in ((kp, C[i].kp), (ki, C[i].ki), (kd, C[i].kd)):
            e[a], e[a + 1] = (v >> 8) & 255, v & 255
    e[23], e[24], e[42] = C[0].pwmmin, C[1].pwmmin, C[2].pwmmin
    e[25], e[26], e[43] = C[0].pwmmax, C[1].pwmmax, C[2].pwmmax
    e[27], e[28], e[29] = C[0].pwmrev, C[1].pwmrev, C[2].pwmrev
    for i, a in enumerate((44, 46, 48)):
        e[a], e[a + 1] = (C[i].ks >> 8) & 255, C[i].ks & 255
    e[50], e[51], e[52] = 1, 25, 25
    if args.fw >= 16:
        e[53], e[54], e[55] = fshift, frate // 100, 0x5A


def save_board():
    e = eeprom
    e[57], e[58], e[59], e[60] = board["view"], board["lflags"], board["sflags"], tilt_deg()
    e[61:63] = struct.pack(">h", board["lv_r"]); e[63:65] = struct.pack(">h", board["lv_p"])
    e[65] = board["baud_idx"]; e[56] = 0xA6


def tilt_deg():                               # 1.8's single tilt limit: the tightest of the four (1.9)
    if args.fw < 19:
        return board["tilt"]
    nz = [t for t in TILT4 if t > 0]
    return min(90, max(1, (min(nz) + 5) // 10)) if nz else 0


def seat_bytes():
    b = b""
    for c in C:
        b += struct.pack(">5h", c.trim, c.tlo, c.thi, c.elo, c.ehi)
    return b + struct.pack(">4h", *TILT4)


def save_seat():                              # [kSV] / [sav]: EEPROM 66-104
    eeprom[67:105] = seat_bytes(); eeprom[66] = 0xB9


def clear_seat():
    for c in C:
        c.trim, c.tlo, c.thi, c.elo, c.ehi = 0, 0, 1023, 0, 1023


def load_seat():                              # [kRL] and power-up; 1.8's single tilt limit as the fallback
    clear_seat()
    TILT4[:] = [board["tilt"] * 10] * 4
    if eeprom[66] == 0xB9:
        for i, c in enumerate(C):
            c.trim, c.tlo, c.thi, c.elo, c.ehi = struct.unpack(">5h", bytes(eeprom[67 + 10 * i: 77 + 10 * i]))
        TILT4[:] = struct.unpack(">4h", bytes(eeprom[97:105]))
    for c in C:
        lo, hi = c.window(); c.target = max(lo, min(hi, c.target))


def seat_saved():
    if eeprom[66] != 0xB9:
        return all((c.trim, c.tlo, c.thi, c.elo, c.ehi) == (0, 0, 1023, 0, 1023) for c in C)
    return bytes(eeprom[67:105]) == seat_bytes()


def sensor_fresh(now):
    return SEAT.out is not None and now - SEAT.last_out < 0.5


def seat_angles():
    """Telemetry angles: levelled, then turned the way the sensor is mounted (board settings)."""
    r = SEAT.out[0] - board["lv_r"] / 100.0
    p = SEAT.out[1] - board["lv_p"] / 100.0
    f = board["sflags"]
    if f & 2: r, p = p, r
    if f & 4: r = -r
    if f & 8: p = -p
    return r, p


def matrix_error(now):
    for i, c in enumerate(C):
        if c.disabled and c.reason == 1: return i + 1
    if any(c.disabled and c.reason == 4 for c in C): return 4
    if (board["sflags"] & 1) and not sensor_fresh(now) and now - t0 > 8: return 5
    return 255


def sensor_tick(now):
    """Called every 10 ms: the sensor sends at its rate; the board levels, turns and checks the tilt limit."""
    if not args.imu or (args.imu_lost_at and now - started > args.imu_lost_at):
        return
    if SEAT.found_at is None:                 # the board tries baud rates until it finds the sensor
        SEAT.found_at = now + (0.4 if BAUDS[board["baud_idx"]] == SEAT.baud else 1.6)
        board["baud_idx"] = BAUDS.index(SEAT.baud)
    if now < SEAT.found_at or now < board["setup_until"]:
        return
    if now - SEAT.last_out < 1.0 / SEAT.rate - 0.002:
        return
    q = lambda d: round(d * 32768 / 180) * 180 / 32768
    SEAT.out = (q(SEAT.roll + SENSOR_OFFSET[0] + random.gauss(0, 0.015)),
                q(SEAT.pitch + SENSOR_OFFSET[1] + random.gauss(0, 0.015)), q(((37.5 + SEAT.yaw + random.gauss(0, 0.05)) + 180) % 360 - 180))
    dt = max(1e-3, now - SEAT.last_out)
    SEAT.last_out = now; SEAT.packets += 3
    if SEAT.all:                              # acceleration (g) and gyro (deg/s) as the sensor sends them, raw
        rr, pp = math.radians(SEAT.out[0]), math.radians(SEAT.out[1])
        ar = (SEAT.vr - SEAT.vr_prev) / dt; ap_ = (SEAT.vp - SEAT.vp_prev) / dt      # deg/s^2
        ay_ = (SEAT.vy - SEAT.vy_prev) / dt
        SEAT.vr_prev, SEAT.vp_prev, SEAT.vy_prev = SEAT.vr, SEAT.vp, SEAT.vy
        lever = 0.35 / 9.81 * math.pi / 180.0                                          # 35 cm above the pivot
        gx = math.sin(pp) + ap_ * lever + random.gauss(0, 0.004)
        gy = -math.sin(rr) * math.cos(pp) - ar * lever + ay_ * 1.0 / 9.81 * math.pi / 180.0 + random.gauss(0, 0.004)
        gz = math.cos(rr) * math.cos(pp) + random.gauss(0, 0.006)
        cl = lambda v: max(-32767, min(32767, int(round(v))))
        SEAT.acc = (cl(gx * 2048), cl(gy * 2048), cl(gz * 2048))
        SEAT.gyro = (cl((SEAT.vr + random.gauss(0, 0.08)) * 16.384), cl((SEAT.vp + random.gauss(0, 0.08)) * 16.384),
                     cl((SEAT.vy + random.gauss(0, 0.05)) * 16.384))
        SEAT.acc_t = now; SEAT.acc_n += 1; SEAT.packets += 0
        SEAT.peak = max(SEAT.peak, int(abs(math.sqrt(gx * gx + gy * gy + gz * gz) - 1.0) * 1000))
    lim = board["tilt"]
    if args.fw >= 19:
        r, p = seat_angles()
        past = ((TILT4[0] and r > TILT4[0] / 10.0) or (TILT4[1] and r < -TILT4[1] / 10.0)
                or (TILT4[2] and p > TILT4[2] / 10.0) or (TILT4[3] and p < -TILT4[3] / 10.0))
        lim = 1 if any(TILT4) else 0
    if any_soft():                            # 1.9.1: the tilt limit waits while the seat creeps back
        board["grace"] = now + 1.5
    if (board["sflags"] & 1) and lim and now >= board["grace"]:
        r, p = seat_angles()
        if (past if args.fw >= 19 else (abs(r) > lim or abs(p) > lim)):
            board["tilt_hits"] += 1
            if board["tilt_hits"] >= 3:
                board["tilt_hits"] = 0
                hit = False
                for c in C:
                    if not c.disabled: c.disabled, c.reason, hit = True, 4, True
                if hit: board["tilt_trips"] += 1
        else:
            board["tilt_hits"] = 0
    else:
        board["tilt_hits"] = 0


def load_eeprom():
    global fshift, frate
    e = eeprom
    w = lambda a: e[a] * 256 + e[a + 1]
    for i, (cut, clip, dz) in enumerate([(1, 2, 5), (6, 7, 10), (31, 32, 35)]):
        C[i].cut, C[i].clip, C[i].dz = e[cut], e[clip], e[dz]
    for i, (kp, ki, kd) in enumerate([(11, 13, 15), (17, 19, 21), (36, 38, 40)]):
        C[i].kp, C[i].ki, C[i].kd = w(kp), w(ki), w(kd); C[i].update_ilim()
    for i, (a, b, r) in enumerate([(23, 25, 27), (24, 26, 28), (42, 43, 29)]):
        C[i].pwmmin, C[i].pwmmax, C[i].pwmrev = e[a], e[b], e[r]
    for i, a in enumerate((44, 46, 48)):
        C[i].ks = max(1, w(a))
    if e[55] == 0x5A:
        fshift, frate = min(8, e[53]), e[54] * 100


def chan_tick(c):
    """One PID cycle for channel c: returns the signed drive (-1..1) and brake flag for its bridge."""
    diff = (c.target << 12) - c.goal_acc
    step = (diff >> fshift) if fshift else diff
    if step == 0: step = diff
    if frate:
        lim = int(frate * 4096 / 4000); step = max(-lim, min(lim, step))
    if c.soft:                                # 1.9.1: creeping after power-up / Enable Motors
        ad = abs(diff)
        sstep = int(SOFT["rate"] * 4096 / 4000)
        if ad <= (2 << 12) or sstep <= 0:
            c.soft = False
        else:
            lim = sstep
            if c.soft_ticks < 2400:
                c.soft_ticks += 1; lim = lim * c.soft_ticks // 2400
            lim = max(1, min(lim, (ad >> 10) + 1))
            step = max(-lim, min(lim, step))
    c.goal_acc += step
    c.goal_q = (c.goal_acc + 128) >> 8
    p = P[potmap[c.i]]
    noise = 5.0 if args.noisy == c.i + 1 else 0.35
    pos = p.pot_x + random.gauss(0, noise)
    TICKS[0] += 1
    if args.glitch == c.i + 1 and (TICKS[0] // 3) % 6000 < 4:     # wiper drops out for ~1 ms every 1.5 s
        pos = random.uniform(150, 300)
    fb = max(0, min(16383, int(round(pos * 16))))
    j = abs(fb - c.fb_q)
    if j > 16 * 16 and c.jumps < 65535: c.jumps += 1
    c.jmax = max(c.jmax, j); c.fmin = min(c.fmin, fb); c.fmax = max(c.fmax, fb)
    c.fb_q = fb
    clo, chi = c.cut_window() if args.fw >= 19 else (c.cut, 1023 - c.cut)
    past = fb > (chi << 4) or fb < (clo << 4)
    if past and c.soft and c.soft_out > 0:   # 1.9.1: coming back from past an end stop after power-up
        out_q = (clo << 4) - fb if fb < (clo << 4) else fb - (chi << 4)
        hard = fb > ((1023 - c.cut) << 4) or fb < (c.cut << 4)
        if not hard and out_q <= c.soft_out + (16 << 4):
            past = False; c.soft_out = min(c.soft_out, out_q)
    elif not past and c.soft_out:             # well inside (16 counts): the end stops count as usual again
        inside = min(fb - (clo << 4), (chi << 4) - fb)
        c.soft_out = 0 if (inside > (16 << 4) or not c.soft) else 1
    if past:
        if not c.disabled:
            c.trips += 1; c.trip_at = fb >> 4; c.reason = 1
        c.disabled = True
    err = c.goal_q - fb; dzq = c.dz << 4
    pt = c.kp * err
    if (-dzq <= err <= dzq) or c.ilim == 0: c.cum = 0
    else:
        c.cum += err; c.cum = max(-c.ilim, min(c.ilim, c.cum))
    it = (c.ki * c.cum) >> 6
    c.kdcnt += 1
    if c.kdcnt >= c.ks:
        d = fb - c.lastpos; c.lastpos = fb; c.kdcnt = 0
        d = max(-4095, min(4095, d))
        c.dterm += (((c.kd * d) << 5) - c.dterm) >> 3
    out = (pt + it - c.dterm) >> power_scale; out = max(-FULL, min(FULL, out))
    c.pid_q = out
    if c.disabled:
        return 0.0, True
    cmax, cmin = (1023 - c.clip) << 4, c.clip << 4
    offq, maxq, revq = c.pwmmin << 4, c.pwmmax << 4, c.pwmrev << 4
    rev = c.pwmrev and not c.soft             # 1.9.1: no PWMrev drive while creeping
    if fb > cmax and rev: return -revq / FULL, False
    if fb < cmin and rev: return revq / FULL, False
    if c.goal_q > fb + dzq or c.goal_q < fb - dzq:
        if out >= 0: return min(out + offq, maxq) / FULL, False
        return -min(-out + offq, maxq) / FULL, False
    return 0.0, True


def phys_tick(p, d, brake):
    if args.reverse == p.i + 1: d = -d
    if args.dead == p.i + 1: d, brake = 0.0, True
    if args.oneway == p.i + 1 and d < 0: d = 0.0
    p.dl += (d - p.dl) * DT / (0.035 if args.hunt == p.i + 1 else TLAG)   # --hunt: sluggish drive -> limit cycle
    fr = FRIC_UP if p.dl > 0 else FRIC_DN
    if args.heavy == p.i + 1 and p.dl > 0:   # --heavy: lifting this corner needs a lot more, worse near the top
        fr = 0.22 + 0.30 * max(0.0, (p.x - 560) / 300.0)
    deff = math.copysign(max(0.0, abs(p.dl) - fr), p.dl) if p.dl != 0 else 0.0
    if brake or deff == 0.0:
        p.v += -p.v * DT / 0.012
    else:
        p.v += (deff * VMAX - p.v) * DT / TAU
    p.x += p.v * DT
    lo_, hi_ = STOPS[p.i]                     # mechanical ends (--stops)
    if p.x < lo_: p.x, p.v = lo_, 0.0
    if p.x > hi_: p.x, p.v = hi_, 0.0
    if args.slip != p.i + 1:
        p.pot_x = p.x
    h = SLOP[p.i] / 2.0                       # --slop: the linkage only follows once the play is taken up
    p.link = min(max(p.link, p.x - h), p.x + h)


def seat_tick():
    """Seat angle from the linkages: stiff (follows within a few ms) or, with --flex, a bouncy spring."""
    x1, x2, x3 = (P[i].link for i in range(3))
    roll_t = DEG_PER_COUNT * (x1 - x2) / 2.0 + SEAT_TILT[0]
    if args.layout == "tl":                   # front lifts, the rear on a pivot; motor 3 turns the seat
        pitch_t = DEG_PER_COUNT * ((x1 + x2) / 2.0 - 512) + SEAT_TILT[1]
        yaw_t = YAW_PER_COUNT * TL_DIR * (x3 - 512)
    else:
        pitch_t = DEG_PER_COUNT * ((x1 + x2) / 2.0 - x3) + SEAT_TILT[1]
        yaw_t = 0.0
    wn, z = (2 * math.pi * 4.0, 0.12) if args.flex else (2 * math.pi * 22.0, 0.8)
    for ang, vel, tgt in (("roll", "vr", roll_t), ("pitch", "vp", pitch_t), ("yaw", "vy", yaw_t)):
        a, v = getattr(SEAT, ang), getattr(SEAT, vel)
        acc = wn * wn * (tgt - a) - 2 * z * wn * v
        v += acc * DT; a += v * DT
        setattr(SEAT, ang, a); setattr(SEAT, vel, v)


samples = []
clients = []
started = time.time()
first_client = [None]


def sim_loop():
    global power_scale
    nxt = time.time()
    rebooted = False
    while True:
        nxt += 0.01
        with lock:
            if args.reboot_at and not rebooted and time.time() - started > args.reboot_at:
                rebooted = True
                boot(cold=True)
                samples.clear()
                for c in list(clients):
                    try: c.close()
                    except OSError: pass
                clients.clear()
            soft_now = any_soft()                 # 1.9.1: full power while creeping; give up after 40 s
            power_scale = 9 if ((time.time() - last_target_t) > 15 and not soft_now) else 7
            if soft_now:
                for c in C:
                    if c.soft and time.time() - c.soft_since > 40: c.soft = False
            for _ in range(SUB):
                out = [chan_tick(c) for c in C]
                for i in range(3):
                    d, br = out[i]
                    phys_tick(P[drvmap[i]], d, br)
                seat_tick()
            now = time.time()
            sensor_tick(now)
            ms = int((now - t0) * 1000)
            status = 16 + sum((1 << i) for i, c in enumerate(C) if c.disabled)
            body = struct.pack("<I", ms & 0xFFFFFFFF)
            for c in C:
                body += struct.pack("<hhhh", c.target, c.goal_q, c.fb_q, c.pid_q)
            if args.fw >= 18 and sensor_fresh(now):
                r, p = seat_angles()
                cl = lambda v: max(-32767, min(32767, int(v * 100)))
                body += struct.pack("<BBhhhH", status, power_scale, cl(r), cl(p), cl(SEAT.out[2]), 70)
            elif args.fw >= 18:
                body += struct.pack("<BBhhhH", status, power_scale, -32768, -32768, -32768, 70)
            else:
                body += struct.pack("<BBhhhH", status, power_scale, 0, 0, 0, 70)
            samples.append(body)
            if len(samples) > 50: del samples[:-50]
        sl = nxt - time.time()
        if sl > 0: time.sleep(sl)
        else: nxt = time.time()


def fake_simhub():
    """Like SimHub's SMC3 output over USB: all three targets, ~200 updates/s, gentle road-like motion."""
    global last_target_t
    while True:
        time.sleep(0.005)
        fc = first_client[0]
        on = SIMHUB[0] or args.simhub or (args.simhub_after and fc and time.time() - fc > args.simhub_after)
        if not on:
            continue
        t = time.time()
        with lock:
            held = args.fw >= 19 and t < HOLD[0]  # 1.9: the page holds the seat - SimHub's positions wait
            for i, c in enumerate(C):
                v = 512 + 120 * math.sin(t * (0.7 + 0.3 * i)) + 25 * math.sin(t * 7.3 + i)
                if args.heavy:                    # a bit more travel, so the heavy corner reaches its hard part
                    v = 512 + 200 * math.sin(t * (0.7 + 0.3 * i)) + 25 * math.sin(t * 7.3 + i)
                if held:
                    usb["held"] = usb.get("held", 0) + 1
                else:
                    c.set_target(int(v))
                usb["tgt"] += 1; usb["ok"] += 1
            usb["last"] = usb["last_tgt"] = t
            last_target_t = t                     # (held or not, like the firmware: SimHub is alive)
            board["last_pos"] = t


def reply2(code, a, b): return b"[" + code.encode() + bytes([a & 255, b & 255]) + b"]"
def replyw(code, v): return b"[" + code.encode() + bytes([(v >> 8) & 255, v & 255]) + b"]"


def diag_frame():
    now = time.time()
    ms = int((now - t0) * 1000)
    since = lambda x: 65535 if not x else min(65535, int((now - x) * 1000))
    pl = struct.pack("<I3BB4I2HIbBB", ms, *reset_flags, 1, usb["ok"], usb["tgt"], usb["bad"], usb["tcp"],
                     since(usb["last"]), since(usb["last_tgt"]), random.randint(800, 4000), -58,
                     (2 | 4 | 8) if args.fw >= 18 else (2 | 8), min(args.fw, 19))
    for c in C:
        pl += struct.pack("<BHhHHHH", c.reason, c.trips, c.trip_at, c.jumps, min(65535, c.jmax), c.fmin, c.fmax)
        c.jmax, c.fmin, c.fmax = 0, c.fb_q, c.fb_q
    if args.fw < 18:
        pl += bytes([64]) + bytes(eeprom[:64])
        return b"SM3D" + bytes([1, len(pl)]) + pl
    pl += bytes([72]) + bytes(eeprom[:72])
    fresh = sensor_fresh(now)
    raw = SEAT.out or (0.0, 0.0, 0.0)
    cl = lambda v: max(-32767, min(32767, int(v * 100)))
    age = 65535 if SEAT.out is None else min(65535, int((now - SEAT.last_out) * 1000))
    rate = int(SEAT.rate) if fresh else 0
    pl += struct.pack("<BBBIHIHHhhhhhHBBB", sensor_state(now), tilt_deg(), board["baud_idx"], BAUDS[board["baud_idx"]],
                      rate, SEAT.packets, SEAT.bad, age, cl(raw[0]), cl(raw[1]), cl(raw[2]), board["lv_r"], board["lv_p"],
                      board["tilt_trips"], board["view"], board["lflags"], matrix_error(now))
    if args.fw < 19:
        return b"SM3D" + bytes([2, len(pl)]) + pl
    for c in C:                               # version 3 (1.9): seat set-up and the rest of the sensor
        pl += struct.pack("<5h", c.trim, c.tlo, c.thi, c.elo, c.ehi)
    pl += struct.pack("<4h", *TILT4)
    fl = (1 if seat_saved() else 0) | (2 if eeprom[66] == 0xB9 else 0) | (4 if EXTRAS_ANY[0] else 0) | \
        (8 if time.time() < HOLD[0] else 0)
    hz = int(SEAT.rate) if (fresh and SEAT.all) else 0
    pl += struct.pack("<BIHH3h3hh", fl, usb.get("held", 0), hz, hz, *SEAT.acc, *SEAT.gyro, 2950)
    if not soft_fw():
        return b"SM3D" + bytes([3, len(pl)]) + pl
    pl += struct.pack("<HB", SOFT["rate"], sum(1 << i for i, c in enumerate(C) if c.soft and not c.disabled))
    if not ota_fw():
        return b"SM3D" + bytes([4, len(pl)]) + pl
    wf = 1 | (6 if OTA["placeholder"] else 0)     # remembered; joined with it + a build with the placeholders
    pl += struct.pack("<3BBBh", *WIFI_FW, wf, OTA["state"], OTA["code"])
    return b"SM3D" + bytes([5, len(pl)]) + pl


def ota_frame():
    """1.9.2 "SM3U": firmware update state."""
    fl = (1 if OTA["progress"] else 0) | 2
    pl = struct.pack("<BhIIBBBB", OTA["state"], OTA["code"], OTA["done"], OTA["total"], fl, *WIFI_FW)
    pl += FW["ver"].encode()[:8].ljust(8, b"\0")
    return b"SM3U" + bytes([1, len(pl)]) + pl


def ota_set(**kw):
    OTA.update(kw)
    OTA["seq"] += 1


def ota_fail(code):
    ota_set(state=6, code=code, progress=False, blocked=False)


def lzss_decode_stream(data):
    """The WiFi module's LZSSDecoder (EI 11, EJ 4), all at once."""
    N, F = 2048, 17
    buf = bytearray(b" " * (N - F) + bytes(N + F))
    r, st, bb, bs, i0 = N - F, 0, 0, 0, 0
    out = bytearray()
    need = {0: 1, 1: 8, 2: 11, 3: 4}
    for byte in data:
        bb = (bb << 8) | byte; bs += 8
        while bs >= need[st]:
            n = need[st]; c = bb >> (bs - n); bb &= (1 << (bs - n)) - 1; bs -= n
            if st == 0: st = 1 if c else 2
            elif st == 1:
                out.append(c); buf[r] = c; r = (r + 1) & (N - 1); st = 0
            elif st == 2: i0 = c; st = 3
            else:
                for k in range(c + 2):
                    v = buf[(i0 + k) & (N - 1)]; out.append(v); buf[r] = v; r = (r + 1) & (N - 1)
                st = 0
    return bytes(out)


def ota_run():
    """[uGO]: what the firmware and its WiFi module do, in a thread."""
    time.sleep(0.3)                           # the brakes
    ota_set(state=2)
    blocking = WIFI_FW < (0, 5, 0)
    try:
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        resp = opener.open(OTA["url"], timeout=10)
    except urllib.error.HTTPError:
        ota_fail(-14); return
    except OSError:
        ota_fail(-6); return
    total = resp.headers.get("Content-Length")
    if total is None:
        ota_fail(-7); return
    total = int(total)
    ota_set(state=3, total=total, done=0, progress=not blocking, blocked=blocking)
    data = bytearray()
    while len(data) < total:
        chunk = resp.read(min(2048, total - len(data)))
        if not chunk:
            ota_fail(-12); return
        data += chunk
        if len(data) >= 12 and int.from_bytes(data[8:12], "little") != 0x23411002:
            ota_fail(-11); return
        if not blocking:
            OTA["done"] = len(data)
        time.sleep(0.04)                      # ~50 kB/s, so the progress shows
    OTA["done"] = len(data)
    if args.ota_fail == "crc":
        data[100] ^= 0x55
    ota_set(state=4, blocked=False)
    time.sleep(0.3)
    if (zlib.crc32(bytes(data[8:])) & 0xFFFFFFFF) != int.from_bytes(data[4:8], "little"):
        ota_fail(-10); return
    binary = lzss_decode_stream(bytes(data[20:]))
    if time.time() - usb["last"] < 3:         # SimHub sending over USB: not while programming
        ota_fail(-100); return
    ota_set(state=5)
    time.sleep(0.45)
    if args.ota_fail == "flash" or time.time() - usb["last"] < 3:
        ota_fail(-104 if args.ota_fail == "flash" else -100); return
    m = re.search(rb"SMC3_R4 v(\d+\.\d+(?:\.\d+)?)", binary)
    new = m.group(1).decode() if m else "?"
    print(f"simboard: programming firmware {new} ({len(binary)} bytes)", flush=True)
    OFFLINE_UNTIL[0] = time.time() + 4.0      # programming; the board restarts
    with lock:
        for cn in list(clients):
            try: cn.close()
            except OSError: pass
        clients.clear()
    time.sleep(1.0)
    with lock:
        FW["ver"] = new
        OTA.update(state=0, code=0, done=0, total=0, progress=False, blocked=False,
                   placeholder=b"YourNetworkName" in binary)
        OTA["seq"] += 1
        for i, c in enumerate(C):             # the seat sank a little while the motors were off
            START[i] = P[potmap[i]].pot_x
        boot(cold=False)
        samples.clear()


def extras_frame():
    """1.9 "SM3X": the sensor's acceleration and gyro, after every telemetry burst once the page asked ([xe1])."""
    now = time.time()
    fresh = 3 if (SEAT.all and SEAT.out is not None and now - SEAT.acc_t < 0.5) else 0
    pl = struct.pack("<3h3hhHBB", *SEAT.acc, *SEAT.gyro, 2950, min(65000, SEAT.peak), fresh, min(255, SEAT.acc_n))
    SEAT.peak = 0; SEAT.acc_n = 0
    return b"SM3X" + bytes([1, len(pl)]) + pl


def sensor_state(now):
    v = board["sflags"] & 15
    if sensor_fresh(now): v |= 0x10
    if SEAT.out is not None and now - SEAT.last_out < 3: v |= 0x20
    if now < board["setup_until"]: v |= 0x40
    if args.fw >= 19 and SEAT.all and sensor_fresh(now): v |= 0x80
    return v


def handle_cmd(c, a, b, out, cl=None):
    global last_target_t, fshift, frate
    w = a * 256 + b
    with lock:
        usb["tcp"] += 1
        if c in "ABC":
            ch = C["ABC".index(c)]
            ch.set_target(w)
            if args.fw >= 14: last_target_t = time.time()
            board["last_pos"] = time.time()
        elif c == "w" and args.fw >= 19:
            VREG[0] = w - 65536 if w >= 32768 else w
        elif c == "k" and args.fw >= 19:
            v = VREG[0]
            if b == ord("="):
                if 64 <= a < 68:
                    if 0 <= v <= 900: TILT4[a - 64] = v
                elif a in (80, 81):
                    if -9000 <= v <= 9000:
                        board["lv_r" if a == 80 else "lv_p"] = v; save_board()
                elif a == 82 and soft_fw():
                    if 0 <= v <= 1000:
                        SOFT["rate"] = v; eeprom[105] = 0xB5; eeprom[106:108] = struct.pack(">h", v)
                elif (a >> 4) < 3:
                    ch, k = C[a >> 4], a & 15
                    if k == 0 and -300 <= v <= 300: ch.trim = v
                    elif k == 1 and 0 <= v <= 1023: ch.tlo = v
                    elif k == 2 and 0 <= v <= 1023: ch.thi = v
                    elif k == 3 and 0 <= v <= 1023: ch.elo = v
                    elif k == 4 and 0 <= v <= 1023: ch.ehi = v
                    lo, hi = ch.window(); ch.target = max(lo, min(hi, ch.target))
            elif (a, b) == (ord("S"), ord("V")): save_seat()
            elif (a, b) == (ord("R"), ord("L")): load_seat()
            elif (a, b) == (ord("C"), ord("L")): clear_seat()
            elif a == ord("H"): HOLD[0] = time.time() + b * 0.1
        elif c == "x" and a == ord("e") and args.fw >= 19 and cl is not None:
            cl["extras"] = b in (1, ord("1"))
            EXTRAS_ANY[0] = cl["extras"]
        elif c == "l" and args.fw >= 18:
            if a <= 2: board["view"], board["lflags"] = a, b; save_board()
        elif c == "i" and args.fw >= 18:
            now = time.time()
            if a < 16:
                board["sflags"] = a
                if b != 255 or args.fw < 19:
                    board["tilt"] = min(b, 60)
                    if args.fw >= 19:
                        TILT4[:] = [board["tilt"] * 10] * 4
                        if eeprom[66] == 0xB9: save_seat()
                save_board()
            elif a == ord("L") and sensor_fresh(now):
                board["lv_r"], board["lv_p"] = int(SEAT.out[0] * 100), int(SEAT.out[1] * 100); save_board()
            elif a == ord("C"):
                board["lv_r"] = board["lv_p"] = 0; save_board()
            elif a == ord("S") and args.imu:
                board["setup_until"] = now + 1.2       # nine commands, 120 ms apart
                if not args.imu_no_tx:
                    code = (b & 0x0F) or 9
                    SEAT.rate, SEAT.baud = {6: 10, 7: 20, 8: 50, 9: 100, 11: 200}.get(code, 100), 115200
                    SEAT.all = bool(b & 0x40) and args.fw >= 19          # [iS 0] (1.8): angles only
                    board["baud_idx"] = 1; save_board()
        elif c in "DEF": C["DEF".index(c)].kp = w
        elif c in "GHI": ch = C["GHI".index(c)]; ch.ki = w; ch.update_ilim()
        elif c in "JKL": C["JKL".index(c)].kd = w
        elif c in "MNO": C["MNO".index(c)].ks = max(1, w)
        elif c in "PQR": ch = C["PQR".index(c)]; ch.pwmmin, ch.pwmmax = a, b
        elif c in "STU":
            ch = C["STU".index(c)]; ch.cut, ch.clip = a, b
            lo, hi = ch.window(); ch.target = max(lo, min(hi, ch.target))
        elif c in "VWX": ch = C["VWX".index(c)]; ch.dz, ch.pwmrev = a, b
        elif c == "d" and a == ord("i") and args.fw >= 15:
            if b == ord("s"):
                for ch in C: ch.disabled, ch.reason = True, 2
            elif ord("1") <= b <= ord("3"): C[b - ord("1")].disabled, C[b - ord("1")].reason = True, 2
        elif c == "f" and args.fw >= 16:
            fshift, frate = min(a, 8), b * 100
        elif c == "u" and ota_fw() and cl is not None:   # 1.9.2: firmware update (from the page only)
            v = VREG[0] & 0xFFFF
            if b == ord("="):
                if a == ord("P"): OTA["port"] = v
                elif a == ord("T"): OTA["token"] = v
                elif a == ord("I"): OTA["ip"][0] = v
                elif a == ord("J"): OTA["ip"][1] = v
            elif (a, b) == (ord("O"), ord("K")) and not (1 <= OTA["state"] <= 5):
                ota_set(state=0, code=0, done=0, total=0)
            elif (a, b) == (ord("G"), ord("O")) and not (1 <= OTA["state"] <= 5):
                OTA.update(code=0, done=0, total=0, progress=False)
                hi, lo = OTA["ip"]
                ip = f"{hi >> 8}.{hi & 255}.{lo >> 8}.{lo & 255}" if (hi or lo) else cl.get("peer", "")
                if not OTA["port"] or not ip:
                    ota_fail(-102)
                elif time.time() - usb["last"] < 3:
                    ota_fail(-100)
                elif WIFI_FW < (0, 4, 0):
                    ota_fail(-101)
                else:
                    OTA["url"] = f"http://{ip}:{OTA['port']}/ota/{OTA['token']:04x}.ota"
                    for ch in C: ch.disabled, ch.reason = True, 5
                    ota_set(state=1)
                    threading.Thread(target=ota_run, daemon=True).start()
        elif c == "z" and a == ord("s"):          # simulator only: fake SimHub on/off
            SIMHUB[0] = (b == ord("1"))
        elif c == "s" and a == ord("a") and b == ord("v"):
            save_eeprom()
            if args.fw >= 19: save_seat()
        elif c == "e" and a == ord("n") and 1 <= OTA["state"] <= 5:
            pass                                  # 1.9.2: not during a firmware update
        elif c == "e" and a == ord("n"):
            if b == ord("a"):
                for ch in C: enable(ch)
            elif ord("1") <= b <= ord("3"): enable(C[b - ord("1")])
            board["grace"] = time.time() + 1.5
        elif c == "r" and a == ord("d"):
            k = chr(b)
            if k in "DEF": out.append(replyw(k, C["DEF".index(k)].kp))
            elif k in "GHI": out.append(replyw(k, C["GHI".index(k)].ki))
            elif k in "JKL": out.append(replyw(k, C["JKL".index(k)].kd))
            elif k in "MNO": out.append(replyw(k, C["MNO".index(k)].ks))
            elif k in "PQR": ch = C["PQR".index(k)]; out.append(reply2(k, ch.pwmmin, ch.pwmmax))
            elif k in "STU": ch = C["STU".index(k)]; out.append(reply2(k, ch.cut, ch.clip))
            elif k in "VWX": ch = C["VWX".index(k)]; out.append(reply2(k, ch.dz, ch.pwmrev))
            elif k == "Y": out.append(reply2("Y", 16 + sum((1 << i) for i, ch in enumerate(C) if ch.disabled), 0))
            elif k == "~": out.append(reply2("~", 25, 25))
            elif k == "$": out.append(replyw("$", 70))
            elif k == "!" and args.fw >= 14:
                out.append(reply2("!", (2 | 4 | 8) if args.fw >= 18 else (2 | 8) if args.fw >= 17 else 0, args.fw))
            elif k == "l" and args.fw >= 18: out.append(reply2("l", board["view"], board["lflags"]))
            elif k == "i" and args.fw >= 18: out.append(reply2("i", sensor_state(time.time()), tilt_deg()))
            elif k == "f" and args.fw >= 16: out.append(reply2("f", fshift, frate // 100))
            elif k == "d" and args.fw >= 17: out.append(diag_frame())
            elif k == "u" and ota_fw(): out.append(ota_frame())


def client(conn):
    if time.time() < OFFLINE_UNTIL[0]:        # being programmed: nobody answers
        conn.close()
        return
    try:
        peer = conn.getpeername()[0]
    except OSError:
        peer = ""
    conn.setblocking(False)
    if first_client[0] is None:
        first_client[0] = time.time()
    with lock:
        clients.append(conn)
    buf = b""; last_send = time.time()
    cl = {"extras": False, "peer": peer}
    ota_seq, ota_last = OTA["seq"], 0.0
    while True:
        time.sleep(0.02)
        if conn not in clients:
            return
        try:
            d = conn.recv(4096)
            if d == b"": return
            buf += d
        except BlockingIOError:
            pass
        except OSError:
            return
        out = []
        while len(buf) >= 5:
            if buf[0:1] == b"[" and buf[4:5] == b"]":
                handle_cmd(chr(buf[1]), buf[2], buf[3], out, cl); buf = buf[5:]
            else:
                buf = buf[1:]
        if OTA["blocked"]:                        # WiFi module 0.4.x: the board's loop waits for the download
            out = []
            time.sleep(0.05)
            continue
        if ota_fw() and (OTA["seq"] != ota_seq or (1 <= OTA["state"] <= 5 and time.time() - ota_last >= 0.25)):
            ota_seq, ota_last = OTA["seq"], time.time()
            out.append(ota_frame())
        if time.time() - last_send >= 0.05:
            last_send = time.time()
            with lock:
                batch = samples[-5:]; samples.clear()
            if batch:
                out.append(b"SM3T" + bytes([1, len(batch)]) + b"".join(batch))
                if cl["extras"]:
                    with lock:
                        out.append(extras_frame())
        try:
            for o in out: conn.sendall(o)
        except OSError:
            return


boot(cold=True)
threading.Thread(target=sim_loop, daemon=True).start()
threading.Thread(target=fake_simhub, daemon=True).start()
srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind((args.bind, args.port)); srv.listen(4)
print("simboard on", args.bind, args.port, vars(args), flush=True)
while True:
    cn, _ = srv.accept()
    threading.Thread(target=client, args=(cn,), daemon=True).start()
