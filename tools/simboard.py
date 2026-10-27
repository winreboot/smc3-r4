"""
Simulated SMC3_R4 board (firmware 1.7, or 1.3-1.6 behaviour with --fw) - try smc3_scope without a rig, and
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
  --fw N               13, 14, 15, 16 or 17 (default 17)
"""
import argparse, math, random, socket, struct, threading, time

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
ap.add_argument("--fw", type=int, default=17)
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


class Phys:                                   # the motor + linkage + pot
    def __init__(self, i):
        self.i = i
        self.x = 512.0 + random.uniform(-30, 30); self.v = 0.0; self.dl = 0.0; self.drive = 0.0; self.brake = True
        self.pot_x = self.x                       # where the pot thinks it is (differs only with --slip)


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
        self.update_ilim()

    def update_ilim(self):
        self.ilim = ((FULL << 13) // self.ki) if self.ki > 0 else 0


lock = threading.Lock()
TICKS = [0]
SIMHUB = [False]                          # switched with [zs1] / [zs0] from any client (simulator only)
state = {}


eeprom = None
P = [Phys(i) for i in range(3)]


def boot(cold=True):
    """Power-up: params from 'EEPROM', motors enabled, counters reset."""
    global C, power_scale, last_target_t, t0, fshift, frate, usb, eeprom, reset_flags
    C = [Chan(i) for i in range(3)]
    fshift, frate = 6, 0
    first = eeprom is None
    if first:                                 # very first power-up: defaults (+ --hunt) become the saved values
        if args.tuned:
            for c in C:
                c.kp, c.ki, c.kd, c.pwmmin, c.dz = 250, 0, 0, 30, 2
                c.update_ilim()
        if args.hunt:
            h = C[args.hunt - 1]
            h.kp, h.ki, h.kd, h.dz, h.pwmmin = 2600, 0, 0, 0, 90
        eeprom = bytearray(64)
        save_eeprom()
        if args.pwmmax0:                      # running-only, like auto-tune's safety stop
            C[args.pwmmax0 - 1].pwmmax = 0
    else:
        load_eeprom()
    for c in C:
        p = P[potmap[c.i]]
        c.fb_q = int(p.pot_x * 16); c.lastpos = c.fb_q
        c.goal_acc = (max(c.clip, min(1023 - c.clip, int(p.pot_x))) << 12); c.goal_q = c.goal_acc >> 8
        c.target = max(c.clip, min(1023 - c.clip, int(p.pot_x)))
        c.disabled, c.reason = False, 0
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
    if fb > ((1023 - c.cut) << 4) or fb < (c.cut << 4):
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
    if fb > cmax and c.pwmrev: return -revq / FULL, False
    if fb < cmin and c.pwmrev: return revq / FULL, False
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
    deff = math.copysign(max(0.0, abs(p.dl) - fr), p.dl) if p.dl != 0 else 0.0
    if brake or deff == 0.0:
        p.v += -p.v * DT / 0.012
    else:
        p.v += (deff * VMAX - p.v) * DT / TAU
    p.x += p.v * DT
    if p.x < 30: p.x, p.v = 30.0, 0.0
    if p.x > 993: p.x, p.v = 993.0, 0.0
    if args.slip != p.i + 1:
        p.pot_x = p.x


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
            power_scale = 9 if (time.time() - last_target_t) > 15 else 7
            for _ in range(SUB):
                out = [chan_tick(c) for c in C]
                for i in range(3):
                    d, br = out[i]
                    phys_tick(P[drvmap[i]], d, br)
            ms = int((time.time() - t0) * 1000)
            status = 16 + sum((1 << i) for i, c in enumerate(C) if c.disabled)
            body = struct.pack("<I", ms & 0xFFFFFFFF)
            for c in C:
                body += struct.pack("<hhhh", c.target, c.goal_q, c.fb_q, c.pid_q)
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
            for i, c in enumerate(C):
                v = 512 + 120 * math.sin(t * (0.7 + 0.3 * i)) + 25 * math.sin(t * 7.3 + i)
                c.target = int(max(c.clip, min(1023 - c.clip, v)))
                usb["tgt"] += 1; usb["ok"] += 1
            usb["last"] = usb["last_tgt"] = t
            last_target_t = t


def reply2(code, a, b): return b"[" + code.encode() + bytes([a & 255, b & 255]) + b"]"
def replyw(code, v): return b"[" + code.encode() + bytes([(v >> 8) & 255, v & 255]) + b"]"


def diag_frame():
    now = time.time()
    ms = int((now - t0) * 1000)
    since = lambda x: 65535 if not x else min(65535, int((now - x) * 1000))
    pl = struct.pack("<I3BB4I2HIbBB", ms, *reset_flags, 1, usb["ok"], usb["tgt"], usb["bad"], usb["tcp"],
                     since(usb["last"]), since(usb["last_tgt"]), random.randint(800, 4000), -58, 2 | 8, 17)
    for c in C:
        pl += struct.pack("<BHhHHHH", c.reason, c.trips, c.trip_at, c.jumps, min(65535, c.jmax), c.fmin, c.fmax)
        c.jmax, c.fmin, c.fmax = 0, c.fb_q, c.fb_q
    pl += bytes([64]) + bytes(eeprom)
    return b"SM3D" + bytes([1, len(pl)]) + pl


def handle_cmd(c, a, b, out):
    global last_target_t, fshift, frate
    w = a * 256 + b
    with lock:
        usb["tcp"] += 1
        if c in "ABC":
            ch = C["ABC".index(c)]
            ch.target = max(ch.clip, min(1023 - ch.clip, w))
            if args.fw >= 14: last_target_t = time.time()
        elif c in "DEF": C["DEF".index(c)].kp = w
        elif c in "GHI": ch = C["GHI".index(c)]; ch.ki = w; ch.update_ilim()
        elif c in "JKL": C["JKL".index(c)].kd = w
        elif c in "MNO": C["MNO".index(c)].ks = max(1, w)
        elif c in "PQR": ch = C["PQR".index(c)]; ch.pwmmin, ch.pwmmax = a, b
        elif c in "STU": ch = C["STU".index(c)]; ch.cut, ch.clip = a, b
        elif c in "VWX": ch = C["VWX".index(c)]; ch.dz, ch.pwmrev = a, b
        elif c == "d" and a == ord("i") and args.fw >= 15:
            if b == ord("s"):
                for ch in C: ch.disabled, ch.reason = True, 2
            elif ord("1") <= b <= ord("3"): C[b - ord("1")].disabled, C[b - ord("1")].reason = True, 2
        elif c == "f" and args.fw >= 16:
            fshift, frate = min(a, 8), b * 100
        elif c == "z" and a == ord("s"):          # simulator only: fake SimHub on/off
            SIMHUB[0] = (b == ord("1"))
        elif c == "s" and a == ord("a") and b == ord("v"):
            save_eeprom()
        elif c == "e" and a == ord("n"):
            if b == ord("a"):
                for ch in C: ch.disabled, ch.reason = False, 0
            elif ord("1") <= b <= ord("3"): C[b - ord("1")].disabled, C[b - ord("1")].reason = False, 0
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
            elif k == "!" and args.fw >= 14: out.append(reply2("!", 2 | 8 if args.fw >= 17 else 0, args.fw))
            elif k == "f" and args.fw >= 16: out.append(reply2("f", fshift, frate // 100))
            elif k == "d" and args.fw >= 17: out.append(diag_frame())


def client(conn):
    conn.setblocking(False)
    if first_client[0] is None:
        first_client[0] = time.time()
    with lock:
        clients.append(conn)
    buf = b""; last_send = time.time()
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
                handle_cmd(chr(buf[1]), buf[2], buf[3], out); buf = buf[5:]
            else:
                buf = buf[1:]
        if time.time() - last_send >= 0.05:
            last_send = time.time()
            with lock:
                batch = samples[-5:]; samples.clear()
            if batch:
                out.append(b"SM3T" + bytes([1, len(batch)]) + b"".join(batch))
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
