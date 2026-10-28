"""
Smoke test: start simulated rigs and the page, run diagnostics, check the results. Standard library only.

    python3 tools/smoke_test.py

1. A healthy rig: quick and full check must find no problems.
2. A rig with motor 2 wired backwards: the full check must say so and leave motor 2 switched off.
3. A rig with a WT901C at its factory 10 Hz: the page finds it, switches it on, sets it to 100 Hz with gyro and
   acceleration, and the quick check is clean.
4. Seat setup (firmware 1.9) on a traction-loss rig (motors 1 and 2 lift the front, motor 3 slides the rear) whose
   seat isn't level: motors 1 and 2 level it, motor 1 stops at the tilt allowed, motor 3 at the traction loss
   allowed (the seat's turn), it saves; the board then has the level trim, travel limits and end stops, and the seat
   reads level.
5. The same on a three-lift rig: all three motors level the seat, and motor 3 stops at the tilt allowed.
6. Soft start (firmware 1.9.1): a seat that rode down while the rig was off creeps back to its centre at power-up,
   no faster than the soft-start speed, and the speed can be changed.
"""
import json, os, subprocess, sys, tempfile, time, urllib.request as u

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "simboard.py")
SCOPE = os.path.join(HERE, "..", "scope", "smc3_scope.py")


def get(base, p):
    return json.loads(u.urlopen(base + p, timeout=15).read())


def post(base, p, o=None):
    return json.loads(u.urlopen(u.Request(base + p, data=json.dumps(o or {}).encode(), method="POST"), timeout=60).read())


def start(port, web, *sim_args):
    data = tempfile.mkdtemp(prefix="smc3-smoke-")
    sim = subprocess.Popen([sys.executable, SIM, str(port), "--tuned", *sim_args], stdout=subprocess.DEVNULL)
    time.sleep(0.5)
    log = open(os.path.join(data, "scope.log"), "w")
    scope = subprocess.Popen([sys.executable, SCOPE, "--host", "127.0.0.1", "--port", str(port), "--web", str(web),
                              "--data", data], stdout=log, stderr=subprocess.STDOUT)
    scope.log_path = log.name
    base = f"http://127.0.0.1:{web}"
    for _ in range(60):
        try:
            if get(base, "/data?since=0")["connected"] and get(base, "/data?since=0")["fwnum"]:
                break
        except OSError:
            pass
        time.sleep(0.5)
    time.sleep(2)
    return sim, scope, base


def run_check(base, kind):
    r = post(base, "/diag/start", {"kind": kind, "ask": False, "problem": "smoke test"})
    assert r["ok"], r
    t0 = time.time()
    while time.time() - t0 < 300:
        s = get(base, "/diag/status")
        if s["state"] not in ("running", "waiting"):
            return s
        time.sleep(0.5)
    raise AssertionError("check did not finish")


def problems(s):
    return [f["title"] for f in s["result"]["findings"] if f["level"] == "fail"]


failed = 0


def check(cond, label):
    global failed
    print(("PASS " if cond else "FAIL ") + label, flush=True)
    failed += 0 if cond else 1


procs = []
try:
    sim, scope, base = start(4710, 8710)
    procs += [sim, scope]
    page = u.urlopen(base + "/").read().decode()
    check("SMC3" in page and "Diagnostics" in page, "page is served")
    s = run_check(base, "quick")
    check(s["state"] == "done" and not problems(s), f"healthy rig, quick check: no problems {problems(s)}")
    s = run_check(base, "full")
    check(s["state"] == "done" and not problems(s), f"healthy rig, full check: no problems {problems(s)}")
    rep = json.loads(u.urlopen(base + "/diag/file?name=" + s["result"]["name"]).read())
    check(list(rep)[:6] == ["about", "made", "kind", "problem", "summary", "findings"] and len(rep["check"]) == 3,
          "diagnostics file is complete")
    sim2, scope2, base2 = start(4711, 8711, "--reverse", "2")
    procs += [sim2, scope2]
    s = run_check(base2, "full")
    check(any("WRONG way" in t for t in problems(s)), f"reversed motor 2 found: {problems(s)}")
    time.sleep(1)
    check(get(base2, "/data?since=0")["samples"][-1]["dis"] == [False, True, False], "motor 2 left off, 1 and 3 back on")
    sim3, scope3, base3 = start(4712, 8712, "--imu", "10")
    procs += [sim3, scope3]
    bs = get(base3, "/board/settings")
    check(bs["ok"] and bs["sensor"]["receiving"] and bs["sensor"]["angles_per_s"] == 10, "sensor found at 10 Hz")
    r = post(base3, "/board/sensor", {"on": True, "tilt": 12})
    check(r["sensor"]["switched_on"] and r["sensor"]["tilt_limit_deg"] == 12, "sensor switched on, tilt limit 12")
    r = post(base3, "/board/sensor/action", {"action": "setup"})
    time.sleep(2)
    check(get(base3, "/board/settings")["sensor"]["angles_per_s"] == 100, "sensor set to 100 Hz")
    check(get(base3, "/data?since=0")["samples"][-1]["roll"] is not None, "seat angles in the telemetry")
    check(get(base3, "/board/settings")["sensor"]["accel_gyro"], "gyro and acceleration arriving")
    time.sleep(1)
    check(get(base3, "/data?since=0")["imu"] is not None, "acceleration and gyro reach the page")
    s = run_check(base3, "quick")
    check(s["state"] == "done" and not problems(s), f"rig with sensor, quick check: no problems {problems(s)}")
    sim4, scope4, base4 = start(4713, 8713, "--imu", "100", "--sensor-on", "--seat-tilt", "1.5:-2.0")
    procs += [sim4, scope4]

    def seat_wait():
        t0 = time.time()
        while time.time() - t0 < 120:
            st = get(base4, "/seat/status")
            if st["action"] is None:
                return st
            time.sleep(0.5)
        raise AssertionError("seat setup did not finish")
    check(post(base4, "/seat/start")["ok"], "seat setup opens")
    post(base4, "/seat/level")
    st = seat_wait()
    check(st["error"] is None and abs(st["live"]["roll_abs"]) < 0.2 and abs(st["live"]["pitch_abs"]) < 0.2,
          f"seat levels itself with motors 1 and 2 (roll {st['live']['roll_abs']:+.2f}, pitch {st['live']['pitch_abs']:+.2f})")
    check(st["tl"] == 2 and st["J"][0][2] == 0 and st["J"][1][2] == 0, "motor 3 slides the rear: not used to level")
    r = post(base4, "/seat/accept")
    check(r["ok"], "level accepted")
    lv = r["status"]["level_pos"]
    post(base4, "/seat/options", {"want": {"roll_right": 5, "roll_left": 5, "front_up": 5, "front_down": 5,
                                           "traction": 4}, "keep_inside": False})   # only some ends are explored here
    post(base4, "/seat/explore", {"motor": 0, "way": 1})
    st = seat_wait()
    e1 = st["ends"][0][1]
    check(e1 and e1["why"] == "angle" and max(e1["roll"], e1["pitch"]) >= 4.8, f"motor 1 stops at the tilt allowed ({e1})")
    post(base4, "/seat/explore", {"motor": 2, "way": 1})
    st = seat_wait()
    e3 = st["ends"][2][1]
    check(e3 and e3["why"] == "yaw" and e3["yaw"] >= 3.8, f"motor 3 stops at the traction loss allowed ({e3})")
    check(st["yaw_k"] and abs(abs(st["yaw_k"]) - 0.02) < 0.003, f"it measured how far the seat turns ({st['yaw_k']})")
    check(post(base4, "/seat/save")["ok"], "save starts")
    st = seat_wait()
    b = get(base4, "/board/settings")["seat"]
    check(st["step"] == "done" and b["saved"] and b["trim"] == [p - 512 for p in lv] and
          b["travel"][0][1] == e1["pot"] - 20 and b["travel"][2][1] == e3["pot"] - 20,
          f"board has the set-up (trim {b['trim']}, travel {b['travel']})")
    time.sleep(1)
    s = get(base4, "/data?since=0")["samples"][-1]
    check(abs(s["roll"]) < 0.2 and abs(s["pitch"]) < 0.2, "the level seat reads 0° / 0°")
    g = get(base4, "/seat/geometry")
    check(g["layout"] == "tl" and g["tl"] == 2 and abs(g["yaw_per_count"] - 0.02) < 0.003, "the 3D view gets the rear slide")

    # 5. three lifts
    sim5, scope5, base5 = start(4714, 8714, "--imu", "100", "--sensor-on", "--layout", "lifts", "--seat-tilt", "1.5:-2.0")
    procs += [sim5, scope5]
    post(base5, "/config", {"layout": "lifts"})
    check(post(base5, "/seat/start")["ok"], "three lifts: seat setup opens")
    post(base5, "/seat/level")
    t0 = time.time()
    while time.time() - t0 < 120 and get(base5, "/seat/status")["action"]:
        time.sleep(0.5)
    st = get(base5, "/seat/status")
    check(st["error"] is None and st["tl"] is None and abs(st["live"]["roll_abs"]) < 0.2 and abs(st["live"]["pitch_abs"]) < 0.2
          and abs(st["J"][1][2]) > 0.01, f"three lifts: all three level the seat (J {st['J']})")
    post(base5, "/seat/accept")
    post(base5, "/seat/options", {"want": {"roll_right": 6, "roll_left": 6, "front_up": 5, "front_down": 5}})
    post(base5, "/seat/explore", {"motor": 2, "way": 1})
    t0 = time.time()
    while time.time() - t0 < 120 and get(base5, "/seat/status")["action"]:
        time.sleep(0.5)
    e = get(base5, "/seat/status")["ends"][2][1]
    check(e and e["why"] == "angle" and abs(e["pitch"]) >= 4.8, f"three lifts: motor 3 stops at the tilt allowed ({e})")
    post(base5, "/seat/cancel")

    # 6. soft start: motor 1 at 300 at power-up, its centre (level trim 18) at 530
    sim6, scope6, base6 = start(4715, 8715, "--seat-setup", "--start", "1:300")
    procs += [sim6, scope6]
    seq, got, t0 = 0, [], time.time()
    while time.time() - t0 < 9:
        d = get(base6, f"/data?since={seq}")
        if d["samples"]:
            got += d["samples"]; seq = d["samples"][-1]["seq"]
        time.sleep(0.25)
    gs = [(x["ms"], x["m"][0]["g"]) for x in got]
    fastest = max((abs(gs[i + 10][1] - gs[i][1]) * 1000.0 / max(1, gs[i + 10][0] - gs[i][0]) for i in range(0, len(gs) - 10, 10)),
                  default=0)
    last = got[-1]["m"][0] if got else {"g": 0, "f": 0}
    check(bool(got) and fastest < 70 and abs(last["g"] - 530) <= 3 and abs(last["f"] - 530) <= 8 and
          not any(x["dis"][0] for x in got), f"soft start: crept to 530 (now {last['f']:.0f}), fastest {fastest:.0f} counts/s")
    r = post(base6, "/board/soft", {"rate": 120})
    check(r.get("ok") and r["soft_start"]["rate"] == 120, f"soft start speed set to 120 ({r.get('soft_start')})")
finally:
    for p in procs:
        p.terminate()
    for p in procs:
        try:
            p.wait(timeout=5)
        except Exception:
            p.kill()
        path = getattr(p, "log_path", None)
        out = open(path).read() if path else ""
        if "Traceback" in out:
            print(out)
            failed += 1
print("FAILED" if failed else "ALL PASSED")
sys.exit(1 if failed else 0)
