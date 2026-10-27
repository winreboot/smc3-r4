"""
Smoke test: start simulated rigs and the page, run diagnostics, check the results. Standard library only.

    python3 tools/smoke_test.py

1. A healthy rig: quick and full check must find no problems.
2. A rig with motor 2 wired backwards: the full check must say so and leave motor 2 switched off.
3. A rig with a WT901C at its factory 10 Hz (firmware 1.8): the page finds it, switches it on, sets it to 100 Hz,
   and the quick check is clean.
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
    s = run_check(base3, "quick")
    check(s["state"] == "done" and not problems(s), f"rig with sensor, quick check: no problems {problems(s)}")
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
