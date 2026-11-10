"""Whole engagement, many times over: a drone dives at the building, the turret's sensor sees it (with noise and delay),
the fire control (turret/firecontrol.py) tracks it, turns the head and fires, and the shot is scored.

    python analysis/turret/engage.py          (~2 min on 4 cores)  -> RESULTS.md

It answers: how good do the sensor and the pan-tilt head have to be, and how much of the building does one turret cover?
The net flight tables come from envelope.py; the scoring rule is checked against the full net model by check_oblique.py.
"""
import math
import sys
import zlib
from multiprocessing import Pool
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]))
from turret.firecontrol import FireControl, NetTable, PanTilt, Tracker, score_shot  # noqa: E402

DEG = math.pi / 180
N_TRIALS = 1000
TICK = 0.005

# sensor suites: angle noise (deg, 1 sigma), range noise (m), unknown range scale error (fraction, fixed per engagement)
SUITES = {
    "camera only (range guessed from image size)": dict(sig_ang=0.3, rng_abs=0.0, rng_rel=0.05, rng_bias=0.20),
    "camera + rangefinder or small radar (0.5 m)": dict(sig_ang=0.3, rng_abs=0.5, rng_rel=0.0, rng_bias=0.0),
    "camera + good radar (0.15 m)": dict(sig_ang=0.2, rng_abs=0.15, rng_rel=0.0, rng_bias=0.0),
}
S_CAM, S_MID, S_GOOD = list(SUITES)
BASE = dict(cls="light", suite=S_MID, u=30.0, b=1.0, elev=20.0, jink=0.0, prefer=0.7, r_det=60.0, rate=30.0, lat=0.08,
            head_rate=180.0, head_accel=1500.0, t_act=0.04, disp=1.5, v_scatter=0.04)
JINKS = (0.0, 0.25, 0.5, 1.0, 2.0)      # sideways weave at 1 Hz, peak acceleration in g
REACH = {"light": (0, 1, 2, 3, 4, 5, 6), "medium": (0, 2, 4, 6, 8, 10), "medium-wide": (0, 2, 4, 6, 8),
         "heavy": (0, 4, 8, 12, 16), "heavy-wide": (0, 2, 4, 6, 8, 10)}


def trial(rng, net, c):
    su = SUITES[c["suite"]]
    e, psi = c["elev"] * DEG, rng.uniform(0, 2 * math.pi)
    a_in = np.array([math.cos(e) * math.cos(psi), math.cos(e) * math.sin(psi), math.sin(e)])
    n_h = np.array([-math.sin(psi), math.cos(psi), 0.0]) * rng.choice([-1.0, 1.0])
    aim_point = c["b"] * n_h
    th = rng.uniform(0, 2 * math.pi)
    w_dir = math.cos(th) * n_h + math.sin(th) * np.cross(a_in, n_h)
    ph, u = rng.uniform(0, 2 * math.pi), c["u"]
    amp = c["jink"] * 9.81 / (2 * math.pi) ** 2          # weave at 1 Hz with this peak sideways acceleration
    s0 = c["r_det"] + 0.2 * u

    def path(t):
        return aim_point + a_in * (s0 - u * t) + w_dir * (amp * math.sin(2 * math.pi * t + ph))

    bias = 1.0 + rng.normal(0, su["rng_bias"]) if su["rng_bias"] > 0 else 1.0
    sig_ang = su["sig_ang"] * DEG
    head = PanTilt(c["head_rate"] * DEG, c["head_accel"] * DEG)
    fc = FireControl(net, c["head_rate"] * DEG, t_act=c["t_act"], prefer=c["prefer"])
    trk = Tracker()
    queue, n_upd, plan, t_first = [], 0, None, None
    t, next_meas, t_end = 0.0, 0.0, s0 / u + 0.2
    while t < t_end:
        if t >= next_meas:
            r = path(t)
            rng_true = float(np.linalg.norm(r))
            if rng_true <= c["r_det"]:
                az = math.atan2(r[1], r[0]) + rng.normal(0, sig_ang)
                el = math.asin(r[2] / rng_true) + rng.normal(0, sig_ang)
                rm = rng_true * bias * (1 + rng.normal(0, su["rng_rel"])) + rng.normal(0, su["rng_abs"])
                queue.append((t + c["lat"], t, az, el, rm))
            next_meas += 1.0 / c["rate"]
        fresh = False
        while queue and queue[0][0] <= t:
            _, tm, az, el, rm = queue.pop(0)
            sig_r = max(su["rng_abs"], (su["rng_bias"] + su["rng_rel"]) * rm, 0.05)
            trk.update(tm, az, el, rm, sig_ang, sig_r)
            n_upd += 1
            fresh = True
            if t_first is None:
                t_first = t
        if trk.x is not None:
            if fresh:
                p, v = trk.state_at(t)
                plan = fc.plan(p, v, t, head.direction())
            if plan is not None:
                a = plan[0]
                head.step(math.atan2(a[1], a[0]), math.asin(max(-1.0, min(1.0, a[2]))), TICK)
                if n_upd >= 6 and fc.should_fire(plan, t, head.direction(), TICK):
                    t_launch = t + c["t_act"] + rng.normal(0, 0.005)
                    while t < t_launch:                       # the head keeps moving until the net leaves
                        head.step(math.atan2(a[1], a[0]), math.asin(max(-1.0, min(1.0, a[2]))), TICK)
                        t += TICK
                    axis = head.direction() + rng.normal(0, c["disp"] * DEG, 3)
                    axis /= np.linalg.norm(axis)
                    ok, xm, miss, inc = score_shot(net, t_launch, axis, path, speed_scale=1 + rng.normal(0, c["v_scatter"]))
                    return ("caught" if ok else "fired, missed"), xm, miss
        t += TICK
    return ("never in reach" if plan is None else "head not ready in time"), float("nan"), float("nan")


def run_config(args):
    key, c = args
    net = NetTable(HERE / "out" / f"net_{c['cls']}.json")
    rng = np.random.default_rng(zlib.crc32(repr(key).encode()))   # same numbers every run
    out = [trial(rng, net, c) for _ in range(N_TRIALS)]
    kinds = [o[0] for o in out]
    miss = [o[2] for o in out if o[0] in ("caught", "fired, missed") and o[2] == o[2]]
    xm = [o[1] for o in out if o[0] == "caught"]
    return key, dict(p=kinds.count("caught") / N_TRIALS, missed=kinds.count("fired, missed") / N_TRIALS,
                     late=kinds.count("head not ready in time") / N_TRIALS, out=kinds.count("never in reach") / N_TRIALS,
                     miss_med=float(np.median(miss)) if miss else float("nan"), x_med=float(np.median(xm)) if xm else float("nan"))


def main():
    cfgs = {}

    def add(key, **kw):
        cfgs[key] = dict(BASE, **kw)

    offsets = [0, 1, 2, 3, 4, 5]
    for s in SUITES:
        for b in offsets:
            add(("offset", s, b), suite=s, b=float(b))
        for u in (20.0, 30.0, 40.0):
            add(("speed", s, u), suite=s, u=u)
    for rate in (60.0, 120.0, 180.0, 360.0):
        for rd in (40.0, 60.0, 80.0):
            add(("head", rate, rd), head_rate=rate, head_accel=rate * 8, r_det=rd)
    for name, rate, lat in (("15 pictures a second, 150 ms delay", 15.0, 0.15), ("30 a second, 80 ms", 30.0, 0.08), ("60 a second, 30 ms", 60.0, 0.03)):
        for b in (1.0, 3.0):
            add(("timing", name, b), rate=rate, lat=lat, b=b)
    for name, kw in (("baseline", {}), ("net thrown 4 deg off instead of 1.5", dict(disp=4.0)),
                     ("net speed varies 10 % instead of 4 %", dict(v_scatter=0.10)),
                     ("valve takes 100 ms instead of 40 ms (known)", dict(t_act=0.10))):
        add(("robust", name), **kw)
    classes = [k for k in REACH if (HERE / "out" / f"net_{k}.json").exists()]
    for cl in classes:
        for b in REACH[cl]:
            add(("reach", cl, b), cls=cl, b=float(b))
        for pref, label in ((0.7, "far"), (0.15, "near")):
            for g in JINKS:
                add(("jink", cl, label, g), cls=cl, prefer=pref, jink=g)
    with Pool(4) as p:
        res = dict(p.map(run_config, list(cfgs.items()), chunksize=1))

    pc = lambda k: f"{res[k]['p'] * 100:.0f} %"                # noqa: E731
    L = ["# Turret engagement simulation", "",
         "Generated by `engage.py`. A drone dives 20° at a point on the building at 30 m/s (108 km/h) unless stated; it is first "
         "seen 60 m out, from a random direction, so the head may have to turn up to 180°. Each cell is the share of "
         f"{N_TRIALS} engagements that end in a catch. Light launcher (2 m net, 36 J) unless stated.", "",
         "Baseline: camera + 0.5 m ranging, 30 measurements a second with 80 ms delay, head turning 180°/s, "
         "40 ms from fire command to the net leaving, net thrown within 1.5° and 4 % of its nominal speed.", "",
         "## How far from the turret can the drone be aimed?", "",
         "| sensor | " + " | ".join(f"{b} m" for b in offsets) + " |", "|---|" + "---|" * len(offsets)]
    for s in SUITES:
        L.append(f"| {s} | " + " | ".join(pc(("offset", s, b)) for b in offsets) + " |")
    L += ["", "Column = how far the drone's aim point is from the turret. Beyond the net's reach nothing helps.", "",
          "## Drone speed (aim point 1 m from the turret)", "", "| sensor | 20 m/s | 30 m/s | 40 m/s |", "|---|---|---|---|"]
    for s in SUITES:
        L.append(f"| {s} | " + " | ".join(pc(("speed", s, u)) for u in (20.0, 30.0, 40.0)) + " |")
    L += ["", "## Head speed and how early the drone is seen (aim point 1 m away)", "",
          "| head turns at | seen at 40 m | seen at 60 m | seen at 80 m |", "|---|---|---|---|"]
    for rate in (60.0, 120.0, 180.0, 360.0):
        L.append(f"| {rate:.0f}°/s | " + " | ".join(pc(("head", rate, rd)) for rd in (40.0, 60.0, 80.0)) + " |")
    L += ["", "## Measurement rate and delay", "", "| sensor timing | aim point 1 m away | 3 m away |", "|---|---|---|"]
    for name in ("15 pictures a second, 150 ms delay", "30 a second, 80 ms", "60 a second, 30 ms"):
        L.append(f"| {name} | {pc(('timing', name, 1.0))} | {pc(('timing', name, 3.0))} |")
    L += ["", "## A drone that does not fly straight (aim point 1 m away)", "",
          "The drone weaves from side to side once a second; the column is the peak sideways acceleration. "
          "'Far' and 'near' are where the fire control prefers to meet the drone within the net's reach.", "",
          "| launcher, meeting point | " + " | ".join("straight" if g == 0 else f"{g:g} g" for g in JINKS) + " |", "|---|" + "---|" * len(JINKS)]
    for cl in classes:
        for label in ("far", "near"):
            xs = [res[("jink", cl, label, g)]["x_med"] for g in JINKS]
            L.append(f"| {cl}, {label} (about {np.nanmedian(xs):.0f} m out) | " + " | ".join(pc(("jink", cl, label, g)) for g in JINKS) + " |")
    L += ["", "## How far from the turret can the drone be aimed, by launcher", ""]
    for cl in classes:
        L += [f"| {cl}: aim point from the turret | " + " | ".join(f"{b} m" for b in REACH[cl]) + " |", "|---|" + "---|" * len(REACH[cl]),
              "| catch | " + " | ".join(pc(("reach", cl, b)) for b in REACH[cl]) + " |", ""]
    L += ["## What else could go wrong (aim point 1 m away)", "", "| change | catch | typical sideways miss |", "|---|---|---|"]
    for k in cfgs:
        if k[0] == "robust":
            L.append(f"| {k[1]} | {pc(k)} | {res[k]['miss_med']:.2f} m |")
    L += ["", "## Why engagements fail (baseline sensor)", "", "| aim point from the turret | caught | fired and missed | head not ready | never in reach |",
          "|---|---|---|---|---|"]
    for b in offsets:
        r = res[("offset", S_MID, b)]
        L.append(f"| {b} m | {r['p'] * 100:.0f} % | {r['missed'] * 100:.0f} % | {r['late'] * 100:.0f} % | {r['out'] * 100:.0f} % |")
    L += ["", "Not modelled: evasion other than a steady weave, several drones at once, rain or darkness on the camera, and what the "
          "netted drone does afterwards (it keeps most of its speed; see the discussion of catch distance).", ""]
    (HERE / "RESULTS.md").write_text("\n".join(L))
    print("\n".join(L))


if __name__ == "__main__":
    main()
