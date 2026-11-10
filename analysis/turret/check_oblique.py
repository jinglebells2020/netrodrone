"""Check the quick shot-scoring rule in turret/firecontrol.py against the full capture model, for drones whose path
passes to one side of the turret and for shots fired slightly early or late.
Run: python analysis/turret/check_oblique.py [class]     (~1 min on 4 cores; the 5 m nets ~3 min)"""
import math
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parents[1]))
from classes import turret_shot  # noqa: E402
from capture import run  # noqa: E402
from turret.firecontrol import NetTable, score_shot  # noqa: E402

U, ELEV = 30.0, math.radians(20.0)
X_MEET = {"light": 5.0, "medium": 7.0, "medium-wide": 5.0, "heavy": 12.0, "heavy-wide": 8.0}   # planned meeting distance (m)
A_IN = np.array([math.cos(ELEV), 0.0, math.sin(ELEV)])
FRACS = [0.0, 0.2, 0.4, 0.6, 0.8]            # how far the path passes from the turret, as a share of the meeting distance
ERRS = [-0.06, -0.03, 0.0, 0.03, 0.06]


def setup(name, b, err):
    net = NetTable(HERE / "out" / f"net_{name}.json")
    P = np.array([0.0, b, 0.0])
    path = lambda t: P + A_IN * (60.0 - U * t)                 # noqa: E731
    v = -U * A_IN
    p0 = path(0.0)
    pv, vv, pp = float(p0 @ v), float(v @ v), float(p0 @ p0)
    xm = X_MEET[name]
    s = (-pv - math.sqrt(pv * pv - vv * (pp - xm ** 2))) / vv
    axis = path(s) / xm
    t_launch = s - net.time_to(xm) + err
    return net, path, v, axis, t_launch


def job(a):
    name, b, err = a
    net, path, v, axis, t_launch = setup(name, b, err)
    r = run(turret_shot(name, axis, -path(t_launch), drone_velocity=v, seed=3))
    full = bool(r["caught"] and r["R_min"] is not None and r["R_min"] <= 0.75)
    quick, xm, c, inc = score_shot(net, t_launch, axis, path)
    return b, err, full, quick, c, inc


if __name__ == "__main__":
    name = sys.argv[1] if len(sys.argv) > 1 else "light"
    with Pool(4) as p:
        offsets = [round(f * X_MEET[name], 2) for f in FRACS]
        res = p.map(job, [(name, b, e) for b in offsets for e in ERRS], chunksize=1)
    agree = sum(f == q for _, _, f, q, _, _ in res)
    print(f"{name} class, drone at {U:.0f} m/s, planned meeting {X_MEET[name]:.0f} m out. Each cell: full model / quick rule (C = caught, - = missed)")
    print("path passes | fired " + "  ".join(f"{e * 1000:+4.0f} ms" for e in ERRS) + " | sideways miss if on time, incidence")
    for b in offsets:
        row = [r for r in res if r[0] == b]
        cells = "  ".join(f"  {'C' if f else '-'}/{'C' if q else '-'}  " for _, _, f, q, _, _ in row)
        on = [r for r in row if r[1] == 0.0][0]
        print(f"   {b:4.1f} m off |       {cells} | {on[4]:.2f} m, {math.degrees(on[5]):.0f} deg")
    print(f"agreement: {agree} of {len(res)} cases")
