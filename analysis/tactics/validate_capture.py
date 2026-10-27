"""Check the fast capture model (capture.py) against the slower numpy models it replaces.
Run: python analysis/tactics/validate_capture.py   (~1 min on 4 cores)

1. drape mode vs drape.py: same scenarios, same seed, same initial packing
2. launch mode in free flight (no target, no downwash) vs the netflight.py variant in down_strike.py
"""
import sys
import time
from pathlib import Path
from multiprocessing import Pool

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from capture import Shot, run  # noqa: E402
import drape  # noqa: E402
from down_strike import down_strike  # noqa: E402

A1 = dict(side=1.5, n=13, m_w=0.020, tether=1.0, start=(-1.0, 0.0, 3.0), v_cmd=(2.5, 0.0, -3.0), h_min=0.8,
          climb_after=0.25, v_climb=3.0, back_kick=0.0, pouch_drop=0.0)
C2 = dict(side=1.5, n=13, m_w=0.020, tether=3.0, start=(-1.0, 0.0, 3.0), v_cmd=(3.0, 0.0, -0.0001), h_min=2.99,
          climb_after=0.6, v_climb=1.0, back_kick=4.0, pouch_drop=0.5)
CASES = {"A1 your drape (1 m lines, dive)": A1, "C2 fixed drape": C2, "C3 fixed drape, target 3 m/s": dict(C2, wind=(-3.0, 0.0, 0.0))}


def _old(kw):
    d = dict(kw)
    d["m_w_back"] = d["m_w_front"] = d.pop("m_w")
    res, _ = drape.run(drape.Drape(**d))
    return res


def _free_down(u, v0=9.3, mw=0.008, side=1.2, n=11):
    s = Shot(mode="launch", side=side, n=n, m_w=mw, v0=v0, start=(0.0, 0.0, 50.0), wind=(-u, 0, 0), target_r=0.0,
             target_dw=0.0, icpt_dw=0.0, t_end=0.9, stop_on_catch=False, seed=1)
    _, rec = run(s, rec_every=100, n_rec=500)
    out = {}
    muzzle = 50.0 - 0.15
    for depth in (2.0, 3.0, 4.0, 5.0):
        row = next((q for q in rec if muzzle - q[3] >= depth), None)
        if row is None:
            continue
        c = row[4:16].reshape(4, 3)[:, :2]
        q = c[[0, 1, 3, 2]]
        area = 0.5 * abs(np.dot(q[:, 0], np.roll(q[:, 1], 1)) - np.dot(q[:, 1], np.roll(q[:, 0], 1)))
        out[depth] = (row[0], row[1], area / side ** 2)   # wind along -x, so the x offset is the lag
    return out


if __name__ == "__main__":
    t0 = time.time()
    with Pool(4) as p:
        olds = p.map(_old, list(CASES.values()))
        refs = p.map(down_strike, [(u, 9.3, 0.008, 1.2) for u in (0.0, 4.0, 6.0)])
    print("== 1) drape mode: capture.py (new) vs drape.py (old)")
    for (name, kw), o in zip(CASES.items(), olds):
        r = run(Shot(mode="drape", t_end=2.2, seed=3, stop_on_catch=False, **kw))
        print(f"{name}\n   new: first contact {r['t_contact']:.3f} s, nodes touching {r['contacts_max']}, "
              f"net vs own props {r['min_clearance']:+.3f} m, tether pull {r['max_tether_N']:.1f} N"
              f"\n   old: first contact {o['t_contact']:.3f} s, nodes touching {o['contacts_max']}, "
              f"net vs own props {o['min_clearance']:+.3f} m, tether pull {o['max_tether_N']:.1f} N")
    print("== 2) V2 net fired straight down while drifting at u: capture.py (new) vs netflight.py variant (old)")
    for (key, old), u in zip(refs, (0.0, 4.0, 6.0)):
        new = _free_down(u)
        for d in (3.0, 4.0):
            tn, lag_n, open_n = new[d]
            to, lag_o, open_o = old[d]
            print(f"   u={u:.0f} m/s, {d:.0f} m below: new t={tn:.2f}s lag {abs(lag_n):.2f} m {open_n * 100:.0f}% open | "
                  f"old t={to:.2f}s lag {abs(lag_o):.2f} m {open_o * 100:.0f}% open")
    print(f"done in {time.time() - t0:.0f}s")
