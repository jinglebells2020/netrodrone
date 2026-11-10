"""Net flight tables for the turret's fire control, from the repo capture model.
Run: python analysis/turret/envelope.py [class ...]     (light + medium ~4 min on 4 cores; the 5 m nets ~8 min each)

For each launcher class this writes out/net_<class>.json with
  t, x, open   net centre distance from the muzzle and open fraction against time (free flight, fired 20 deg up)
  band         catch half-width against distance: how far off the aim line a hovering 0.25 m drone can be and still be
               caught (strict rule of analysis/tactics: 12+ mesh points touching and spread over a 60 deg patch)
"""
import json
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from classes import CLASSES, energy, turret_shot  # noqa: E402
from capture import run  # noqa: E402

ELEV = np.radians(20.0)
AX = np.array([np.cos(ELEV), 0.0, np.sin(ELEV)])
OFFS = np.round(np.arange(0, 21) * 0.15, 2)
RANGES = {"light": np.arange(1.5, 10.1, 0.5), "medium": np.arange(2.0, 15.1, 1.0), "heavy": np.arange(4.0, 24.1, 2.0),
          "medium-wide": np.arange(1.5, 9.1, 0.75), "heavy-wide": np.arange(2.0, 12.6, 1.5)}


def free_flight(name):
    s = turret_shot(name, AX, (0.0, 0.0, 0.0), t_end=1.6, target_r=0.0, stop_on_catch=False)
    _, rec = run(s, rec_every=max(1, int(0.002 / s.dt)), n_rec=900)
    t = rec[:, 0]
    c = rec[:, 4:16].reshape(-1, 4, 3)
    area = 0.5 * np.linalg.norm(np.cross(c[:, 3] - c[:, 0], c[:, 1] - c[:, 2]), axis=1)
    x = rec[:, 1:4] @ AX
    keep = np.concatenate([[True], np.diff(np.maximum.accumulate(x)) > 0])     # outbound part only
    return t[keep], x[keep], area[keep] / CLASSES[name]["side"] ** 2


def _hover(a):
    name, R, ic = a
    c = float(OFFS[ic])
    r = run(turret_shot(name, AX, (-R * AX[0], c, -R * AX[2]), hover=True, t_end=1.6, seed=1 + ic))
    return name, float(R), ic, bool(r["caught"] and r["R_min"] is not None and r["R_min"] <= 0.75)


def main(names):
    jobs = [(nm, R, ic) for nm in names for R in RANGES[nm] for ic in range(len(OFFS))]
    with Pool(4) as p:
        res = p.map(_hover, jobs, chunksize=4)
    (HERE / "out").mkdir(exist_ok=True)
    for nm in names:
        t, x, op = free_flight(nm)
        band = []
        for R in RANGES[nm]:
            hw = 0.0
            for ic in range(len(OFFS)):
                if (nm, float(R), ic, True) in res:
                    hw = float(OFFS[ic]) + 0.075
                else:
                    break
            band.append((float(R), round(hw, 3)))
        out = dict(name=nm, energy_J=energy(nm), **{k: CLASSES[nm][k] for k in ("side", "m_w", "v0", "theta")},
                   t=[round(float(v), 4) for v in t], x=[round(float(v), 3) for v in x], open=[round(float(v), 3) for v in op], band=band)
        (HERE / "out" / f"net_{nm}.json").write_text(json.dumps(out))
        ok = [(R, hw) for R, hw in band if hw > 0]
        print(f"{nm}: {energy(nm):.0f} J, net reaches {x.max():.1f} m; catches a hovering drone from {ok[0][0]:.1f} to {ok[-1][0]:.1f} m, "
              f"half-width up to {max(h for _, h in ok):.2f} m; time to 5 m {np.interp(5.0, x, t):.2f} s")


if __name__ == "__main__":
    main(sys.argv[1:] or ["light", "medium"])
