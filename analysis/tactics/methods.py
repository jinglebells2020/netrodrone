"""The capture methods compared by compare_methods.py, and the aim-offset grid each one is swept over.

Target frame (see capture.py): the target sits at the origin, the air moves at -u along x when the
target flies at +u, and the interceptor shadows it. Every run places the interceptor at an aim offset
(along x, across y) from the ideal spot; compare_eval.py then weights those runs by a random aiming error.
"""
import csv

import numpy as np

V2 = dict(mode="launch", side=1.2, n=11, m_w=0.008, v0=9.3, theta=22.0)        # V2 piston, M120 spring (conservative)
LATEX = dict(mode="launch", side=2.0, n=17, m_w=0.015, v0=20.0, theta=22.0)    # latex-band launcher: estimate, not designed
DRAPE_FIX = dict(mode="drape", side=1.5, n=13, m_w=0.020, tether=3.0, v_cmd=(3.0, 0.0, -0.0001), h_min=2.99,
                 climb_after=0.6, v_climb=1.0, back_kick=4.0, pouch_drop=0.5, t_end=1.8)
DRAPE_ORIG = dict(mode="drape", side=1.5, n=13, m_w=0.020, tether=1.0, v_cmd=(2.5, 0.0, -3.0), h_min=0.8,
                  climb_after=0.25, v_climb=3.0, back_kick=0.0, pouch_drop=0.0, t_end=1.8)

STEP = 0.15          # grid spacing of aim offsets (m)
SPEEDS = [0.0, 2.0, 4.0, 6.0]   # target speed over the ground (m/s); the interceptor flies along with it


def axis(lo, hi):
    return np.round(np.arange(round(lo / STEP), round(hi / STEP) + 1) * STEP, 3)


# name -> (Shot parameters, interceptor start from the 2 aim offsets (a = along/height, c = across),
#          grid centre vs target speed, a offsets, c offsets (mirrored, the setups are symmetric across))
VARIANTS = {
    # forward-firing V2 head, muzzle 0.15 m below the props, interceptor D behind and `a` above the target
    "fwd_v2_D2.25":    (dict(V2, axis=(1.0, 0, 0), drop=0.15), lambda a, c: (-2.25, c, a), lambda u: 1.5, axis(-1.8, 1.8), axis(0, 1.05)),
    "fwd_v2_D3.0":     (dict(V2, axis=(1.0, 0, 0), drop=0.15), lambda a, c: (-3.0, c, a), lambda u: 1.5, axis(-1.8, 1.8), axis(0, 1.05)),
    # V2 head pointing straight down, H above the target
    "down_v2_H3.0":    (dict(V2, axis=(0, 0, -1.0)), lambda a, c: (a, c, 3.0), lambda u: 0.06 * u, axis(-1.2, 1.2), axis(0, 1.05)),
    "down_v2_H4.0":    (dict(V2, axis=(0, 0, -1.0)), lambda a, c: (a, c, 4.0), lambda u: 0.08 * u, axis(-1.2, 1.2), axis(0, 1.05)),
    # latex launcher (estimate) pointing down
    "down_latex_H3.5": (dict(LATEX, axis=(0, 0, -1.0)), lambda a, c: (a, c, 3.5), lambda u: 0.05 * u, axis(-1.65, 1.65), axis(0, 1.35)),
    "down_latex_H5.0": (dict(LATEX, axis=(0, 0, -1.0)), lambda a, c: (a, c, 5.0), lambda u: 0.07 * u, axis(-1.65, 1.65), axis(0, 1.35)),
    # drape: release point along the pass (a) and sideways offset (c), 3 m above the target
    "drape_fixed":     (DRAPE_FIX, lambda a, c: (-1.0 + a, c, 3.0), lambda u: 0.0, axis(-3.0, 3.0), axis(0, 1.05)),
    "drape_original":  (DRAPE_ORIG, lambda a, c: (-1.0 + a, c, 3.0), lambda u: 0.0, axis(-2.0, 2.0), axis(0, 1.05)),
}

# method label -> variants (the best variant at each speed counts, e.g. the better firing height)
GROUPS = {
    "V2 launcher, fire down from above": ["down_v2_H3.0", "down_v2_H4.0"],
    "Latex launcher, fire down (estimate)": ["down_latex_H3.5", "down_latex_H5.0"],
    "Drape, fixed version": ["drape_fixed"],
    "V2 launcher, fire forward from above-behind": ["fwd_v2_D2.25", "fwd_v2_D3.0"],
    "Drape, your original version": ["drape_original"],
}

COLUMNS = ["variant", "speed", "ia", "ic", "aim_along", "aim_across", "caught", "t_catch", "min_clearance",
           "max_tether_N", "contacts_max", "R_min", "open_at_catch"]


def write_rows(rows, path):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        for r in sorted(rows, key=lambda r: (r[0], r[1], r[2], r[3])):
            w.writerow([r[0], f"{r[1]:g}", r[2], r[3], f"{r[4]:.3f}", f"{r[5]:.3f}", int(bool(r[6])),
                        "" if r[7] is None else f"{r[7]:.5f}", f"{r[8]:.4f}", f"{r[9]:.2f}", r[10],
                        "" if r[11] is None else f"{r[11]:.4f}", "" if r[12] is None else f"{r[12]:.4f}"])


def read_rows(path):
    out = []
    with open(path, newline="") as f:
        for d in csv.DictReader(f):
            num = lambda k: None if d[k] == "" else float(d[k])  # noqa: E731
            out.append((d["variant"], float(d["speed"]), int(d["ia"]), int(d["ic"]), float(d["aim_along"]),
                        float(d["aim_across"]), d["caught"] == "1", num("t_catch"), float(d["min_clearance"]),
                        float(d["max_tether_N"]), int(d["contacts_max"]), num("R_min"), num("open_at_catch")))
    return out
