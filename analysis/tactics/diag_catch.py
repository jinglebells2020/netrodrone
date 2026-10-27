"""Look inside individual catches: how spread out the net is when it reaches the target.
Run: python analysis/tactics/diag_catch.py   (~10 s)

This is how the 'strict' rule was chosen. A forward shot from above-behind lands open (R_min ~0.3-0.5);
a bundle of line hitting the target gives R_min close to 1.
  R_min   spread of the contact patch (1 = one spot, 0.5 = a whole hemisphere covered)
  open    corner-to-corner area of the net when it first catches, as a fraction of a flat net
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from capture import Shot, run  # noqa: E402
from methods import VARIANTS  # noqa: E402

CASES = [("fwd_v2_D3.0", 0.0, 1.53), ("fwd_v2_D3.0", 0.0, 0.6), ("fwd_v2_D2.25", 0.0, 0.5), ("fwd_v2_D2.25", 4.0, 1.1),
         ("fwd_v2_D2.25", 6.0, 1.45), ("fwd_v2_D2.25", 6.0, 0.5), ("down_v2_H4.0", 0.0, 0.0), ("down_v2_H4.0", 6.0, 0.45),
         ("down_v2_H3.0", 0.0, 0.0), ("down_latex_H5.0", 0.0, 0.0), ("drape_fixed", 0.0, 0.78), ("drape_fixed", 0.0, 0.0),
         ("drape_fixed", 4.0, 1.1), ("drape_original", 0.0, -0.38)]

if __name__ == "__main__":
    for nm, u, a in CASES:
        base, place, _, _, _ = VARIANTS[nm]
        for c in (0.0, 0.45):
            r = run(Shot(**dict(base, start=place(a, c), wind=(-u, 0.0, 0.0), seed=7)))
            if r["caught"]:
                v = r["v_at_catch"]
                print(f"{nm:16s} u={u:.0f} aim {a:+.2f},{c:.2f}: catch {r['t_catch']:.2f}s  R_min {r['R_min']:.2f}  "
                      f"open {r['open_at_catch']:.2f}  nodes {r['contacts_post']:3d}  net v ({v[0]:+.1f},{v[1]:+.1f},{v[2]:+.1f}) m/s")
            else:
                print(f"{nm:16s} u={u:.0f} aim {a:+.2f},{c:.2f}: miss (max nodes {r['contacts_max']})")
