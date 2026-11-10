"""How much compressed air does one shot need?  Run: python analysis/turret/pneumatic.py   (~1 min)

Uses the repo's shot model (analysis/ballistics.py) unchanged: the spring is removed, the piston is parked, and the
'cylinder' becomes a pre-charged air chamber that dumps through a valve into the four barrels. The valve is assumed to
open instantly; a real one must open in a few milliseconds because the whole shot lasts about 10 ms.
"""
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[0]))
sys.path.insert(0, str(HERE))
from ballistics import Gun, run, R, TATM, PATM  # noqa: E402
from classes import CLASSES, energy  # noqa: E402

# class -> barrel bore (mm), weight travel (mm), O-ring drag per weight (N), and the chamber / pressure / valve cases to try
CASES = {
    "light":  (16.0, 150.0, 3.0, [(0.10, 6, 10), (0.20, 6, 10), (0.20, 8, 10), (0.30, 8, 15)]),
    "medium": (25.0, 250.0, 3.0, [(0.50, 8, 12), (0.75, 8, 19), (1.00, 8, 19), (1.00, 10, 19)]),
    "heavy":  (32.0, 400.0, 5.0, [(3.0, 8, 25), (4.0, 8, 32), (5.0, 8, 32), (4.0, 10, 32)]),
}


def shot(chamber_l, p_bar, valve_mm, bore_mm, travel_mm, m_w, f_slide):
    bore = 0.05
    g = Gun(bore=bore, stroke=chamber_l * 1e-3 / (np.pi / 4 * bore ** 2), v_dead1=1e-6, m_piston=1.0, m_spring=0.0, f_piston=0.0,
            k=0.0, c0=0.0, port_d=valve_mm * 1e-3, port_cd=0.7, v_dead2=20e-6, n_proj=4, proj_d=bore_mm * 1e-3, m_proj=m_w,
            f_break=2.5 * f_slide, f_slide=f_slide, barrel_len=travel_mm * 1e-3, leak_area=4 * 0.5e-6)
    p1 = PATM + p_bar * 1e5
    v1 = g.v_dead1 + g.Ap * g.stroke
    init = np.array([p1 * v1 / (R * TATM), TATM, PATM * g.v_dead2 / (R * TATM), TATM])
    res, _ = run(g.params(), g.n_proj * g.f_break, g.barrel_len, 2e-7, 0.2, 10 ** 9, init)
    v = max(float(res[0]), 0.0)
    return v, 0.5 * 4 * m_w * v * v, float(res[1])


if __name__ == "__main__":
    print("| class | target | chamber | pressure | valve bore | barrels (bore x travel) | weight speed | energy | shot lasts |")
    print("|---|---|---|---|---|---|---|---|---|")
    for name, (bore, travel, fs, cases) in CASES.items():
        c = CLASSES[name]
        for ch, p, vd in cases:
            v, e, t = shot(ch, p, vd, bore, travel, c["m_w"], fs)
            print(f"| {name} | {c['v0']:.0f} m/s, {energy(name):.0f} J | {ch:.2f} L | {p} bar | {vd} mm | {bore:.0f} x {travel:.0f} mm | "
                  f"{v:.0f} m/s | {e:.0f} J | {t * 1e3:.0f} ms |")
