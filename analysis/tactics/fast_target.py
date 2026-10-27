"""What changes when the target flies at 150 km/h (42 m/s)?
Run: python analysis/tactics/fast_target.py   (~1 min)

1. Head-on from a hovering interceptor: when to fire, and how much timing slack the net gives
   (the net drops while it flies, so it is only on the target's path for part of its open time).
2. Simple physics: air load on an open net, dodge offsets, impact energy, what an anchored net
   must hold, where a wrapped (props stopped) target lands, warning time from detection range.
"""
import sys
import math
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # analysis/ (netflight.py)
from netflight import NetShot, simulate_net, summarize  # noqa: E402

V = 150 / 3.6


def main():
    print(f"150 km/h = {V:.2f} m/s")
    # design point net shot from a hovering launcher (the target's speed makes no wind on the net)
    s = NetShot(side=1.2, m_net=0.009 * 1.2 ** 2 + 0.002, m_w=0.008, v0=9.3, theta=22.0, n=11, t_end=1.2)
    rec = simulate_net(s, record_every=50)
    full = s.side ** 2
    op = [r for r in rec if r["area"] >= 0.7 * full]
    win = [r for r in op if r["vx_net"] > 3.0]
    print("summary", summarize(s, rec))
    print(f">=70% open: t {op[0]['t']:.3f}-{op[-1]['t']:.3f} s, x {op[0]['x_net']:.2f}-{op[-1]['x_net']:.2f} m, "
          f"drop {-op[0]['z_net']:.2f}-{-op[-1]['z_net']:.2f} m")
    for name, w in (("open>=70%", op), ("open and moving >3 m/s", win)):
        D = [r["x_net"] + V * r["t"] for r in w]
        print(f"head-on, {name}: fire when the target is {min(D):.1f}-{max(D):.1f} m away; open for {w[-1]['t'] - w[0]['t']:.3f} s")

    # the net falls while it flies: aim so its centre crosses the target's path, then see how long it stays within +-tol
    t = np.array([r["t"] for r in op]); z = np.array([r["z_net"] for r in op]); x = np.array([r["x_net"] for r in op])
    for tol in (0.3, 0.45):
        best = (0.0, None)
        for i in range(len(t)):
            ok = np.abs(z - z[i]) <= tol
            j0 = i
            while j0 > 0 and ok[j0 - 1]:
                j0 -= 1
            j1 = i
            while j1 < len(t) - 1 and ok[j1 + 1]:
                j1 += 1
            if t[j1] - t[j0] > best[0]:
                best = (t[j1] - t[j0], (t[j0], t[j1], x[j0] + V * t[j0], x[j1] + V * t[j1]))
        sl, (a, b, d0, d1) = best
        print(f"height tolerance +-{tol} m: timing slack {sl * 1000:.0f} ms (net open {a:.2f}-{b:.2f} s), "
              f"fire at {d0:.1f}-{d1:.1f} m -> range slack {d1 - d0:.1f} m")
    print(f"time for the target to cross a 1.2 m net side-on at {V:.1f} m/s: {1.2 / V * 1000:.0f} ms")

    # air load on an open net (broadside lines, Cd 1.2) at various airspeeds
    n = 11
    L0 = 1.2 / (n - 1)
    line_len = 2 * n * (n - 1) * L0
    for v in (4, 8, 12, 42, 52):
        F = 0.5 * 1.2 * 1.2 * s.line_d * line_len * v ** 2
        print(f"open-net drag at {v:>2} m/s: {F:6.2f} N = {F / (s.m_net * 9.81):6.1f}x net weight, "
              f"net+weights slow at {F / (s.m_net + 4 * 0.008) / 9.81:5.1f} g")
    for g_ in (0.3, 1, 3):
        for tt in (0.4, 0.6):
            print(f"sideways {g_} g for {tt} s -> {0.5 * g_ * 9.81 * tt * tt:.2f} m")
    for M in (0.8, 1.5, 3.0):
        E = 0.5 * M * V ** 2
        print(f"target {M} kg: {E:.0f} J (= dropped from {E / (M * 9.81):.0f} m); "
              f"speed after a free 47 g net wraps it {M * V / (M + 0.047):.1f} m/s")
    for d in (0.5, 1.0, 2.0):
        a = V ** 2 / (2 * d)
        print(f"anchored net stopping 1.5 kg in {d} m: {a / 9.81:.0f} g, {1.5 * a:.0f} N ({1.5 * a / 9.81:.0f} kgf)")
    print(f"net tethered to a 1.5 kg interceptor: both end up at {1.5 * V / 3.0:.1f} m/s")
    for M, CdA in ((1.5, 0.02), (1.5, 0.03)):
        for h in (5, 10, 20):
            px, pz, vx, vz, dt = 0.0, h, V, 0.0, 1e-3
            while pz > 0:
                sp = math.hypot(vx, vz)
                k = 0.5 * 1.2 * CdA / M
                vx += -k * sp * vx * dt
                vz += (-9.81 - k * sp * vz) * dt
                px += vx * dt
                pz += vz * dt
            print(f"wrapped target, CdA {CdA}: from {h} m up it lands {px:.0f} m further on at {math.hypot(vx, vz):.0f} m/s")
    for R in (100, 200, 300):
        print(f"heard at {R} m -> {R / V:.1f} s until it arrives")


if __name__ == "__main__":
    main()
