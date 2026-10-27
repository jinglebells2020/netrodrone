"""(1) Downward strike while interceptor + target both move sideways at speed u.
   (2) Latex-band launcher (faster, heavier weights, bigger net) vs the V2 piston.
Uses the repo net model with: carrier velocity added to every body at launch, gravity direction option.
Run: python analysis/tactics/down_strike.py   (~2 min on 4 cores)"""
import sys, numpy as np
from pathlib import Path
from multiprocessing import Pool
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # analysis/ (netflight.py)
import netflight as nf
from netflight import NetShot
src = open(nf.__file__).read()
src = src.replace("def simulate_net(s: NetShot, record_every=250, snap_times=()):",
                  "def simulate_net(s: NetShot, record_every=250, snap_times=(), u_carrier=(0.0, 0.0, 0.0), gvec=(0.0, 0.0, -9.81)):")
src = src.replace("    mw = s.m_w\n", "    mw = s.m_w\n    U = np.array(u_carrier, float)\n    V += U\n    WV += U\n")
src = src.replace("    g = np.array([0, 0, -9.81])", "    g = np.array(gvec, float)")
src = src.replace("rec.append(dict(t=it * s.dt,", "rec.append(dict(y_net=com[1], vy_net=comv[1], t=it * s.dt,")
ns = {}; exec(compile(src, "nf_variant", "exec"), ns); sim = ns["simulate_net"]

def net_mass(side):
    return 0.009 * side ** 2 + 0.002

def down_strike(args):
    u, v0, mw, side = args
    s = NetShot(side=side, m_net=net_mass(side), m_w=mw, v0=v0, theta=22.0, n=11, t_end=0.9)
    rec = sim(s, u_carrier=(0.0, u, 0.0), gvec=(9.81, 0.0, 0.0))   # launch axis +X = straight down, motion along +Y
    out = {}
    for depth in (2.0, 3.0, 4.0, 5.0):
        r = next((r for r in rec if r["x_net"] >= depth), None)
        if r:
            out[depth] = (r["t"], r["y_net"] - u * r["t"], r["area"] / side ** 2)
    return ("down", u, v0, mw, side), out

def forward(args):
    u, v0, mw, side, theta = args
    s = NetShot(side=side, m_net=net_mass(side), m_w=mw, v0=v0, theta=theta, n=11, t_end=0.9)
    rec = sim(s, u_carrier=(u, 0.0, 0.0))
    o70 = next((r["x_net"] - u * r["t"] for r in rec if r["area"] >= 0.7 * side ** 2), None)
    o90 = next((r["x_net"] - u * r["t"] for r in rec if r["area"] >= 0.9 * side ** 2), None)
    win = [r["x_net"] - u * r["t"] for r in rec if r["area"] >= 0.7 * side ** 2 and r["vx_net"] - u > 3.0]
    dmax = max(r["x_net"] - u * r["t"] for r in rec)
    drop = next((-r["z_net"] for r in rec if r["x_net"] - u * r["t"] >= (win[-1] if win else 1e9)), None)
    return ("fwd", u, v0, mw, side, theta), (o70, o90, (win[0], win[-1]) if win else None, dmax)

if __name__ == "__main__":
    with Pool(4) as p:
        downs = p.map(down_strike, [(u, 9.3, 0.008, 1.2) for u in (0.0, 2.0, 4.0, 6.0)])
        fwds = p.map(forward, [(0.0, 9.3, 0.008, 1.2, 22.0), (0.0, 20.0, 0.015, 2.0, 22.0), (0.0, 20.0, 0.015, 2.0, 18.0),
                               (4.0, 20.0, 0.015, 2.0, 22.0), (8.0, 20.0, 0.015, 2.0, 22.0)])
        downs2 = p.map(down_strike, [(u, 20.0, 0.015, 2.0) for u in (0.0, 4.0)])
    print("== (1) V2 launcher (9.3 m/s, 4x8 g, 1.2 m net) fired straight DOWN while both drones drift sideways at u")
    for k, out in downs:
        u = k[1]
        cells = "  ".join(f"{d:.0f} m below: t={v[0]:.2f}s, net centre {v[1]:+.2f} m vs target, {v[2]*100:3.0f}% open" for d, v in out.items())
        print(f"  u={u:3.0f} m/s | {cells}")
    print("== (2) forward shots: V2 vs latex launcher (20 m/s, 4x15 g, 2.0 m net)")
    for k, (o70, o90, win, dmax) in fwds:
        _, u, v0, mw, side, th = k
        f = lambda x: f"{x:.1f} m" if x is not None else "-"
        w = f"{win[0]:.1f}-{win[1]:.1f} m" if win else "none"
        print(f"  v0={v0:4.1f} m/s, {mw*1000:.0f} g, {side} m net, {th:.0f} deg, drone airspeed {u:.0f} m/s: 70% open {f(o70)}, 90% {f(o90)}, window {w}, reaches {dmax:.1f} m")
    print("== (3) latex launcher fired DOWN")
    for k, out in downs2:
        cells = "  ".join(f"{d:.0f} m: {v[1]:+.2f} m offset, {v[2]*100:3.0f}% open" for d, v in out.items())
        print(f"  u={k[1]:.0f} m/s | {cells}")
