"""Net shot fired from a drone that keeps flying at speed u (same direction as the shot).
Air is still, so the net meets a headwind of u. Positions are measured relative to the drone.
Also: firing straight down from a hover (gravity helps).
Run: python analysis/tactics/airspeed.py   (~1 min on 4 cores)"""
import sys, numpy as np
from pathlib import Path
from multiprocessing import Pool
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # analysis/ (netflight.py)
import netflight as nf
from netflight import NetShot

src = open(nf.__file__).read()
# make a variant that adds a carrier velocity to every body at launch, and optionally rotates the shot
src = src.replace("def simulate_net(s: NetShot, record_every=250, snap_times=()):",
                  "def simulate_net(s: NetShot, record_every=250, snap_times=(), u_carrier=(0.0, 0.0, 0.0), down=False):")
src = src.replace("    mw = s.m_w\n", "    mw = s.m_w\n    U = np.array(u_carrier, float)\n    V += U\n    WV += U\n")
# firing straight down == gravity pointing along the launch direction (+X); opening plane stays YZ
src = src.replace("    g = np.array([0, 0, -9.81])", "    g = np.array([9.81, 0, 0]) if down else np.array([0, 0, -9.81])")
ns = {}
exec(compile(src, "netflight_variant", "exec"), ns)
sim = ns["simulate_net"]

def run(args):
    u, down = args
    s = NetShot(side=1.2, m_net=0.009 * 1.44 + 0.002, m_w=0.008, v0=9.3, theta=22.0, n=11, t_end=0.9)
    rec = sim(s, u_carrier=(u, 0.0, 0.0), down=down)
    out = []
    for r in rec:
        t = r["t"]
        if down:
            rel = r["x_net"]             # distance below the drone (launch axis = down)
        else:
            rel = r["x_net"] - u * t      # distance ahead of the drone
        out.append((t, rel, r["area"] / 1.44, r["vx_net"] - u))
    return u, down, out

if __name__ == "__main__":
    jobs = [(0.0, False), (4.0, False), (8.0, False), (12.0, False), (0.0, True)]
    with Pool(4) as p:
        res = p.map(run, jobs)
    for u, down, out in res:
        o70 = next(((t, d) for t, d, a, v in out if a >= 0.7), None)
        o90 = next(((t, d) for t, d, a, v in out if a >= 0.9), None)
        dmax = max(d for t, d, a, v in out)
        if not down:
            win = [d for t, d, a, v in out if a >= 0.7 and v > 3.0]
            w = f"{win[0]:.1f}-{win[-1]:.1f} m" if win else "none"
            print(f"flying {u:4.1f} m/s: 70% open at {o70[1] if o70 else float('nan'):.1f} m, 90% at "
                  f"{(o90[1] if o90 else float('nan')):.1f} m, net gets at most {dmax:.1f} m ahead of the drone, window (>3 m/s faster than drone) {w}")
        else:
            win = [d for t, d, a, v in out if a >= 0.7 and v > 3.0]
            print(f"fired straight DOWN from hover: 70% open {o70[1]:.1f} m below, 90% open {o90[1]:.1f} m below, "
                  f"window {win[0]:.1f}-{win[-1]:.1f} m below (still >3 m/s); net reaches {dmax:.1f} m in 0.9 s")
