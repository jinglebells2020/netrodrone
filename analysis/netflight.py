"""
Net flight: lumped mass-spring model of a square net pulled out of the launcher cup by 4 corner weights.

  - net = n x n grid of point masses joined by tension-only lines (with damping, so picking up
    slack line is inelastic, like a real net paying out of a cup)
  - the net starts bunched up in the cup at rest; the 4 weights leave the muzzles at v0 and
    half-angle theta from the launch axis and pull it out through 10 cm corner tethers
  - air drag on every line segment (cross-flow, Cd 1.2) and on the weights, gravity
  Launch along +X, gravity -Z.  SI units.

'capture window' = range of distances where the net is >= 70 % open AND its centre still moves
faster than 3 m/s (enough to wrap a small drone).
"""
import numpy as np
from dataclasses import dataclass


@dataclass
class NetShot:
    side: float = 1.2           # net side length (m)
    n: int = 13                 # nodes per side
    m_net: float = 0.012        # net mass (kg)
    line_d: float = 0.4e-3      # mesh line diameter (m)
    m_w: float = 0.010          # each corner weight (kg)
    v0: float = 10.0            # weight muzzle speed (m/s)
    theta: float = 22.0         # half-angle of weights from axis (deg)
    r0: float = 0.030           # muzzle radius from axis (m)
    tether: float = 0.10        # weight-to-corner tether length (m)
    pack_r: float = 0.020       # packed net radius in cup (m)
    pack_len: float = 0.030     # packed net depth (m)
    w_d: float = 0.012          # weight diameter (m) for drag
    w_len: float = 0.018
    EA: float = 1500.0          # line axial stiffness used in the sim (N)
    zeta: float = 0.6           # damping ratio of line elements
    t_end: float = 0.9
    dt: float = 2e-5
    seed: int = 1


def simulate_net(s: NetShot, record_every=250, snap_times=()):
    rng = np.random.default_rng(s.seed)
    n = s.n
    N = n * n
    L0 = s.side / (n - 1)
    # node masses: line mass split evenly
    m = np.full(N, s.m_net / N)
    # edges (grid)
    E = []
    for i in range(n):
        for j in range(n):
            a = i * n + j
            if j + 1 < n: E.append((a, a + 1))
            if i + 1 < n: E.append((a, a + n))
    E = np.array(E)
    rest = np.full(len(E), L0)
    # packed initial positions: random inside small cylinder in front of origin
    P = np.zeros((N, 3))
    rr = s.pack_r * np.sqrt(rng.random(N)); ph = 2 * np.pi * rng.random(N)
    P[:, 0] = s.pack_len * rng.random(N)
    P[:, 1] = rr * np.cos(ph); P[:, 2] = rr * np.sin(ph)
    V = np.zeros((N, 3))
    # weights: 4 extra bodies tied to corners with tether
    corners = np.array([0, n - 1, N - n, N - 1])
    th = np.radians(s.theta)
    dirs = []
    W = np.zeros((4, 3)); WV = np.zeros((4, 3))
    for k, ang in enumerate([225, 315, 135, 45]):  # corner layout around axis
        a = np.radians(ang)
        radial = np.array([0, np.cos(a), np.sin(a)])
        d = np.array([np.cos(th), 0, 0]) + np.sin(th) * radial
        W[k] = s.r0 * radial
        WV[k] = s.v0 * d
    # tether edges: weight k <-> corner node
    mw = s.m_w
    g = np.array([0, 0, -9.81])
    rho = 1.2
    Aw = s.w_d * s.w_len   # side-on area, conservative
    Cdw = 0.9
    k_line = s.EA / L0
    k_teth = s.EA / s.tether
    # damping coefficient per element: c = 2 zeta sqrt(k m_eff)
    c_line = 2 * s.zeta * np.sqrt(k_line * (s.m_net / N))
    m_node = s.m_net / N
    c_teth = 2 * s.zeta * np.sqrt(k_teth * (m_node * mw / (m_node + mw)))
    steps = int(s.t_end / s.dt)
    rec = []
    snaps = {}
    snap_steps = {int(round(t / s.dt)): t for t in snap_times}
    for it in range(steps):
        F = np.zeros((N, 3)); FW = np.zeros((4, 3))
        # net lines
        d = P[E[:, 1]] - P[E[:, 0]]
        l = np.linalg.norm(d, axis=1) + 1e-12
        u = d / l[:, None]
        dv = V[E[:, 1]] - V[E[:, 0]]
        ext = l - rest
        vrel = np.einsum("ij,ij->i", dv, u)
        T = np.where(ext > 0, k_line * ext + c_line * vrel, 0.0)
        T = np.maximum(T, 0.0)
        f = T[:, None] * u
        np.add.at(F, E[:, 0], f); np.add.at(F, E[:, 1], -f)
        # line drag: each element, cross-flow velocity (mean of endpoints)
        vm = 0.5 * (V[E[:, 0]] + V[E[:, 1]])
        vn = vm - np.einsum("ij,ij->i", vm, u)[:, None] * u
        vnm = np.linalg.norm(vn, axis=1)
        seg_len = np.minimum(l, rest)  # drag area of the physical line (slack lines bunch up)
        fd = -0.5 * rho * 1.2 * s.line_d * seg_len[:, None] * vnm[:, None] * vn
        # axial skin drag small
        np.add.at(F, E[:, 0], 0.5 * fd); np.add.at(F, E[:, 1], 0.5 * fd)
        # tethers
        dt_ = P[corners] - W
        lt = np.linalg.norm(dt_, axis=1) + 1e-12
        ut = dt_ / lt[:, None]
        vr = np.einsum("ij,ij->i", V[corners] - WV, ut)
        Tt = np.where(lt > s.tether, k_teth * (lt - s.tether) + c_teth * vr, 0.0)
        Tt = np.maximum(Tt, 0)
        FW += Tt[:, None] * ut
        np.add.at(F, corners, -Tt[:, None] * ut)
        # weight drag + gravity
        sp = np.linalg.norm(WV, axis=1)
        FW += -0.5 * rho * Cdw * Aw * sp[:, None] * WV
        FW += mw * g
        F += m[:, None] * g
        # integrate (semi-implicit Euler)
        V += F / m[:, None] * s.dt
        P += V * s.dt
        WV += FW / mw * s.dt
        W += WV * s.dt
        if it in snap_steps:
            snaps[snap_steps[it]] = (P.copy(), W.copy(), E, corners)
        if it % record_every == 0:
            # opening = area of quadrilateral of the 4 net corners projected on the YZ plane
            c = P[corners][:, 1:]
            q = c[[0, 1, 3, 2]]
            area = 0.5 * abs(np.dot(q[:, 0], np.roll(q[:, 1], 1)) - np.dot(q[:, 1], np.roll(q[:, 0], 1)))
            com = (P * m[:, None]).sum(0) / m.sum()
            comv = (V * m[:, None]).sum(0) / m.sum()
            wcom = W.mean(0); wv = WV.mean(0)
            rec.append(dict(t=it * s.dt, area=area, x_net=com[0], z_net=com[2], vx_net=comv[0], x_w=wcom[0], z_w=wcom[2],
                            vx_w=wv[0], span=np.linalg.norm(c[0] - c[3])))
    if snap_times:
        return rec, snaps
    return rec


def summarize(s: NetShot, rec):
    full = s.side ** 2
    out = {}
    for frac in [0.5, 0.7, 0.9]:
        hit = next((r for r in rec if r["area"] >= frac * full), None)
        out[f"open{int(frac*100)}"] = (round(hit["x_net"], 2), round(hit["t"], 3), round(hit["vx_net"], 2), round(hit["z_net"], 2)) if hit else None
    mx = max(rec, key=lambda r: r["area"])
    out["max_open_frac"] = round(mx["area"] / full, 2)
    out["x_at_max_open"] = round(mx["x_net"], 2)
    # 'effective capture window': distance range where net >= 70% open and moving > 3 m/s
    win = [r for r in rec if r["area"] >= 0.7 * full and r["vx_net"] > 3.0]
    out["capture_window_x"] = (round(win[0]["x_net"], 2), round(win[-1]["x_net"], 2)) if win else None
    return out


if __name__ == "__main__":
    import time
    t0 = time.time()
    s = NetShot()
    rec = simulate_net(s)
    print("time %.1fs" % (time.time() - t0))
    print(summarize(s, rec))
