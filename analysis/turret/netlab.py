"""Net model for comparing launcher ideas: any number of weights, any net outline, a sideways ('spiral') launch angle,
and a net bundle that is itself pushed out of the cup.

Same physics as analysis/tactics/capture.py (tension-only lines with damping, drag on every line, gravity, a 0.25 m
target that the net can wrap, the same 'caught' rule), but with a rest length per line and a mass per mesh point so the
net does not have to be a square with four corners. `python analysis/turret/netlab.py` checks it against capture.py.
"""
import math
import sys
from pathlib import Path

import numpy as np
from numba import njit

G, RHO, EA, ZETA = 9.81, 1.2, 1500.0, 0.6


@njit(cache=True)
def _fly(P, V, m, E, L0, kE, cE, line_d, W, WV, mw, att, link_len, k_link, c_link, Aw, tr, k_c, dt, t_end,
         catch_nodes, post_catch, rec_every, rec):
    N, Ne, Nw = P.shape[0], E.shape[0], W.shape[0]
    F = np.zeros((N, 3))
    FW = np.zeros((Nw, 3))
    steps = int(t_end / dt)
    t_catch, R_min, nrec, peak = -1.0, 2.0, 0, 0.0
    for it in range(steps):
        t = it * dt
        for i in range(N):
            F[i, 0] = 0.0; F[i, 1] = 0.0; F[i, 2] = -G * m[i]
        for e in range(Ne):
            a = E[e, 0]; b = E[e, 1]
            dx = P[b, 0] - P[a, 0]; dy = P[b, 1] - P[a, 1]; dz = P[b, 2] - P[a, 2]
            l = math.sqrt(dx * dx + dy * dy + dz * dz) + 1e-12
            ux = dx / l; uy = dy / l; uz = dz / l
            if l > L0[e]:
                vrel = (V[b, 0] - V[a, 0]) * ux + (V[b, 1] - V[a, 1]) * uy + (V[b, 2] - V[a, 2]) * uz
                T = kE[e] * (l - L0[e]) + cE[e] * vrel
                if T > 0.0:
                    if T > peak:
                        peak = T
                    F[a, 0] += T * ux; F[a, 1] += T * uy; F[a, 2] += T * uz
                    F[b, 0] -= T * ux; F[b, 1] -= T * uy; F[b, 2] -= T * uz
            vmx = 0.5 * (V[a, 0] + V[b, 0]); vmy = 0.5 * (V[a, 1] + V[b, 1]); vmz = 0.5 * (V[a, 2] + V[b, 2])
            vpar = vmx * ux + vmy * uy + vmz * uz
            vnx = vmx - vpar * ux; vny = vmy - vpar * uy; vnz = vmz - vpar * uz
            vnn = math.sqrt(vnx * vnx + vny * vny + vnz * vnz)
            seg = l if l < L0[e] else L0[e]
            cf = -0.5 * RHO * 1.2 * line_d * seg * vnn * 0.5
            F[a, 0] += cf * vnx; F[a, 1] += cf * vny; F[a, 2] += cf * vnz
            F[b, 0] += cf * vnx; F[b, 1] += cf * vny; F[b, 2] += cf * vnz
        ncont = 0
        if tr > 0.0:
            for i in range(N):
                r = math.sqrt(P[i, 0] ** 2 + P[i, 1] ** 2 + P[i, 2] ** 2)
                if r < tr + 0.04:
                    ncont += 1
                if r < tr:
                    nx = P[i, 0] / r; ny = P[i, 1] / r; nz = P[i, 2] / r
                    vr = V[i, 0] * nx + V[i, 1] * ny + V[i, 2] * nz
                    fn = k_c * (tr - r) - math.sqrt(k_c * m[i]) * (vr if vr < 0.0 else 0.0)
                    F[i, 0] += fn * nx; F[i, 1] += fn * ny; F[i, 2] += fn * nz
        for k in range(Nw):
            FW[k, 0] = 0.0; FW[k, 1] = 0.0; FW[k, 2] = -G * mw[k]
            c = att[k]
            dx = P[c, 0] - W[k, 0]; dy = P[c, 1] - W[k, 1]; dz = P[c, 2] - W[k, 2]
            ll = math.sqrt(dx * dx + dy * dy + dz * dz) + 1e-12
            ux = dx / ll; uy = dy / ll; uz = dz / ll
            if ll > link_len:
                vr = (V[c, 0] - WV[k, 0]) * ux + (V[c, 1] - WV[k, 1]) * uy + (V[c, 2] - WV[k, 2]) * uz
                Tl = k_link * (ll - link_len) + c_link[k] * vr
                if Tl > 0.0:
                    FW[k, 0] += Tl * ux; FW[k, 1] += Tl * uy; FW[k, 2] += Tl * uz
                    F[c, 0] -= Tl * ux; F[c, 1] -= Tl * uy; F[c, 2] -= Tl * uz
            sp = math.sqrt(WV[k, 0] ** 2 + WV[k, 1] ** 2 + WV[k, 2] ** 2)
            cf = -0.5 * RHO * 0.9 * Aw * sp
            FW[k, 0] += cf * WV[k, 0]; FW[k, 1] += cf * WV[k, 1]; FW[k, 2] += cf * WV[k, 2]
            if tr > 0.0:
                rr = math.sqrt(W[k, 0] ** 2 + W[k, 1] ** 2 + W[k, 2] ** 2)
                if rr < tr:
                    fn = k_c * 5.0 * (tr - rr) / rr
                    FW[k, 0] += fn * W[k, 0]; FW[k, 1] += fn * W[k, 1]; FW[k, 2] += fn * W[k, 2]
        for i in range(N):
            V[i, 0] += F[i, 0] / m[i] * dt; V[i, 1] += F[i, 1] / m[i] * dt; V[i, 2] += F[i, 2] / m[i] * dt
            P[i, 0] += V[i, 0] * dt; P[i, 1] += V[i, 1] * dt; P[i, 2] += V[i, 2] * dt
        for k in range(Nw):
            WV[k, 0] += FW[k, 0] / mw[k] * dt; WV[k, 1] += FW[k, 1] / mw[k] * dt; WV[k, 2] += FW[k, 2] / mw[k] * dt
            W[k, 0] += WV[k, 0] * dt; W[k, 1] += WV[k, 1] * dt; W[k, 2] += WV[k, 2] * dt
        if rec_every > 0 and it % rec_every == 0 and nrec < rec.shape[0]:
            sx = 0.0; sy = 0.0; sz = 0.0
            for i in range(N):
                sx += P[i, 0]; sy += P[i, 1]; sz += P[i, 2]
            rec[nrec, 0] = t; rec[nrec, 1] = sx / N; rec[nrec, 2] = sy / N; rec[nrec, 3] = sz / N
            for k in range(Nw):
                c = att[k]
                rec[nrec, 4 + 3 * k] = P[c, 0]; rec[nrec, 5 + 3 * k] = P[c, 1]; rec[nrec, 6 + 3 * k] = P[c, 2]
            nrec += 1
        if ncont >= catch_nodes:
            sx = 0.0; sy = 0.0; sz = 0.0; cnt = 0
            for i in range(N):
                r = math.sqrt(P[i, 0] ** 2 + P[i, 1] ** 2 + P[i, 2] ** 2)
                if r < tr + 0.04:
                    sx += P[i, 0] / r; sy += P[i, 1] / r; sz += P[i, 2] / r; cnt += 1
            Rr = math.sqrt(sx * sx + sy * sy + sz * sz) / cnt
            if Rr < R_min:
                R_min = Rr
            if t_catch < 0.0:
                t_catch = t
        if t_catch >= 0.0 and t >= t_catch + post_catch:
            break
    return t_catch, R_min, nrec, peak


def polygon_net(n_sides, area, h=0.15, phase=None):
    """Square-mesh net (mesh size h) cut to a regular polygon of the given area, with a rope round the edge.
    -> points (N, 2), lines (Ne, 2), rest lengths, attachment point for each of the n_sides weights, their directions"""
    R = math.sqrt(2 * area / (n_sides * math.sin(2 * math.pi / n_sides)))
    phase = math.pi / n_sides if phase is None else phase               # 4 sides -> corners at 45 deg, as in capture.py
    ang = phase + 2 * math.pi * np.arange(n_sides) / n_sides
    vx, vy = R * np.cos(ang), R * np.sin(ang)
    apothem = R * math.cos(math.pi / n_sides)
    half = int(math.ceil(R / h)) + 1
    off = 0.5 * h if int(round(2 * apothem / h)) % 2 == 1 and n_sides == 4 else 0.0
    idx, pts = {}, []
    for i in range(-half, half + 1):
        for j in range(-half, half + 1):
            x, y = i * h + off, j * h + off
            inside = True
            for k in range(n_sides):                                   # inside every edge of the polygon
                k2 = (k + 1) % n_sides
                if (vx[k2] - vx[k]) * (y - vy[k]) - (vy[k2] - vy[k]) * (x - vx[k]) < -1e-9:
                    inside = False
                    break
            if inside:
                idx[(i, j)] = len(pts)
                pts.append((x, y))
    pts = np.array(pts)
    edges, rest, nbr = [], [], np.zeros(len(pts), int)
    for (i, j), a in idx.items():
        for d in ((1, 0), (0, 1)):
            b = idx.get((i + d[0], j + d[1]))
            if b is not None:
                edges.append((a, b)); rest.append(h); nbr[a] += 1; nbr[b] += 1
    rim = np.where(nbr < 4)[0]
    rim = rim[np.argsort(np.arctan2(pts[rim, 1], pts[rim, 0]))]
    have = set((min(a, b), max(a, b)) for a, b in edges)
    for q in range(len(rim)):                                          # rope round the edge
        a, b = int(rim[q]), int(rim[(q + 1) % len(rim)])
        if (min(a, b), max(a, b)) not in have:
            edges.append((a, b)); rest.append(float(np.linalg.norm(pts[a] - pts[b]))); have.add((min(a, b), max(a, b)))
    att = np.array([int(rim[np.argmin((pts[rim, 0] - vx[k]) ** 2 + (pts[rim, 1] - vy[k]) ** 2)]) for k in range(n_sides)])
    return pts, np.array(edges, dtype=np.int64), np.array(rest), att, ang


def launch(n_weights=4, area=4.0, m_net=0.011, line_d=0.4e-3, m_w_total=0.080, w_area=0.0004, v0=30.0, theta=16.0, skew=0.0,
           v_net=0.0, axis=(1.0, 0.0, 0.0), start=(0.0, 0.0, 0.0), target_r=0.0, t_end=1.2, seed=1, h=0.15, tether=0.10,
           r0=0.05, pack_r=0.03, pack_len=0.05, n_rec=0):
    """One shot. skew = sideways launch angle of each weight around the axis (deg): the 'spiral'.
    v_net = speed at which the folded net itself is pushed out along the axis (0 = it waits to be dragged out)."""
    rng = np.random.default_rng(seed)
    pts, E, L0, att, ang = polygon_net(n_weights, area, h)
    N = len(pts)
    m = np.full(N, m_net / N)
    kE = EA / L0
    cE = 2 * ZETA * np.sqrt(kE * (m_net / N))
    a = np.array(axis, float); a /= np.linalg.norm(a)
    ref = np.array([1.0, 0, 0]) if abs(a[2]) > 0.9 else np.array([0, 1.0, 0])
    e1 = ref - a * np.dot(ref, a); e1 /= np.linalg.norm(e1)
    e2 = np.cross(a, e1)
    M = np.array(start, float)
    rr = pack_r * np.sqrt(rng.random(N)); ph = 2 * np.pi * rng.random(N)
    P = M + np.outer(pack_len * rng.random(N), a) + np.outer(rr * np.cos(ph), e1) + np.outer(rr * np.sin(ph), e2)
    V = np.tile(v_net * a, (N, 1))
    th, sk = math.radians(theta), math.radians(skew)
    mw = np.full(n_weights, m_w_total / n_weights)
    W = np.zeros((n_weights, 3)); WV = np.zeros((n_weights, 3))
    for k in range(n_weights):
        radial = math.cos(ang[k]) * e1 + math.sin(ang[k]) * e2
        tang = np.cross(a, radial)
        W[k] = M + r0 * radial
        WV[k] = v0 * (math.cos(th) * a + math.sin(th) * (math.cos(sk) * radial + math.sin(sk) * tang))
    k_link = EA / tether
    c_link = 2 * ZETA * np.sqrt(k_link * (m[0] * mw / (m[0] + mw)))
    dt = min(2e-5, 0.6 * 2e-5 * ((m_net / N) / (0.01496 / 121)) ** 0.5)
    rec = np.zeros((max(n_rec, 1), 4 + 3 * n_weights))
    t_catch, R_min, nrec, peak = _fly(P, V, m, E, L0, kE, cE, line_d, W, WV, mw, att, tether, k_link, c_link, w_area,
                                      target_r, 400.0, dt, t_end, 12, 0.10, max(1, int(0.004 / dt)) if n_rec else 0, rec)
    out = dict(caught=bool(t_catch >= 0 and R_min <= 0.75), t_catch=t_catch if t_catch >= 0 else None, peak_tension=peak,
               n_points=N, line_m=float(L0.sum()))
    if n_rec:
        r = rec[:nrec]
        c = r[:, 4:].reshape(-1, n_weights, 3)
        ctr = c.mean(axis=1)
        u, v = (c - ctr[:, None, :]) @ e1, (c - ctr[:, None, :]) @ e2
        area_t = 0.5 * np.abs(np.sum(u * np.roll(v, -1, axis=1) - np.roll(u, -1, axis=1) * v, axis=1))    # outline of the weights
        # how far the whole pattern of weights has turned about the axis, relative to the directions they were fired in
        twist = np.degrees(np.angle(np.mean(np.exp(1j * (np.arctan2(v, u) - ang[None, :])), axis=1)))
        out.update(t=r[:, 0], x=(r[:, 1:4] - M) @ a, open=area_t / area, twist=twist)
    return out


if __name__ == "__main__":
    # check against capture.py: same square 2 m net, 4 x 20 g at 30 m/s, 16 deg, fired 20 deg up, free flight
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from classes import turret_shot
    from capture import run
    ax = (math.cos(math.radians(20)), 0.0, math.sin(math.radians(20)))
    s = turret_shot("light", ax, (0.0, 0.0, 0.0), t_end=1.0, target_r=0.0, stop_on_catch=False)
    _, rec = run(s, rec_every=max(1, int(0.004 / s.dt)), n_rec=400)
    c = rec[:, 4:16].reshape(-1, 4, 3)
    ref_open = 0.5 * np.linalg.norm(np.cross(c[:, 3] - c[:, 0], c[:, 1] - c[:, 2]), axis=1) / 4.0
    ref_x = rec[:, 1:4] @ np.array(ax)
    new = launch(axis=ax, t_end=1.0, n_rec=400)
    print("distance   open (capture.py)   open (netlab.py)")
    for d in (2, 3, 4, 5, 6, 8):
        print(f"  {d} m        {np.interp(d, ref_x, ref_open) * 100:5.0f} %           {np.interp(d, new['x'], new['open']) * 100:5.0f} %")
    print(f"time to 5 m: {np.interp(5, ref_x, rec[:, 0]):.3f} s vs {np.interp(5, new['x'], new['t']):.3f} s;  mesh points {new['n_points']}, line {new['line_m']:.0f} m")
