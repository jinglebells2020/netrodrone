"""
One capture model for every method we came up with, so they can be compared on equal terms.

Target frame: the target hovers at the origin (a 0.25 m sphere with its rotor downwash column).
If the target moves at +u along x, the air moves at -u (wind). The interceptor 'shadows' the target,
so in this frame it is stationary when it fires (launch) or flies a prescribed pass (drape).

  net     : n x n tension-only mass-spring grid, line drag on every segment (same as netflight.py/drape.py)
  launch  : 4 weights fired from a muzzle on the interceptor (V2 piston or latex), interceptor climbs away
  drape   : back weights released (optionally kicked down), front corners tethered to the interceptor,
            front tethers let go when the net grabs the target (or at release_t_max)
  caught  : at least `catch_nodes` net nodes touching the target at the same time
            (same trigger the drape release uses; mesh spacing kept ~0.12 m for every net).
            run() also returns R_min, the spread of the contact patch over the 0.1 s after that:
            1 = all touching one spot (a bundle of line), 0.5 = net over a whole hemisphere.
            compare_eval.py counts a strict catch when R_min <= 0.75 (net covers >= a 60 deg cap).
Compiled with numba; same physics and constants as drape.py (checked by validate_capture.py).
"""
import numpy as np
from numba import njit
from dataclasses import dataclass

G = 9.81
RHO = 1.2
EA = 1500.0
ZETA = 0.6


@njit(cache=True)
def _air_z(x, y, z, wz, Ix, Iy, Iz, tr, tdw, ir, idw):
    az = wz
    if x * x + y * y < (tr + 0.05) ** 2 and z > -2.0 and z < 1.5:
        az -= tdw
    dx = x - Ix
    dy = y - Iy
    if dx * dx + dy * dy < ir * ir and z < Iz + 0.2 and z > Iz - 2.5:
        az -= idw
    return az


@njit(cache=True)
def _core(P, V, m_node, E, L0, k_line, c_line, line_d,
          W, WV, mw, corners, link_len, k_link, c_link, Cdw, Aw,
          tethered, tether, k_teth, c_teth, arm_f,
          I, kin, wind, tr, tdw, ir, idw, k_c,
          dt, t_end, release_contacts, release_t_max, catch_nodes, clear_t0, stop_on_catch,
          rec_every, rec, post_catch, side):
    N = P.shape[0]
    Ne = E.shape[0]
    VI = np.zeros(3)
    F = np.zeros((N, 3))
    FW = np.zeros((4, 3))
    steps = int(t_end / dt)
    kmode = int(kin[0])
    phase = 0
    t_hmin = 0.0
    t_contact = -1.0
    t_catch = -1.0
    t_release = -1.0
    contacts_max = 0
    min_clear = 1e9
    t_minclear = -1.0
    max_teth = 0.0
    c_contact = 2 * 0.5 * np.sqrt(k_c * m_node)
    wx = wind[0]
    wy = wind[1]
    wz = wind[2]
    nrec = 0
    R_min = 2.0
    open_catch = -1.0
    vrel_catch = np.zeros(3)
    cmax_post = 0
    for it in range(steps):
        t = it * dt
        # ---- interceptor
        if kmode == 0:
            if phase == 0 and I[2] <= kin[4]:
                phase = 1
                t_hmin = t
            if phase == 1 and t - t_hmin >= kin[5]:
                phase = 2
            if phase == 0:
                vcx = kin[1]; vcy = kin[2]; vcz = kin[3]
            elif phase == 1:
                vcx = kin[1]; vcy = 0.0; vcz = 0.0
            else:
                vcx = kin[1] * 0.5; vcy = 0.0; vcz = kin[6]
        else:
            vcx = 0.0; vcy = 0.0; vcz = 0.0
            if t >= kin[8]:
                vcz = kin[6]
        tau = kin[7]
        VI[0] += (vcx - VI[0]) * dt / tau
        VI[1] += (vcy - VI[1]) * dt / tau
        VI[2] += (vcz - VI[2]) * dt / tau
        I[0] += VI[0] * dt
        I[1] += VI[1] * dt
        I[2] += VI[2] * dt
        Ix = I[0]; Iy = I[1]; Iz = I[2]
        # ---- net: gravity, lines, drag
        for i in range(N):
            F[i, 0] = 0.0; F[i, 1] = 0.0; F[i, 2] = -G * m_node
        for e in range(Ne):
            a = E[e, 0]; b = E[e, 1]
            dx = P[b, 0] - P[a, 0]; dy = P[b, 1] - P[a, 1]; dz = P[b, 2] - P[a, 2]
            l = np.sqrt(dx * dx + dy * dy + dz * dz) + 1e-12
            ux = dx / l; uy = dy / l; uz = dz / l
            if l > L0:
                vrel = (V[b, 0] - V[a, 0]) * ux + (V[b, 1] - V[a, 1]) * uy + (V[b, 2] - V[a, 2]) * uz
                T = k_line * (l - L0) + c_line * vrel
                if T > 0.0:
                    F[a, 0] += T * ux; F[a, 1] += T * uy; F[a, 2] += T * uz
                    F[b, 0] -= T * ux; F[b, 1] -= T * uy; F[b, 2] -= T * uz
            mx = 0.5 * (P[a, 0] + P[b, 0]); my = 0.5 * (P[a, 1] + P[b, 1]); mz = 0.5 * (P[a, 2] + P[b, 2])
            az = _air_z(mx, my, mz, wz, Ix, Iy, Iz, tr, tdw, ir, idw)
            vmx = 0.5 * (V[a, 0] + V[b, 0]) - wx
            vmy = 0.5 * (V[a, 1] + V[b, 1]) - wy
            vmz = 0.5 * (V[a, 2] + V[b, 2]) - az
            vpar = vmx * ux + vmy * uy + vmz * uz
            vnx = vmx - vpar * ux; vny = vmy - vpar * uy; vnz = vmz - vpar * uz
            vnn = np.sqrt(vnx * vnx + vny * vny + vnz * vnz)
            seg = l if l < L0 else L0
            cf = -0.5 * RHO * 1.2 * line_d * seg * vnn * 0.5
            F[a, 0] += cf * vnx; F[a, 1] += cf * vny; F[a, 2] += cf * vnz
            F[b, 0] += cf * vnx; F[b, 1] += cf * vny; F[b, 2] += cf * vnz
        # ---- target contact
        ncont = 0
        for i in range(N):
            r = np.sqrt(P[i, 0] ** 2 + P[i, 1] ** 2 + P[i, 2] ** 2)
            if r < tr + 0.04:
                ncont += 1
            if r < tr:
                nx = P[i, 0] / r; ny = P[i, 1] / r; nz = P[i, 2] / r
                vr = V[i, 0] * nx + V[i, 1] * ny + V[i, 2] * nz
                fn = k_c * (tr - r) - c_contact * (vr if vr < 0.0 else 0.0)
                F[i, 0] += fn * nx; F[i, 1] += fn * ny; F[i, 2] += fn * nz
        if ncont > contacts_max:
            contacts_max = ncont
        if ncont > 0 and t_contact < 0.0:
            t_contact = t
        # ---- weights
        for k in range(4):
            FW[k, 0] = 0.0; FW[k, 1] = 0.0; FW[k, 2] = -G * mw[k]
            c = corners[k]
            dx = P[c, 0] - W[k, 0]; dy = P[c, 1] - W[k, 1]; dz = P[c, 2] - W[k, 2]
            ll = np.sqrt(dx * dx + dy * dy + dz * dz) + 1e-12
            ux = dx / ll; uy = dy / ll; uz = dz / ll
            if ll > link_len:
                vr = (V[c, 0] - WV[k, 0]) * ux + (V[c, 1] - WV[k, 1]) * uy + (V[c, 2] - WV[k, 2]) * uz
                Tl = k_link * (ll - link_len) + c_link * vr
                if Tl > 0.0:
                    FW[k, 0] += Tl * ux; FW[k, 1] += Tl * uy; FW[k, 2] += Tl * uz
                    F[c, 0] -= Tl * ux; F[c, 1] -= Tl * uy; F[c, 2] -= Tl * uz
            az = _air_z(W[k, 0], W[k, 1], W[k, 2], wz, Ix, Iy, Iz, tr, tdw, ir, idw)
            vax = WV[k, 0] - wx; vay = WV[k, 1] - wy; vaz = WV[k, 2] - az
            sp = np.sqrt(vax * vax + vay * vay + vaz * vaz)
            cf = -0.5 * RHO * Cdw * Aw * sp
            FW[k, 0] += cf * vax; FW[k, 1] += cf * vay; FW[k, 2] += cf * vaz
            rr = np.sqrt(W[k, 0] ** 2 + W[k, 1] ** 2 + W[k, 2] ** 2)
            if rr < tr:
                fn = k_c * 5.0 * (tr - rr) / rr
                FW[k, 0] += fn * W[k, 0]; FW[k, 1] += fn * W[k, 1]; FW[k, 2] += fn * W[k, 2]
        # ---- front tethers (drape)
        if tethered:
            for q in range(2):
                k = 2 + q
                Ax = Ix + arm_f[q, 0]; Ay = Iy + arm_f[q, 1]; Az = Iz + arm_f[q, 2]
                dx = Ax - W[k, 0]; dy = Ay - W[k, 1]; dz = Az - W[k, 2]
                ll = np.sqrt(dx * dx + dy * dy + dz * dz) + 1e-12
                ux = dx / ll; uy = dy / ll; uz = dz / ll
                if ll > tether:
                    vr = (VI[0] - WV[k, 0]) * ux + (VI[1] - WV[k, 1]) * uy + (VI[2] - WV[k, 2]) * uz
                    Tt = k_teth * (ll - tether) + c_teth * vr
                    if Tt > 0.0:
                        FW[k, 0] += Tt * ux; FW[k, 1] += Tt * uy; FW[k, 2] += Tt * uz
                        if Tt > max_teth:
                            max_teth = Tt
            if (ncont >= release_contacts or t >= release_t_max) and t > 0.05:
                tethered = False
                t_release = t
        # ---- integrate (semi-implicit Euler)
        for i in range(N):
            V[i, 0] += F[i, 0] / m_node * dt; V[i, 1] += F[i, 1] / m_node * dt; V[i, 2] += F[i, 2] / m_node * dt
            P[i, 0] += V[i, 0] * dt; P[i, 1] += V[i, 1] * dt; P[i, 2] += V[i, 2] * dt
        for k in range(4):
            WV[k, 0] += FW[k, 0] / mw[k] * dt; WV[k, 1] += FW[k, 1] / mw[k] * dt; WV[k, 2] += FW[k, 2] / mw[k] * dt
            W[k, 0] += WV[k, 0] * dt; W[k, 1] += WV[k, 1] * dt; W[k, 2] += WV[k, 2] * dt
        # ---- clearance between own props and net / weights
        if t > clear_t0:
            cm = 1e9
            for i in range(N + 4):
                if i < N:
                    x = P[i, 0]; y = P[i, 1]; z = P[i, 2]
                else:
                    k = i - N
                    if tethered and k >= 2:
                        continue
                    x = W[k, 0]; y = W[k, 1]; z = W[k, 2]
                dzz = abs(z - Iz)
                dr = np.sqrt((x - Ix) ** 2 + (y - Iy) ** 2)
                if dzz < 0.12:
                    cl = dr - ir
                else:
                    hx = dr - ir
                    if hx < 0.0:
                        hx = 0.0
                    cl = np.sqrt(hx * hx + (dzz - 0.12) ** 2)
                if cl < cm:
                    cm = cl
            if cm < min_clear:
                min_clear = cm
                t_minclear = t
        # ---- record
        if rec_every > 0 and it % rec_every == 0 and nrec < rec.shape[0]:
            sx = 0.0; sy = 0.0; sz = 0.0
            for i in range(N):
                sx += P[i, 0]; sy += P[i, 1]; sz += P[i, 2]
            rec[nrec, 0] = t
            rec[nrec, 1] = sx / N; rec[nrec, 2] = sy / N; rec[nrec, 3] = sz / N
            for k in range(4):
                c = corners[k]
                rec[nrec, 4 + 3 * k] = P[c, 0]; rec[nrec, 5 + 3 * k] = P[c, 1]; rec[nrec, 6 + 3 * k] = P[c, 2]
            rec[nrec, 16] = ncont
            nrec += 1
        if ncont >= catch_nodes:
            # spread of the contact patch over the target: mean resultant length of contact directions
            # (1 = all touching at one spot, i.e. a bundle; 0.5 = net spread over a whole hemisphere)
            sx = 0.0; sy = 0.0; sz = 0.0; cnt = 0
            for i in range(N):
                r = np.sqrt(P[i, 0] ** 2 + P[i, 1] ** 2 + P[i, 2] ** 2)
                if r < tr + 0.04:
                    sx += P[i, 0] / r; sy += P[i, 1] / r; sz += P[i, 2] / r; cnt += 1
            Rr = np.sqrt(sx * sx + sy * sy + sz * sz) / cnt
            if Rr < R_min:
                R_min = Rr
            if ncont > cmax_post:
                cmax_post = ncont
            if t_catch < 0.0:
                t_catch = t
                c0 = corners[0]; c1 = corners[1]; c2 = corners[2]; c3 = corners[3]
                d1x = P[c3, 0] - P[c0, 0]; d1y = P[c3, 1] - P[c0, 1]; d1z = P[c3, 2] - P[c0, 2]
                d2x = P[c1, 0] - P[c2, 0]; d2y = P[c1, 1] - P[c2, 1]; d2z = P[c1, 2] - P[c2, 2]
                cx = d1y * d2z - d1z * d2y; cy = d1z * d2x - d1x * d2z; cz = d1x * d2y - d1y * d2x
                open_catch = 0.5 * np.sqrt(cx * cx + cy * cy + cz * cz) / (side * side)
                for i in range(N):
                    vrel_catch[0] += V[i, 0] / N; vrel_catch[1] += V[i, 1] / N; vrel_catch[2] += V[i, 2] / N
        if stop_on_catch and t_catch >= 0.0 and t >= t_catch + post_catch:
            break
    return t_contact, t_catch, contacts_max, min_clear, t_minclear, max_teth, t_release, nrec, R_min, open_catch, vrel_catch[0], vrel_catch[1], vrel_catch[2], cmax_post


@dataclass
class Shot:
    mode: str = "launch"            # 'launch' or 'drape'
    side: float = 1.2
    n: int = 11
    m_w: float = 0.008
    line_d: float = 0.4e-3
    # launch
    v0: float = 9.3
    theta: float = 22.0
    axis: tuple = (0.0, 0.0, -1.0)  # launch direction
    muzzle: float = 0.15            # muzzle distance from interceptor centre along the axis
    drop: float = 0.0               # muzzle this far below the prop plane (forward-firing mount)
    r0: float = 0.03
    pack_r: float = 0.02
    pack_len: float = 0.03
    tether_w: float = 0.10          # weight-to-corner tether (launch)
    t_escape: float = 0.10          # interceptor starts climbing away this long after firing
    v_escape: float = 2.0
    # drape
    tether: float = 3.0
    v_cmd: tuple = (3.0, 0.0, -0.0001)
    h_min: float = 2.99
    climb_after: float = 0.6
    v_climb: float = 1.0
    back_kick: float = 4.0
    pouch_drop: float = 0.5
    release_contacts: int = 12
    release_t_max: float = 1.6
    # common
    start: tuple = (0.0, 0.0, 3.5)  # interceptor position relative to the target at t = 0
    wind: tuple = (0.0, 0.0, 0.0)
    tau: float = 0.12
    target_r: float = 0.25
    target_dw: float = 7.0
    icpt_r: float = 0.30
    icpt_dw: float = 8.0
    k_c: float = 400.0
    t_end: float = 0.9
    dt: float = 2e-5
    seed: int = 1
    catch_nodes: int = 12
    stop_on_catch: bool = True
    post_catch: float = 0.10          # keep simulating this long after the first catch (spread metrics)


def _grid(n):
    E = []
    for i in range(n):
        for j in range(n):
            a = i * n + j
            if j + 1 < n: E.append((a, a + 1))
            if i + 1 < n: E.append((a, a + n))
    return np.array(E, dtype=np.int64)


def run(s: Shot, rec_every=0, n_rec=0):
    rng = np.random.default_rng(s.seed)
    n = s.n; N = n * n; L0 = s.side / (n - 1)
    m_net = 0.009 * s.side ** 2 + 0.002
    m_node = m_net / N
    E = _grid(n)
    k_line = EA / L0
    c_line = 2 * ZETA * np.sqrt(k_line * m_node)
    I = np.array(s.start, float)
    corners = np.array([0, n - 1, N - n, N - 1], dtype=np.int64)
    arm_f = np.array([[0.25, -0.25, -0.05], [0.25, 0.25, -0.05]])
    arm_b = np.array([[-0.25, -0.25, -0.05], [-0.25, 0.25, -0.05]])
    mw = np.full(4, s.m_w)
    if s.mode == "launch":
        a = np.array(s.axis, float); a /= np.linalg.norm(a)
        ref = np.array([1.0, 0, 0]) if abs(a[2]) > 0.9 else np.array([0, 1.0, 0])
        e1 = ref - a * np.dot(ref, a); e1 /= np.linalg.norm(e1)
        e2 = np.cross(a, e1)
        M = I + s.muzzle * a + np.array([0.0, 0.0, -s.drop])
        rr = s.pack_r * np.sqrt(rng.random(N)); ph = 2 * np.pi * rng.random(N)
        P = M + np.outer(s.pack_len * rng.random(N), a) + np.outer(rr * np.cos(ph), e1) + np.outer(rr * np.sin(ph), e2)
        V = np.zeros((N, 3))
        th = np.radians(s.theta)
        W = np.zeros((4, 3)); WV = np.zeros((4, 3))
        for k, ang in enumerate([225, 315, 135, 45]):
            al = np.radians(ang)
            radial = np.cos(al) * e1 + np.sin(al) * e2
            W[k] = M + s.r0 * radial
            WV[k] = s.v0 * (np.cos(th) * a + np.sin(th) * radial)
        link_len = s.tether_w
        k_link = EA / link_len
        c_link = 2 * ZETA * np.sqrt(k_link * (m_node * s.m_w / (m_node + s.m_w)))
        Aw = 0.012 * 0.018
        tethered = False
        kin = np.array([1.0, 0, 0, 0, 0, 0, s.v_escape, s.tau, s.t_escape])
        k_teth = 1.0; c_teth = 0.0; clear_t0 = 0.15
    else:
        P = I + np.column_stack([rng.uniform(-0.08, 0.08, N), rng.uniform(-0.08, 0.08, N),
                                 rng.uniform(-0.18, -0.08, N) - s.pouch_drop])
        V = np.zeros((N, 3))
        W = np.vstack([I + arm_b + np.array([0, 0, -0.03 - s.pouch_drop]), I + arm_f + np.array([0, 0, -0.03 - s.pouch_drop])])
        WV = np.zeros((4, 3)); WV[:2, 2] = -s.back_kick
        link_len = 0.03
        k_link = 3000.0
        c_link = 2 * 0.6 * np.sqrt(k_link * m_node)
        Aw = 0.015 * 0.02
        tethered = True
        kin = np.array([0.0, s.v_cmd[0], s.v_cmd[1], s.v_cmd[2], s.h_min, s.climb_after, s.v_climb, s.tau, 0.0])
        k_teth = EA / s.tether
        c_teth = 2 * 0.6 * np.sqrt(k_teth * s.m_w)
        clear_t0 = 0.15
    rec = np.zeros((max(n_rec, 1), 17))
    out = _core(P, V, m_node, E, L0, k_line, c_line, s.line_d,
                W, WV, mw, corners, link_len, k_link, c_link, 0.9, Aw,
                tethered, s.tether, k_teth, c_teth, arm_f,
                I, kin, np.array(s.wind, float), s.target_r, s.target_dw, s.icpt_r, s.icpt_dw, s.k_c,
                s.dt, s.t_end, s.release_contacts, s.release_t_max, s.catch_nodes, clear_t0, s.stop_on_catch,
                rec_every, rec, s.post_catch, s.side)
    (t_contact, t_catch, cmax, min_clear, t_minclear, max_teth, t_release, nrec,
     R_min, open_catch, vcx, vcy, vcz, cmax_post) = out
    res = dict(caught=t_catch >= 0, t_catch=t_catch if t_catch >= 0 else None,
               t_contact=t_contact if t_contact >= 0 else None, contacts_max=int(cmax),
               min_clearance=min_clear, t_minclear=t_minclear, max_tether_N=max_teth,
               t_release=t_release if t_release >= 0 else None,
               R_min=R_min if t_catch >= 0 else None, open_at_catch=open_catch if t_catch >= 0 else None,
               v_at_catch=(vcx, vcy, vcz) if t_catch >= 0 else None, contacts_post=int(cmax_post))
    if rec_every > 0:
        return res, rec[:nrec]
    return res
