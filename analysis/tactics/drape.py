"""
'Drape' capture: interceptor above-behind a target releases the 2 back corner weights and dives forward/down
holding the 2 front corners on tethers, dragging the net over the target; front corners released on capture.

Frame: target centre at origin (hovering, or wind = relative air flow if it moves); x forward, z up.
Model: n x n mass-spring net (tension-only, damped), line drag with RELATIVE air velocity, gravity,
target = sphere (r 0.25 m) with penalty contact + its rotor downwash column, interceptor = prescribed path
with its own downwash column; front weights hang on tension-only tethers from the interceptor's front arms.
"""
import numpy as np
from dataclasses import dataclass, field


@dataclass
class Drape:
    side: float = 1.5
    n: int = 13
    m_w_back: float = 0.020
    m_w_front: float = 0.020
    tether: float = 1.0
    start: tuple = (-1.0, 0.0, 3.0)      # interceptor start (m) relative to target
    v_cmd: tuple = (2.5, 0.0, -3.0)       # commanded dive velocity
    h_min: float = 0.8                    # interceptor stops descending at this height above target
    climb_after: float = 0.25             # s after reaching h_min, start climbing
    v_climb: float = 3.0
    tau: float = 0.12                     # interceptor velocity response time constant
    back_kick: float = 0.0                # downward speed given to back weights at release (spring push)
    wind: tuple = (0.0, 0.0, 0.0)         # air velocity in target frame (target moving at -wind)
    release_contacts: int = 12            # release front tethers when this many net nodes touch the target
    release_t_max: float = 1.6
    t_end: float = 2.2
    dt: float = 2e-5
    target_r: float = 0.25
    target_dw: float = 7.0                # target downwash speed
    icpt_dw: float = 8.0                  # interceptor downwash speed
    icpt_r: float = 0.30                  # interceptor prop radius (keep-out)
    seed: int = 3
    pouch_drop: float = 0.0                # net pouch hangs this far below the props (boom/tray)


def run(s: Drape, snap_every=0.1):
    rng = np.random.default_rng(s.seed)
    n = s.n; N = n * n; L0 = s.side / (n - 1)
    m_net = 0.009 * s.side ** 2 + 0.002
    m = np.full(N, m_net / N)
    E = []
    for i in range(n):
        for j in range(n):
            a = i * n + j
            if j + 1 < n: E.append((a, a + 1))
            if i + 1 < n: E.append((a, a + n))
    E = np.array(E); rest = np.full(len(E), L0)
    # node index: i along x (0 = back edge), j along y
    back = np.array([0, n - 1])                 # i=0 row corners (back)
    front = np.array([N - n, N - 1])            # i=n-1 row corners (front)
    I = np.array(s.start, float); VI = np.zeros(3)
    arm_f = np.array([[0.25, -0.25, -0.05], [0.25, 0.25, -0.05]])
    arm_b = np.array([[-0.25, -0.25, -0.05], [-0.25, 0.25, -0.05]])
    # stowed net: bunched in a pouch under the interceptor
    P = I + np.column_stack([rng.uniform(-0.08, 0.08, N), rng.uniform(-0.08, 0.08, N), rng.uniform(-0.18, -0.08, N) - s.pouch_drop])
    V = np.zeros((N, 3))
    WB = I + arm_b + np.array([0, 0, -0.03 - s.pouch_drop]); WBV = np.zeros((2, 3)); WBV[:, 2] = -s.back_kick
    WF = I + arm_f + np.array([0, 0, -0.03 - s.pouch_drop]); WFV = np.zeros((2, 3))
    g = np.array([0, 0, -9.81]); rho = 1.2
    EA = 1500.0; k_line = EA / L0
    c_line = 2 * 0.6 * np.sqrt(k_line * m_net / N)
    k_link = 3000.0; c_link = 2 * 0.6 * np.sqrt(k_link * m_net / N)   # weight <-> corner node (short rope)
    k_teth = EA / s.tether; c_teth = 2 * 0.6 * np.sqrt(k_teth * s.m_w_front)
    k_c = 400.0
    wind = np.array(s.wind, float)
    phase = "dive"; t_hmin = None; released = False; t_release = None; t_contact = None
    min_clear = 1e9; max_teth = 0.0; contacts_max = 0; t_minclear = None; ph_minclear = None
    snaps = []; steps = int(s.t_end / s.dt); next_snap = 0.0
    for it in range(steps):
        t = it * s.dt
        # ---- interceptor commanded velocity
        if phase == "dive" and I[2] <= s.h_min:
            phase = "level"; t_hmin = t
        if phase == "level" and t - t_hmin >= s.climb_after:
            phase = "climb"
        if phase == "dive":
            vc = np.array(s.v_cmd, float)
        elif phase == "level":
            vc = np.array([s.v_cmd[0], 0, 0.0])
        else:
            vc = np.array([s.v_cmd[0] * 0.5, 0, s.v_climb])
        VI += (vc - VI) * (s.dt / s.tau)
        I = I + VI * s.dt

        def air(X):
            a = np.tile(wind, (len(X), 1))
            rt = np.hypot(X[:, 0], X[:, 1])
            m1 = (rt < s.target_r + 0.05) & (X[:, 2] > -2.0) & (X[:, 2] < 1.5)
            a[m1, 2] -= s.target_dw
            ri = np.hypot(X[:, 0] - I[0], X[:, 1] - I[1])
            m2 = (ri < s.icpt_r) & (X[:, 2] < I[2] + 0.2) & (X[:, 2] > I[2] - 2.5)
            a[m2, 2] -= s.icpt_dw
            return a

        F = m[:, None] * g
        d = P[E[:, 1]] - P[E[:, 0]]; l = np.linalg.norm(d, axis=1) + 1e-12; u = d / l[:, None]
        vrel = np.einsum("ij,ij->i", V[E[:, 1]] - V[E[:, 0]], u)
        T = np.maximum(np.where(l > rest, k_line * (l - rest) + c_line * vrel, 0.0), 0.0)
        f = T[:, None] * u
        np.add.at(F, E[:, 0], f); np.add.at(F, E[:, 1], -f)
        mid = 0.5 * (P[E[:, 0]] + P[E[:, 1]])
        vm = 0.5 * (V[E[:, 0]] + V[E[:, 1]]) - air(mid)
        vn = vm - np.einsum("ij,ij->i", vm, u)[:, None] * u
        fd = -0.5 * rho * 1.2 * 0.4e-3 * np.minimum(l, rest)[:, None] * np.linalg.norm(vn, axis=1)[:, None] * vn
        np.add.at(F, E[:, 0], 0.5 * fd); np.add.at(F, E[:, 1], 0.5 * fd)
        # target contact (sphere)
        r = np.linalg.norm(P, axis=1); inside = r < s.target_r
        if inside.any():
            nr = P[inside] / r[inside][:, None]
            pen = s.target_r - r[inside]
            vr = np.einsum("ij,ij->i", V[inside], nr)
            F[inside] += (k_c * pen - 2 * 0.5 * np.sqrt(k_c * m_net / N) * np.minimum(vr, 0))[:, None] * nr
        ncont = int((r < s.target_r + 0.04).sum())
        contacts_max = max(contacts_max, ncont)
        if ncont > 0 and t_contact is None:
            t_contact = t
        # corner weights <-> corner nodes (short links)
        FWB = s.m_w_back * g + np.zeros((2, 3)); FWF = s.m_w_front * g + np.zeros((2, 3))
        for W, WV, idx, FW, mw in ((WB, WBV, back, FWB, s.m_w_back), (WF, WFV, front, FWF, s.m_w_front)):
            dd = P[idx] - W; ll = np.linalg.norm(dd, axis=1) + 1e-12; uu = dd / ll[:, None]
            vr = np.einsum("ij,ij->i", V[idx] - WV, uu)
            Tl = np.maximum(np.where(ll > 0.03, k_link * (ll - 0.03) + c_link * vr, 0.0), 0.0)
            FW += Tl[:, None] * uu; F[idx] -= Tl[:, None] * uu
            vrel_air = WV - air(W)
            FW += -0.5 * rho * 0.9 * (0.015 * 0.02) * np.linalg.norm(vrel_air, axis=1)[:, None] * vrel_air
            rr = np.linalg.norm(W, axis=1); ins = rr < s.target_r
            if ins.any():
                FW[ins] += (k_c * 5 * (s.target_r - rr[ins]))[:, None] * (W[ins] / rr[ins][:, None])
        # front tethers to interceptor arms
        if not released:
            A = I + arm_f
            dd = A - WF; ll = np.linalg.norm(dd, axis=1) + 1e-12; uu = dd / ll[:, None]
            vr = np.einsum("ij,ij->i", VI - WFV, uu)
            Tt = np.maximum(np.where(ll > s.tether, k_teth * (ll - s.tether) + c_teth * vr, 0.0), 0.0)
            FWF += Tt[:, None] * uu
            max_teth = max(max_teth, float(Tt.max()))
            if (ncont >= s.release_contacts or t >= s.release_t_max) and t > 0.05:
                released = True; t_release = t
        # integrate
        V += F / m[:, None] * s.dt; P += V * s.dt
        WBV += FWB / s.m_w_back * s.dt; WB += WBV * s.dt
        WFV += FWF / s.m_w_front * s.dt; WF += WFV * s.dt
        # clearance between interceptor props and any net node / weight (after the net has left the pouch)
        if t > 0.15:
            allp = np.vstack([P, WB, WF if released else WB])
            dz = np.abs(allp[:, 2] - I[2]); dr = np.hypot(allp[:, 0] - I[0], allp[:, 1] - I[1])
            clear = np.where(dz < 0.12, dr - s.icpt_r, np.hypot(np.maximum(dr - s.icpt_r, 0), dz - 0.12))
            cm = float(clear.min())
            if cm < min_clear:
                min_clear = cm; t_minclear = t; ph_minclear = phase
        if t >= next_snap - 1e-9:
            snaps.append((t, P.copy(), WB.copy(), WF.copy(), I.copy(), released))
            next_snap += snap_every
    # wrapped? net nodes below target centre both in front and behind, plus contact
    final_r = np.linalg.norm(P, axis=1)
    on_target = int((final_r < s.target_r + 0.06).sum())
    band = np.abs(P[:, 1]) < 0.5
    front_low = ((P[:, 0] > 0.0) & (P[:, 2] < 0.0) & band & (np.abs(P[:, 0]) < 0.8)).any()
    back_low = ((P[:, 0] < 0.0) & (P[:, 2] < 0.0) & band & (np.abs(P[:, 0]) < 0.8)).any()
    over_top = ((final_r < s.target_r + 0.06) & (P[:, 2] > 0.1)).sum()
    return dict(t_contact=t_contact, t_release=t_release, contacts_max=contacts_max, on_target_end=on_target,
                over_top_end=int(over_top), wrapped=bool(front_low and back_low and on_target >= 6),
                min_clearance=min_clear, t_minclear=t_minclear, phase_minclear=ph_minclear, max_tether_N=max_teth, h_min_reached=float(min(sn[4][2] for sn in snaps))), snaps
