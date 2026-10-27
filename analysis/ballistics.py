"""
Internal ballistics: V2 AEG spring-piston -> air -> nozzle -> launcher manifold -> 4 net weights.

Model
  CV1 = cylinder air (shrinks as the piston moves forward)
  CV2 = everything downstream of the nozzle: nozzle/plenum dead volume + barrels behind the weights
  - adiabatic ideal gas in both volumes, compressible (choked / unchoked) orifice flow between them
  - leaks: piston-head seal (CV1 -> air) and weight blow-by (CV2 -> air)
  - piston + spring (1/3 of spring mass moving), O-ring friction, stops at the cylinder head
  - weights: breakaway (O-ring static friction) + sliding friction, exit at the end of travel
  Fixed-step RK4, compiled with numba.  SI units.

Spring calibration
  AEG spring ratings (M100, M120, M150 ...) are defined as roughly the muzzle speed in m/s of a
  0.20 g BB in a typical AEG.  No force data is published, so the spring rate k is found by
  simulating a stock AEG (0.20 g BB, 6.03 x 363 mm barrel) and solving for the k that gives the
  rated BB speed.  Two loss assumptions bracket the truth:
    CAL_A ("conservative"): almost loss-free gun -> the weakest spring that can reach the rating
    CAL_B ("expected"):     seal leaks, spring surge, hop-up drag -> ~35-50 % BB efficiency
  The net-launcher predictions are then made with the same calibrated spring.
"""
import numpy as np
from dataclasses import dataclass, replace, asdict
from numba import njit

GAM = 1.4
R = 287.05
CV = R / (GAM - 1)
CP = GAM * CV
PATM = 101325.0
TATM = 293.15
RC = (2 / (GAM + 1)) ** (GAM / (GAM - 1))
CHOKE = (2 / (GAM + 1)) ** ((GAM + 1) / (2 * (GAM - 1)))


@njit(cache=True)
def mdot(cda, pu, Tu, pd):
    if pu <= pd or cda <= 0.0:
        return 0.0
    r = pd / pu
    if r <= RC:
        return cda * pu * np.sqrt(GAM / (R * Tu)) * CHOKE
    return cda * pu * np.sqrt(2 * GAM / ((GAM - 1) * R * Tu) * (r ** (2 / GAM) - r ** ((GAM + 1) / GAM)))


@njit(cache=True)
def deriv(s, P, pist_lock, proj_lock):
    (Ap, Ab, stroke, vd1, vd2, m_eff, k, c0, fpist, cda, M, Fslide, leak_cda) = (
        P[0], P[1], P[2], P[3], P[4], P[5], P[6], P[7], P[8], P[9], P[10], P[11], P[12])
    leak1_cda, sp_loss, hop_len, hop_f = P[13], P[14], P[15], P[16]
    valve_cda, bleed_cda, pvent_cda = P[17], P[18], P[19]
    x, v, m1, T1, m2, T2, y, u = s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7]
    V1 = vd1 + Ap * (stroke - x)
    V2 = vd2 + Ab * y
    p1 = m1 * R * T1 / V1
    p2 = m2 * R * T2 / V2
    if p1 >= p2:
        md = mdot(cda, p1, T1, p2)
        h = CP * T1
    else:
        md = -mdot(cda, p2, T2, p1)
        h = CP * T2
    ml = mdot(leak_cda + bleed_cda, p2, T2, PATM)          # blow-by + bleed, out of CV2
    ml -= mdot(bleed_cda + valve_cda, PATM, TATM, p2)       # bleed + check valve, into CV2 (vacuum only)
    ml1 = mdot(leak1_cda, p1, T1, PATM)
    ml1 -= mdot(pvent_cda, PATM, TATM, p1)                  # vented piston head refills CV1 (vacuum only)
    out = np.zeros(8)
    if pist_lock:
        dx = 0.0
        dv = 0.0
    else:
        Fs = k * (c0 + stroke - x)
        if v > 0:
            Fs *= (1.0 - sp_loss)
        F = Fs - (p1 - PATM) * Ap
        if v > 0:
            F -= fpist
        elif v < 0:
            F += fpist
        dx = v
        dv = F / m_eff
    if proj_lock:
        dy = 0.0
        du = 0.0
    else:
        Fp = (p2 - PATM) * Ab
        fs = Fslide
        if y < hop_len:
            fs += hop_f
        if u >= 0:
            Fp -= fs
        else:
            Fp += fs
        dy = u
        du = Fp / M
    dV1 = -Ap * dx
    dV2 = Ab * dy
    dm1 = -md - ml1
    dU1 = -p1 * dV1 - md * h - ml1 * CP * (T1 if ml1 > 0 else TATM)
    dm2 = md - ml
    dU2 = -p2 * dV2 + md * h - ml * CP * (T2 if ml > 0 else TATM)
    out[0] = dx
    out[1] = dv
    out[2] = dm1
    out[3] = (dU1 - CV * T1 * dm1) / (m1 * CV)
    out[4] = dm2
    out[5] = (dU2 - CV * T2 * dm2) / (m2 * CV)
    out[6] = dy
    out[7] = du
    return out


@njit(cache=True)
def run(P, Fbreak, barrel_len, dt, t_end, rec_every, init):
    Ap, Ab, stroke, vd1, vd2 = P[0], P[1], P[2], P[3], P[4]
    rho0 = PATM / (R * TATM)
    s = np.zeros(8)
    s[2] = rho0 * (vd1 + Ap * stroke)
    s[3] = TATM
    s[4] = rho0 * vd2
    s[5] = TATM
    if init[0] > 0:            # gas state at release from the cocking phase
        s[2], s[3], s[4], s[5] = init[0], init[1], init[2], init[3]
    pist_lock = False
    proj_lock = True
    impact_v = -1.0
    t_exit = -1.0
    v_exit = -1.0
    pk1 = PATM
    pk2 = PATM
    n = int(t_end / dt)
    nrec = n // rec_every + 1
    rec = np.zeros((nrec, 6))
    ri = 0
    t = 0.0
    for i in range(n):
        # breakaway check for projectile
        if proj_lock:
            V2 = vd2 + Ab * s[6]
            p2 = s[4] * R * s[5] / V2
            if (p2 - PATM) * Ab > Fbreak:
                proj_lock = False
        k1 = deriv(s, P, pist_lock, proj_lock)
        k2 = deriv(s + 0.5 * dt * k1, P, pist_lock, proj_lock)
        k3 = deriv(s + 0.5 * dt * k2, P, pist_lock, proj_lock)
        k4 = deriv(s + dt * k3, P, pist_lock, proj_lock)
        s = s + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        t += dt
        # piston limits
        if not pist_lock:
            if s[0] >= stroke:
                impact_v = s[1]
                s[0] = stroke
                s[1] = 0.0
                pist_lock = True
            elif s[0] < 0.0:
                s[0] = 0.0
                if s[1] < 0:
                    s[1] = 0.0
        # projectile can't go backwards past start
        if not proj_lock:
            if s[6] < 0.0:
                s[6] = 0.0
                s[7] = 0.0
                proj_lock = True
        V1 = vd1 + Ap * (stroke - s[0])
        V2 = vd2 + Ab * s[6]
        p1 = s[2] * R * s[3] / V1
        p2 = s[4] * R * s[5] / V2
        if p1 > pk1:
            pk1 = p1
        if p2 > pk2:
            pk2 = p2
        if i % rec_every == 0 and ri < nrec:
            rec[ri, 0] = t
            rec[ri, 1] = s[0]
            rec[ri, 2] = s[1]
            rec[ri, 3] = p1
            rec[ri, 4] = p2
            rec[ri, 5] = s[7]
            ri += 1
        if s[6] >= barrel_len:
            t_exit = t
            v_exit = s[7]
            break
    res = np.array([v_exit, t_exit, impact_v, pk1, pk2, s[6], s[7], s[0], s[1]])
    return res, rec[:ri]


@njit(cache=True)
def cock_phase(P, t_cock, dt):
    """piston pulled from rest (x=stroke) to cocked (x=0) at constant speed; weights seated.
    returns gas state (m1, T1, m2, T2) at release and the minimum pressures reached"""
    Ap, stroke, vd1, vd2 = P[0], P[2], P[3], P[4]
    rho0 = PATM / (R * TATM)
    s = np.zeros(8)
    s[0] = stroke
    s[1] = -stroke / t_cock
    s[2] = rho0 * vd1
    s[3] = TATM
    s[4] = rho0 * vd2
    s[5] = TATM
    n = int(t_cock / dt)
    pmin1 = PATM
    pmin2 = PATM
    for i in range(n):
        k1 = deriv(s, P, True, True)
        k2 = deriv(s + 0.5 * dt * k1, P, True, True)
        k3 = deriv(s + 0.5 * dt * k2, P, True, True)
        k4 = deriv(s + dt * k3, P, True, True)
        # piston motion is prescribed: dV1/dt = -Ap*dx with dx = s[1]
        ds = dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        V1 = vd1 + Ap * (stroke - s[0])
        V1n = vd1 + Ap * (stroke - (s[0] + s[1] * dt))
        p1 = s[2] * R * s[3] / V1
        # adiabatic expansion work for the moving piston (deriv() sees a locked piston)
        dU = -p1 * (V1n - V1)
        s = s + ds
        s[3] += dU / (s[2] * CV)
        s[0] += s[1] * dt
        V1 = vd1 + Ap * (stroke - s[0])
        p1 = s[2] * R * s[3] / V1
        p2 = s[4] * R * s[5] / vd2
        if p1 < pmin1:
            pmin1 = p1
        if p2 < pmin2:
            pmin2 = p2
    return np.array([s[2], s[3], s[4], s[5], pmin1, pmin2])


@dataclass
class Gun:
    # cylinder / piston
    bore: float = 23.0e-3          # cylinder inner diameter
    stroke: float = 52.0e-3        # piston travel
    v_dead1: float = 1.0e-6        # cylinder dead volume at end of stroke (head recess, port)
    m_piston: float = 0.020        # piston assembly mass
    m_spring: float = 0.010        # spring mass (1/3 counted as moving)
    f_piston: float = 3.0          # piston o-ring sliding friction, N
    # spring (linear): F = k*(c0 + stroke - x)
    k: float = 900.0               # N/m
    c0: float = 55e-3              # preload compression with piston forward
    # port between cylinder and downstream volume
    port_d: float = 5.0e-3
    port_cd: float = 0.80
    # downstream
    v_dead2: float = 0.5e-6        # nozzle + manifold volume
    n_proj: int = 1
    proj_d: float = 6.03e-3        # barrel bore per projectile
    m_proj: float = 0.20e-3        # mass per projectile
    f_break: float = 2.0           # breakaway force per projectile (retention / hop-up)
    f_slide: float = 0.3           # sliding friction per projectile
    barrel_len: float = 0.363      # travel to exit
    leak_area: float = 0.3e-6      # total blow-by area from CV2 to atmosphere, m^2
    leak_cd: float = 0.6
    leak1_area: float = 0.0        # piston head seal leak, CV1 -> atmosphere, m^2
    spring_loss: float = 0.0       # fraction of spring force lost while extending (surge / hysteresis)
    hop_len: float = 0.0           # length of extra-friction zone at start of travel (hop-up rubber)
    hop_f: float = 0.0             # extra sliding friction in that zone (total), N
    # refill paths (matter when the launcher is sealed between shots)
    valve_area: float = 0.0        # one-way check valve atmosphere -> plenum, m^2 (opens on vacuum)
    bleed_area: float = 0.0        # small open bleed hole plenum <-> atmosphere, m^2
    pvent_area: float = 0.0        # vented AEG piston head: atmosphere -> cylinder on the cocking stroke, m^2
    cock_time: float = 0.0         # >0: simulate the cocking stroke first (s); 0: start at 1 atm

    @property
    def Ap(self):
        return np.pi / 4 * self.bore ** 2

    @property
    def Ab(self):
        return self.n_proj * np.pi / 4 * self.proj_d ** 2

    def spring_force(self, x):
        return self.k * (self.c0 + self.stroke - x)

    @property
    def spring_energy(self):
        return 0.5 * self.k * ((self.c0 + self.stroke) ** 2 - self.c0 ** 2)

    @property
    def swept(self):
        return self.Ap * self.stroke

    def params(self):
        m_eff = self.m_piston + self.m_spring / 3
        cda = self.port_cd * np.pi / 4 * self.port_d ** 2
        return np.array([self.Ap, self.Ab, self.stroke, self.v_dead1, self.v_dead2, m_eff, self.k, self.c0,
                         self.f_piston, cda, self.n_proj * self.m_proj, self.n_proj * self.f_slide,
                         self.leak_cd * self.leak_area, 0.6 * self.leak1_area, self.spring_loss,
                         self.hop_len, self.hop_f, 0.6 * self.valve_area, 0.6 * self.bleed_area,
                         0.6 * self.pvent_area])


def simulate(g: Gun, dt=2e-7, t_end=0.06, trace=False):
    init = np.zeros(4)
    cock = None
    if g.cock_time > 0:
        cock = cock_phase(g.params(), g.cock_time, 1e-6)
        init = cock[:4]
    res, rec = run(g.params(), g.n_proj * g.f_break, g.barrel_len, dt, t_end, 50 if trace else 10**9, init)
    v_exit, t_exit, impact_v, pk1, pk2, y, u, x, v = res
    out = dict(v_exit=v_exit if v_exit > 0 else None, t_exit=t_exit if t_exit > 0 else None,
               impact_v=impact_v if impact_v >= 0 else None, peak_p1_bar=(pk1 - PATM) / 1e5,
               peak_p2_bar=(pk2 - PATM) / 1e5, y_final=y, u_final=u, E_spring=g.spring_energy,
               F_max=g.spring_force(0.0), F_min=g.spring_force(g.stroke))
    vv = out["v_exit"] if out["v_exit"] else u
    out["E_proj"] = 0.5 * g.n_proj * g.m_proj * vv ** 2
    out["eff"] = out["E_proj"] / g.spring_energy
    if trace:
        out["trace"] = rec
    if cock is not None:
        V1 = g.v_dead1 + g.Ap * g.stroke
        out["p_cyl_at_release_bar_abs"] = cock[0] * R * cock[1] / V1 / 1e5
        out["p_min_cyl_bar_abs"] = cock[4] / 1e5
    return out


def calibrate_k(target_v, base: Gun, lo=100.0, hi=6000.0):
    """spring rate so a 0.20 g BB leaves at target_v m/s (that is what the AEG 'M' number means)"""
    for _ in range(36):
        mid = 0.5 * (lo + hi)
        v = simulate(replace(base, k=mid))["v_exit"] or 0.0
        if v < target_v:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


AEG_BB = Gun()                     # stock AEG used for the spring calibration
CAL_A = AEG_BB
CAL_B = replace(AEG_BB, leak1_area=0.25e-6, spring_loss=0.22, leak_area=0.9e-6, f_break=3.0,
                hop_len=0.012, hop_f=3.0, f_piston=5.0)
CALIBRATIONS = {"A (conservative)": CAL_A, "B (expected)": CAL_B}

# ---- the netrodrone launcher design point (matches cad/build_netrodrone.py) ----
LAUNCHER = dict(
    n_proj=4,              # four corner weights
    proj_d=12.0e-3,        # barrel bore
    barrel_len=0.042,      # weight travel inside the barrel
    port_d=4.0e-3,         # stock AEG nozzle bore (3.5-5 mm changes speed only +-6 %)
    port_cd=0.80,
    v_dead2=3.0e-6,        # nozzle + seal sleeve + plenum + feed holes
    f_break=4.0,           # O-ring breakaway force per weight
    f_slide=1.2,           # O-ring sliding friction per weight
    leak_area=4 * 0.2e-6,  # blow-by past 4 greased O-rings
    hop_len=0.0, hop_f=0.0,
    valve_area=np.pi / 4 * 4.0e-3 ** 2,   # refill check valve (6 mm BB on a 4 mm seat)
    bleed_area=np.pi / 4 * 1.0e-3 ** 2,   # 1 mm bleed hole (lets air out while loading weights)
    cock_time=0.025,       # sector gear pulls the piston back in ~25 ms (11.1 V, stock gears)
)


def spring_k(rating, cal=CAL_A):
    """spring rate (N/m) for an 'M' rating under calibration cal"""
    return calibrate_k(rating, cal)


def net_launcher(cal, k, m_weight, **overrides):
    """Gun object for the net launcher with spring k (N/m) and weight mass m_weight (kg)"""
    kw = dict(LAUNCHER)
    kw.update(overrides)
    return replace(cal, k=k, m_proj=m_weight, **kw)


if __name__ == "__main__":
    for cname, cal in CALIBRATIONS.items():
        for m in (100, 120, 150):
            k = spring_k(m, cal)
            g = replace(cal, k=k)
            bb = simulate(g)
            r = simulate(net_launcher(cal, k, 0.008), t_end=0.12)
            print(f"cal {cname:17s} M{m}: k={k:4.0f} N/m  F_max={g.spring_force(0):5.1f} N  E_spring={g.spring_energy:.2f} J  "
                  f"| net weights 4x8 g: {r['v_exit']:.1f} m/s, {r['E_proj']:.2f} J, p_max={r['peak_p2_bar']:.1f} bar")
