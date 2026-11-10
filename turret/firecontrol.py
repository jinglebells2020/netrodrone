"""Fire control for the net turret: track a drone, decide where to point and when to fire, and score a shot.

Pure numpy, no hardware. It is used by the engagement simulation (analysis/turret/engage.py) and is meant to run on
the turret itself, first in 'virtual fire' mode (track real drones, log the shots it would have taken and score them).

Frame: turret at the origin, x/y horizontal, z up, metres and seconds.
"""
import json
import math

import numpy as np


class NetTable:
    """How the net flies (analysis/turret/out/net_<class>.json): where its centre is against time, and how far off
    the aim line a drone can be at each distance and still be caught."""

    def __init__(self, path):
        d = json.load(open(path))
        self.name = d["name"]
        self.t = np.array(d["t"])
        self.x = np.array(d["x"])
        self.bx = np.array([b[0] for b in d["band"]])
        self.bw = np.array([b[1] for b in d["band"]])
        ok = self.bw > 0
        self.x_min = float(self.bx[ok][0])
        self.x_max = float(self.bx[ok][-1])

    def time_to(self, x):
        return float(np.interp(x, self.x, self.t))

    def dist_at(self, tau):
        return float(np.interp(tau, self.t, self.x))

    def half_width(self, x):
        if x < self.x_min - 0.25 or x > self.x_max + 0.25:
            return 0.0
        return float(np.interp(x, self.bx, self.bw))


class Tracker:
    """Extended Kalman filter, constant-velocity target, measurements = (azimuth, elevation, range)."""

    def __init__(self, accel_noise=4.0):
        self.q = accel_noise            # how hard the target is assumed to manoeuvre (m/s2, 1 sigma)
        self.x = None                   # [px, py, pz, vx, vy, vz]
        self.P = None
        self.t = None

    def _predict_to(self, t):
        dt = t - self.t
        if dt <= 0:
            return
        F = np.eye(6)
        F[0, 3] = F[1, 4] = F[2, 5] = dt
        q = self.q ** 2
        Q = np.zeros((6, 6))
        for i in range(3):
            Q[i, i] = q * dt ** 4 / 4
            Q[i, i + 3] = Q[i + 3, i] = q * dt ** 3 / 2
            Q[i + 3, i + 3] = q * dt ** 2
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.t = t

    def update(self, t, az, el, rng, sig_ang, sig_rng):
        if self.x is None:
            ce = math.cos(el)
            p = rng * np.array([ce * math.cos(az), ce * math.sin(az), math.sin(el)])
            self.x = np.concatenate([p, np.zeros(3)])
            self.P = np.diag([sig_rng ** 2 + (rng * sig_ang) ** 2] * 3 + [60.0 ** 2] * 3)
            self.t = t
            return
        self._predict_to(t)
        px, py, pz = self.x[:3]
        rho2 = px * px + py * py
        rho = math.sqrt(rho2)
        r = math.sqrt(rho2 + pz * pz)
        H = np.zeros((3, 6))
        H[0, 0], H[0, 1] = -py / rho2, px / rho2
        H[1, 0], H[1, 1], H[1, 2] = -px * pz / (r * r * rho), -py * pz / (r * r * rho), rho / (r * r)
        H[2, 0], H[2, 1], H[2, 2] = px / r, py / r, pz / r
        y = np.array([az - math.atan2(py, px), el - math.atan2(pz, rho), rng - r])
        y[0] = (y[0] + math.pi) % (2 * math.pi) - math.pi
        Rm = np.diag([sig_ang ** 2, sig_ang ** 2, sig_rng ** 2])
        S = H @ self.P @ H.T + Rm
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

    def state_at(self, t):
        dt = t - self.t
        return self.x[:3] + self.x[3:] * dt, self.x[3:].copy()


class PanTilt:
    """Two-axis head with a rate limit and an acceleration limit, driven toward a commanded direction."""

    def __init__(self, max_rate, max_accel, az=0.0, el=math.radians(20.0), gain=12.0):
        self.w, self.a, self.k = max_rate, max_accel, gain
        self.az, self.el, self.raz, self.rel = az, el, 0.0, 0.0

    def step(self, az_cmd, el_cmd, dt):
        e_az = (az_cmd - self.az + math.pi) % (2 * math.pi) - math.pi
        e_el = el_cmd - self.el
        for err, name in ((e_az, "raz"), (e_el, "rel")):
            # fastest rate that can still stop on the target: sqrt(2 a |err|), also capped by the gain and the rate limit
            want = math.copysign(min(self.w, self.k * abs(err), math.sqrt(2 * self.a * abs(err))), err)
            cur = getattr(self, name)
            setattr(self, name, cur + max(-self.a * dt, min(self.a * dt, want - cur)))
        self.az += self.raz * dt
        self.el += self.rel * dt

    def direction(self):
        ce = math.cos(self.el)
        return np.array([ce * math.cos(self.az), ce * math.sin(self.az), math.sin(self.el)])


def _angle(a, b):
    return math.acos(max(-1.0, min(1.0, float(np.dot(a, b)))))


class FireControl:
    """Pick the meeting point on the drone's predicted path and the moment to fire.

    The net is thrown at the point where the drone will be when the net gets there. Far meeting points are preferred
    (more margin); if the head cannot get there in time a nearer one is used."""

    def __init__(self, net, max_rate, t_act=0.04, prefer=0.7, max_oblique=math.radians(50.0), aim_tol=math.radians(2.0)):
        self.net, self.w, self.t_act = net, max_rate, t_act
        self.x_pref = net.x_min + prefer * (net.x_max - net.x_min)      # prefer: 0 = meet as near as possible, 1 = as far
        self.cos_obl, self.aim_tol = math.cos(max_oblique), aim_tol

    def plan(self, p, v, now, head_dir):
        """-> (aim direction, time to fire, meeting distance) or None when the path never comes within reach"""
        vv = float(v @ v)
        pv = float(p @ v)
        pp = float(p @ p)
        if vv < 1.0:
            return None
        best = None
        # preferred meeting distance first, then nearer ones (if the head needs more time), then farther ones (if the
        # path passes too far out for a near meeting)
        near = np.arange(self.x_pref, self.net.x_min - 1e-9, -0.25)
        far = np.arange(self.x_pref + 0.25, self.net.x_max + 1e-9, 0.25)
        for x in list(near) + list(far):
            disc = pv * pv - vv * (pp - x * x)
            if disc < 0.0:
                continue
            s = (-pv - math.sqrt(disc)) / vv                      # time until the drone is x metres away, inbound
            if s <= 0.0:
                continue
            a = (p + v * s) / x
            if -float(v @ a) / math.sqrt(vv) < self.cos_obl:
                continue
            t_fire = now + s - self.net.time_to(x) - self.t_act
            if best is None or (best[1] < now <= t_fire):
                best = (a, t_fire, float(x))
            if t_fire - now >= _angle(head_dir, a) / self.w:      # the head can get there in time
                return (a, t_fire, float(x))
        return best

    def should_fire(self, plan, now, head_dir, dt):
        return plan is not None and now >= plan[1] - dt / 2 and _angle(head_dir, plan[0]) <= self.aim_tol


def score_shot(net, t_launch, axis, path, speed_scale=1.0, max_incidence=math.radians(50.0), step=0.001):
    """Would this shot have caught the drone?  path(t) -> drone position (3,) at time t.

    The net centre moves out along `axis`; the drone is caught if, when it crosses the plane of the net, it is inside
    the catch half-width for that distance. A net met at an angle looks narrower by cos(incidence).
    -> (caught, meeting distance, lateral miss, incidence angle)"""
    tau = 0.0
    prev = None
    while tau < 1.6:
        r = path(t_launch + tau)
        xi = float(r @ axis)
        xn = net.dist_at(tau * speed_scale)
        if xi <= xn:
            if prev is None:
                return False, xn, float("nan"), float("nan")
            c = float(np.linalg.norm(r - xi * axis))
            vd = (r - prev[0]) / step
            vn = (xn - prev[1]) / step
            rel = vn * axis - vd
            inc = _angle(rel / np.linalg.norm(rel), axis)
            ok = inc <= max_incidence and c <= net.half_width(xn) * math.cos(inc)
            return bool(ok), xn, c, inc
        prev = (r, xn)
        tau += step
    return False, float("nan"), float("nan"), float("nan")
