"""Launcher classes studied for the roof turret, and how each is set up in the capture model (analysis/tactics/capture.py).

A class = net + four weights + muzzle speed + spread angle. The numbers are design targets, not built hardware:
  light   2 m net, 4 x 20 g at 30 m/s  (36 J)   safe enough to develop the turret with
  medium  3 m net, 4 x 50 g at 40 m/s  (160 J)
  heavy   5 m net, 4 x 100 g at 60 m/s (720 J)  the energy class of commercial net pods
  medium-wide / heavy-wide: the same shots with the barrels spread wider (26 / 22 deg instead of 14 / 10)
Mesh points are 0.15 m apart for every net; line 0.4-0.5 mm braided UHMWPE.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tactics"))
from capture import Shot  # noqa: E402

CLASSES = {
    "light":  dict(side=2.0, n=14, line_d=0.4e-3, m_net=0.011, m_w=0.020, w_area=0.0004, v0=30.0, theta=16.0),
    "medium": dict(side=3.0, n=21, line_d=0.5e-3, m_net=0.040, m_w=0.050, w_area=0.0008, v0=40.0, theta=14.0),
    "heavy":  dict(side=5.0, n=34, line_d=0.5e-3, m_net=0.085, m_w=0.100, w_area=0.0012, v0=60.0, theta=10.0),
    # same energy, barrels spread wider: the net opens sooner and is wider close in (for drones that do not fly straight)
    "medium-wide": dict(side=3.0, n=21, line_d=0.5e-3, m_net=0.040, m_w=0.050, w_area=0.0008, v0=40.0, theta=26.0),
    "heavy-wide":  dict(side=5.0, n=34, line_d=0.5e-3, m_net=0.085, m_w=0.100, w_area=0.0012, v0=60.0, theta=22.0),
}


def energy(name):
    c = CLASSES[name]
    return 0.5 * 4 * c["m_w"] * c["v0"] ** 2


def time_step(c, fast=False):
    """explicit integrator: lighter mesh points need a smaller step; halve it again for high closing speeds"""
    dt = min(2e-5, 0.6 * 2e-5 * ((c["m_net"] / c["n"] ** 2) / (0.01496 / 121)) ** 0.5)
    return dt * (0.5 if fast else 1.0)


def turret_shot(name, axis, start, drone_velocity=(0.0, 0.0, 0.0), hover=False, t_end=1.6, seed=1, **over):
    """Shot fired by a fixed turret, expressed in the drone's frame (the frame capture.py works in).

    axis            unit vector the launcher points along (ground frame)
    start           turret position relative to the drone at the moment of firing
    drone_velocity  drone velocity over the ground; the turret and the still air then move at -velocity in the drone's frame
    """
    c = dict(CLASSES[name])
    c.update(over)
    u = -np.array(drone_velocity, float)
    fast = float(np.linalg.norm(u)) >= 15.0
    return Shot(mode="launch", axis=tuple(axis), start=tuple(start), muzzle=0.0, r0=0.05, pack_r=0.03, pack_len=0.05,
                icpt_dw=0.0, target_dw=(7.0 if hover else 0.0), v_escape=0.0, t_end=t_end, seed=seed,
                u0=tuple(u), wind=tuple(u), dt=time_step(c, fast), **c)
