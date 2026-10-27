"""
netrodrone - V2-gearbox powered net launcher, parametric CAD build (build123d / OpenCascade).

Run:   python cad/build_netrodrone.py            (writes cad/out/step/*.step and cad/out/stl/*.stl)

Input: cad/source/gearbox_v2_original.step  -- the original "GearBox V2 lower" model.
       It holds 3 bodies.  The newest matched pair is used:
         BLOCK (1)       -> gearbox half A (thick half, 18 mm, cylinder axis 3 mm inside it)
         BLOCK (1) (1)   -> gearbox half B (thin half, 12 mm + locating islands)
       BLOCK (the first, symmetric version) is not used.

All component fits from the original (cylinder, cylinder-head seat + nozzle pins, piston rails,
spring-guide seat, 8 mm bushings, motor pocket, anti-reversal latch) are kept *exactly*: every
change below is a boolean add/cut on the original bodies, placed only where there was solid
material or empty space outside the original block.

Assembly frame used for every exported part (mm):
    X = 0 at the gearbox front face, +X toward the rear (spring guide).  The net fires toward -X.
    Y = 0 on the cylinder axis, +Y up (top face of the gearbox is Y = +25).
    Z across the gearbox; the two halves meet at Z = -3 (half A is Z -3..+15, half B is Z -15..-3).
"""
from __future__ import annotations

import math
import os
import sys

from build123d import (Align, Axis, Box, Compound, Cone, Cylinder, GeomType, Location, Plane, Pos,
                       RegularPolygon, Rot, Solid, Vector, chamfer, export_step, export_stl, extrude,
                       fillet, import_step)

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "source", "gearbox_v2_original.step")
OUT = os.path.join(HERE, "out")

# --------------------------------------------------------------------------------------------
# Parameters (edit these, then re-run)
# --------------------------------------------------------------------------------------------
# frame offsets of the original file (measured from the bodies themselves at load time)
LEN = 163.0                 # gearbox length (front face -> rear face)
TOP = 25.0                  # top face height above cylinder axis
BOT0 = -76.194              # original bottom face (pocket was open here)
SPLIT = -3.0                # split plane between halves (Z)
HALF = 15.0                 # outer faces at Z = +/-15

# gearbox changes
BOTTOM_WALL = 5.0           # new closed bottom wall thickness
CHAMFER = 1.0               # outer edge chamfer
CABLE_SLOT = dict(x0=24.0, x1=34.0, z0=-6.0, z1=6.0)   # motor-lead exit through the bottom wall

M3_CLEAR = 3.4              # clearance hole
M3_CBORE_D, M3_CBORE_H = 6.5, 5.0      # socket-head counterbore in half A (M3x25 SHCS)
M3_NUT_AF, M3_NUT_H = 5.8, 2.8         # hex nut trap in half B (M3 DIN 934, 5.5 AF x 2.4)
INSERT_D, INSERT_H = 4.0, 6.5          # M3 heat-set insert hole (M3 x 5.7 x 4.6 inserts)
DOWEL_D, DOWEL_H = 3.1, 7.0            # 3 mm steel dowel pin, 7 mm into each half

# joint bolts (X, Y). Top band and the two solid blocks need no boss; bottom ones get a boss.
BOLTS_SOLID = [(16, 19), (55, 19), (110, 19), (157, 19), (24, -22), (52, -22), (151, -17)]
BOLTS_BOSS = [(12, -71), (95, -71), (147, -71)]
BOSS_R = 5.0
POCKET_Z = 7.0              # gear pocket floors are at Z = +/-7
DOWELS = [(36, -27), (159, -45)]
TOP_INSERTS = [(30, 8), (80, 8), (130, 8), (30, -8), (80, -8), (130, -8)]   # (X, Z) on top face

# launcher-head mounting tabs (part of the gearbox halves, forward of the front face)
TAB = dict(x0=-12.0, x1=1.5, y0=-30.0, y1=TOP, t=4.0)
TAB_SCREWS = [(-4.0, 17.0), (-4.0, -24.0)]              # (X, Y) M3 x 8 through the tabs

# launcher head (4 barrels)
YL = -10.0                  # launcher axis height (head top stays under the gearbox top face)
THETA = 22.0                # half-angle of the barrels from the launch axis (deg)
R_BREECH = 10.0             # barrel axis radius at the breech
X_BREECH = -20.0            # breech (bottom of the bores) on the launch axis
BORE = 12.2                 # printed bore (prints ~12.0); projectile O-ring seals it
BARREL_WALL = 2.5
BARREL_LEN = 66.0           # breech -> muzzle along the barrel axis (42 travel + 22 projectile + 2)
FEED_D = 5.0                # breech feed hole
PLENUM_R, PLENUM_X = 9.0, (-16.0, -13.0)
FLANGE = dict(x0=-16.0, x1=0.0, y0=-30.0, y1=24.5, zh=10.75)   # 0.25 mm clearance to each tab
SOCKET_D, SOCKET_DEPTH = 11.6, 12.0      # seal-sleeve socket on the nozzle axis
INLET_D = 4.5
CUP_FLOOR_A = 14.0          # cup floor, axial distance in front of the breech
CUP_R0 = 1.4                # cup radius = CUP_R0 + a*tan(THETA)  (keeps a 2 mm wall to the bores)
CUP_WALL = 1.6
# refill check valve.  Without it the sealed launcher pulls a vacuum while the piston is cocked and
# the shot fails (see analysis/RESULTS.md).  Ball = one 6 mm airsoft BB, dropped in during a print pause.
VALVE = dict(y=-16.0, z=0.0, seat_d=4.0, chamber_d=7.2, x_seat=-2.4, x_chamber=(-4.0, -11.5),
             slot=(3.5, 8.0))
BLEED = dict(y=-14.0, z=5.0, d=1.2)      # small bleed: lets air out while loading the weights
VENT_GROOVE = dict(y0=-31.0, y1=-12.5, zh=6.0, depth=1.2)   # rear-face groove to open air

# projectile (net weight)
PROJ_D = 11.7
PROJ_L = 22.0
ORING_CS, ORING_ROOT_D = 2.0, 8.6        # O-ring 8 x 2 mm NBR, groove root diameter
ORING_GROOVE_W, ORING_GROOVE_X = 2.6, 2.2  # groove width, distance from the rear face
CORE_D, CORE_L = 8.2, 15.5              # cavity for an 8 x 15 (or 16) mm steel pin
TETHER_HOLE_D, TETHER_HOLE_X = 1.8, 3.5  # cross hole, distance from the nose

# drone mount plate (optional): bolts to the 6 top inserts, slots for 20 mm straps / zip ties
PLATE = dict(x0=16.0, x1=144.0, zh=22.0, t=3.0, slot_w=3.5, slot_l=22.0, slot_x=(24.0, 136.0))

# nozzle seal sleeve (print in TPU 95A).  Measure your nozzle tip and set NOZZLE_TIP_OD.
NOZZLE_TIP_OD = 6.0
SLEEVE_OD = 11.4
SLEEVE_SQUEEZE = 0.4
# pick by how far the nozzle TIP sticks out of the gearbox front face (P, mm):
#   P 10..12 -> S5,  7..10 -> S8,  4..7 -> S11,  1..4 -> S14,  -2..1 -> S17
SLEEVE_LENGTHS = {"S5": 5.0, "S8": 8.0, "S11": 11.0, "S14": 14.0, "S17": 17.0}


# --------------------------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def cyl_z(x, y, z0, z1, r):
    return Pos(x, y, z0) * Cylinder(r, z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def cyl_y(x, z, y0, y1, r):
    return Pos(x, y0, z) * (Rot(-90, 0, 0) * Cylinder(r, y1 - y0, align=(Align.CENTER, Align.CENTER, Align.MIN)))


def cyl_x(y, z, x0, x1, r):
    return Pos(x0, y, z) * (Rot(0, 90, 0) * Cylinder(r, x1 - x0, align=(Align.CENTER, Align.CENTER, Align.MIN)))


def cyl_dir(origin, direction, r, s0, s1):
    """cylinder of radius r along unit 'direction' from origin+s0*d to origin+s1*d"""
    d = Vector(*direction).normalized()
    o = Vector(*origin) + d * s0
    return Plane(origin=o, z_dir=d) * Cylinder(r, s1 - s0, align=(Align.CENTER, Align.CENTER, Align.MIN))


def hex_z(x, y, z0, z1, af):
    r = af / math.sqrt(3)
    return Pos(x, y, z0) * extrude(RegularPolygon(r, 6), amount=z1 - z0)


def single(shape):
    """return the largest solid of a boolean result"""
    sols = shape.solids()
    return max(sols, key=lambda s: s.volume) if len(sols) > 1 else sols[0]


# --------------------------------------------------------------------------------------------
# gearbox halves
# --------------------------------------------------------------------------------------------
def _axis_of(solid):
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    for f in solid.faces():
        if f.geom_type == GeomType.CYLINDER:
            c = BRepAdaptor_Surface(f.wrapped).Cylinder()
            if abs(c.Radius() - 12.75) < 1e-6:
                loc = c.Axis().Location()
                return loc.Y(), loc.Z()
    raise RuntimeError("cylinder bore (r=12.75) not found")


def load_original_halves():
    sols = import_step(SRC).solids()
    by_y = sorted(sols, key=lambda s: s.bounding_box().min.Y)
    s0, s1, s2 = by_y          # BLOCK, BLOCK (1), BLOCK (1) (1)
    x0 = s1.bounding_box().min.X
    y1, z1 = _axis_of(s1)
    y2, z2 = _axis_of(s2)
    A = Pos(-x0, -y1, -z1) * s1
    B = Rot(180, 0, 0) * (Pos(-x0, -y2, -z2) * s2)
    return A, B, (Pos(-x0, -_axis_of(s0)[0], -_axis_of(s0)[1]) * s0)


def chamfered_envelope(x0, x1, y0, y1, z0, z1, c):
    b = box(x0, x1, y0, y1, z0, z1)
    return chamfer(b.edges(), c)


def build_gearbox():
    A, B, _ = load_original_halves()
    zA = (SPLIT, HALF)
    zB = (-HALF, SPLIT)
    ybot = BOT0 - BOTTOM_WALL

    # 1) close the open bottom of the gear pocket
    A = A + box(0, LEN, ybot, BOT0 + 0.2, *zA)
    B = B + box(0, LEN, ybot, BOT0 + 0.2, *zB)

    # 2) bolt bosses bridging the pocket where bottom bolts pass
    for x, y in BOLTS_BOSS:
        A = A + cyl_z(x, y, SPLIT, POCKET_Z + 0.2, BOSS_R)
        B = B + cyl_z(x, y, -POCKET_Z - 0.2, SPLIT, BOSS_R)

    # 3) chamfer all outer edges (split-face edges stay sharp)
    env = chamfered_envelope(0, LEN, ybot, TOP, -HALF, HALF, CHAMFER)
    A = single(A & env)
    B = single(B & env)

    # 4) launcher mounting tabs (flush with each outer face, reaching forward of the front face)
    tA = box(TAB["x0"], TAB["x1"], TAB["y0"], TAB["y1"], HALF - TAB["t"], HALF)
    tB = box(TAB["x0"], TAB["x1"], TAB["y0"], TAB["y1"], -HALF, -HALF + TAB["t"])
    tA = chamfer(tA.edges().filter_by(Axis.Y).group_by(Axis.X)[0], 1.0)
    tB = chamfer(tB.edges().filter_by(Axis.Y).group_by(Axis.X)[0], 1.0)
    A = single(A + tA)
    B = single(B + tB)

    # 5) holes
    for x, y in BOLTS_SOLID + BOLTS_BOSS:
        A = A - cyl_z(x, y, SPLIT - 1, HALF + 1, M3_CLEAR / 2)
        A = A - cyl_z(x, y, HALF - M3_CBORE_H, HALF + 1, M3_CBORE_D / 2)
        B = B - cyl_z(x, y, -HALF - 1, SPLIT + 1, M3_CLEAR / 2)
        B = B - hex_z(x, y, -HALF - 1, -HALF + M3_NUT_H, M3_NUT_AF)
    for x, y in DOWELS:
        A = A - cyl_z(x, y, SPLIT - 0.1, SPLIT + DOWEL_H, DOWEL_D / 2)
        B = B - cyl_z(x, y, SPLIT - DOWEL_H, SPLIT + 0.1, DOWEL_D / 2)
    for x, z in TOP_INSERTS:
        hole = cyl_y(x, z, TOP - INSERT_H, TOP + 1, INSERT_D / 2)
        if z > SPLIT:
            A = A - hole
        else:
            B = B - hole
    for x, y in TAB_SCREWS:
        A = A - cyl_z(x, y, HALF - TAB["t"] - 1, HALF + 1, M3_CLEAR / 2)
        B = B - cyl_z(x, y, -HALF - 1, -HALF + TAB["t"] + 1, M3_CLEAR / 2)
    cs = CABLE_SLOT
    slot = box(cs["x0"], cs["x1"], ybot - 1, BOT0 + 0.5, cs["z0"], cs["z1"])
    A = A - slot
    B = B - slot
    return single(A), single(B)


# --------------------------------------------------------------------------------------------
# launcher head
# --------------------------------------------------------------------------------------------
def barrel_axes():
    t = math.radians(THETA)
    axes = []
    for phi in (45, 135, 225, 315):
        p = math.radians(phi)
        radial = Vector(0, math.sin(p), math.cos(p))
        o = Vector(X_BREECH, YL, 0) + radial * R_BREECH
        d = Vector(-math.cos(t), math.sin(t) * math.sin(p), math.sin(t) * math.cos(p))
        axes.append((o, d, radial))
    return axes


def build_head():
    t = math.radians(THETA)
    tan = math.tan(t)
    f = FLANGE
    flange = box(f["x0"], f["x1"], f["y0"], f["y1"], -f["zh"], f["zh"])
    flange = fillet(flange.edges().filter_by(Axis.X), 1.5)
    # Print support envelope: a 45-degree cone that starts at the flange width just in front of the
    # gearbox tabs.  The breech block is this cone, and the barrel tubes are trimmed to it, so every
    # surface facing the print bed is self-supporting (head prints rear face down, barrels up).
    x_c0 = TAB["x0"] - 0.5                     # cone starts just in front of the tabs
    r_c0 = f["zh"]
    cone45 = Pos(x_c0, YL, 0) * (Rot(0, -90, 0) * Cone(r_c0, r_c0 + 120.0, 120.0, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    x_bb1 = X_BREECH - 4.0                     # top of the solid breech block
    breech = cone45 & box(x_bb1, x_c0, YL - 60, YL + 60, -60, 60)
    body = flange + breech
    axes = barrel_axes()
    r_out = BORE / 2 + BARREL_WALL
    for o, d, _ in axes:
        body = body + (cyl_dir(o, d, r_out, -4.0, BARREL_LEN) & cone45)
    # net cup: solid cone core, then hollowed
    a0, a1 = 4.0, BARREL_LEN * math.cos(t) + 1.0
    def cup_cone(r_off, a_start, a_end):
        r_s = CUP_R0 + a_start * tan + r_off
        r_e = CUP_R0 + a_end * tan + r_off
        return Pos(X_BREECH - a_start, YL, 0) * (Rot(0, -90, 0) * Cone(r_s, r_e, a_end - a_start, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    body = body + cup_cone(CUP_WALL, a0, a1)
    # ---- cuts
    body = body - cup_cone(0.0, CUP_FLOOR_A, a1 + 1.0)
    for o, d, radial in axes:
        body = body - cyl_dir(o, d, BORE / 2, 0.0, BARREL_LEN + 2.0)
        body = body - cyl_dir(o, d, FEED_D / 2, -8.0, 0.5)
        # tether notch at the muzzle on the cup side
        tip = o + d * (BARREL_LEN - 2.5)
        inward = Vector(0, -radial.Y, -radial.Z)
        notch = Plane(origin=tip, x_dir=d, z_dir=inward) * Box(7.0, 2.6, 14.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
        body = body - notch
    # plenum + link to the inlet
    body = body - cyl_x(YL, 0, PLENUM_X[0], PLENUM_X[1], PLENUM_R)
    body = body - box(PLENUM_X[0], PLENUM_X[1], YL, 1.8, -2.5, 2.5)
    # refill check valve (6 mm BB on a conical seat; slot retainer toward the plenum)
    v = VALVE
    body = body - cyl_x(v["y"], v["z"], v["x_seat"] - 0.01, 1.0, v["seat_d"] / 2)              # seat hole to rear face
    seat_len = (v["chamber_d"] - v["seat_d"]) / 2
    body = body - (Pos(v["x_seat"], v["y"], v["z"]) * (Rot(0, -90, 0) * Cone(v["seat_d"] / 2, v["chamber_d"] / 2, seat_len,
                                                                              align=(Align.CENTER, Align.CENTER, Align.MIN))))
    body = body - cyl_x(v["y"], v["z"], v["x_chamber"][1], v["x_chamber"][0] + 0.01, v["chamber_d"] / 2)
    sw, sl = v["slot"]
    body = body - box(PLENUM_X[1] - 0.2, v["x_chamber"][1] + 0.01, v["y"] - sw / 2, v["y"] + sw / 2, v["z"] - sl / 2, v["z"] + sl / 2)
    # bleed hole and the vent groove on the rear face that leads both to open air below the flange
    b = BLEED
    body = body - cyl_x(b["y"], b["z"], PLENUM_X[1] - 0.2, 1.0, b["d"] / 2)
    g = VENT_GROOVE
    body = body - box(-g["depth"], 1.0, f["y0"] - 1.0, g["y1"], -g["zh"], g["zh"])
    # socket for the TPU seal sleeve (nozzle comes in from the gearbox) + inlet
    body = body - cyl_x(0, 0, -SOCKET_DEPTH, 1.0, SOCKET_D / 2)
    body = body - cyl_x(0, 0, PLENUM_X[1] - 0.5, -SOCKET_DEPTH + 0.1, INLET_D / 2)
    # small entry chamfer for the sleeve
    body = body - (Pos(0.01, 0, 0) * (Rot(0, -90, 0) * Cone(SOCKET_D / 2 + 0.8, SOCKET_D / 2, 0.8, align=(Align.CENTER, Align.CENTER, Align.MIN))))
    # heat-set inserts in the flange sides (screws come through the gearbox tabs)
    for x, y in TAB_SCREWS:
        body = body - cyl_z(x, y, f["zh"] - INSERT_H, f["zh"] + 1, INSERT_D / 2)
        body = body - cyl_z(x, y, -f["zh"] - 1, -f["zh"] + INSERT_H, INSERT_D / 2)
    return single(body)


# --------------------------------------------------------------------------------------------
# projectile (net weight) - built along +Z (rear face at Z=0, nose at Z=PROJ_L)
# --------------------------------------------------------------------------------------------
def build_projectile():
    p = Cylinder(PROJ_D / 2, PROJ_L, align=(Align.CENTER, Align.CENTER, Align.MIN))
    p = chamfer(p.edges().group_by(Axis.Z)[-1], 1.5)              # rounded-ish nose
    p = chamfer(p.edges().group_by(Axis.Z)[0], 0.4)               # rear edge (helps loading)
    groove = Pos(0, 0, ORING_GROOVE_X) * (Cylinder(PROJ_D / 2 + 1, ORING_GROOVE_W, align=(Align.CENTER, Align.CENTER, Align.MIN))
                                          - Cylinder(ORING_ROOT_D / 2, ORING_GROOVE_W, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    p = p - groove
    p = p - Pos(0, 0, -0.1) * Cylinder(CORE_D / 2, CORE_L + 0.1, align=(Align.CENTER, Align.CENTER, Align.MIN))
    p = p - cyl_x(0, PROJ_L - TETHER_HOLE_X, -10, 10, TETHER_HOLE_D / 2)
    return single(p)


def build_sleeve(length):
    d_in = NOZZLE_TIP_OD - SLEEVE_SQUEEZE
    s = Cylinder(SLEEVE_OD / 2, length, align=(Align.CENTER, Align.CENTER, Align.MIN)) - \
        Cylinder(d_in / 2, length, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # lead-in cone on the nozzle side (Z = length)
    s = s - Pos(0, 0, length - 1.0) * Cone(d_in / 2, d_in / 2 + 0.8, 1.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return single(s)


def build_mount_plate():
    pl = PLATE
    plate = box(pl["x0"], pl["x1"], TOP, TOP + pl["t"], -pl["zh"], pl["zh"])
    plate = fillet(plate.edges().filter_by(Axis.Y), 4.0)
    for x, z in TOP_INSERTS:
        plate = plate - cyl_y(x, z, TOP - 1, TOP + pl["t"] + 1, M3_CLEAR / 2)
        plate = plate - cyl_y(x, z, TOP + pl["t"] - 1.7, TOP + pl["t"] + 1, 3.2)   # button-head recess
    for xs in pl["slot_x"]:
        plate = plate - box(xs - pl["slot_w"] / 2, xs + pl["slot_w"] / 2, TOP - 1, TOP + pl["t"] + 1,
                            -pl["slot_l"] / 2, pl["slot_l"] / 2)
    # lightening + extra M3 grid for other drones (10 mm pitch on the centre line)
    for x in range(40, 125, 10):
        if all(abs(x - ix) > 6 for ix, _ in TOP_INSERTS):
            plate = plate - cyl_y(x, 0, TOP - 1, TOP + pl["t"] + 1, M3_CLEAR / 2)
    return single(plate)


# --------------------------------------------------------------------------------------------
# placement helpers for the assembly
# --------------------------------------------------------------------------------------------
def projectile_locations(seated=True):
    locs = []
    for o, d, _ in barrel_axes():
        locs.append(Plane(origin=o, z_dir=d).location)
    return locs


def print_orient(shape, kind):
    """rotate parts so their natural flat face sits on Z=0 for printing"""
    if kind == "half_A":       # outer face (Z=+15) down
        s = Rot(180, 0, 0) * shape
    elif kind == "half_B":     # outer face (Z=-15) down
        s = shape
    elif kind == "head":       # rear face (X=0) down, barrels up:  new Z = -X
        s = Rot(0, 90, 0) * shape
    elif kind == "plate":      # flat
        s = Rot(90, 0, 0) * shape
    else:
        s = shape
    bb = s.bounding_box()
    return Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * s


def main(write=True):
    os.makedirs(os.path.join(OUT, "step"), exist_ok=True)
    os.makedirs(os.path.join(OUT, "stl"), exist_ok=True)
    A, B = build_gearbox()
    head = build_head()
    proj = build_projectile()
    sleeves = {k: build_sleeve(v) for k, v in SLEEVE_LENGTHS.items()}
    plate = build_mount_plate()
    parts = {"gearbox_half_A": A, "gearbox_half_B": B, "launcher_head": head, "net_weight": proj, "mount_plate": plate}
    for k, s in sleeves.items():
        parts[f"nozzle_seal_sleeve_{k}"] = s
    for name, shp in parts.items():
        print(f"{name:28s} valid={shp.is_valid}  volume={shp.volume/1000:7.1f} cm3")
    if write:
        export_step(A, os.path.join(OUT, "step", "gearbox_half_A.step"))
        export_step(B, os.path.join(OUT, "step", "gearbox_half_B.step"))
        export_step(head, os.path.join(OUT, "step", "launcher_head.step"))
        export_step(proj, os.path.join(OUT, "step", "net_weight.step"))
        export_step(plate, os.path.join(OUT, "step", "mount_plate.step"))
        for k, s in sleeves.items():
            export_step(s, os.path.join(OUT, "step", f"nozzle_seal_sleeve_{k}.step"))
        # assembly (projectiles seated in the barrels, M sleeve in the socket)
        asm = [A, B, head, plate]
        for loc in projectile_locations():
            asm.append(loc * proj)
        asm.append(Pos(-SOCKET_DEPTH, 0, 0) * (Rot(0, 90, 0) * sleeves["S8"]))
        export_step(Compound(asm), os.path.join(OUT, "step", "netrodrone_assembly.step"))
        # print-oriented STL
        stl = os.path.join(OUT, "stl")
        export_stl(print_orient(A, "half_A"), os.path.join(stl, "gearbox_half_A.stl"), tolerance=0.02, angular_tolerance=0.1)
        export_stl(print_orient(B, "half_B"), os.path.join(stl, "gearbox_half_B.stl"), tolerance=0.02, angular_tolerance=0.1)
        export_stl(print_orient(head, "head"), os.path.join(stl, "launcher_head.stl"), tolerance=0.02, angular_tolerance=0.1)
        one = print_orient(proj, "proj")
        four = Compound([Pos(dx, dy, 0) * one for dx in (-9.0, 9.0) for dy in (-9.0, 9.0)])
        export_stl(four, os.path.join(stl, "net_weight_x4.stl"), tolerance=0.01, angular_tolerance=0.1)
        export_stl(print_orient(plate, "plate"), os.path.join(stl, "mount_plate.stl"), tolerance=0.02, angular_tolerance=0.1)
        for k, s in sleeves.items():
            export_stl(print_orient(s, "sleeve"), os.path.join(stl, f"nozzle_seal_sleeve_{k}_TPU.stl"), tolerance=0.01, angular_tolerance=0.1)
    return parts


if __name__ == "__main__":
    main(write="--dry" not in sys.argv)
