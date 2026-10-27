"""
Geometry checks for the netrodrone build.  Run:  python cad/verify.py

1. every part is a valid closed solid
2. no collisions between parts in the assembly (gearbox halves, launcher head, net weights, seal sleeve,
   plus stand-in bodies for the AEG cylinder and motor sized from the original pockets)
3. the original component cavities are untouched: material added inside them is only the bottom bolt
   bosses (+0.2 mm overlap where the new bottom wall meets the old bottom edge)
4. minimum wall around every new bolt / dowel / insert hole
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import build_netrodrone as bn  # noqa: E402
from build123d import Box, Pos, Rot, Cylinder, Align  # noqa: E402

ok = True


def report(name, good, detail=""):
    global ok
    ok &= bool(good)
    print(f"[{'PASS' if good else 'FAIL'}] {name} {detail}")


def vol(s):
    return 0.0 if s is None else sum(x.volume for x in s.solids())


def main():
    A0, B0, _ = bn.load_original_halves()
    A, B = bn.build_gearbox()
    head = bn.build_head()
    proj = bn.build_projectile()
    sleeve = bn.build_sleeve(bn.SLEEVE_LENGTHS["S8"])

    # 1. validity
    for n, s in [("half A", A), ("half B", B), ("launcher head", head), ("net weight", proj), ("seal sleeve", sleeve)]:
        report(f"{n} is a valid solid", s.is_valid and len(s.solids()) == 1)

    # 2. collisions
    cyl_tube = bn.cyl_x(0, 0, 12.8 + 0.2, 85.0 - 0.2, 24.4 / 2)                    # AEG cylinder stand-in
    motor = bn.cyl_x(-48.9, 0, 20.0, 72.5, 13.0)    # motor body stand-in (dia 26), clear of the 1-1.5 mm end fillets
    parts = {"A": A, "B": B, "head": head, "cylinder": cyl_tube, "motor": motor,
             "sleeve": Pos(-bn.SOCKET_DEPTH, 0, 0) * (Rot(0, 90, 0) * sleeve)}
    for k, loc in enumerate(bn.projectile_locations()):
        parts[f"weight{k}"] = loc * proj
    names = list(parts)
    worst = 0.0
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = parts[names[i]], parts[names[j]]
            if a.bounding_box().to_align_offset if False else True:
                v = vol(a & b)
                if v > 0.05:  # mm^3; coplanar contact produces ~0 volume slivers
                    report(f"collision {names[i]} x {names[j]}", False, f"{v:.2f} mm3")
                worst = max(worst, v)
    report("no part collisions in the assembly", worst <= 0.05, f"(largest overlap {worst:.3f} mm3)")

    # 3. original geometry untouched except in the intended zones (ray casting along Z)
    from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
    from OCP.gp import gp_Lin, gp_Pnt, gp_Dir

    def hits(inter, x, y):
        inter.Perform(gp_Lin(gp_Pnt(x, y, -100), gp_Dir(0, 0, 1)), -1e9, 1e9)
        return tuple(sorted({round(inter.Pnt(k + 1).Z(), 2) for k in range(inter.NbPnt())})) if inter.IsDone() else ()

    def expected_zone(x, y):
        if x < bn.CHAMFER + 0.3 or x > bn.LEN - bn.CHAMFER - 0.3 or y > bn.TOP - bn.CHAMFER - 0.3:
            return "edge chamfer"
        if y < bn.BOT0 + 0.5:
            return "new bottom wall"
        for hx, hy in bn.BOLTS_SOLID + bn.BOLTS_BOSS:
            if math.hypot(x - hx, y - hy) < bn.M3_CBORE_D / 2 + 0.4:
                return "bolt hole"
        for hx, hy in bn.BOLTS_BOSS:
            if math.hypot(x - hx, y - hy) < bn.BOSS_R + 0.4:
                return "boss"
        for hx, hy in bn.DOWELS:
            if math.hypot(x - hx, y - hy) < bn.DOWEL_D / 2 + 0.4:
                return "dowel"
        for hx, hz in bn.TOP_INSERTS:
            if abs(x - hx) < bn.INSERT_D / 2 + 0.4 and y > bn.TOP - bn.INSERT_H - 0.4:
                return "top insert"
        return None

    step = 0.7
    for nm, orig, new in [("A", A0, A), ("B", B0, B)]:
        io, inw = IntCurvesFace_ShapeIntersector(), IntCurvesFace_ShapeIntersector()
        io.Load(orig.wrapped, 1e-4); inw.Load(new.wrapped, 1e-4)
        changed, unexpected = 0, []
        nx, ny = int(bn.LEN / step), int((bn.TOP - bn.BOT0) / step)
        for i in range(nx):
            x = 0.35 + i * step
            for j in range(ny):
                y = bn.BOT0 + 0.35 + j * step
                if hits(io, x, y) != hits(inw, x, y):
                    changed += 1
                    if expected_zone(x, y) is None:
                        unexpected.append((round(x, 1), round(y, 1)))
        report(f"half {nm}: original geometry unchanged outside intended zones", not unexpected,
               f"({changed} of {nx*ny} rays changed, all in bolt/dowel/insert/boss/bottom/chamfer zones)" if not unexpected
               else f"unexpected changes at {unexpected[:12]}")

    # 4. wall around new holes: ring of thickness w around each hole must not hit empty space
    def min_wall(part, x, y, z0, z1, r, ring=1.2):
        ringshape = bn.cyl_z(x, y, z0, z1, r + ring) - bn.cyl_z(x, y, z0 - 1, z1 + 1, r + 0.01)
        return vol(ringshape - part)

    for nm, part, zr in [("A", A, (bn.SPLIT + 0.3, bn.HALF - bn.M3_CBORE_H - 0.3)), ("B", B, (-bn.HALF + bn.M3_NUT_H + 0.3, bn.SPLIT - 0.3))]:
        bad = []
        for x, y in bn.BOLTS_SOLID + bn.BOLTS_BOSS:
            v = min_wall(part, x, y, *zr, bn.M3_CLEAR / 2)
            if v > 0.5:
                bad.append(((x, y), round(v, 1)))
        report(f"half {nm}: >=1.2 mm wall around all bolt shanks", not bad, str(bad) if bad else "")
        # nut trap / counterbore: 1.0 mm ring
        bad = []
        for x, y in bn.BOLTS_SOLID + bn.BOLTS_BOSS:
            if nm == "A":
                v = min_wall(part, x, y, bn.HALF - bn.M3_CBORE_H + 0.2, bn.HALF - 0.2, bn.M3_CBORE_D / 2, 1.0)
            else:
                v = min_wall(part, x, y, -bn.HALF + 0.2, -bn.HALF + bn.M3_NUT_H - 0.2, bn.M3_NUT_AF / math.sqrt(3), 1.0)
            if v > 0.5:
                bad.append(((x, y), round(v, 1)))
        report(f"half {nm}: >=1.0 mm wall around {'counterbores' if nm == 'A' else 'nut traps'}", not bad, str(bad) if bad else "")
        bad = []
        for x, y in bn.DOWELS:
            z0, z1 = (bn.SPLIT + 0.2, bn.SPLIT + bn.DOWEL_H - 0.2) if nm == "A" else (bn.SPLIT - bn.DOWEL_H + 0.2, bn.SPLIT - 0.2)
            v = min_wall(part, x, y, z0, z1, bn.DOWEL_D / 2)
            if v > 0.5:
                bad.append(((x, y), round(v, 1)))
        report(f"half {nm}: >=1.2 mm wall around dowel holes", not bad, str(bad) if bad else "")

    # top-face insert holes (along Y)
    bad = []
    for x, z in bn.TOP_INSERTS:
        part = A if z > bn.SPLIT else B
        ring = bn.cyl_y(x, z, bn.TOP - bn.INSERT_H + 0.2, bn.TOP - 0.2, bn.INSERT_D / 2 + 1.2) - \
            bn.cyl_y(x, z, bn.TOP - bn.INSERT_H - 1, bn.TOP + 1, bn.INSERT_D / 2 + 0.01)
        v = vol(ring - part)
        if v > 0.5:
            bad.append(((x, z), round(v, 1)))
    report(">=1.2 mm wall around top-face insert holes", not bad, str(bad) if bad else "")

    # head air paths: valve + bleed + vent groove must be open from the plenum to open air
    from build123d import Vector
    def empty_along(pts):
        return all(not head.is_inside(Vector(*p)) for p in pts)
    v, b, g = bn.VALVE, bn.BLEED, bn.VENT_GROOVE
    valve_path = [(x, v["y"], v["z"]) for x in [i * -0.25 for i in range(0, 56)]]          # X 0 .. -13.75
    report("head: check-valve bore open from rear face into the plenum", empty_along(valve_path))
    bleed_path = [(x, b["y"], b["z"]) for x in [i * -0.25 for i in range(0, 56)]]
    report("head: bleed hole open from rear face into the plenum", empty_along(bleed_path))
    groove_path = [(-0.6, y, 0.0) for y in [-31.5 + i * 0.25 for i in range(0, 70)]]
    report("head: vent groove reaches open air below the flange", empty_along(groove_path))
    ball_r = 5.95 / 2
    report("head: 6 mm BB cannot pass the valve retainer slot", v["slot"][0] < 2 * ball_r - 1.0, f"(slot {v['slot'][0]} mm)")
    report("head: 6 mm BB cannot pass the seat hole", v["seat_d"] < 2 * ball_r - 1.0, f"(seat {v['seat_d']} mm)")

    # head: wall between net cup and each bore, and around the side inserts
    t = math.radians(bn.THETA)
    wall = (bn.R_BREECH - bn.BORE / 2 / math.cos(t) - bn.CUP_R0) * math.cos(t)
    report("head: cup-to-bore wall >= 1.8 mm", wall >= 1.8, f"({wall:.2f} mm)")
    print("\nOVERALL:", "PASS" if ok else "FAIL")
    return ok


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
