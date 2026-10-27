"""Cut-away of the launcher (how a shot works) -> docs/images/how_it_works.png
Run: python docs/figures/how_it_works.py   (~5 min: exact ray-cast sections of the CAD)"""
import sys, math
import numpy as np
from pathlib import Path
REPO = Path(__file__).resolve().parents[2]
IMG = REPO / "docs" / "images"
sys.path.insert(0, str(REPO / "cad"))
import build_netrodrone as bn
from build123d import *
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.gp import gp_Lin, gp_Pnt, gp_Dir
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle as MRect, Circle, FancyArrowPatch, Polygon

SURF, INK, INK2, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9"
plt.rcParams.update({"figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF, "font.size": 9.5,
                     "text.color": INK})


def raster(shape, origin, ex, ey, xr, yr, res=0.12):
    """boolean image of the planar section (rows along ex). exact along rows via ray hits."""
    o, ex, ey = Vector(*origin), Vector(*ex).normalized(), Vector(*ey).normalized()
    inter = IntCurvesFace_ShapeIntersector(); inter.Load(shape.wrapped, 1e-5)
    xs = np.arange(xr[0], xr[1], res); ys = np.arange(yr[0], yr[1], res)
    img = np.zeros((len(ys), len(xs)), bool)
    for j, v in enumerate(ys):
        p = o + ey * v + ex * xr[0]
        inter.Perform(gp_Lin(gp_Pnt(p.X, p.Y, p.Z), gp_Dir(ex.X, ex.Y, ex.Z)), -1e-6, xr[1] - xr[0] + 1)
        if not inter.IsDone() or inter.NbPnt() == 0:
            continue
        ts = sorted(xr[0] + inter.WParameter(k + 1) for k in range(inter.NbPnt()))
        ts = [xr[0]] + ts + [xr[1]]
        for a, b in zip(ts[:-1], ts[1:]):
            if b - a < 1e-6:
                continue
            q = o + ey * v + ex * ((a + b) / 2)
            if shape.is_inside(q):
                i0 = int(np.searchsorted(xs, a)); i1 = int(np.searchsorted(xs, b))
                img[j, i0:i1] = True
    return img, xs, ys


def show(ax, img, xs, ys, color, alpha=1.0, z=1):
    rgba = np.zeros(img.shape + (4,))
    c = matplotlib.colors.to_rgb(color)
    rgba[img] = (*c, alpha)
    ax.imshow(rgba, origin="lower", extent=[xs[0], xs[-1], ys[0], ys[-1]], interpolation="nearest", zorder=z)


A, B = bn.build_gearbox(); head = bn.build_head(); proj = bn.build_projectile()
sleeve = Pos(-bn.SOCKET_DEPTH, 0, 0) * (Rot(0, 90, 0) * bn.build_sleeve(bn.SLEEVE_LENGTHS["S8"]))
C = dict(A="#d9894a", head="#5fa04e", w="#e3b23c", part="#a9a8a2", dark="#5d5c58", sleeve="#222222")

fig = plt.figure(figsize=(15, 10.2))
gs = fig.add_gridspec(2, 2, height_ratios=[1.25, 1], width_ratios=[1.35, 1])
ax = fig.add_subplot(gs[0, :])
XR, YR = (-92, 168), (-84, 32)
# ---- panel 1: Z = 1.0 plane (inside half A, through the cylinder/nozzle axis region)
for shp, col in [(A, C["A"]), (head, C["head"]), (sleeve, C["sleeve"])]:
    img, xs, ys = raster(shp, (0, 0, 1.0), (1, 0, 0), (0, 1, 0), XR, YR, res=0.18)
    show(ax, img, xs, ys, col)
# schematic AEG parts (illustrative)
part_kw = dict(facecolor=C["part"], edgecolor=C["dark"], lw=0.7, zorder=3)
ax.add_patch(MRect((13.0, -12.2), 72.0, 0.7, **part_kw)); ax.add_patch(MRect((13.0, 11.5), 72.0, 0.7, **part_kw))   # cylinder tube
ax.add_patch(MRect((13.0, -11.5), 7.0, 23.0, **part_kw))                                                           # cylinder head
ax.add_patch(MRect((3.0, -4.7), 10.0, 9.4, **part_kw))                                                             # head nub
ax.add_patch(Polygon([(12.0, 5.5), (-1.0, 5.5), (-1.0, 3.2), (-6.5, 3.2), (-6.5, -3.2), (-1.0, -3.2), (-1.0, -5.5), (12.0, -5.5)], closed=True, **part_kw))  # nozzle
ax.add_patch(Polygon([(60.0, 11.4), (68.0, 11.4), (68.0, 9.5), (146.0, 9.5), (146.0, -9.5), (68.0, -9.5), (68.0, -11.4), (60.0, -11.4)], closed=True, **part_kw))  # piston (cocked)
zz = np.linspace(68.0, 155.5, 60); ax.plot(zz, 7.5 * np.sign(np.sin(np.linspace(0, 29 * np.pi, 60))) * 0.9, color=C["dark"], lw=0.8, zorder=4)  # spring
ax.add_patch(MRect((155.0, -10.9), 2.2, 21.8, **part_kw)); ax.add_patch(MRect((140.0, -6.0), 22.0, 12.0, **part_kw))  # spring guide
for (gx, gy, r, nm) in [(95.5, -25.0, 17.5, "sector"), (116.0, -29.41, 18.0, "spur"), (112.5, -48.83, 18.0, "bevel")]:
    ax.add_patch(Circle((gx, gy), r, fill=False, ls=(0, (3, 2)), color=C["dark"], lw=0.8, zorder=4))
    ax.text(gx, gy, nm, ha="center", va="center", fontsize=8, color=INK2, zorder=5)
ax.add_patch(MRect((20.0, -61.9), 52.5, 26.0, facecolor="#8e8d88", edgecolor=C["dark"], lw=0.7, zorder=3))
ax.text(46, -48.9, "motor", ha="center", va="center", color="white", fontsize=9, zorder=5)
ax.plot([72.5, 88], [-48.9, -48.9], color=C["dark"], lw=1.5, zorder=4)
# air path arrows
arw = dict(arrowstyle="-|>", mutation_scale=14, color="#2a78d6", lw=2.0, zorder=6)
for a, b in [((55, 0), (22, 0)), ((10, 0), (-3, 0)), ((-6, 0), (-12.8, 0)), ((-13.5, 0), (-14.5, -6)), ((-16, -10), (-30, -10))]:
    ax.add_patch(FancyArrowPatch(a, b, **arw))
ax.set_xlim(*XR); ax.set_ylim(*YR); ax.set_aspect("equal")
ax.set_xticks(range(-80, 170, 20)); ax.tick_params(colors=MUTED, labelsize=8); ax.grid(color=GRID, lw=0.5)
for s in ax.spines.values(): s.set_color(GRID)
ax.set_title("Side section through the cylinder axis (half A cut, mm).  Grey = your AEG parts, drawn schematically.", loc="left", fontweight="bold", fontsize=11)
def marker(a, x, y, n):
    a.add_patch(Circle((x, y), 4.2, facecolor="#0b0b0b", edgecolor="none", zorder=8))
    a.text(x, y - 0.2, str(n), color="white", ha="center", va="center", fontsize=9.5, fontweight="bold", zorder=9)
marker(ax, 95.5, -6.0, 1); marker(ax, 120, 15.5, 2); marker(ax, -3.0, 9.5, 3); marker(ax, -23.0, -17.0, 4)
ax.text(-88, 27.5, "launcher head", color=INK2, fontsize=9.5)
ax.text(2, 27.5, "gearbox half A (your geometry, kept; new bolts, bosses, bottom wall, tabs)", color=INK2, fontsize=9.5)
ax.text(-16.5, 13.5, "TPU seal\nsleeve", color=INK2, fontsize=8, ha="right")
ax.add_patch(Circle((-4.6, -16.0), 2.975, facecolor="#f2f2ee", edgecolor=C["dark"], lw=0.8, zorder=7))
ax.annotate("check valve:\n6 mm BB lets air in\nwhile cocking, seals\nduring the shot", xy=(-5.5, -18.5), xytext=(-60, -62),
            color=INK2, fontsize=8.5, arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))

# ---- panel 2: plane through the launch axis and one barrel
ax2 = fig.add_subplot(gs[1, 0])
phi = math.radians(45)
origin = (0, bn.YL, 0); ex = (1, 0, 0); ey = (0, math.sin(phi), math.cos(phi))
XR2, YR2 = (-90, 2), (-46, 46)
imgs = []
for shp, col in [(head, C["head"])] + [(loc * proj, C["w"]) for loc in bn.projectile_locations()]:
    img, xs, ys = raster(shp, origin, ex, ey, XR2, YR2, res=0.12)
    show(ax2, img, xs, ys, col)
# o-rings (8x2) drawn at the groove positions
t = math.radians(bn.THETA)
for sgn in (1, -1):
    for o, d, rad in bn.barrel_axes():
        pass
for k, (o, d, radial) in enumerate(bn.barrel_axes()):
    # only barrels lying in this plane (phi=45 -> radial (0,.707,.707) and phi=225)
    proj_r = radial.Y * ey[1] + radial.Z * ey[2]
    if abs(abs(proj_r) - 1) > 1e-6:
        continue
    for side in (1, -1):
        sc = bn.ORING_GROOVE_X + bn.ORING_GROOVE_W / 2
        cpt = o + d * sc
        nrm = Vector(math.sin(t) * proj_r, -math.cos(t) * 0, 0)
        # perpendicular in plane coords
        px = (cpt.X) ; py = (cpt - Vector(*origin)).dot(Vector(*ey))
        perp = np.array([math.sin(t), math.cos(t) * np.sign(proj_r)]) * side * (bn.ORING_ROOT_D / 2 + 1.0)
        dxy = np.array([-math.cos(t), math.sin(t) * np.sign(proj_r)])
        perp = np.array([-dxy[1], dxy[0]]) * side * (bn.ORING_ROOT_D / 2 + 1.0)
        ax2.add_patch(Circle((px + perp[0], py + perp[1]), 1.0, color="#222222", zorder=5))
ax2.add_patch(FancyArrowPatch((-12, 0), (-15.5, 0), **arw))
for sgn in (1, -1):
    ax2.add_patch(FancyArrowPatch((-15.0, sgn * 3.0), (-19.5, sgn * 7.0), **arw))
    ax2.add_patch(FancyArrowPatch((-48, sgn * 27.0), (-80, sgn * 40.5), arrowstyle="-|>", mutation_scale=14, color="#eb6834", lw=2.0, zorder=6))
ax2.text(-62, 0, "net\nfolded\nin cup", ha="center", va="center", color=INK2, fontsize=9)
marker(ax2, -62, 12, 5)
ax2.text(-88, 43, "weights leave at ~10 m/s,\n22° outward", color=INK2, fontsize=9, va="top")
ax2.text(-40, -40, "8 g weight: printed body + 8 mm steel pin,\nsealed by an 8 x 2 mm O-ring", color=INK2, fontsize=9)
ax2.set_xlim(*XR2); ax2.set_ylim(*YR2); ax2.set_aspect("equal")
ax2.tick_params(colors=MUTED, labelsize=8); ax2.grid(color=GRID, lw=0.5)
for s in ax2.spines.values(): s.set_color(GRID)
ax2.set_title("Launcher head, section along two opposite barrels", loc="left", fontweight="bold", fontsize=11)

# ---- panel 3: text box on what was changed / numbers
ax3 = fig.add_subplot(gs[1, 1]); ax3.axis("off")
txt = ("How a shot works\n"
       " 1  motor turns the gears; the sector gear pulls the\n"
       "    piston back and compresses the spring\n"
       " 2  sector gear lets go: the spring drives the piston\n"
       "    forward and squeezes the air (peak 2-3 bar)\n"
       " 3  air: cylinder -> AEG nozzle -> TPU seal sleeve\n"
       "    (while cocking, air refills through the BB check valve)\n"
       " 4  plenum feeds the 4 barrels, 22 deg outward;\n"
       "    weights leave after ~11 ms at ~10 m/s\n"
       " 5  each weight drags one net corner out of the cup:\n"
       "    70 % open at ~1.5 m, 90 % open at ~2 m\n\n"
       "Predicted with an M120 spring (8 g weights, 1.2 m net):\n"
       "    weight speed       9.3 - 10.0 m/s\n"
       "    shot duration      11 ms after release\n"
       "    peak air pressure  2.4 - 2.9 bar\n"
       "    capture window     1.5 m to 3.6 - 3.8 m\n"
       "    net drop           ~0.4 m at 1.5 m, ~0.7 m at 2 m\n\n"
       "Must-haves for it to work:\n"
       "    BB check valve in the head (without it: no launch)\n"
       "    O-ring on every weight (without it: no launch)\n"
       "    barrel volume <= cylinder volume (~22 cc)")
ax3.text(0.0, 1.0, txt, va="top", family="DejaVu Sans Mono", fontsize=9.3, color=INK, linespacing=1.25)
fig.tight_layout()
fig.savefig(IMG / "how_it_works.png", dpi=125)
print("ok")
