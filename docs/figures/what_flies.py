"""What gets launched, and the one-way valve ball that stays inside -> docs/images/what_flies_out.png
Run: python docs/figures/what_flies.py   (~3 min)"""
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
from matplotlib.patches import Circle, FancyArrowPatch, Rectangle as MRect

# reuse the exact ray-cast section helpers from the how-it-works figure
src = (Path(__file__).resolve().parent / "how_it_works.py").read_text()
exec(src[src.index("def raster"):src.index("A, B = bn.build_gearbox()")])

SURF, INK, INK2, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9"
GREEN, GOLD, BLUE, RED, GREY = "#5fa04e", "#e3b23c", "#2a78d6", "#d03b3b", "#b9b8b2"
plt.rcParams.update({"figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF, "font.size": 10.5})

head = bn.build_head(); proj = bn.build_projectile()
fig = plt.figure(figsize=(15.5, 8.6))
gs = fig.add_gridspec(2, 2, width_ratios=[1.45, 1], height_ratios=[1, 1])

# ---------------- A: what flies out
ax = fig.add_subplot(gs[:, 0])
phi = math.radians(45)
origin = (0, bn.YL, 0); ex = (1, 0, 0); ey = (0, math.sin(phi), math.cos(phi))
XR, YR = (-122, 16), (-57, 57)
img, xs, ys = raster(head, origin, ex, ey, XR, YR, res=0.15); show(ax, img, xs, ys, GREEN)
for loc in bn.projectile_locations():
    img, xs, ys = raster(loc * proj, origin, ex, ey, XR, YR, res=0.15); show(ax, img, xs, ys, GOLD, z=3)
# folded net in the cup (schematic zig-zag)
zz = np.linspace(-38, -74, 14)
ax.plot(zz, np.array([1 if i % 2 else -1 for i in range(14)]) * np.linspace(6, 16, 14), color="#6d6c68", lw=1.3, zorder=4)
# tethers from weight noses to the net
for sgn in (1, -1):
    ax.plot([-44, -80.5, -72], [sgn * 23.0, sgn * 37.5, sgn * 17.5], color="#6d6c68", lw=1.1, ls=(0, (3, 2)), zorder=4)
# air push arrows behind the weights
arw = dict(arrowstyle="-|>", mutation_scale=16, color=BLUE, lw=2.2, zorder=6)
for sgn in (1, -1):
    ax.add_patch(FancyArrowPatch((-14.5, sgn * 3.0), (-19.5, sgn * 7.5), **arw))
ax.add_patch(FancyArrowPatch((13.0, 0), (-12.5, 0), **arw))
# flight arrows
fly = dict(arrowstyle="-|>", mutation_scale=18, color=GOLD, lw=3.0, zorder=6)
for sgn in (1, -1):
    ax.add_patch(FancyArrowPatch((-86, sgn * 42.5), (-114, sgn * 53.8), **fly))
ax.add_patch(FancyArrowPatch((-78, 0), (-114, 0), arrowstyle="-|>", mutation_scale=18, color="#6d6c68", lw=3.0, zorder=6))
ax.text(-120, 36, "FLIES OUT:\n4 net weights, 8 g each\n(printed slug + steel pin)\nabout 10 m/s", color=INK, fontsize=10.5, va="top", fontweight="bold")
ax.text(-120, 5, "FLIES OUT:\nthe net,\npulled by the\n4 weights", color=INK, fontsize=10.5, va="bottom", fontweight="bold")
ax.text(8.0, 3.0, "air from\nthe gearbox", color=BLUE, fontsize=10, ha="center", va="bottom")
ax.text(-62, -27, "tether (10 cm line)\nweight -> net corner", color=INK2, fontsize=9, ha="center", va="top")
ax.text(-30, -47, "weight sits here before the shot\n(O-ring seals it in the barrel)", color=INK2, fontsize=9.5, ha="center")
ax.annotate("", xy=(-30, -22), xytext=(-30, -40.5), arrowprops=dict(arrowstyle="-", color=MUTED, lw=0.8))
ax.set_xlim(*XR); ax.set_ylim(*YR); ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
for s in ax.spines.values(): s.set_visible(False)
ax.set_title("What gets launched (launcher head cut in half through two barrels)", loc="left", fontweight="bold", fontsize=12)

# ---------------- B/C: the valve ball, two moments
v = bn.VALVE
def valve_panel(ax, ball_x, title, state):
    o2 = (0, 0, 0)
    XR2, YR2 = (-17.5, 7.5), (-28.0, -6.0)
    img, xs, ys = raster(head, o2, (1, 0, 0), (0, 1, 0), XR2, YR2, res=0.05); show(ax, img, xs, ys, GREEN)
    ax.add_patch(MRect((0.0, -28), 7.5, 22, facecolor=GREY, edgecolor="none", zorder=1))
    ax.text(3.8, -7.3, "gearbox\nfront", ha="center", va="top", fontsize=9, color=INK2)
    ax.add_patch(Circle((ball_x, v["y"]), 2.975, facecolor="#fafaf7", edgecolor=INK, lw=1.4, zorder=6))
    ax.text(ball_x, v["y"], "BB", ha="center", va="center", fontsize=8.5, zorder=7)
    if state == "open":
        a = dict(arrowstyle="-|>", mutation_scale=13, color=BLUE, lw=2.0, zorder=8)
        ax.add_patch(FancyArrowPatch((-0.6, -26.5), (-0.6, -19.0), **a))
        ax.add_patch(FancyArrowPatch((-1.2, -16.0), (-4.2, -16.0), **a))
        ax.add_patch(FancyArrowPatch((-5.5, -13.4), (-12.6, -13.4), **a))
        ax.add_patch(FancyArrowPatch((-13.8, -16.0), (-15.8, -13.0), **a))
        ax.text(-16.8, -26.8, "outside air comes in\n(through the groove\nunder the head)", fontsize=9.3, color=BLUE, va="bottom", zorder=10, bbox=dict(facecolor="white", edgecolor="none", alpha=0.92, pad=2.5))
    else:
        a = dict(arrowstyle="-|>", mutation_scale=13, color=RED, lw=2.0, zorder=8)
        ax.add_patch(FancyArrowPatch((-15.5, -12.5), (-8.5, -15.0), **a))
        ax.plot([-1.6, 0.6], [-17.4, -14.6], color=RED, lw=2.4, zorder=9); ax.plot([-1.6, 0.6], [-14.6, -17.4], color=RED, lw=2.4, zorder=9)
        ax.text(-16.8, -26.8, "air pushes the BB into\nthe 4 mm hole: closed.\nAll air goes to the weights", fontsize=9.3, color=RED, va="bottom", zorder=10, bbox=dict(facecolor="white", edgecolor="none", alpha=0.92, pad=2.5))
    ax.text(-16.8, -7.0, "to the weights\n(plenum)", fontsize=9, color=INK2, va="top", zorder=10, bbox=dict(facecolor="white", edgecolor="none", alpha=0.92, pad=2.5))
    ax.set_xlim(*XR2); ax.set_ylim(*YR2); ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values(): s.set_visible(False)
    ax.set_title(title, loc="left", fontweight="bold", fontsize=11)

valve_panel(fig.add_subplot(gs[0, 1]), -8.4, "Valve ball, while the gears pull the piston back", "open")
valve_panel(fig.add_subplot(gs[1, 1]), -4.6, "Valve ball, during the shot", "closed")
fig.text(0.635, 0.012, "The BB never leaves the head. Its little room has a 4 mm hole on one side\n"
         "and a 3.5 mm slot on the other, both too small for a 6 mm ball.\n"
         "It is only a one-way door for air.", fontsize=9.5, color=INK2)
fig.tight_layout(rect=(0, 0.06, 1, 1))
fig.savefig(IMG / "what_flies_out.png", dpi=120)
print("ok")
