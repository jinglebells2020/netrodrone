"""Side-view snapshots of the drape tactic (your version vs the fixed one) -> docs/images/drape_tactic.png
Run drape_run.py first (it writes out/drape_results.pkl)."""
import pickle
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

HERE = Path(__file__).resolve().parent
IMG = HERE.parents[1] / "docs" / "images"
with open(HERE / "out" / "drape_results.pkl", "rb") as f:
    D = pickle.load(f)
SURF, INK, INK2, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9"
plt.rcParams.update({"figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF, "font.size": 10})
n = 13
E = []
for i in range(n):
    for j in range(n):
        a = i * n + j
        if j + 1 < n: E.append((a, a + 1))
        if i + 1 < n: E.append((a, a + n))
E = np.array(E)
rows = [("A1 your idea: 1 m tethers, dive (2.5 fwd, 3 down) to 0.8 m", "Your version: dive down holding the front corners (1 m tethers)"),
        ("C2 = C1 + net pouch 0.5 m below props (boom)", "Fixed version: stay 3 m up, 3 m tethers, net pouch on a boom, back weights pushed down")]
times = [0.2, 0.4, 0.6, 0.8]
fig, axs = plt.subplots(2, 4, figsize=(16, 9.0))
fig.subplots_adjust(left=0.01, right=0.99, top=0.9, bottom=0.07, wspace=0.04, hspace=0.32)
for r, (key, title) in enumerate(rows):
    res, snaps = D[key]
    for c, tt in enumerate(times):
        ax = axs[r, c]
        t, P, WB, WF, I, rel = min(snaps, key=lambda s: abs(s[0] - tt))
        ax.add_patch(Rectangle((-0.3, -1.5), 0.6, 3.0, color="#cde2fb", alpha=0.6, lw=0, zorder=0))
        ax.add_patch(Circle((0, 0), 0.25, facecolor="#8c8b86", edgecolor=INK, lw=0.8, zorder=3))
        for a, b in E:
            ax.plot([P[a, 0], P[b, 0]], [P[a, 2], P[b, 2]], color="#5d5c58", lw=0.45, zorder=2)
        ax.scatter(WB[:, 0], WB[:, 2], s=28, color="#eb6834", zorder=5, label="back weights (released)")
        ax.scatter(WF[:, 0], WF[:, 2], s=28, color="#1baf7a", zorder=5, label="front weights")
        if not rel:
            for k in range(2):
                ax.plot([I[0] + 0.25, WF[k, 0]], [I[2] - 0.05, WF[k, 2]], color="#1baf7a", lw=0.9, ls=(0, (3, 2)), zorder=4)
        # interceptor body + prop keep-out zone
        allp = np.vstack([P, WB])
        dr = np.hypot(allp[:, 0] - I[0], allp[:, 1] - I[1]); dz = np.abs(allp[:, 2] - I[2])
        hit = ((dr < 0.30) & (dz < 0.12)).any()
        ax.add_patch(Rectangle((I[0] - 0.3, I[2] - 0.04), 0.6, 0.08, color="#d03b3b" if hit else "#2a78d6", zorder=6))
        ax.text(I[0], I[2] + 0.18, "NET IN PROPS" if hit else "your drone", ha="center", fontsize=8.5,
                color="#d03b3b" if hit else "#2a78d6", fontweight="bold" if hit else "normal", zorder=7)
        ax.set_xlim(-2.4, 3.4); ax.set_ylim(-1.6, 3.6); ax.set_aspect("equal")
        ax.set_xticks([]); ax.set_yticks([]); ax.grid(False)
        for sp in ax.spines.values(): sp.set_color(GRID)
        ax.set_title(f"t = {t:.1f} s", fontsize=10, color=INK2)
        if c == 0:
            fig.text(0.012, 0.955 if r == 0 else 0.495, title, fontsize=12, fontweight="bold", va="center")
    axs[r, 0].text(0.0, -0.42, "target", ha="center", fontsize=8.5, color=INK2, zorder=7)
axs[0, 3].legend(loc="upper right", fontsize=8.5, frameon=True, framealpha=0.9)
fig.text(0.01, 0.01, "Side view, target hovering. Light blue = target's downwash (pulls the net into its rotors). "
         "Model shows the net reaching the target; real capture happens when the lines touch its spinning props.",
         fontsize=9, color=INK2)
fig.savefig(IMG / "drape_tactic.png", dpi=110)
print("ok")
