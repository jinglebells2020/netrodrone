"""Chart of the method comparison -> docs/images/method_comparison.png
Run: python analysis/tactics/compare_fig.py   (needs out/compare_methods.csv)"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.ticker import FuncFormatter  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from compare_eval import evaluate_all  # noqa: E402
from methods import SPEEDS  # noqa: E402

IMG = HERE.parents[1] / "docs" / "images"
SURF, INK, INK2, MUTED, GRID, BASE = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
SERIES = [  # fixed categorical order (slots 1-4 of the reference palette); colour follows the method
    ("V2 launcher, fire down from above", "V2 gearbox, fire straight down (3–4 m above)", "#2a78d6"),
    ("Latex launcher, fire down (estimate)", "latex bands, fire down (estimate, 2 m net)", "#eb6834"),
    ("Drape, fixed version", "drape, fixed version (3 m lines)", "#1baf7a"),
    ("V2 launcher, fire forward from above-behind", "V2 gearbox, fire forward from 1–1.8 m above, 2–3 m behind", "#eda100"),
]


def main():
    ev = evaluate_all()
    plt.rcParams.update({"figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
                         "font.family": "DejaVu Sans", "font.size": 10.5, "text.color": INK,
                         "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED})
    fig, axes = plt.subplots(1, 2, figsize=(13.0, 6.4), sharey=True)
    panels = [(0.25, "Automatic aiming (camera + rangefinder), error ±0.25 m"), (0.5, "Manual piloting, error ±0.5 m")]
    for ax, (sigma, title) in zip(axes, panels):
        for key, label, col in SERIES:
            ys = [ev[("strict", sigma, key, u)]["p"] * 100 for u in SPEEDS]
            ax.plot(SPEEDS, ys, color=col, lw=2.0, solid_capstyle="round", solid_joinstyle="round", zorder=3, label=label)
            ax.plot(SPEEDS, ys, "o", ms=6.5, color=col, mec=SURF, mew=1.6, zorder=4)
        ax.set_title(title, loc="left", fontsize=11.5, color=INK, pad=10)
        ax.set_xlim(-0.2, 6.3)
        ax.set_ylim(-3, 103)
        ax.set_xticks(SPEEDS)
        ax.set_xticklabels([f"{u:.0f}\n{u * 3.6:.0f} km/h" if u > 0 else "0\nhover" for u in SPEEDS])
        ax.set_xlabel("target speed (m/s)", color=INK2)
        ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:.0f}%"))
        ax.set_yticks([0, 25, 50, 75, 100])
        ax.grid(axis="y", color=GRID, lw=0.8)
        ax.set_axisbelow(True)
        for sp in ("top", "right", "left"):
            ax.spines[sp].set_visible(False)
        ax.spines["bottom"].set_color(BASE)
        ax.tick_params(length=0)
    axes[0].set_ylabel("chance of a catch per attempt", color=INK2)
    h, lab = axes[0].get_legend_handles_labels()
    fig.legend(h, lab, loc="upper left", bbox_to_anchor=(0.055, 0.935), ncol=2, frameon=False, fontsize=10.5,
               handlelength=1.8, columnspacing=1.6, labelcolor=INK2)
    fig.text(0.055, 0.962, "Chance of catching the target in one attempt", fontsize=14, fontweight="bold", color=INK)
    fig.text(0.055, 0.025,
             "Same simulated target for every method: a 0.25 m drone with its own downwash; your interceptor flies along with it at the same speed.\n"
             "Catch = net spread over it (12+ net points touching, covering at least a 60° patch). Best aim point per method, including aiming slightly ahead.\n"
             "Aiming error is random (normal, per axis). Latex launcher is an estimate, not a designed part. Your original drape is left out: its net hit your own props in every run.",
             fontsize=9, color=MUTED, linespacing=1.5)
    fig.tight_layout(rect=(0.04, 0.12, 1, 0.84))
    fig.savefig(IMG / "method_comparison.png", dpi=130)
    print("wrote", IMG / "method_comparison.png")


if __name__ == "__main__":
    main()
