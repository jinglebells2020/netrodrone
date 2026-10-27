"""
Regenerates every number and figure in the README.

    python analysis/run_analysis.py          (~3 min on 4 cores)

Outputs
    analysis/RESULTS.md                     tables
    docs/images/fig_launch_trace.png        pressure + speed during one shot (M120)
    docs/images/fig_weight_speed.png        weight speed vs weight mass for M100/M120/M150
    docs/images/fig_net_opening.png         simulated net opening, front view snapshots + opening vs distance
    docs/images/fig_capture_window.png      distance window where the net can catch a drone
"""
import os
import sys
from dataclasses import replace
from multiprocessing import Pool

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from ballistics import CAL_A, CAL_B, simulate, spring_k, net_launcher, PATM  # noqa: E402
from netflight import NetShot, simulate_net, summarize  # noqa: E402

IMG = os.path.join(HERE, "..", "docs", "images")
os.makedirs(IMG, exist_ok=True)

# chart styling (reference palette: categorical slots 1-3, light surface, recessive chrome)
SURF, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
SPRING_COLORS = {100: "#2a78d6", 120: "#eb6834", 150: "#1baf7a"}
plt.rcParams.update({
    "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
    "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED,
    "text.color": INK, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6, "grid.linestyle": "-",
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 10, "axes.titlesize": 11,
    "axes.titleweight": "bold", "axes.titlecolor": INK, "legend.frameon": False, "lines.linewidth": 1.8,
})

M_WEIGHT = 0.008      # design weight mass
THETA = 22.0
NET_SIDE = 1.2
NET_MASS = 0.009 * NET_SIDE ** 2 + 0.002   # ~9 g/m2 braided PE mesh + perimeter line


def weight_speeds():
    """v0 for each spring/calibration/weight mass"""
    out = {}
    ks = {(c, m): spring_k(m, cal) for c, cal in (("A", CAL_A), ("B", CAL_B)) for m in (100, 120, 150)}
    for (c, m), k in ks.items():
        cal = CAL_A if c == "A" else CAL_B
        for mw in (0.006, 0.008, 0.010, 0.012):
            r = simulate(net_launcher(cal, k, mw), t_end=0.12)
            out[(c, m, mw)] = r
    return ks, out


def _net_job(args):
    key, v0, theta, side, snaps = args
    s = NetShot(side=side, m_net=0.009 * side ** 2 + 0.002, m_w=M_WEIGHT, v0=v0, theta=theta, n=11, t_end=0.8)
    if snaps:
        rec, sn = simulate_net(s, snap_times=snaps)
    else:
        rec, sn = simulate_net(s), None
    return key, summarize(s, rec), [(r["t"], r["x_net"], r["area"] / side ** 2, r["vx_net"], r["z_net"]) for r in rec], sn


def fig_launch_trace(ks):
    g = net_launcher(CAL_B, ks[("B", 120)], M_WEIGHT)
    r = simulate(g, t_end=0.12, trace=True)
    tr = r["trace"]
    t = tr[:, 0] * 1e3
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8.5, 6.2), sharex=True)
    c_cyl, c_man = "#2a78d6", "#eb6834"
    a1.plot(t, (tr[:, 3] - PATM) / 1e5, color=c_cyl, label="cylinder (behind the piston head)")
    a1.plot(t, (tr[:, 4] - PATM) / 1e5, color=c_man, label="launcher plenum / barrels")
    a1.set_ylabel("gauge pressure (bar)")
    a1.set_title("One shot, M120 spring, 4 x 8 g weights (time from sector-gear release)", loc="left")
    a1.legend(loc="upper left")
    a2.plot(t, tr[:, 2], color=c_cyl, label="piston")
    a2.plot(t, tr[:, 5], color=c_man, label="net weights")
    a2.set_ylabel("speed (m/s)")
    a2.set_xlabel("time (ms)")
    a2.legend(loc="upper left")
    te = r["t_exit"] * 1e3
    a2.text(te + 0.2, r["v_exit"], f"weights leave the barrels:\n{r['v_exit']:.1f} m/s at {te:.1f} ms",
            color=INK2, fontsize=9, va="center")
    for a in (a1, a2):
        a.axvline(te, color=AXIS, lw=0.8)
    a2.set_xlim(0, te + 4.2)
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "fig_launch_trace.png"), dpi=130)
    plt.close(fig)
    return r


def fig_weight_speed(speeds):
    fig, ax = plt.subplots(figsize=(8.5, 4.6))
    mws = np.array([0.006, 0.008, 0.010, 0.012])
    for m in (100, 120, 150):
        va = np.array([speeds[("A", m, mw)]["v_exit"] for mw in mws])
        vb = np.array([speeds[("B", m, mw)]["v_exit"] for mw in mws])
        col = SPRING_COLORS[m]
        ax.fill_between(mws * 1e3, va, vb, color=col, alpha=0.18, lw=0)
        ax.plot(mws * 1e3, (va + vb) / 2, color=col, marker="o", ms=4.5, label=f"M{m} spring")
        ax.text(12.15, (va[-1] + vb[-1]) / 2, f"M{m}", color=INK2, va="center", fontsize=9)
    ax.axvline(8, color=AXIS, lw=0.8)
    ax.text(8.08, ax.get_ylim()[0] + 0.3 if False else 6.2, "design weight 8 g", color=MUTED, fontsize=8.5)
    ax.set_xlabel("mass of each net weight (g)")
    ax.set_ylabel("muzzle speed (m/s)")
    ax.set_title("Weight speed leaving the barrels (band = conservative to expected spring model)", loc="left")
    ax.set_xlim(5.6, 12.8)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "fig_weight_speed.png"), dpi=130)
    plt.close(fig)


def fig_net_opening(snapdata, curves):
    times = sorted(snapdata[1].keys())
    fig = plt.figure(figsize=(12, 6.4))
    gs = fig.add_gridspec(2, len(times), height_ratios=[1.15, 1])
    summary, rec = snapdata[0], snapdata[2]
    xs_at = {round(t, 3): x for t, x, *_ in rec}
    for i, t in enumerate(times):
        ax = fig.add_subplot(gs[0, i])
        P, W, E, corners = snapdata[1][t]
        for a, b in E:
            ax.plot([P[a, 1], P[b, 1]], [P[a, 2], P[b, 2]], color="#7a7974", lw=0.5)
        for k in range(4):
            ax.plot([W[k, 1], P[corners[k], 1]], [W[k, 2], P[corners[k], 2]], color=MUTED, lw=0.6)
        ax.scatter(W[:, 1], W[:, 2], s=22, color="#eb6834", zorder=5)
        yc, zc = np.mean(P[:, 1]), np.mean(P[:, 2])
        ax.set_xlim(yc - 0.95, yc + 0.95); ax.set_ylim(zc - 0.95, zc + 0.95)
        ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
        xm = np.mean(P[:, 0])
        ax.set_title(f"t = {t:.2f} s\n{xm:.1f} m out, {-zc*100:.0f} cm drop", fontsize=9.5, fontweight="normal", color=INK2)
        ax.grid(False)
        for sp in ax.spines.values():
            sp.set_visible(True); sp.set_color(GRID)
    ax2 = fig.add_subplot(gs[1, :])
    for m in (100, 120, 150):
        cur = np.array(curves[m])
        ax2.plot(cur[:, 1], cur[:, 2] * 100, color=SPRING_COLORS[m], label=f"M{m} spring")
        j = np.argmax(cur[:, 2])
        ax2.text(cur[-1, 1] + 0.05, cur[-1, 2] * 100, f"M{m}", color=INK2, fontsize=9, va="center")
    ax2.axhline(70, color=AXIS, lw=0.8)
    ax2.text(0.05, 72, "70 % open", color=MUTED, fontsize=8.5)
    ax2.set_xlim(0, 5.0); ax2.set_ylim(0, 105)
    ax2.set_xlabel("distance of net centre from launcher (m)")
    ax2.set_ylabel("net opening (% of full area)")
    ax2.legend(loc="lower right")
    fig.suptitle(f"Simulated net shot: {NET_SIDE:.1f} m net, 4 x 8 g weights, {THETA:.0f}° spread "
                 f"(top: front view, M120 spring, conservative model)", x=0.01, ha="left", fontsize=11, fontweight="bold")
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "fig_net_opening.png"), dpi=130)
    plt.close(fig)


def fig_capture_window(win):
    fig, ax = plt.subplots(figsize=(8.5, 3.6))
    rows = []
    for m in (150, 120, 100):
        for c in ("B", "A"):
            rows.append((m, c))
    for i, (m, c) in enumerate(rows):
        w = win[(m, c)]
        if not w["capture_window_x"]:
            continue
        x0, x1 = w["capture_window_x"]
        ax.barh(i, x1 - x0, left=x0, height=0.6, color=SPRING_COLORS[m], alpha=1.0 if c == "B" else 0.55)
        o90 = w["open90"]
        if o90:
            ax.plot([o90[0], o90[0]], [i - 0.3, i + 0.3], color=INK, lw=1.2)
        ax.text(x1 + 0.06, i, f"{x0:.1f}-{x1:.1f} m", va="center", color=INK2, fontsize=9)
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels([f"M{m}  {'expected' if c == 'B' else 'conservative'}" for m, c in rows])
    ax.set_xlim(0, 5.6)
    ax.set_axisbelow(True)
    ax.set_xlabel("distance from launcher (m)")
    ax.set_title("Capture window: net >= 70 % open and still moving > 3 m/s   (| = 90 % open)", loc="left")
    ax.grid(axis="y", visible=False)
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, "fig_capture_window.png"), dpi=130)
    plt.close(fig)


def main():
    ks, speeds = weight_speeds()
    trace = fig_launch_trace(ks)
    fig_weight_speed(speeds)
    jobs = []
    snaps = (0.05, 0.10, 0.15, 0.20, 0.30, 0.40)
    for m in (100, 120, 150):
        for c in ("A", "B"):
            v0 = speeds[(c, m, M_WEIGHT)]["v_exit"]
            jobs.append(((m, c, THETA), v0, THETA, NET_SIDE, snaps if (m == 120 and c == "A") else ()))
    for th in (18, 26, 30):
        v0 = speeds[("A", 120, M_WEIGHT)]["v_exit"]
        jobs.append(((120, "A", th), v0, th, NET_SIDE, ()))
    with Pool(4) as p:
        res = p.map(_net_job, jobs)
    win = {}
    curves = {}
    snapdata = None
    for key, summ, rec, sn in res:
        m, c, th = key
        if th == THETA:
            win[(m, c)] = summ
            if c == "A":
                curves[m] = [(t, x, a) for t, x, a, v, z in rec if t <= 0.6]
        if sn:
            snapdata = (summ, sn, rec)
    angle = {key[2]: summ for key, summ, rec, sn in res if key[0] == 120 and key[1] == "A"}
    fig_net_opening(snapdata, curves)
    fig_capture_window(win)

    # ---- tables
    L = []
    L.append("# Simulation results\n")
    L.append("Generated by `analysis/run_analysis.py`. Design point: 4 barrels, bore 12 mm, 42 mm weight travel, "
             "stock AEG nozzle (4 mm bore), O-ring sealed weights, 22° spread, 1.2 m net.\n")
    L.append("## Spring calibration (from the AEG rating: M120 = ~120 m/s with a 0.20 g BB)\n")
    L.append("| spring | model | rate k (N/mm) | force cocked (N) | stored energy (J) |")
    L.append("|---|---|---|---|---|")
    for m in (100, 120, 150):
        for c, cal in (("A", CAL_A), ("B", CAL_B)):
            g = replace(cal, k=ks[(c, m)])
            L.append(f"| M{m} | {'conservative' if c == 'A' else 'expected'} | {g.k/1000:.2f} | {g.spring_force(0):.0f} | {g.spring_energy:.2f} |")
    L.append("\n## Weight speed leaving the barrels (m/s), conservative - expected\n")
    L.append("| spring | 6 g | 8 g (design) | 10 g | 12 g | energy to weights @ 8 g (J) | peak pressure @ 8 g (bar) |")
    L.append("|---|---|---|---|---|---|---|")
    for m in (100, 120, 150):
        cells = []
        for mw in (0.006, 0.008, 0.010, 0.012):
            cells.append(f"{speeds[('A', m, mw)]['v_exit']:.1f} - {speeds[('B', m, mw)]['v_exit']:.1f}")
        e = f"{speeds[('A', m, M_WEIGHT)]['E_proj']:.2f} - {speeds[('B', m, M_WEIGHT)]['E_proj']:.2f}"
        p = f"{speeds[('A', m, M_WEIGHT)]['peak_p1_bar']:.1f} - {speeds[('B', m, M_WEIGHT)]['peak_p1_bar']:.1f}"
        L.append(f"| M{m} | " + " | ".join(cells) + f" | {e} | {p} |")
    L.append(f"\nShot duration (release to weights out): {trace['t_exit']*1e3:.1f} ms (M120). "
             f"Piston hits the cylinder head at <= {max((speeds[k]['impact_v'] or 0) for k in speeds):.1f} m/s "
             "(the air in front of it cushions the stop).\n")
    L.append("## Net flight (1.2 m net, 4 x 8 g, 22°)\n")
    L.append("| spring | model | weight speed (m/s) | 70 % open at | 90 % open at | capture window |")
    L.append("|---|---|---|---|---|---|")
    for m in (100, 120, 150):
        for c in ("A", "B"):
            w = win[(m, c)]
            o70 = f"{w['open70'][0]:.1f} m" if w["open70"] else "-"
            o90 = f"{w['open90'][0]:.1f} m" if w["open90"] else "-"
            cw = f"{w['capture_window_x'][0]:.1f} - {w['capture_window_x'][1]:.1f} m" if w["capture_window_x"] else "none"
            L.append(f"| M{m} | {'conservative' if c == 'A' else 'expected'} | {speeds[(c, m, M_WEIGHT)]['v_exit']:.1f} | {o70} | {o90} | {cw} |")
    L.append("\n## What breaks it, what doesn't (M120, 8 g weights)\n")
    L.append("| change from the design | weight speed (m/s) | change |")
    L.append("|---|---|---|")
    area = lambda d_mm: np.pi / 4 * (d_mm * 1e-3) ** 2
    base = net_launcher(CAL_A, ks[("A", 120)], M_WEIGHT)
    v0 = simulate(base, t_end=0.12)["v_exit"]
    cases = [
        ("design point (check valve + 1 mm bleed, O-rings, 4 mm nozzle)", {}),
        ("**no check valve** (system sealed between shots)", dict(valve_area=0.0)),
        ("no check valve, but a vented AEG piston head (~2 mm2)", dict(valve_area=0.0, pvent_area=2e-6)),
        ("**no O-rings on the weights** (0.1 mm printed gap)", dict(leak_area=4 * 3.8e-6)),
        ("worn / dry O-rings (1 mm2 leak each)", dict(leak_area=4 * 1.0e-6)),
        ("high O-ring friction (3 N each)", dict(f_slide=3.0)),
        ("nozzle bore 3.5 mm", dict(port_d=3.5e-3)),
        ("nozzle bore 5.0 mm", dict(port_d=5.0e-3)),
        ("bleed hole 1.5 mm", dict(bleed_area=area(1.5))),
        ("no bleed hole", dict(bleed_area=0.0)),
        ("fast cocking (15 ms)", dict(cock_time=0.015)),
        ("slow cocking (40 ms)", dict(cock_time=0.040)),
        ("weight travel 36 mm", dict(barrel_len=0.036)),
        ("weight travel 48 mm", dict(barrel_len=0.048)),
        ("4 x 14 mm bores, 100 mm travel (barrels too big)", dict(proj_d=14e-3, barrel_len=0.100)),
    ]
    for nm, kw in cases:
        r = simulate(replace(base, **kw), t_end=0.15)
        v = r["v_exit"] or 0.0
        tag = f"{(v / v0 - 1) * 100:+.0f} %" if v > 0 else "**weights do not leave the barrels**"
        L.append(f"| {nm} | {v:.1f} | {tag} |")
    L.append("\n## Spread angle (M120, conservative)\n")
    L.append("| half-angle | 90 % open at | capture window |")
    L.append("|---|---|---|")
    for th in sorted(angle):
        w = angle[th]
        o90 = f"{w['open90'][0]:.1f} m" if w["open90"] else "not reached"
        cw = f"{w['capture_window_x'][0]:.1f} - {w['capture_window_x'][1]:.1f} m" if w["capture_window_x"] else "none"
        L.append(f"| {th:g}° | {o90} | {cw} |")
    with open(os.path.join(HERE, "RESULTS.md"), "w") as f:
        f.write("\n".join(L) + "\n")
    print("\n".join(L))


if __name__ == "__main__":
    main()
