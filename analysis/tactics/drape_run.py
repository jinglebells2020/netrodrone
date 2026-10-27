"""The 'drape' tactic: your version (dive holding the front corners) and the fixed version.
Run: python analysis/tactics/drape_run.py   (~1 min on 4 cores)
Writes out/drape_results.pkl (snapshots for drape_fig.py; not committed) and prints the summary."""
import sys
import pickle
from pathlib import Path
from multiprocessing import Pool

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from drape import Drape, run  # noqa: E402

OUT = HERE / "out"

SC = {
    "A1 your idea: 1 m tethers, dive (2.5 fwd, 3 down) to 0.8 m": Drape(),
    "A2 same, target moving 2 m/s": Drape(wind=(-2.0, 0, 0)),
    "A3 speedy drop (2 fwd, 5 down) to 0.5 m": Drape(v_cmd=(2.0, 0, -5.0), h_min=0.5),
    "B1 refined: 2 m tethers, shallow pass (3 fwd, 1.5 down) to 1.6 m, back weights kicked 3 m/s": Drape(tether=2.0, v_cmd=(3.0, 0, -1.5), h_min=1.6, back_kick=3.0),
    "B2 refined, target moving 2 m/s": Drape(tether=2.0, v_cmd=(3.0, 0, -1.5), h_min=1.6, back_kick=3.0, wind=(-2.0, 0, 0)),
    "B3 refined, target moving 4 m/s": Drape(tether=2.0, v_cmd=(3.0, 0, -1.5), h_min=1.6, back_kick=3.0, wind=(-4.0, 0, 0)),
    "C1 3 m tethers, NO dive (level pass 3 m above), back weights kicked 4 m/s": Drape(tether=3.0, v_cmd=(3.0, 0, -0.0001), h_min=2.99, climb_after=0.6, v_climb=1.0, back_kick=4.0),
    "C2 = C1 + net pouch 0.5 m below props (boom)": Drape(tether=3.0, v_cmd=(3.0, 0, -0.0001), h_min=2.99, climb_after=0.6, v_climb=1.0, back_kick=4.0, pouch_drop=0.5),
    "C3 = C2, target moving 3 m/s": Drape(tether=3.0, v_cmd=(3.0, 0, -0.0001), h_min=2.99, climb_after=0.6, v_climb=1.0, back_kick=4.0, pouch_drop=0.5, wind=(-3.0, 0, 0)),
}


def job(k):
    res, snaps = run(SC[k], snap_every=0.1)
    return k, res, snaps


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    with Pool(4) as p:
        out = p.map(job, list(SC))
    with open(OUT / "drape_results.pkl", "wb") as f:
        pickle.dump({k: (r, s) for k, r, s in out}, f)
    for k, r, s in out:
        print(f"{k}\n   first contact {r['t_contact'] if r['t_contact'] is None else round(r['t_contact'], 2)} s, "
              f"front released {None if r['t_release'] is None else round(r['t_release'], 2)} s, max nodes touching {r['contacts_max']}, "
              f"closest net-to-own-props {r['min_clearance']:+.2f} m at t={r['t_minclear']:.2f}s ({r['phase_minclear']}), "
              f"max tether pull {r['max_tether_N']:.1f} N, lowest height {r['h_min_reached']:.2f} m")
