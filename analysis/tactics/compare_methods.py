"""Sweep every capture method over target speed x aim offset, same target, same air, same 'caught' rule.
Run: python analysis/tactics/compare_methods.py   (~12 min on 4 cores, 6704 runs)
Writes out/compare_methods.csv (one row per run). Then run compare_eval.py and compare_fig.py."""
import sys
import time
from pathlib import Path
from multiprocessing import Pool

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from capture import Shot, run  # noqa: E402
from methods import VARIANTS, SPEEDS, write_rows  # noqa: E402

CSV = HERE / "out" / "compare_methods.csv"


def job(args):
    name, u, ia, ic = args
    base, place, centre, A, C = VARIANTS[name]
    a = centre(u) + A[ia]
    c = C[ic]
    r = run(Shot(**dict(base, start=place(a, c), wind=(-u, 0.0, 0.0), seed=1 + 1000 * ia + ic, post_catch=0.10)))
    return (name, u, ia, ic, a, c, r["caught"], r["t_catch"], r["min_clearance"], r["max_tether_N"],
            r["contacts_max"], r["R_min"], r["open_at_catch"])


if __name__ == "__main__":
    jobs = [(nm, u, ia, ic) for nm, v in VARIANTS.items() for u in SPEEDS for ia in range(len(v[3])) for ic in range(len(v[4]))]
    jobs.sort(key=lambda j: (j[2] * 31 + j[3] * 7 + int(j[1])) % 97)   # mix slow (drape) and fast runs
    print(len(jobs), "runs", flush=True)
    t0 = time.time()
    out = []
    with Pool(4) as p:
        for i, r in enumerate(p.imap_unordered(job, jobs, chunksize=16)):
            out.append(r)
            if (i + 1) % 1000 == 0:
                print(f"{i + 1}/{len(jobs)}  {time.time() - t0:.0f}s", flush=True)
    CSV.parent.mkdir(exist_ok=True)
    write_rows(out, CSV)
    print(f"done {time.time() - t0:.0f}s -> {CSV}", flush=True)
