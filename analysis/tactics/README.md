# Tactics studies

How to use the launcher on a drone, and which capture method works best. The findings are
written up in [docs/RESEARCH.md](../../docs/RESEARCH.md). The head-to-head numbers are in
[RESULTS.md](RESULTS.md).

All scripts run from the repo root with the packages in `requirements.txt`.

| script | what it answers | time (4 cores) |
|---|---|---|
| `airspeed.py` | firing forward while the drone flies at 0–12 m/s, and firing straight down from a hover | ~1 min |
| `down_strike.py` | firing down while both drones drift sideways (how far the net lags), plus the latex-band estimate | ~1 min |
| `drape.py`, `drape_run.py` | the drape tactic (your version and the fixed one); numpy model | ~1 min |
| `drape_fig.py` | side-view snapshots → `docs/images/drape_tactic.png` (run `drape_run.py` first) | seconds |
| `fast_target.py` | what changes at 150 km/h | ~1 min |
| `capture.py` | one fast (numba) model for every method: same target, same air, same "caught" rule | – |
| `validate_capture.py` | checks `capture.py` against `drape.py` and `netflight.py` | ~1 min |
| `methods.py` | the methods compared and the aim-offset grid for each | – |
| `compare_methods.py` | the sweep: every method × target speed × aim offset (6,704 runs) → `out/compare_methods.csv` | ~12 min |
| `compare_eval.py` | chance of a catch per attempt for a random aiming error → `RESULTS.md` | seconds |
| `compare_fig.py` | the chart → `docs/images/method_comparison.png` | seconds |
| `diag_catch.py` | inside single catches: how spread out the net is when it lands (why the strict rule) | seconds |

```bash
python analysis/tactics/compare_methods.py    # optional: out/compare_methods.csv is already committed
python analysis/tactics/compare_eval.py
python analysis/tactics/compare_fig.py
```

## How the comparison works

- **Target:** a 0.25 m sphere hovering at the origin, with a 7 m/s rotor downwash column
  that pulls nearby lines down into it.
- **Moving target:** handled in the target's own frame, so the air moves at −u. Your interceptor
  shadows the target, so it is stationary in this frame when it fires. In the drape it flies a
  set pass over the target.
- **Net:** the same mass-spring net as `netflight.py`, with mesh points about 0.12 m apart for
  every net size.
- **Caught (strict):** 12 or more net points touch the target, and in the next 0.1 s the net
  spreads over at least a 60° patch of it. A collapsed bundle of line landing on the target
  doesn't count. The loose rule (12 points touching) is reported too.
- **Aiming:** each run starts the interceptor at an offset from the ideal spot, on a 0.15 m
  grid. `compare_eval.py` weights the runs by a random aiming error (normal; ±0.25 m for
  automatic aiming, ±0.5 m manual) around the best aim point. The best point includes aiming
  slightly ahead, and the better firing height or distance.
- **Not modelled:** props wrapping the lines, the target dodging, gusts.
