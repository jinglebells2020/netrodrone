# Roof turret studies

A fixed turret on the building: a sensor finds an incoming drone, a pan-tilt head points a net launcher at where the
drone will be, and the net is thrown into its path. These scripts size the launcher and work out how good the sensor,
the head and the fire control have to be. Results are in [RESULTS.md](RESULTS.md). Nothing here is built hardware yet.

All scripts run from the repo root with the packages in `requirements.txt`.

| script | what it answers | time (4 cores) |
|---|---|---|
| `classes.py` | the launcher sizes compared (net, weights, speed, spread) | – |
| `envelope.py` | how each net flies and where it can catch → `out/net_<class>.json` (committed) | 4–20 min |
| `pneumatic.py` | how much compressed air one shot needs | ~1 min |
| `check_oblique.py` | checks the quick shot-scoring rule against the full net model | ~1 min |
| `engage.py` | the whole engagement, 1,000 times per case → `RESULTS.md` | ~3 min |
| `netlab.py` | net model for any number of weights, any net outline, a sideways ('spiral') launch and a pushed-out net; checks itself against `capture.py` | ~10 s |
| `launch_variants.py` | launcher ideas compared at equal energy → [VARIANTS.md](VARIANTS.md) | ~4 min |

The fire control itself is in [`turret/firecontrol.py`](../../turret/firecontrol.py): the tracker, the head model, the
choice of meeting point and firing time, and the shot-scoring rule. It has no hardware in it, so the same code can run on
the real turret, first in "virtual fire" mode (track real drones, log the shots it would have taken, score them).

## How the engagement model works

- **Drone:** flies a straight 20° dive at a point on the building, at 20–40 m/s, from a random direction. It can also
  weave from side to side once a second.
- **Sensor:** gives direction and range a set number of times a second, with noise and delay. "Camera only" guesses
  range from the size of the drone in the picture, so its range is off by an unknown 20 %.
- **Tracker:** a Kalman filter that assumes roughly constant velocity.
- **Fire control:** picks the point on the predicted path where net and drone should meet, turns the head there, and
  fires so that the net arrives at the same moment as the drone.
- **Net:** flies as the capture model in `analysis/tactics/capture.py` says (tables from `envelope.py`).
- **Caught:** when the drone crosses the plane of the net it must be within the catch half-width for that distance,
  narrowed by the cosine of the angle it arrives at. `check_oblique.py` compares this rule with the full net model.

## Not modelled

Evasion other than a steady side-to-side weave, several drones at once, darkness and weather on the camera, the valve's opening
time, and what a netted drone does afterwards: it keeps most of its speed, so a catch a few metres out does not stop
it reaching the building.
