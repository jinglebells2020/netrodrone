# Figure scripts

These scripts regenerate the pictures in `docs/images/` that come from the CAD model. The
charts come from `analysis/run_analysis.py` and `analysis/tactics/`.

| script | makes | time |
|---|---|---|
| `how_it_works.py` | `how_it_works.png`: cut-away of a shot | ~5 min |
| `what_flies.py` | `what_flies_out.png`: what gets launched, and the valve ball that stays inside | ~3 min |
| `render_docs.py` | `assembly.png`, `exploded.png`, `launcher_head_front/rear.png`, `net_weight.png`, `before_after.png` | ~5 min |

Both cut-aways are exact sections of the CAD, made by casting rays through the solids. The 3D
renders use three.js in headless Chromium (`viewer/`). They need Node.js, Playwright with
Chromium, and three.js:

```bash
npm i -g playwright && npx playwright install chromium
cd docs/figures/viewer && npm ci && cd -
python docs/figures/render_docs.py
```
