"""3D renders used in the README -> docs/images/{assembly,exploded,launcher_head_front,launcher_head_rear,net_weight,before_after}.png
Run: python docs/figures/render_docs.py   (~5 min)

Needs Node.js and Playwright with Chromium (`npm i -g playwright && npx playwright install chromium`),
plus three.js in docs/figures/viewer (`cd docs/figures/viewer && npm ci`).
"""
import shutil
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
IMG = REPO / "docs" / "images"
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / "cad"))
import build_netrodrone as bn  # noqa: E402
from build123d import Pos, Rot  # noqa: E402
from scene import part, render  # noqa: E402

out = tempfile.mkdtemp(prefix="netrodrone_renders_")
A, B = bn.build_gearbox()
head = bn.build_head()
proj = bn.build_projectile()
plate = bn.build_mount_plate()
sleeve = bn.build_sleeve(bn.SLEEVE_LENGTHS["S8"])
cyl_tube = bn.cyl_x(0, 0, 13.0, 84.8, 12.2) - bn.cyl_x(0, 0, 12.0, 86, 11.5)
motor = bn.cyl_x(-48.9, 0, 20.0, 72.5, 13.0) + bn.cyl_x(-48.9, 0, 72.5, 86.0, 1.2)
weights = [loc * proj for loc in bn.projectile_locations()]
C = dict(A="#d9894a", B="#4f86c6", head="#5fa04e", w="#e3b23c", plate="#8c8c8c", int="#b0b0b0", sl="#222222")
# assembled
parts = [part(A, "A", C["A"]), part(B, "B", C["B"]), part(head, "head", C["head"]), part(plate, "plate", C["plate"])]
parts += [part(w, f"w{i}", C["w"]) for i, w in enumerate(weights)]
render(parts, [
    {"name": "iso", "dir": [-1.0, 0.75, 1.0], "up": [0, 1, 0]},
    {"name": "iso_back", "dir": [1.0, 0.6, -1.0], "up": [0, 1, 0]},
    {"name": "front", "dir": [-1, 0.0, 0.0], "up": [0, 1, 0]},
], f"{out}/asm", W=1600, H=1000)
# exploded
ex = [part(Pos(0, 0, 45) * A, "A", C["A"]), part(Pos(0, 0, -40) * B, "B", C["B"]), part(Pos(-45, 0, 0) * head, "head", C["head"]),
      part(Pos(0, 40, 0) * plate, "plate", C["plate"]), part(cyl_tube, "cyl", C["int"]), part(motor, "motor", "#707070"),
      part(Pos(-45 - 12, 0, 0) * (Rot(0, 90, 0) * sleeve), "sl", C["sl"])]
for i, loc in enumerate(bn.projectile_locations()):
    o, d, _ = bn.barrel_axes()[i]
    ex.append(part(Pos(-45, 0, 0) * Pos(*(d * 75)) * (loc * proj), f"w{i}", C["w"]))
render(ex, [{"name": "exploded", "dir": [-1.0, 0.8, 1.15], "up": [0, 1, 0]}], f"{out}/asm", W=1800, H=1150)
# halves, original vs improved
A0, B0, S0 = bn.load_original_halves()
render([part(A0, "A0", "#c9a27a")], [{"name": "orig_inner", "dir": [-0.35, -0.5, -1.0], "up": [0, 1, 0]}], f"{out}/cmpA", W=1300, H=900)
render([part(A, "A", C["A"])], [{"name": "new_inner", "dir": [-0.35, -0.5, -1.0], "up": [0, 1, 0]}], f"{out}/cmpA", W=1300, H=900)
render([part(B0, "B0", "#8fb0d8")], [{"name": "orig_inner", "dir": [-0.35, -0.5, 1.0], "up": [0, 1, 0]}], f"{out}/cmpB", W=1300, H=900)
render([part(B, "B", C["B"])], [{"name": "new_inner", "dir": [-0.35, -0.5, 1.0], "up": [0, 1, 0]}], f"{out}/cmpB", W=1300, H=900)
render([part(head, "head", C["head"])], [{"name": "rear", "dir": [1.0, 0.35, 0.5], "up": [0, 1, 0]},
                                         {"name": "front", "dir": [-1.0, 0.45, 0.6], "up": [0, 1, 0]}], f"{out}/headv", W=1100, H=1000)
render([part(proj, "p", C["w"])], [{"name": "iso", "dir": [1, -1, 0.7], "up": [0, 0, 1]}], f"{out}/weight", W=700, H=800)

# copy into docs/images under the names the README uses
for src, dst in [("asm_iso", "assembly"), ("asm_exploded", "exploded"), ("headv_front", "launcher_head_front"),
                 ("headv_rear", "launcher_head_rear"), ("weight_iso", "net_weight")]:
    shutil.copy(f"{out}/{src}.png", IMG / f"{dst}.png")

# before / after: your original halves next to the improved ones
from PIL import Image, ImageDraw, ImageFont  # noqa: E402
ims = [Image.open(f"{out}/{n}.png") for n in ("cmpA_orig_inner", "cmpA_new_inner", "cmpB_orig_inner", "cmpB_new_inner")]
W, H = ims[0].size
canvas = Image.new("RGB", (W * 2, H * 2), (252, 252, 251))
for im, (x, y) in zip(ims, [(0, 0), (W, 0), (0, H), (W, H)]):
    canvas.paste(im.convert("RGB"), (x, y))
draw = ImageDraw.Draw(canvas)
try:
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 34)
except OSError:
    font = ImageFont.load_default()
for txt, (x, y) in [("Half A - your original", (30, 20)), ("Half A - improved", (W + 30, 20)),
                    ("Half B - your original", (30, H + 20)), ("Half B - improved", (W + 30, H + 20))]:
    draw.text((x, y), txt, fill=(11, 11, 11), font=font)
canvas = canvas.resize((canvas.width * 2 // 3, canvas.height * 2 // 3), Image.LANCZOS)
canvas.save(IMG / "before_after.png")
shutil.rmtree(out)
print("wrote renders to", IMG)
