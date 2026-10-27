"""Export build123d shapes to a JSON scene for the three.js headless renderer."""
import json, subprocess, os

VIEWER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "viewer")


def part(shape, name, color, opacity=1.0, tol=0.08, ang=0.2, edges=True, noclip=False):
    verts, tris = shape.tessellate(tol, ang)
    pos = []
    for v in verts:
        pos += [round(v.X, 4), round(v.Y, 4), round(v.Z, 4)]
    idx = [i for t in tris for i in t]
    return {"name": name, "color": color, "opacity": opacity, "positions": pos, "indices": idx,
            "edges": edges, "noclip": noclip}


def render(parts, views, out_prefix, W=1400, H=1000):
    scene_f = out_prefix + "_scene.json"
    views_f = out_prefix + "_views.json"
    with open(scene_f, "w") as f:
        json.dump({"parts": parts}, f)
    with open(views_f, "w") as f:
        json.dump(views, f)
    r = subprocess.run(["node", os.path.join(VIEWER, "render.js"), scene_f, views_f, out_prefix, str(W), str(H)],
                       capture_output=True, text=True, cwd=VIEWER)
    print(r.stdout[-2000:], r.stderr[-2000:])
    os.remove(scene_f); os.remove(views_f)


ISO = [1, -1, 1]
VIEWS_STD = [
    {"name": "top", "dir": [0, 0, 1], "up": [0, 1, 0]},
    {"name": "bottom", "dir": [0, 0, -1], "up": [0, 1, 0]},
    {"name": "iso", "dir": [1, -1.2, 1.1], "up": [0, 0, 1]},
    {"name": "iso_under", "dir": [1, -1.2, -1.1], "up": [0, 0, -1]},
]
