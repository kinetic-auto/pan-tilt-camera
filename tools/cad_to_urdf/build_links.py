"""Group STEP leaf parts into URDF links, compute joint frames from the B-rep
cylinder axes, and export per-link meshes (meters, link frame)."""
import json, os, sys, numpy as np, trimesh

PARTS = sys.argv[1]           # dir with parts.json + PLYs (mm, assembly coords)
CYL = sys.argv[2]             # cyl.json from cyl_axes.py
OUT = sys.argv[3]             # output dir (meshes/ + frames.json)
os.makedirs(os.path.join(OUT, "meshes", "visual"), exist_ok=True)
os.makedirs(os.path.join(OUT, "meshes", "collision"), exist_ok=True)
P = json.load(open(os.path.join(PARTS, "parts.json")))
C = json.load(open(CYL))

# ---------------------------------------------------------------- grouping
def classify(path):
    leaf = path.split("/")[-1]
    if "Mounting Base" in path: return "base_link", "base_plate"
    if "Thrust Bearing" in path: return "base_link", "thrust_bearing"
    # pan horn + its center screw are bolted to the base hub -> fixed
    if leaf in ("DC11_A01_HORN_REF_1", "FHS_M2_5X06_NYLOK_1"): return "base_link", "pan_horn"
    # tilt horn + its screw ride with the fork
    if leaf in ("DC11_A01_HORN_REF", "FHS_M2_5X06_NYLOK"): return "tilt_link", "tilt_hardware"
    if "Tilt Fork" in path: return "tilt_link", "tilt_fork"
    if "57155K552" in path or "CirPattern1" in path: return "tilt_link", "tilt_hardware"   # idler bearing
    if "/NONE" in path: return "camera_link", "camera"
    if "Panning Adapter" in path: return "pan_link", "pan_shroud"
    if "98089A373" in path or "94669A006" in path: return "pan_link", "servo"  # shoulder bolt + spacer: fixed to case
    if "2XL-430" in path: return "pan_link", "servo"
    raise ValueError("unclassified part: " + path)

# ---------------------------------------------------------------- axes (mm)
def axis_of(leaf, r=None):
    c = [c for c in C if c["path"].split("/")[-1] == leaf and (r is None or abs(c["r"] - r) < 1e-3)]
    c = max(c, key=lambda c: c["area"])
    return np.array(c["pt"]), np.array(c["dir"])

pan_pt, pan_dir = axis_of("DC11_A01_HORN_REF_1", 10.25)
tilt_pt, tilt_dir = axis_of("DC11_A01_HORN_REF", 10.25)       # dir points case -> horn (outward)
lens_pt, lens_dir = axis_of("NONE", 7.5)
assert abs(abs(pan_dir[2]) - 1) < 1e-6 and abs(pan_pt[0]) < 1e-6 and abs(pan_pt[1]) < 1e-6

cam = [trimesh.load(os.path.join(PARTS, p["file"]), process=False) for p in P if "/NONE" in p["path"]]
camv = np.vstack([m.vertices for m in cam])
# optical axis direction = toward the front of the lens (away from the servo)
if np.dot(camv.mean(0) - tilt_pt, lens_dir) < 0: lens_dir = -lens_dir
look = lens_dir / np.linalg.norm(lens_dir)

theta = np.arctan2(look[1], look[0])                  # CAD pan heading of the camera
elev = np.arcsin(look[2])                              # CAD camera elevation (+ = up)
u = tilt_dir / np.linalg.norm(tilt_dir)

def Rz(a): c, s = np.cos(a), np.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
def Ry(a): c, s = np.cos(a), np.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

R_pan = Rz(theta)
assert np.allclose(R_pan[:, 1], u, atol=2e-3), (R_pan[:, 1], u)   # tilt axis == pan_link +y
q_tilt_cad = -elev                                      # +y rotation pitches nose DOWN
R_tilt = R_pan @ Ry(q_tilt_cad)
assert np.allclose(R_tilt[:, 0], look, atol=2e-3)

base_top = max(p["hi"][2] for p in P if "Mounting Base" in p["path"])
o_pan = np.array([0, 0, base_top])                      # pan joint on the hub top
# tilt origin: point on tilt axis closest to the pan axis
s = -np.dot(tilt_pt[:2], u[:2]) / np.dot(u[:2], u[:2])
o_tilt = tilt_pt + s * u

# optical axis expressed in the tilt frame
lens_in_tilt = R_tilt.T @ (lens_pt - o_tilt)
front = np.max((camv - o_tilt) @ look)                  # lens front along the optical axis
o_cam = o_tilt + R_tilt @ np.array([front, lens_in_tilt[1], lens_in_tilt[2]])

frames = {
    "base_link": (np.eye(3), np.zeros(3)),
    "pan_link": (R_pan, o_pan),
    "tilt_link": (R_tilt, o_tilt),
    "camera_link": (R_tilt, o_cam),
}

# ---------------------------------------------------------------- meshes
groups = {}
for p in P:
    link, grp = classify(p["path"])
    m = trimesh.load(os.path.join(PARTS, p["file"]), process=False)
    groups.setdefault((link, grp), []).append(m)

budget = {"thrust_bearing": 8000, "camera": 20000, "servo": 20000, "pan_shroud": 8000,
          "base_plate": 6000, "tilt_fork": 6000, "tilt_hardware": 6000, "pan_horn": 3000}
report = {}
for (link, grp), ms in groups.items():
    R, o = frames[link]
    m = trimesh.util.concatenate(ms)
    m.vertices = ((m.vertices - o) @ R) / 1000.0         # R^T (v - o), mm -> m
    m.merge_vertices(); m.remove_unreferenced_vertices()
    n0 = len(m.faces)
    if n0 > budget[grp]:
        m = m.simplify_quadric_decimation(face_count=budget[grp])
    fn = f"{link}__{grp}.stl"
    m.export(os.path.join(OUT, "meshes", "visual", fn))
    report[fn] = (n0, len(m.faces))

# collision: one convex hull per link
for link in frames:
    ms = [trimesh.load(os.path.join(OUT, "meshes", "visual", f)) for f in report if f.startswith(link + "__")]
    hull = trimesh.util.concatenate(ms).convex_hull
    hull.export(os.path.join(OUT, "meshes", "collision", f"{link}.stl"))
    report[f"collision/{link}.stl"] = (None, len(hull.faces))

def mm(v): return (np.asarray(v) / 1000.0).round(6).tolist()
info = dict(
    cad_pan_heading_deg=float(np.degrees(theta)), cad_tilt_deg=float(np.degrees(q_tilt_cad)),
    cad_camera_elevation_deg=float(np.degrees(elev)),
    pan_joint_xyz=mm(o_pan),
    tilt_joint_xyz=mm(R_pan.T @ (o_tilt - o_pan)),
    camera_joint_xyz=mm(R_tilt.T @ (o_cam - o_tilt)),
    axis_offset_d_mm=float(np.linalg.norm((R_pan.T @ (o_tilt - o_pan))[:2])),
    tilt_axis_height_above_base_mm=float(o_tilt[2]),
    optical_axis_to_tilt_axis_mm=float(abs(np.dot(lens_pt - tilt_pt, np.cross(u, look)))),
    optical_axis_to_pan_axis_mm=float(abs(lens_in_tilt[1])),
    lens_front_from_tilt_axis_mm=float(front),
    meshes=report,
    groups={f"{l}/{g}": len(v) for (l, g), v in groups.items()},
    frames={k: dict(R=v[0].round(6).tolist(), o_mm=v[1].round(4).tolist()) for k, v in frames.items()},
)
json.dump(info, open(os.path.join(OUT, "frames.json"), "w"), indent=1)
print(json.dumps({k: v for k, v in info.items() if k not in ("frames",)}, indent=1))
