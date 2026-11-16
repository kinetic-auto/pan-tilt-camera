"""Load a STEP assembly with XCAF, tessellate every leaf instance in assembly
coordinates, and save: parts.json (name, path, bbox) + one PLY per leaf."""
import sys, json, os
import numpy as np
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_Label
try:  # cadquery-ocp >= 7.9
    from OCP.TDF import TDF_LabelSequence
except ImportError:  # older cadquery-ocp
    from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.IFSelect import IFSelect_RetDone
import trimesh

step, outdir = sys.argv[1], sys.argv[2]
os.makedirs(outdir, exist_ok=True)

doc = TDocStd_Document(TCollection_ExtendedString("doc"))
r = STEPCAFControl_Reader()
r.SetNameMode(True)
assert r.ReadFile(step) == IFSelect_RetDone
r.Transfer(doc)
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

def name_of(lab):
    a = TDataStd_Name()
    if lab.FindAttribute(TDataStd_Name.GetID_s(), a):
        return a.Get().ToExtString()
    return "?"

def mesh_shape(shape, loc):
    BRepMesh_IncrementalMesh(shape, float(os.environ.get("DEFL", "0.05")), False, float(os.environ.get("ADEFL", "0.3")), True)
    V, F = [], []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face(exp.Current())
        l = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, l)
        if tri is not None:
            trsf = loc.Multiplied(l).Transformation()
            off = sum(len(v) for v in V)
            pts = []
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(trsf)
                pts.append((p.X(), p.Y(), p.Z()))
            rev = face.Orientation().name == "TopAbs_REVERSED"
            tris = []
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                tris.append((a - 1 + off, c - 1 + off, b - 1 + off) if rev else (a - 1 + off, b - 1 + off, c - 1 + off))
            V.append(pts); F.extend(tris)
        exp.Next()
    if not V:
        return None
    return trimesh.Trimesh(np.vstack([np.array(v) for v in V]), np.array(F), process=False)

parts = []
def walk(lab, loc, path):
    if st.IsAssembly_s(lab):
        comps = TDF_LabelSequence()
        st.GetComponents_s(lab, comps)
        for c in list(comps):
            ref = TDF_Label()
            st.GetReferredShape_s(c, ref)
            cloc = loc.Multiplied(st.GetLocation_s(c))
            walk(ref, cloc, path + [name_of(c) if name_of(c) != "?" else name_of(ref)])
    else:
        shape = st.GetShape_s(lab)
        m = mesh_shape(shape, loc)
        if m is None:
            return
        idx = len(parts)
        fn = f"{idx:03d}.ply"
        m.export(os.path.join(outdir, fn))
        lo, hi = m.bounds
        parts.append(dict(i=idx, name=name_of(lab), path="/".join(path), file=fn,
                          lo=lo.round(2).tolist(), hi=hi.round(2).tolist(),
                          center=((lo + hi) / 2).round(2).tolist(), nf=len(m.faces)))

roots = TDF_LabelSequence()
st.GetFreeShapes(roots)
for rl in list(roots):
    walk(rl, TopLoc_Location(), [name_of(rl)])
json.dump(parts, open(os.path.join(outdir, "parts.json"), "w"), indent=1)
print(len(parts), "leaf parts")
