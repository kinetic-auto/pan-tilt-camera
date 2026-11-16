"""Report cylindrical-face axes (assembly coords, mm) of leaf parts whose path matches a filter."""
import sys, json, re, os
import numpy as np
sys.argv += []
exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'load_step.py')).read().split("parts = []")[0].replace("os.makedirs(outdir, exist_ok=True)", ""))
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp

pat = re.compile(sys.argv[3])
out = []
def walk(lab, loc, path):
    if st.IsAssembly_s(lab):
        comps = TDF_LabelSequence(); st.GetComponents_s(lab, comps)
        for c in list(comps):
            ref = TDF_Label(); st.GetReferredShape_s(c, ref)
            walk(ref, loc.Multiplied(st.GetLocation_s(c)), path + [name_of(c) if name_of(c) != "?" else name_of(ref)])
    else:
        p = "/".join(path)
        if not pat.search(p): return
        shape = st.GetShape_s(lab).Moved(loc)
        exp = TopExp_Explorer(shape, TopAbs_FACE)
        while exp.More():
            f = TopoDS.Face(exp.Current())
            s = BRepAdaptor_Surface(f)
            if s.GetType() == GeomAbs_Cylinder:
                c = s.Cylinder(); ax = c.Axis()
                g = GProp_GProps(); BRepGProp.SurfaceProperties_s(f, g)
                L, D = ax.Location(), ax.Direction()
                d = np.array([D.X(), D.Y(), D.Z()]); l = np.array([L.X(), L.Y(), L.Z()])
                cm = g.CentreOfMass(); cm = np.array([cm.X(), cm.Y(), cm.Z()])
                # project centroid onto axis -> a point on the axis near the face
                pt = l + d * np.dot(cm - l, d)
                out.append(dict(path=p, r=round(c.Radius(), 3), area=round(g.Mass(), 2),
                                pt=pt.round(3).tolist(), dir=d.round(5).tolist()))
            exp.Next()
roots = TDF_LabelSequence(); st.GetFreeShapes(roots)
for rl in list(roots): walk(rl, TopLoc_Location(), [name_of(rl)])
json.dump(out, open(sys.argv[2], "w"), indent=1)
print(len(out), "cylinder faces")
