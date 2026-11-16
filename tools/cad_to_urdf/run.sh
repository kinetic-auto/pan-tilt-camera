#!/usr/bin/env bash
# Regenerate meshes/ from an Onshape STEP export of the whole assembly.
#   pip install cadquery-ocp trimesh fast_simplification scipy
#   ./run.sh ../../cad/Pan-Tilt-Camera.step
# Prints the joint origins it measured -> copy them into urdf/pan_tilt.urdf.xacro.
# Part grouping (which part rides on which link) is by part name in build_links.py:classify().
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"; PKG="$(cd "$HERE/../.." && pwd)"
STEP="$1"; TMP="$(mktemp -d)"
DEFL=0.15 ADEFL=0.5 python3 "$HERE/load_step.py" "$STEP" "$TMP/parts"
python3 "$HERE/cyl_axes.py" "$STEP" "$TMP/cyl.json" "HORN_REF|NONE"
python3 "$HERE/build_links.py" "$TMP/parts" "$TMP/cyl.json" "$TMP/out"
cp -r "$TMP/out/meshes/." "$PKG/meshes/"
echo "meshes updated in $PKG/meshes  (frames.json kept in $TMP/out)"
