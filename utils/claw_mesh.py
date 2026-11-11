"""
Extract the claw (gripper body + two fingers) from the manufacturer's STEP assembly as a compact wireframe for the
tkinter canvas: utils/models/claw_mesh.json, a few hundred straight edges in the tool frame.

    python -X utf8 utils/claw_mesh.py ["C:/Users/ASUS/Downloads/Polar-R400-Z400 模型图.STEP"] [--cache DIR] [--out utils/models/claw_mesh.json]

Where the claw is in the file (Polar R400 Z400, SolidWorks export; instance names are the STEP's NAUOxx ids, the same
ones docs/cad_parts.json lists, see utils/step_extract.py):

    NAUO21/NAUO85/NAUO90/NAUO97   the gripper body, 145 x 52.5 x 129.2 mm
    NAUO23/NAUO81 + NAUO23/NAUO82 finger plate 7 x 43.9 x 134 mm and its 38 mm jaw pad, CAD +x side
    NAUO22/NAUO83 + NAUO22/NAUO84 the mirrored finger and pad, CAD -x side

CAD frame (hardware.json cad._note): x across the platform, y UP, z along the platform; the wrist (j4, the Z axis the
controller reports) is at CAD x 1.2, z -90.2 (hardware.json cad.checks.j4_cad_xz) and the fingers point along CAD +z.
Tool frame: x_t = z_c + 90.2 (along the fingers), y_t = x_c - 1.2 (across), z_t = y_c - Z0 (up); (z, x, y) is a cyclic
permutation of (x, y, z) so the frame stays right-handed. The file does not say where the controller's reported z sits on
the claw (cad.mapping.k is null), so Z0 is chosen to put the finger bottom at z_t = -25, which is where layout.json
volumes.claw puts it.

Method: the five solids are read once with STEPCAFControl_Reader (the 75 MB file takes minutes) and cached as .brep in
--cache (default %TEMP%/claw_mesh_cache); tessellated with BRepMesh (1.0 mm linear, 0.35 rad angular); the triangle
edges are kept where the adjacent triangle normals differ by more than 25 deg or the edge has one triangle (boundary), plus
the tangent boundaries of rounded edges (a BRep edge between a plane and a curved face, so a filleted box edge still
shows its outline); features smaller than --min-feature-mm (screw holes with their chamfers: connected components
of the edge graph) are dropped, stacked concentric rings (the collar on the Z shaft) collapse to one ring each, and
collinear runs are merged with --tol (0.5 mm, also the sagitta kept on arcs). Coordinates are rounded to 0.1 mm. The
script prints the checks against hardware.json cad.claw.
"""
import argparse
import json
import math
import os
import sys
import tempfile
import time
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_STEP = r"C:\Users\ASUS\Downloads\Polar-R400-Z400 模型图.STEP"

# instance paths in the STEP assembly (root name omitted) -> which claw part they belong to
PARTS = {
    "body": ["NAUO21/NAUO85/NAUO90/NAUO97"],
    "finger_pos": ["NAUO23/NAUO81", "NAUO23/NAUO82"],
    "finger_neg": ["NAUO22/NAUO83", "NAUO22/NAUO84"],
}
WRIST_CAD_X, WRIST_CAD_Z = 1.2, -90.2       # hardware.json cad.checks.j4_cad_xz
FINGER_BOTTOM_Z = -25.0                      # layout.json volumes.claw z[0]
KNOWN = {"fingertip_x": 205.0, "finger_len": 134.0, "finger_h": 44.0, "inner_gap": 80.0,
         "body_half_w": 72.0, "body_behind": 41.0}


# ----------------------------------------------------------------------------------------------------- STEP -> BREP
def extract_shapes(step_path, wanted, cache_dir):
    """Return {path: TopoDS_Shape} for the wanted instance paths, located in the assembly frame. Cached as .brep."""
    from OCP.BRep import BRep_Builder
    from OCP.BRepTools import BRepTools
    from OCP.TopoDS import TopoDS_Shape

    os.makedirs(cache_dir, exist_ok=True)
    cache_file = lambda p: os.path.join(cache_dir, p.replace("/", "__") + ".brep")
    shapes = {}
    for p in wanted:
        if os.path.exists(cache_file(p)):
            s = TopoDS_Shape()
            BRepTools.Read_s(s, cache_file(p), BRep_Builder())
            shapes[p] = s
    missing = [p for p in wanted if p not in shapes]
    if not missing:
        print(f"all {len(shapes)} solids from cache {cache_dir}")
        return shapes

    from OCP.IFSelect import IFSelect_RetDone
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDF import TDF_Label, TDF_LabelSequence
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopLoc import TopLoc_Location
    from OCP.XCAFApp import XCAFApp_Application
    from OCP.XCAFDoc import XCAFDoc_DocumentTool

    def name_of(label):
        n = TDataStd_Name()
        return n.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), n) else ""

    t0 = time.time()
    print(f"reading {step_path} for {missing} ...", flush=True)
    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString("MDTV-XCAF"))
    app.InitDocument(doc)
    rd = STEPCAFControl_Reader()
    rd.SetNameMode(True)
    rd.SetColorMode(False)
    if rd.ReadFile(step_path) != IFSelect_RetDone:
        sys.exit("could not read the STEP file")
    print(f"read ({time.time() - t0:.0f} s); transferring", flush=True)
    rd.Transfer(doc)
    print(f"transferred ({time.time() - t0:.0f} s)", flush=True)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    want = set(missing)

    def walk(label, loc, trail):
        ref = TDF_Label()
        if st.IsReference_s(label):
            st.GetReferredShape_s(label, ref)
        else:
            ref = label
        here = trail + [name_of(label) or name_of(ref)]
        if st.IsAssembly_s(ref):
            comps = TDF_LabelSequence()
            st.GetComponents_s(ref, comps)
            for i in range(1, comps.Length() + 1):
                c = comps.Value(i)
                walk(c, loc.Multiplied(st.GetLocation_s(c)), here)
        else:
            path = "/".join(here[1:])          # drop the root assembly's name
            if path in want:
                shapes[path] = st.GetShape_s(ref).Located(loc)

    free = TDF_LabelSequence()
    st.GetFreeShapes(free)
    for i in range(1, free.Length() + 1):
        walk(free.Value(i), TopLoc_Location(), [])
    still = [p for p in missing if p not in shapes]
    if still:
        sys.exit(f"not found in the STEP: {still}")
    for p in missing:
        BRepTools.Write_s(shapes[p], cache_file(p))
    print(f"{len(missing)} solids extracted and cached ({time.time() - t0:.0f} s)", flush=True)
    return shapes


# --------------------------------------------------------------------------------------------- BREP -> wireframe
def triangulate(shape, lin=1.0, ang=0.35):
    """Mesh the shape; return welded vertices (N,3), triangles (M,3) with per-triangle face index, planar flag per face
    and the BRep edges as index polylines with the ids of the faces they border."""
    from OCP.BRep import BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.GeomAbs import GeomAbs_Plane
    from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopExp import TopExp, TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS
    from OCP.TopTools import TopTools_IndexedDataMapOfShapeListOfShape, TopTools_IndexedMapOfShape

    BRepMesh_IncrementalMesh(shape, lin, False, ang, True)
    verts, vindex = [], {}
    tris, tri_face, planar = [], [], []

    def vid(p):
        key = (round(p[0], 2), round(p[1], 2), round(p[2], 2))
        i = vindex.get(key)
        if i is None:
            i = vindex[key] = len(verts)
            verts.append(key)
        return i

    face_map = TopTools_IndexedMapOfShape()
    TopExp.MapShapes_s(shape, TopAbs_FACE, face_map)
    face_tri = {}   # face index -> (triangulation, location, node ids)
    for fi in range(1, face_map.Extent() + 1):
        face = TopoDS.Face_s(face_map.FindKey(fi))
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is None:
            continue
        trsf = loc.Transformation()
        ids = []
        for n in range(1, tri.NbNodes() + 1):
            p = tri.Node(n).Transformed(trsf)
            ids.append(vid((p.X(), p.Y(), p.Z())))
        rev = face.Orientation() == TopAbs_REVERSED
        planar.append(BRepAdaptor_Surface(face).GetType() == GeomAbs_Plane)
        for t in range(1, tri.NbTriangles() + 1):
            a, b, c = tri.Triangle(t).Get()
            tri_ids = (ids[a - 1], ids[c - 1], ids[b - 1]) if rev else (ids[a - 1], ids[b - 1], ids[c - 1])
            if len(set(tri_ids)) == 3:
                tris.append(tri_ids)
                tri_face.append(fi - 1)
        face_tri[fi] = (tri, loc, ids)

    # BRep edges as index polylines, with the faces each borders
    edge_faces = TopTools_IndexedDataMapOfShapeListOfShape()
    TopExp.MapShapesAndAncestors_s(shape, TopAbs_EDGE, TopAbs_FACE, edge_faces)
    brep_edges = []
    for ei in range(1, edge_faces.Extent() + 1):
        edge = TopoDS.Edge_s(edge_faces.FindKey(ei))
        faces = [face_map.FindIndex(f) for f in edge_faces.FindFromIndex(ei)]
        faces = sorted(set(faces))
        poly_ids = None
        for fi in faces:
            if fi not in face_tri:
                continue
            tri, loc, ids = face_tri[fi]
            pot = BRep_Tool.PolygonOnTriangulation_s(edge, tri, loc)
            if pot is None:
                continue
            nodes = pot.Nodes()
            poly_ids = [ids[nodes.Value(k) - 1] for k in range(nodes.Lower(), nodes.Upper() + 1)]
            break
        if poly_ids and len(poly_ids) >= 2:
            brep_edges.append((poly_ids, [f - 1 for f in faces]))
    return np.array(verts, float), np.array(tris, int), np.array(tri_face, int), planar, brep_edges


def feature_edges(verts, tris, tri_face, planar, brep_edges, crease_deg=25.0):
    """Index pairs (i, j) of the edges to draw."""
    n = np.cross(verts[tris[:, 1]] - verts[tris[:, 0]], verts[tris[:, 2]] - verts[tris[:, 0]])
    ln = np.linalg.norm(n, axis=1)
    ok = ln > 1e-9
    n[ok] /= ln[ok][:, None]
    cos_crease = math.cos(math.radians(crease_deg))
    owners = defaultdict(list)
    for t, (a, b, c) in enumerate(tris):
        for i, j in ((a, b), (b, c), (c, a)):
            owners[(min(i, j), max(i, j))].append(t)
    keep = set()
    for e, ts in owners.items():
        if len(ts) != 2:
            keep.add(e)                          # boundary or non-manifold
        elif float(n[ts[0]] @ n[ts[1]]) < cos_crease:
            keep.add(e)                          # crease
    # tangent boundaries of rounded edges: a BRep edge between a plane and a curved face
    for poly, faces in brep_edges:
        if len(faces) == 2 and planar[faces[0]] != planar[faces[1]]:
            for i, j in zip(poly, poly[1:]):
                if i != j:
                    keep.add((min(i, j), max(i, j)))
    return keep


def polylines(edges):
    """Chain edge pairs into polylines (open chains first, then closed loops)."""
    adj = defaultdict(set)
    for i, j in edges:
        adj[i].add(j)
        adj[j].add(i)
    unused = set(edges)
    out = []

    def walk(start, nxt):
        chain = [start, nxt]
        unused.discard((min(start, nxt), max(start, nxt)))
        while True:
            cur, prev = chain[-1], chain[-2]
            if len(adj[cur]) != 2:
                return chain
            (other,) = adj[cur] - {prev} if len(adj[cur] - {prev}) == 1 else (None,)
            if other is None:
                return chain
            e = (min(cur, other), max(cur, other))
            if e not in unused:
                return chain
            unused.discard(e)
            chain.append(other)
            if other == start:
                return chain

    for v in sorted(adj, key=lambda v: (len(adj[v]) == 2, v)):   # start chains at junctions and ends
        for w in sorted(adj[v]):
            e = (min(v, w), max(v, w))
            if e in unused:
                out.append(walk(v, w))
    return out


def simplify(points, tol=0.15):
    """Drop interior points that lie within tol of the line through their neighbours (collinear runs), iteratively."""
    pts = list(points)
    closed = len(pts) > 2 and pts[0] == pts[-1]
    changed = True
    while changed and len(pts) > 2:
        changed = False
        out = [pts[0]]
        k = 1
        while k < len(pts) - 1:
            a, b, c = np.array(out[-1]), np.array(pts[k]), np.array(pts[k + 1])
            ac = c - a
            d = np.linalg.norm(np.cross(b - a, ac)) / max(np.linalg.norm(ac), 1e-9)
            if d < tol and float((b - a) @ ac) >= 0:
                changed = True
                k += 1
                continue
            out.append(pts[k])
            k += 1
        out.append(pts[-1])
        pts = out
    if closed and len(pts) > 3:
        # the seam point of a closed loop may itself be collinear
        a, b, c = np.array(pts[-2]), np.array(pts[0]), np.array(pts[1])
        ac = c - a
        if np.linalg.norm(np.cross(b - a, ac)) / max(np.linalg.norm(ac), 1e-9) < tol:
            pts = pts[1:-1] + [pts[1]]
    return pts


def drop_small_features(verts, edges, min_mm):
    """Remove the edges of connected components (a screw hole with its chamfer rings, a small boss) whose extent is
    under min_mm. A hole's loops touch nothing else, so the component extent is the feature's size."""
    adj = defaultdict(set)
    for i, j in edges:
        adj[i].add(j)
        adj[j].add(i)
    seen, keep, dropped = set(), set(), 0
    for v in adj:
        if v in seen:
            continue
        comp, stack = [], [v]
        while stack:
            u = stack.pop()
            if u in seen:
                continue
            seen.add(u)
            comp.append(u)
            stack.extend(adj[u] - seen)
        if np.ptp(verts[comp], axis=0).max() >= min_mm:
            keep.update(comp)
        else:
            dropped += 1
    return {e for e in edges if e[0] in keep}, dropped


def wireframe(shape, to_tool, lin=1.0, ang=0.35, crease_deg=25.0, min_feature_mm=10.0, tol=0.5, dup_mm=2.5,
              min_edge_mm=0.8):
    verts, tris, tri_face, planar, brep_edges = triangulate(shape, lin, ang)
    verts_t = to_tool(verts)
    edges = feature_edges(verts_t, tris, tri_face, planar, brep_edges, crease_deg)
    stats = {"triangles": len(tris), "raw_feature_edges": len(edges)}
    edges, stats["dropped_features"] = drop_small_features(verts_t, edges, min_feature_mm)
    chains = polylines(edges)
    stats["chains"] = len(chains)
    stats["dropped_duplicate_loops"] = 0
    out_v, out_index, out_e = [], {}, []
    kept_loops = []   # (centroid, extent) of closed loops already kept: stacked concentric rings collapse to one

    def vid(p):
        key = (round(p[0], 1), round(p[1], 1), round(p[2], 1))
        key = tuple(0.0 if v == 0 else v for v in key)
        i = out_index.get(key)
        if i is None:
            i = out_index[key] = len(out_v)
            out_v.append(list(key))
        return i

    for chain in chains:
        pts = [tuple(verts_t[i]) for i in chain]
        if pts[0] == pts[-1] and len(pts) > 3:
            arr = np.array(pts[:-1])
            c, ext = arr.mean(axis=0), np.ptp(arr, axis=0)
            if any(np.abs(c - c2).max() < dup_mm and np.abs(ext - e2).max() < dup_mm for c2, e2 in kept_loops):
                stats["dropped_duplicate_loops"] += 1
                continue
            kept_loops.append((c, ext))
        pts = simplify(pts, tol)
        ids = [vid(p) for p in pts]
        for i, j in zip(ids, ids[1:]):
            if i != j and [i, j] not in out_e and [j, i] not in out_e:
                if np.linalg.norm(np.subtract(out_v[i], out_v[j])) < min_edge_mm:
                    continue                  # grip ribs, chamfer slivers: invisible on the canvas
                out_e.append([i, j])
    used = sorted({i for e in out_e for i in e})
    remap = {old: new for new, old in enumerate(used)}
    out_v = [out_v[i] for i in used]
    out_e = [[remap[i], remap[j]] for i, j in out_e]
    stats["vertices"], stats["edges"] = len(out_v), len(out_e)
    return {"v": out_v, "e": out_e}, stats


# ------------------------------------------------------------------------------------------------------- assembly
def bbox(shape):
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    return box.Get()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("step", nargs="?", default=DEFAULT_STEP)
    ap.add_argument("--cache", default=os.path.join(tempfile.gettempdir(), "claw_mesh_cache"))
    ap.add_argument("--out", default=os.path.join(HERE, "utils", "models", "claw_mesh.json"))
    ap.add_argument("--min-feature-mm", type=float, default=10.0, help="drop features (holes, bosses) smaller than this")
    ap.add_argument("--crease-deg", type=float, default=25.0)
    ap.add_argument("--tol", type=float, default=0.5, help="collinear merge tolerance (also the sagitta kept on arcs)")
    ap.add_argument("--min-edge-mm", type=float, default=0.8, help="drop edges shorter than this (grip ribs)")
    ap.add_argument("--dry-run", action="store_true", help="print the checks, write nothing")
    args = ap.parse_args()

    wanted = [p for ps in PARTS.values() for p in ps]
    shapes = extract_shapes(args.step, wanted, args.cache)

    # the finger plates fix Z0: finger bottom (CAD y min of the plates) -> FINGER_BOTTOM_Z
    plate_boxes = [bbox(shapes[PARTS[k][0]]) for k in ("finger_pos", "finger_neg")]
    finger_bottom_cad_y = min(b[1] for b in plate_boxes)
    z0 = finger_bottom_cad_y - FINGER_BOTTOM_Z

    def to_tool(p):
        p = np.asarray(p, float)
        return np.stack([p[..., 2] - WRIST_CAD_Z, p[..., 0] - WRIST_CAD_X, p[..., 1] - z0], axis=-1)

    from OCP.BRep import BRep_Builder
    from OCP.TopoDS import TopoDS_Compound

    parts, report, boxes = {}, {}, {}
    for name, paths in PARTS.items():
        comp = TopoDS_Compound()
        bld = BRep_Builder()
        bld.MakeCompound(comp)
        for p in paths:
            bld.Add(comp, shapes[p])
        mesh, stats = wireframe(comp, to_tool, crease_deg=args.crease_deg, min_feature_mm=args.min_feature_mm,
                                tol=args.tol, min_edge_mm=args.min_edge_mm)
        parts[name] = mesh
        report[name] = stats
        v = np.array(mesh["v"])
        boxes[name] = (v.min(axis=0), v.max(axis=0))
        print(f"{name:11s} {stats}  tool bbox min {np.round(v.min(axis=0), 1).tolist()} max {np.round(v.max(axis=0), 1).tolist()}")

    # inner faces from the finger PLATES (not the pads), in the tool frame
    inner = {}
    for name in ("finger_pos", "finger_neg"):
        b = to_tool(np.array(bbox(shapes[PARTS[name][0]])).reshape(2, 3))
        lo, hi = np.minimum(b[0], b[1]), np.maximum(b[0], b[1])
        inner[name] = float(lo[1]) if name == "finger_pos" else float(hi[1])
        parts[name]["side"] = 1 if name == "finger_pos" else -1
        parts[name]["inner_y"] = round(inner[name], 1)
    pad_inner = {}
    for name in ("finger_pos", "finger_neg"):
        b = to_tool(np.array(bbox(shapes[PARTS[name][1]])).reshape(2, 3))
        lo, hi = np.minimum(b[0], b[1]), np.maximum(b[0], b[1])
        pad_inner[name] = float(lo[1]) if name == "finger_pos" else float(hi[1])

    fp = boxes["finger_pos"]
    fn = boxes["finger_neg"]
    bd = boxes["body"]
    plate = to_tool(np.array(bbox(shapes[PARTS["finger_pos"][0]])).reshape(2, 3))
    plo, phi = np.minimum(plate[0], plate[1]), np.maximum(plate[0], plate[1])
    checks = {
        "fingertip_x": round(float(max(fp[1][0], fn[1][0])), 1),
        "finger_len": round(float(phi[0] - plo[0]), 1),
        "finger_h": round(float(phi[2] - plo[2]), 1),
        "inner_gap": round(inner["finger_pos"] - inner["finger_neg"], 1),
        "body_half_w": round(float(max(bd[1][1], -bd[0][1])), 1),
        "body_behind": round(float(-bd[0][0]), 1),
        "finger_outer_y": round(float(max(fp[1][1], -fn[0][1])), 1),
        "pad_inner_gap": round(pad_inner["finger_pos"] - pad_inner["finger_neg"], 1),
        "body_x": [round(float(bd[0][0]), 1), round(float(bd[1][0]), 1)],
        "z_range": [round(float(min(b[0][2] for b in boxes.values())), 1), round(float(max(b[1][2] for b in boxes.values())), 1)],
    }
    print("\ncheck            model   known")
    for k, known in KNOWN.items():
        flag = "" if abs(checks[k] - known) <= 3 else "   <-- differs"
        print(f"{k:14s} {checks[k]:8.1f} {known:7.1f}{flag}")
    for k in ("finger_outer_y", "pad_inner_gap", "body_x", "z_range"):
        print(f"{k:14s} {checks[k]}")
    total_e = sum(len(p["e"]) for p in parts.values())
    print(f"\nedges: " + ", ".join(f"{k} {len(p['e'])}" for k, p in parts.items()) + f", total {total_e}")

    out = {
        "source": os.path.basename(args.step),
        "units": "mm",
        "frame": "tool: origin on the wrist axis at the reported z, x along the fingers, y across, z up",
        "jaw_ref_note": "finger positions as modelled; inner_y is each finger's inner face y in this model "
                        "(the finger plate; the 38 mm jaw pads on the plates reach 2.4 mm further in, see checks.pad_inner_gap)",
        "z_note": f"the STEP does not place the controller's reported z on the claw (hardware.json cad.mapping.k is null): "
                  f"z = 0 is set so the finger bottom is at {FINGER_BOTTOM_Z:.0f}, as layout.json volumes.claw has it "
                  f"(CAD y {finger_bottom_cad_y:.1f} -> z 0 at CAD y {z0:.1f})",
        "cad_solids": PARTS,
        "parts": parts,
        "checks": checks,
    }
    if args.dry_run:
        print("dry run, nothing written")
        return
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, separators=(",", ":"))
    print(f"written {args.out} ({os.path.getsize(args.out) / 1024:.0f} kB)")


if __name__ == "__main__":
    main()
