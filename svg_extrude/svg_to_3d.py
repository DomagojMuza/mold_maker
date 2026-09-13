"""svg_to_3d - extrude an SVG outline to a solid, then offset ONE side of it.

    ..\offset_bench\.venv\Scripts\python.exe svg_to_3d.py logo.svg out.stl \
        --height 10 --grow 4 --grow-axis x --grow-side max

Two independent "offset one side" operations, either or both:

  --grow N        in-plane: dilate the outline by N mm on one half only (split
                  at --grow-at along --grow-axis). The other half is untouched,
                  so the parting line gets a step. Negative N carves that half in.
  --offset N      out-of-plane: move the whole top (or --face bottom) cap along
                  Z by N mm. On a plain extrude this just makes the slab taller.

1 SVG user unit = 1 mm (pipeline_v2 convention). Group/element transforms are
baked in; <rect>/<circle>/<polygon>/... become paths; holes by containment nesting.
"""
from __future__ import annotations

import argparse
import sys

import numpy as np
import trimesh
from shapely.affinity import scale as shapely_scale
from shapely.geometry import Polygon, MultiPolygon, box as shp_box
from shapely.geometry.polygon import orient as shp_orient
from shapely.ops import unary_union
from svgpathtools import Document

_JOIN = {"mitre": 2, "round": 1, "bevel": 3}


def load_rings(svg_path: str, density: float) -> list[np.ndarray]:
    """Every closed contour in the SVG as an (N, 2) point loop, transforms baked."""
    doc = Document(svg_path)
    rings: list[np.ndarray] = []
    for path in doc.paths():
        for sub in path.continuous_subpaths():
            length = sub.length()
            if length <= 1e-6:
                continue
            n = max(int(length * density), 24)
            ts = np.linspace(0.0, 1.0, n, endpoint=False)
            pts = np.array([(p.real, p.imag) for p in (sub.point(t) for t in ts)])
            rings.append(pts)
    return rings


def rings_to_polygons(rings: list[np.ndarray], flip_y: bool, simplify: float,
                      scale: float) -> MultiPolygon:
    polys = []
    for r in rings:
        r = r.copy()
        if flip_y:
            r[:, 1] = -r[:, 1]
        p = Polygon(r)
        if not p.is_valid:
            p = p.buffer(0)
        if p.area > 1e-6:
            polys.append(p)
    if not polys:
        raise SystemExit("no usable closed contours in SVG")

    # nest by containment: each ring's parent is the smallest ring that contains
    # it; even depth = solid, odd depth = hole. (Point-in-count is unreliable - a
    # big ring's interior point can land inside a small ring it encloses.)
    order = sorted(range(len(polys)), key=lambda i: polys[i].area, reverse=True)
    depth = [0] * len(polys)
    parent: list[int | None] = [None] * len(polys)
    for pos, i in enumerate(order):
        outer = [j for j in order[:pos] if polys[j].contains(polys[i])]
        if outer:
            parent[i] = min(outer, key=lambda j: polys[j].area)
            depth[i] = depth[parent[i]] + 1

    out = []
    for i, p in enumerate(polys):
        if depth[i] % 2:
            continue  # hole - folded into its parent below
        holes = [polys[j].exterior.coords for j in range(len(polys)) if parent[j] == i]
        out.append(Polygon(p.exterior.coords, holes))

    mp = MultiPolygon(out)
    if scale != 1.0:
        mp = shapely_scale(mp, xfact=scale, yfact=scale, origin=(0, 0))
    if simplify > 0:
        mp = mp.simplify(simplify)
    return _as_multipolygon(mp)


def _as_multipolygon(g) -> MultiPolygon:
    if g.is_empty:
        raise SystemExit("geometry is empty")
    if g.geom_type == "Polygon":
        return MultiPolygon([g])
    if g.geom_type == "MultiPolygon":
        return g
    polys = [p for p in getattr(g, "geoms", []) if p.geom_type == "Polygon" and p.area > 1e-9]
    if not polys:
        raise SystemExit(f"no polygons left after {g.geom_type}")
    return MultiPolygon(polys)


def grow_one_side(mp: MultiPolygon, amount: float, axis: str, side: str,
                  at: float | None, join: str) -> tuple[MultiPolygon, float]:
    """Dilate the outline by `amount` mm on one half-plane only; return (geom, line)."""
    minx, miny, maxx, maxy = mp.bounds
    big = 10.0 * max(maxx - minx, maxy - miny, 1.0) + 10.0 * abs(amount)
    if axis == "x":
        line = at if at is not None else 0.5 * (minx + maxx)
        max_half = shp_box(line, miny - big, maxx + big, maxy + big)
        min_half = shp_box(minx - big, miny - big, line, maxy + big)
    else:
        line = at if at is not None else 0.5 * (miny + maxy)
        max_half = shp_box(minx - big, line, maxx + big, maxy + big)
        min_half = shp_box(minx - big, miny - big, maxx + big, line)
    grow_region, keep_region = (max_half, min_half) if side == "max" else (min_half, max_half)

    buffered = mp.buffer(amount, join_style=_JOIN[join], mitre_limit=10.0)
    grown = _as_multipolygon(buffered).intersection(grow_region)
    kept = mp.intersection(keep_region)
    return _as_multipolygon(unary_union([grown, kept])), line


def extrude(mp: MultiPolygon, height: float) -> trimesh.Trimesh:
    parts = [trimesh.creation.extrude_polygon(poly, height=height) for poly in mp.geoms]
    mesh = trimesh.util.concatenate(parts) if len(parts) > 1 else parts[0]
    mesh.merge_vertices()
    mesh.process()
    return mesh


def taper(mp: MultiPolygon, widen: float, height: float, face: str,
          steps: int = 10) -> trimesh.Trimesh:
    """Draft the walls by a **true uniform offset**: the cross-section at height
    fraction t is `outline.buffer(widen * t)` (t measured from the un-widened
    cap). Built as `steps` stacked extruded slabs boolean-unioned, so every edge
    of the moving cap ends up exactly `widen` mm out - same on +X, -X, +Y, -Y -
    and the un-widened cap is the plain extrude, vertex-for-vertex.

    `steps` slabs -> a fine staircase wall; raise for smoother, lower for speed.
    Pieces closer than `2 * widen` fuse where their drafts meet (physical)."""
    slabs = []
    dz = height / steps
    for k in range(steps):
        frac = k / (steps - 1)
        if face == "bottom":
            frac = 1.0 - frac
        d = widen * frac
        layer = _as_multipolygon(mp.buffer(d, join_style=_JOIN["round"])) if abs(d) > 1e-9 else mp
        lo = k * dz if k == 0 else k * dz - 0.4 * dz            # overlap neighbours so the
        hi = (k + 1) * dz if k == steps - 1 else (k + 1) * dz + 0.4 * dz   # union has no coplanar faces
        for poly in layer.geoms:
            s = trimesh.creation.extrude_polygon(poly, height=hi - lo)
            s.apply_translation((0.0, 0.0, lo))
            slabs.append(s)

    mesh = trimesh.boolean.union(slabs)
    if isinstance(mesh, list):
        mesh = trimesh.util.concatenate(mesh)
    mesh.merge_vertices()
    mesh.process()
    if mesh.volume <= 0 or not mesh.is_winding_consistent:
        raise SystemExit(f"widen {widen:g} mm makes this outline degenerate")
    return mesh


def offset_face(mesh: trimesh.Trimesh, face: str, offset: float, tol: float = 1e-5) -> None:
    zmin, zmax = mesh.bounds[0][2], mesh.bounds[1][2]
    if offset < 0 and abs(offset) >= (zmax - zmin):
        raise SystemExit(
            f"--offset {offset} would collapse a slab only {zmax - zmin:.3f} mm thick")
    if face == "top":
        sel = mesh.vertices[:, 2] >= zmax - tol
        dz = offset
    else:  # bottom
        sel = mesh.vertices[:, 2] <= zmin + tol
        dz = -offset
    mesh.vertices[sel, 2] += dz


def negative_mold(model: trimesh.Trimesh, *, wall: float, floor: float,
                  clearance: float, freeboard: float) -> trimesh.Trimesh:
    """Press `model` into a tray and subtract it -> a negative mould.

    The tray is a solid base (footprint = model bbox + `clearance`, thickness
    `floor` under the deepest point) with a `wall`-thick rim rising `freeboard`
    mm above the base top. Subtracting the model leaves a model-shaped pocket in
    the base, its mouth flush with the base top, an open basin inside the walls
    above it. Bottom of the result sits at z = 0."""
    lo, hi = model.bounds
    hm = float(hi[2] - lo[2])
    ix0, iy0 = lo[0] - clearance, lo[1] - clearance
    ix1, iy1 = hi[0] + clearance, hi[1] + clearance
    iw, idp = ix1 - ix0, iy1 - iy0
    cx, cy = 0.5 * (ix0 + ix1), 0.5 * (iy0 + iy1)
    base_h = floor + hm
    total_h = base_h + freeboard

    base = trimesh.creation.box(extents=(iw, idp, base_h))
    base.apply_translation((cx, cy, base_h / 2.0))

    outer = trimesh.creation.box(extents=(iw + 2 * wall, idp + 2 * wall, total_h))
    outer.apply_translation((cx, cy, total_h / 2.0))
    hollow = trimesh.creation.box(extents=(iw, idp, total_h + 2.0))
    hollow.apply_translation((cx, cy, total_h / 2.0))
    walls = trimesh.boolean.difference([outer, hollow])
    tray = trimesh.boolean.union([base, walls])

    m = model.copy()
    m.apply_translation((0.0, 0.0, floor - lo[2]))          # bottom on the base top - hm
    if m.body_count > 1:
        # draft pushes neighbouring pieces into each other; resolve those
        # overlaps into clean single geometry BEFORE the subtraction, or the
        # CSG leaves self-crossing walls between them.
        m = trimesh.boolean.union(list(m.split(only_watertight=False)))
        if isinstance(m, list):
            m = trimesh.util.concatenate(m)
    mold = trimesh.boolean.difference([tray, m])
    if isinstance(mold, list):
        mold = trimesh.util.concatenate(mold)
    mold.merge_vertices()
    mold.process()
    return mold


def _to_meshlib(mesh: trimesh.Trimesh):
    import io
    from meshlib import mrmeshpy as mm
    b = io.BytesIO()
    mesh.export(b, file_type="stl")
    b.seek(0)
    return mm.loadMesh(b, extension="*.stl")


def _from_meshlib(ml) -> trimesh.Trimesh:
    import io
    from meshlib import mrmeshpy as mm
    b = io.BytesIO()
    mm.saveMesh(ml, "*.stl", b)
    b.seek(0)
    return trimesh.load(b, file_type="stl", force="mesh")


def mesh_health(mesh: trimesh.Trimesh) -> dict:
    """Cheap MeshLib defect scan: self-intersecting + degenerate face counts."""
    try:
        from meshlib import mrmeshpy as mm
        ml = _to_meshlib(mesh)
        return {"self_intersections": int(mm.localFindSelfIntersections(ml).count()),
                "degenerate_faces": int(mm.findDegenerateFaces(mm.MeshPart(ml)).count())}
    except Exception as e:  # noqa: BLE001 - never block a build on the scan
        return {"health_error": f"{type(e).__name__}: {e}"}


def heal_mesh(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    """Remove self-intersections with MeshLib's local **Relax** pass - it nudges
    only the vertices in the offending wall patches (no detail deleted, no full
    remesh) and is a no-op when the mesh is already clean. The bottom plane is
    then pinned back to its exact z so `--widen`'s flat base is untouched."""
    from meshlib import mrmeshpy as mm

    z0 = float(mesh.vertices[:, 2].min())
    z1 = float(mesh.vertices[:, 2].max())
    tol = 1e-3 * max(z1 - z0, 1.0)

    ml = _to_meshlib(mesh)
    st = mm.SelfIntersections.Settings()
    st.method = mm.SelfIntersections.Settings.Method.Relax
    st.maxExpand = 2
    mm.SelfIntersections.fix(ml, st)
    out = _from_meshlib(ml)

    z = out.vertices[:, 2]
    z[np.abs(z - z0) <= tol] = z0            # pin the base flat exactly
    z[np.abs(z - z1) <= tol] = z1            # and the top
    out.vertices[:, 2] = z
    out.merge_vertices()
    out.fix_normals()
    return out


def build(svg: str, *, height: float = 10.0, scale: float = 1.0,
          widen: float = 0.0, widen_face: str = "top", widen_steps: int = 10,
          offset: float = 0.0, face: str = "top",
          inflate: float = 0.0, inflate_join: str = "round",
          grow: float = 0.0, grow_axis: str = "x", grow_side: str = "max",
          grow_at: float | None = None, grow_join: str = "mitre",
          mold: bool = False, mold_wall: float = 4.0, mold_floor: float = 4.0,
          mold_clearance: float = 3.0, mold_freeboard: float = 3.0,
          heal: bool = False, check: bool = True,
          density: float = 1.0, simplify: float = 0.05,
          flip_y: bool = True) -> tuple[trimesh.Trimesh, dict]:
    """Full SVG -> solid pipeline. Returns (mesh, info). Shared by the CLI and GUI."""
    rings = load_rings(svg, density)
    if not rings:
        raise SystemExit("no closed contours in SVG")

    mp = rings_to_polygons(rings, flip_y=flip_y, simplify=simplify, scale=scale)
    info: dict = {"contours": len(rings), "polys": len(mp.geoms),
                  "outline_bbox": np.round(mp.bounds, 3).tolist()}

    if inflate:
        mp = _as_multipolygon(mp.buffer(inflate * scale, join_style=_JOIN[inflate_join],
                                        mitre_limit=10.0))
        info["polys"] = len(mp.geoms)
        info["inflated_bbox"] = np.round(mp.bounds, 3).tolist()

    if grow:
        at = grow_at if grow_at is None else grow_at * scale
        mp, line = grow_one_side(mp, grow * scale, grow_axis, grow_side, at, grow_join)
        info["grow_line"] = round(line, 3)
        info["polys"] = len(mp.geoms)

    h_mm = height * scale
    mesh = (taper(mp, widen * scale, h_mm, widen_face, max(widen_steps, 2))
            if widen else extrude(mp, h_mm))

    if offset:
        offset_face(mesh, face, offset)
        mesh.merge_vertices()
        mesh.process()

    info["model_bbox"] = np.round(mesh.extents, 3).tolist()

    if heal:
        before = mesh_health(mesh)
        if before.get("self_intersections") or before.get("degenerate_faces"):
            mesh = heal_mesh(mesh)
            info["healed_from"] = before

    if mold:
        mesh = negative_mold(mesh, wall=mold_wall, floor=mold_floor,
                             clearance=mold_clearance, freeboard=mold_freeboard)
        info["mold"] = True
        if heal and mesh_health(mesh).get("self_intersections"):
            mesh = heal_mesh(mesh)

    if check:
        info.update(mesh_health(mesh))

    info.update(verts=len(mesh.vertices), faces=len(mesh.faces),
                watertight=bool(mesh.is_watertight), components=int(mesh.body_count),
                volume=round(float(mesh.volume), 2),
                bbox=np.round(mesh.extents, 3).tolist())
    return mesh, info


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("svg")
    ap.add_argument("out", help="output mesh (.stl/.obj/.ply/.3mf)")
    ap.add_argument("--height", type=float, default=10.0,
                    help="extrusion depth in mm (default 10)")

    i = ap.add_argument_group("inflate the whole outline (all sides, both axes)")
    i.add_argument("--inflate", type=float, default=0.0,
                   help="offset the whole outline out by this many mm - every side, "
                        "holes shrink (- offsets it in)")
    i.add_argument("--inflate-join", choices=tuple(_JOIN), default="round",
                   help="corner style of the inflated outline (default round)")

    g = ap.add_argument_group("offset ONE side only (in-plane, split at a parting line)")
    g.add_argument("--grow", type=float, default=0.0,
                   help="dilate the outline by this many mm on one half (- carves in)")
    g.add_argument("--grow-axis", choices=("x", "y"), default="x",
                   help="axis the parting line is perpendicular to (default x)")
    g.add_argument("--grow-side", choices=("max", "min"), default="max",
                   help="which half to grow: max = +axis side (default), min = -axis side")
    g.add_argument("--grow-at", type=float, default=None,
                   help="world-mm position of the parting line (default: bbox centre)")
    g.add_argument("--grow-join", choices=tuple(_JOIN), default="mitre",
                   help="corner style of the grown outline (default mitre)")

    w = ap.add_argument_group("taper: uniformly offset one cap's outline in X/Y")
    w.add_argument("--widen", type=float, default=0.0,
                   help="move every edge of one cap out this many mm (- shrinks it); "
                        "the other cap keeps the exact SVG outline")
    w.add_argument("--widen-face", choices=("top", "bottom"), default="top",
                   help="which cap gets the wider outline (default top)")
    w.add_argument("--widen-steps", type=int, default=10,
                   help="stacked slabs across the taper - higher = smoother wall, slower")

    f = ap.add_argument_group("offset one cap (out-of-plane, Z)")
    f.add_argument("--offset", type=float, default=0.0,
                   help="move one cap by this many mm along Z (+ grows, - carves)")
    f.add_argument("--face", choices=("top", "bottom"), default="top",
                   help="which cap --offset moves (default top)")

    n = ap.add_argument_group("negative mould (subtract the model from a tray)")
    n.add_argument("--mold", action="store_true",
                   help="wrap the model in a floor+walls tray and subtract it")
    n.add_argument("--mold-wall", type=float, default=4.0, help="wall thickness mm (default 4)")
    n.add_argument("--mold-floor", type=float, default=4.0,
                   help="floor thickness under the cavity mm (default 4)")
    n.add_argument("--mold-clearance", type=float, default=3.0,
                   help="gap between model bbox and wall inner face mm (default 3)")
    n.add_argument("--mold-freeboard", type=float, default=3.0,
                   help="wall height above the model top mm (default 3)")

    ap.add_argument("--heal", action="store_true",
                    help="repair self-intersections / degenerate faces with MeshLib "
                         "(local CutAndFill, SDF fallback) before the mould step")
    ap.add_argument("--no-check", action="store_true",
                    help="skip the MeshLib defect scan of the result")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="multiply SVG coords + height (default 1: 1 unit = 1 mm)")
    ap.add_argument("--density", type=float, default=1.0,
                    help="samples per mm along curves (default 1)")
    ap.add_argument("--simplify", type=float, default=0.05,
                    help="shapely simplify tolerance mm (0 = off)")
    ap.add_argument("--no-flip-y", action="store_true",
                    help="keep SVG's native Y-down orientation")
    args = ap.parse_args()

    mesh, info = build(
        args.svg, height=args.height, scale=args.scale,
        widen=args.widen, widen_face=args.widen_face, widen_steps=args.widen_steps,
        offset=args.offset, face=args.face,
        inflate=args.inflate, inflate_join=args.inflate_join,
        grow=args.grow, grow_axis=args.grow_axis, grow_side=args.grow_side,
        grow_at=args.grow_at, grow_join=args.grow_join,
        mold=args.mold, mold_wall=args.mold_wall, mold_floor=args.mold_floor,
        mold_clearance=args.mold_clearance, mold_freeboard=args.mold_freeboard,
        heal=args.heal, check=not args.no_check,
        density=args.density, simplify=args.simplify, flip_y=not args.no_flip_y)

    for k, v in info.items():
        print(f"{k}={v}")
    if info["components"] > 1:
        print(f"note: {info['components']} disjoint solids (SVG holds separate shapes)")

    mesh.export(args.out)
    print(f"wrote {args.out}")

    rc = 0
    if not info["watertight"]:
        print("WARNING: result is not watertight", file=sys.stderr)
        rc = 1
    if info.get("self_intersections"):
        print(f"WARNING: {info['self_intersections']} self-intersecting faces"
              f"{'' if args.heal else ' - re-run with --heal'}", file=sys.stderr)
        rc = 1
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
