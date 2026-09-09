#!/usr/bin/env python3
"""
make_mold2.py - glove-mould pipeline, no Blender.

Same geometry design as core/glove_mold.py (rigid shell + CORE_OFFSET silicone
gap + base cradle + clamping lip + 2/4-way split with registration pins) but
built on MeshLib instead of bpy:

  offset  : mrmeshpy.offsetMesh  - OpenVDB narrow-band SDF + dual marching
            cubes, adaptive output. One call per offset; no stepped
            vertex-push + full voxel-remesh loop, so triangle count does not
            compound stage over stage.
  boolean : mrmeshpy.boolean     - exact CSG. Handles the concave-shape split
            that Blender's halfspace-box INTERSECT silently mangled.

    <venv>/python make_mold2.py <master.stl> [outdir] [--two] [--novents]
                                 [--nofeet] [--nopins] [--voxel N]

Reusable entry point for the GUI: `generate(cfg, master, pour=, vents=, feet=)`.
"""
import sys, os, time, math
from dataclasses import dataclass, field
from typing import Optional, List, Tuple

import meshlib.mrmeshpy as mr

_t0 = time.time()
def log(m): print(f"[v2] {time.time()-_t0:6.1f}s  {m}", flush=True)


# ============================================================ config
@dataclass
class Config:
    SHELL_OFFSET: float = 4.2
    CORE_OFFSET: float = 3.0
    FLANGE_REACH: float = 10.0
    FLANGE_THICK: float = 6.0
    FOUR_PIECE: bool = True
    VOXEL: Optional[float] = None      # None -> auto from model size (diag/200, clamped)
    SPLIT_X: Optional[float] = None    # parting-plane X (world mm); None -> model bbox centre
    SPLIT_Y: Optional[float] = None    # parting-plane Y

    ADD_PINS: bool = True
    PIN_RADIUS: float = 2.0
    PIN_CLEAR: float = 0.25
    PIN_COUNT: Optional[int] = None

    POUR_SHAPE: str = "round"         # pour post + bore cross-section: "round" | "square"
    POUR_R: float = 6.0
    POUR_BORE: float = 3.0
    POUR_RES_H: float = 12.0
    FUNNEL_H: float = 6.0

    ADD_VENTS: bool = True
    VENT_BORE: float = 1.0
    VENT_POST_R: float = 2.5
    VENT_MIN_SEP: float = 25.0
    VENT_MAX_N: int = 3

    ADD_FEET: bool = True
    STAB_R: float = 5.0
    STAB_TIP_R: float = 2.0
    STAB_N: Optional[int] = None

    ADD_CRADLE: bool = True
    ADD_LIP: bool = True
    CRADLE_MARGIN: float = 10.0
    CRADLE_RECESS: float = 2.0
    CRADLE_FLOOR: float = 3.0
    CRADLE_TOL: float = 0.5
    LIP_THICK: float = 3.0
    LIP_INSET: float = 2.0

    EXPORT_MAX_ERR: float = 0.08     # mm, adaptive decimation of each exported mesh


# ============================================================ helpers
def V(x, y, z): return mr.Vector3f(float(x), float(y), float(z))

def bbox(m):
    b = m.computeBoundingBox()
    return (b.min.x, b.min.y, b.min.z), (b.max.x, b.max.y, b.max.z)

def nfaces(m): return m.topology.numValidFaces()

UNION = mr.BooleanOperation.Union
DIFF  = mr.BooleanOperation.DifferenceAB
ISECT = mr.BooleanOperation.Intersection

def boolop(a, b, op):
    r = mr.boolean(a, b, op, mr.AffineXf3f())
    if not r.valid():
        raise RuntimeError(f"boolean {op} failed: {r.errorString}")
    return r.mesh

def offset(mesh, dist, voxel, raw=True):
    p = mr.OffsetParameters()
    p.voxelSize = voxel
    if raw:
        # HoleWindingRule: tolerates messy source STLs AND is ~3x faster here than
        # the default OpenVDB sign mode (measured on these models). Kept on always.
        p.signDetectionMode = mr.SignDetectionMode.HoleWindingRule
    return mr.offsetMesh(mr.MeshPart(mesh), float(dist), p)

def box(cx, cy, cz, sx, sy, sz):
    return mr.makeCube(V(sx, sy, sz), V(cx - sx/2, cy - sy/2, cz - sz/2))

def cyl(x, y, z0, z1, r, res=48):
    m = mr.makeCylinder(float(r), float(z1 - z0), res)
    m.transform(mr.AffineXf3f.translation(V(x, y, z0)))
    return m

def cone(x, y, z0, z1, r0, r1, res=48):
    m = mr.makeCylinderAdvanced(float(r0), float(r1), 0.0, 2*math.pi, float(z1 - z0), res)
    m.transform(mr.AffineXf3f.translation(V(x, y, z0)))
    return m

def prism(x, y, z0, z1, r, shape):
    """vertical bar z0->z1: round cylinder (r = radius) or square (r = half-side)."""
    if shape == "square":
        return box(x, y, (z0 + z1) / 2, 2 * r, 2 * r, z1 - z0)
    return cyl(x, y, z0, z1, r)

def ball(x, y, z, r):
    m = mr.makeUVSphere(float(r), 24, 24)
    m.transform(mr.AffineXf3f.translation(V(x, y, z)))
    return m

def ray(mesh, origin, direction):
    d = V(*direction)
    n = (d.x*d.x + d.y*d.y + d.z*d.z) ** 0.5
    h = mr.rayMeshIntersect(mesh, mr.Line3f(V(*origin), V(d.x/n, d.y/n, d.z/n)))
    return None if h is None else (h.proj.point.x, h.proj.point.y, h.proj.point.z)

def keep_largest(m):
    keep = mr.MeshComponents.getLargestComponent(mr.MeshPart(m))
    drop = m.topology.getValidFaces() - keep
    if drop.any():
        m.deleteFaces(drop)
        m.pack()
    return m

def slab_below(m, cx, cy, ztop, span):
    """bottom slice of m up to ztop - cradle/lip offsets only need geometry
    near the base, and offsetting a slice is much cheaper than the full mesh."""
    (_, _, zlo), _ = bbox(m)
    return boolop(m, box(cx, cy, (zlo - 5.0 + ztop) / 2, span, span, ztop - (zlo - 5.0)), ISECT)


# ============================================================ pipeline
#
# Two phases so a GUI can build the shell, let the user place vents / feet on the
# ACTUAL shell, then finish:
#     st = build_shell(cfg, master, pour=, split=)     # heal..pour, no vents/feet/split
#     res = finish_mould(cfg, st, vents=, feet=)        # vents, feet, lip, cradle, split
# generate() just chains them (the one-shot CLI path).

def generate(cfg: Config, master, pour=None, vents=None, feet=None, split=None):
    return finish_mould(cfg, build_shell(cfg, master, pour=pour, split=split),
                        vents=vents, feet=feet)


def build_shell(cfg: Config, master, pour=None, split=None):
    """Phase A - heal, offsets, flange, cavity carve, flat bottom, pour inlet.
    `split` = (x, y) parting-plane position in world mm, else cfg.SPLIT_* , else
    model centre. Returns a `state` dict consumed by finish_mould()."""
    (mnx, mny, mnz), (mxx, mxy, mxz) = bbox(master)
    cx, cy = (mnx + mxx) / 2, (mny + mxy) / 2
    size = (mxx - mnx, mxy - mny, mxz - mnz)
    maxdim = max(size)
    diag = (size[0]**2 + size[1]**2 + size[2]**2) ** 0.5
    span = maxdim * 4.0
    cutz = mnz
    sxs, sys_ = (split if split else (cfg.SPLIT_X, cfg.SPLIT_Y))
    split_x = sxs if sxs is not None else cx
    split_y = sys_ if sys_ is not None else cy
    vs = cfg.VOXEL if cfg.VOXEL else min(max(diag / 200.0, 0.30), 1.5)
    log(f"master {nfaces(master):,} tris  bbox {tuple(round(s,1) for s in size)}  "
        f"voxel {vs:.3f}  split ({split_x:.1f},{split_y:.1f})")

    # -- heal the source ONCE (SDF round-trip). Raw STLs have self-intersections
    # / non-manifold edges that make later slab clips and booleans fail.
    master = offset(master, 0.0, vs, raw=True)
    log(f"healed master {nfaces(master):,} tris")

    outer  = offset(master, cfg.SHELL_OFFSET, vs, raw=True)
    cavity = offset(master, cfg.CORE_OFFSET,  vs, raw=True)            # silicone-gap solid
    log(f"outer {nfaces(outer):,}  cavity {nfaces(cavity):,}")

    # -- flange bands sit ON the parting planes (split_x / split_y), not the
    # bbox centre, so the cut lands in flange meat not thin shell.
    (_, _, ozlo), (_, _, ozhi) = bbox(outer)
    if cfg.FLANGE_THICK > 0:
        fblob = offset(outer, cfg.FLANGE_REACH, vs)
        zc = (ozlo - 10.0 + ozhi) / 2
        zh = ozhi - (ozlo - 10.0)
        outer = boolop(outer, boolop(fblob, box(cx, split_y, zc, span, cfg.FLANGE_THICK, zh), ISECT), UNION)
        if cfg.FOUR_PIECE:
            outer = boolop(outer, boolop(fblob, box(split_x, cy, zc, cfg.FLANGE_THICK, span, zh), ISECT), UNION)
        log(f"flange merged   outer {nfaces(outer):,}")

    body = boolop(outer, cavity, DIFF)
    body = boolop(body, box(cx, cy, cutz + span, 4*maxdim, 4*maxdim, 2*span), ISECT)   # flat open bottom
    log(f"body {nfaces(body):,}  (cavity carved, bottom open @ z={cutz:.1f})")

    # -- pour inlet: post + bore + funnel at the cavity apex; POUR_SHAPE round|square --
    def apex(mesh, pts):
        if pts:
            p = pts[0]
            h = ray(mesh, (p[0], p[1], mxz + span), (0, 0, -1))
            return h or p
        (x0, y0, _), (x1, y1, z1) = bbox(mesh)
        best, g = None, 14
        for i in range(g):
            for j in range(g):
                x = x0 + (x1 - x0) * (i + .5) / g
                y = y0 + (y1 - y0) * (j + .5) / g
                h = ray(mesh, (x, y, z1 + span), (0, 0, -1))
                if h and (best is None or h[2] > best[2]):
                    best = h
        return best

    shp = cfg.POUR_SHAPE
    sp = apex(cavity, pour)
    res_top = sp[2] + cfg.POUR_RES_H
    pbore = min(cfg.POUR_BORE, cfg.POUR_R - 1.5)
    fbot = res_top - min(cfg.FUNNEL_H, cfg.POUR_RES_H - 1.0)
    body = boolop(body, prism(sp[0], sp[1], sp[2] + 0.5, res_top, cfg.POUR_R, shp), UNION)
    if shp == "square":            # square counterbore mouth instead of a cone
        body = boolop(body, box(sp[0], sp[1], (fbot + res_top + 1) / 2,
                                2*(cfg.POUR_R - 1), 2*(cfg.POUR_R - 1), res_top + 1 - fbot), DIFF)
    else:
        body = boolop(body, cone(sp[0], sp[1], fbot, res_top + 1.0, pbore, cfg.POUR_R - 1.0), DIFF)
    body = boolop(body, prism(sp[0], sp[1], sp[2] - 3.0, fbot + 0.5, pbore, shp), DIFF)   # bore
    log(f"pour {shp} @ ({sp[0]:.0f},{sp[1]:.0f})  R{cfg.POUR_R}/bore{pbore}  top z={res_top:.1f}")

    return dict(master=master, body=body, cavity=cavity, sp=sp, res_top=res_top,
                cx=cx, cy=cy, cutz=cutz, span=span, maxdim=maxdim, vs=vs,
                mnz=mnz, mxz=mxz, size=size, split_x=split_x, split_y=split_y)


def finish_mould(cfg: Config, st, vents=None, feet=None):
    """Phase B - air vents, stab feet, clamping lip, cradle, split + pins.
    `vents` / `feet` are [(x,y,z), ...] picked on the SHELL (or None -> auto).
    Returns dict(pieces=[(tag, Mesh)], cradle=Mesh|None)."""
    master, body, cavity = st["master"], st["body"], st["cavity"]
    sp, res_top = st["sp"], st["res_top"]
    cx, cy, cutz, span = st["cx"], st["cy"], st["cutz"], st["span"]
    maxdim, vs, mnz, mxz, size = st["maxdim"], st["vs"], st["mnz"], st["mxz"], st["size"]
    split_x, split_y = st["split_x"], st["split_y"]

    # -- 4. air vents --------------------------------------------
    if cfg.ADD_VENTS:
        if vents:
            vpts = [ray(cavity, (p[0], p[1], mxz + span), (0, 0, -1)) or p for p in vents]
        else:
            (x0, y0, _), (x1, y1, z1) = bbox(cavity)
            cand, g = [], 16
            for i in range(g):
                for j in range(g):
                    x = x0 + (x1 - x0) * (i + .5) / g
                    y = y0 + (y1 - y0) * (j + .5) / g
                    h = ray(cavity, (x, y, z1 + span), (0, 0, -1))
                    if h:
                        cand.append(h)
            cand.sort(key=lambda p: -p[2])
            vpts = []
            for c in cand:
                if math.dist(c[:2], sp[:2]) < cfg.VENT_MIN_SEP:
                    continue
                if any(math.dist(c[:2], q[:2]) < cfg.VENT_MIN_SEP for q in vpts):
                    continue
                vpts.append(c)
                if len(vpts) >= cfg.VENT_MAX_N:
                    break
        done = 0
        for vp in vpts:
            try:                                   # one bad vent must not sink the run
                post = boolop(cyl(vp[0], vp[1], vp[2] + 0.5, res_top, cfg.VENT_POST_R), cavity, DIFF)
                nb = boolop(body, post, UNION)
                nb = boolop(nb, cyl(vp[0], vp[1], vp[2] - 2.0, res_top + 1.0, cfg.VENT_BORE), DIFF)
                body, done = nb, done + 1
            except RuntimeError as e:
                log(f"  vent @ ({vp[0]:.0f},{vp[1]:.0f}) skipped: {e}")
        log(f"vents: {done}/{len(vpts)}")

    # -- 5. stabilisation feet ----------------------------------
    if cfg.ADD_FEET:
        foot_top = res_top + 1.0
        (bx0, by0, bz0), (bx1, by1, bz1) = bbox(body)
        bcx, bcy = (bx0 + bx1) / 2, (by0 + by1) / 2
        placed = 0
        if feet:
            spots = [(p[0], p[1]) for p in feet]
        else:
            per = 2 * ((bx1 - bx0) + (by1 - by0))
            n = max(4, cfg.STAB_N or int(per / 80))
            spots = []
            midz = bz0 + (mxz - bz0) * 0.3
            for k in range(n):
                th = 2*math.pi * k / n + math.pi / n
                dx, dy = math.cos(th), math.sin(th)
                hz = ray(body, (bcx, bcy, midz), (dx, dy, 0))
                if not hz:
                    continue
                spots.append((hz[0] - dx * cfg.STAB_R * 2.0, hz[1] - dy * cfg.STAB_R * 2.0))
        for fx, fy in spots:
            hit = ray(body, (fx, fy, bz1 + 20), (0, 0, -1))
            base = (hit[2] if hit else bz0) - 5.0
            if foot_top - base < 1.0:
                continue
            try:                                   # one bad foot must not sink the run
                f = boolop(cone(fx, fy, base, foot_top, cfg.STAB_R, cfg.STAB_TIP_R), cavity, DIFF)
                body = boolop(body, f, UNION)
                placed += 1
            except RuntimeError as e:
                log(f"  foot @ ({fx:.0f},{fy:.0f}) skipped: {e}")
        log(f"feet: {placed}/{len(spots)}")

    body = keep_largest(body)

    # -- 6. clamping lip (annulus: hole = cavity, so it overlaps the body wall)
    body_prelip = body
    if cfg.ADD_LIP:
        reach = cfg.SHELL_OFFSET + cfg.CRADLE_MARGIN - cfg.LIP_INSET
        m_lip = slab_below(master, cx, cy, cutz + cfg.LIP_THICK + reach + 2.0, span)   # only base needed
        disc = offset(m_lip, reach, vs)
        disc = boolop(disc, box(cx, cy, cutz + cfg.LIP_THICK / 2, span, span, cfg.LIP_THICK), ISECT)
        # carve the centre with master+(CORE_OFFSET+0.6): that surface sits INSIDE the
        # shell wall (CORE_OFFSET..SHELL_OFFSET), so the lip overlaps the body wall by
        # ~0.6 mm (clean union, no coincident faces -> no perimeter tunnel) yet never
        # pokes into the cavity void.
        lip_hole = offset(m_lip, cfg.CORE_OFFSET + 0.6, vs)
        lip = boolop(disc, lip_hole, DIFF)
        body = boolop(body, lip, UNION)
        body = keep_largest(body)
        log(f"lip merged   body {nfaces(body):,}")

    # -- 7. cradle -----------------------------------------------
    cradle = None
    if cfg.ADD_CRADLE:
        rim_z = cutz + cfg.CRADLE_RECESS
        reach = cfg.CRADLE_TOL + cfg.CRADLE_MARGIN
        pre = slab_below(body_prelip, cx, cy, rim_z + reach + 2.0, span)
        shell_pre = offset(pre, cfg.CRADLE_TOL, vs)                 # fixed outer edge (pre-lip)
        outer_blob = offset(shell_pre, cfg.CRADLE_MARGIN, vs)
        bslc = slab_below(body, cx, cy, rim_z + cfg.CRADLE_TOL + 2.0, span)
        shell_cut = offset(bslc, cfg.CRADLE_TOL, vs)                # groove fits the lip (post-lip)
        mslc = slab_below(master, cx, cy, rim_z + cfg.CRADLE_TOL + 2.0, span)
        model_cut = offset(mslc, cfg.CRADLE_TOL, vs)
        bot_z = cutz - cfg.CRADLE_FLOOR
        plate = boolop(outer_blob, box(cx, cy, (rim_z + bot_z) / 2, span, span, rim_z - bot_z), ISECT)
        plate = boolop(plate, shell_cut, DIFF)
        plate = boolop(plate, model_cut, DIFF)
        cradle = keep_largest(plate)
        log(f"cradle {nfaces(cradle):,}")

    # -- 8. split + registration pins --------------------------
    n_pins = cfg.PIN_COUNT or max(1, int(size[2] / 40) + 1)
    if n_pins == 1:
        pin_zs = [(mnz + mxz) / 2]
    else:
        lo, hi = mnz + 0.15 * size[2], mnz + 0.85 * size[2]
        pin_zs = [lo + (hi - lo) * k / (n_pins - 1) for k in range(n_pins)]
    inset = cfg.PIN_RADIUS + 2.0

    def edge_x(plus, pz):   # scan along X at the parting line y=split_y for the shell edge
        h = ray(body, (split_x + (span if plus else -span), split_y, pz), (-1 if plus else 1, 0, 0))
        return None if h is None else (h[0] - inset if plus else h[0] + inset)

    def edge_y(plus, pz):
        h = ray(body, (split_x, split_y + (span if plus else -span), pz), (0, -1 if plus else 1, 0))
        return None if h is None else (h[1] - inset if plus else h[1] + inset)

    def key(piece, pt, male):
        if male:
            return boolop(piece, ball(*pt, cfg.PIN_RADIUS), UNION)
        return boolop(piece, ball(*pt, cfg.PIN_RADIUS + cfg.PIN_CLEAR), DIFF)

    def quad_clip(xlo, ylo):
        bx = split_x - span if xlo else split_x
        by = split_y - span if ylo else split_y
        return mr.makeCube(V(span, span, 2*span), V(bx, by, cutz - span))

    pieces = []
    if cfg.FOUR_PIECE:
        combos = [("xl_yl", 1, 1), ("xh_yl", 0, 1), ("xl_yh", 1, 0), ("xh_yh", 0, 0)]
        for tag, xlo, ylo in combos:
            p = boolop(body, quad_clip(xlo, ylo), ISECT)
            p = keep_largest(p)
            if cfg.ADD_PINS:
                for pz in pin_zs:
                    px = edge_x(not xlo, pz)
                    if px is not None:
                        p = key(p, (px, split_y, pz), male=not ylo)
                    py = edge_y(not ylo, pz)
                    if py is not None:
                        p = key(p, (split_x, py, pz), male=not xlo)
            pieces.append((tag, p))
    else:
        for tag, ylo in (("yl", 1), ("yh", 0)):
            by = split_y - span if ylo else split_y
            p = boolop(body, mr.makeCube(V(2*span, span, 2*span), V(cx - span, by, cutz - span)), ISECT)
            p = keep_largest(p)
            if cfg.ADD_PINS:
                for pz in pin_zs:
                    for plus in (False, True):
                        px = edge_x(plus, pz)
                        if px is not None:
                            p = key(p, (px, split_y, pz), male=not ylo)
            pieces.append((tag, p))
    log(f"split -> {len(pieces)} pieces" + ("  + pins" if cfg.ADD_PINS else ""))

    return dict(pieces=pieces, cradle=cradle, pour=sp, res_top=res_top)


# ============================================================ cli / io
def export(mesh, path, max_err, voxel=None, reheal=False):
    keep_largest(mesh)
    if reheal:
        # SDF round-trip removes tiny self-intersections / stray handles the
        # boolean chain can leave (e.g. flange-recess x shell-recess on the cradle)
        try:
            mesh = offset(mesh, 0.0, voxel)
            keep_largest(mesh)
        except Exception:
            pass
    s = mr.DecimateSettings()
    s.maxError = max_err
    s.packMesh = True
    mr.decimateMesh(mesh, s)
    mr.saveMesh(mesh, path)
    return mesh, nfaces(mesh), os.path.getsize(path)


def validate(mesh, maxdim, kind="piece"):
    """geometry sanity for the test gate.

    NOTE on euler: a mould piece that carries the pour or a vent legitimately has
    a channel through it -> genus > 0 -> euler < 2. That is correct, not a defect.
    So the gate checks watertight + single component + a sane bounding box (this is
    what catches the Blender-style cutter-box-fragment garbage: bbox ~4x model),
    and only flags euler when it is wildly negative (real corruption) or when a
    CRADLE - which has no channels by design - comes out non-genus-0."""
    import trimesh, io
    buf = io.BytesIO()
    mr.saveMesh(mesh, "*.stl", buf)
    buf.seek(0)
    tm = trimesh.load(buf, file_type="stl", force="mesh")
    comps = len(tm.split(only_watertight=False))
    ext = float(max(tm.extents))
    euler = int(tm.euler_number)
    ok = (bool(tm.is_watertight) and comps == 1 and ext < 1.7 * maxdim and euler > -8)
    if kind == "cradle":
        ok = ok and euler >= 0        # 2 = clean tray, 0 = one stray micro-handle (tolerated)
    return dict(watertight=bool(tm.is_watertight), euler=euler, comps=comps,
                vol_ml=float(tm.volume) / 1000.0, ext=ext, ok=ok)


def main():
    args = sys.argv[1:]
    if not args:
        raise SystemExit("usage: make_mold2.py <master.stl> [outdir] [--two] [--novents] "
                         "[--nofeet] [--nopins] [--nocradle] [--nolip] [--voxel N] [--split X,Y] "
                         "[--split-x N] [--split-y N] [--pour-shape round|square]")
    src = os.path.abspath(args[0])
    outdir = os.path.dirname(src)
    cfg = Config()
    i = 1
    while i < len(args):
        a = args[i]
        if   a == "--two":      cfg.FOUR_PIECE = False
        elif a == "--novents":  cfg.ADD_VENTS = False
        elif a == "--nofeet":   cfg.ADD_FEET = False
        elif a == "--nopins":   cfg.ADD_PINS = False
        elif a == "--nocradle": cfg.ADD_CRADLE = False
        elif a == "--nolip":    cfg.ADD_LIP = False
        elif a == "--voxel":    cfg.VOXEL = float(args[i+1]); i += 1
        elif a == "--split":    cfg.SPLIT_X, cfg.SPLIT_Y = (float(v) for v in args[i+1].split(",")); i += 1
        elif a == "--split-x":  cfg.SPLIT_X = float(args[i+1]); i += 1
        elif a == "--split-y":  cfg.SPLIT_Y = float(args[i+1]); i += 1
        elif a == "--pour-shape": cfg.POUR_SHAPE = args[i+1]; i += 1        # round | square
        elif not a.startswith("--"): outdir = os.path.abspath(a)
        i += 1
    os.makedirs(outdir, exist_ok=True)
    stem = os.path.splitext(os.path.basename(src))[0]

    master = mr.loadMesh(src)
    (mnx, mny, mnz), (mxx, mxy, mxz) = bbox(master)
    maxdim = max(mxx - mnx, mxy - mny, mxz - mnz)
    res = generate(cfg, master)

    log("--- export + validate ---")
    bad = 0
    for tag, p in res["pieces"]:
        m2, n, b = export(p, os.path.join(outdir, f"{stem}_mould_{tag}.stl"), cfg.EXPORT_MAX_ERR)
        v = validate(m2, maxdim, "piece")
        flag = "" if v["ok"] else "  <-- CHECK"
        bad += 0 if v["ok"] else 1
        log(f"  {tag:6} {n:>7,}f {b/1e6:5.2f}MB  wt={v['watertight']} euler={v['euler']} "
            f"comp={v['comps']} vol={v['vol_ml']:.1f}ml ext={v['ext']:.0f}{flag}")
    if res["cradle"] is not None:
        m2, n, b = export(res["cradle"], os.path.join(outdir, f"{stem}_cradle.stl"),
                          cfg.EXPORT_MAX_ERR, voxel=cfg.VOXEL, reheal=True)
        v = validate(m2, maxdim, "cradle")
        flag = "" if v["ok"] else "  <-- CHECK"
        bad += 0 if v["ok"] else 1
        log(f"  cradle {n:>7,}f {b/1e6:5.2f}MB  wt={v['watertight']} euler={v['euler']} "
            f"comp={v['comps']} vol={v['vol_ml']:.1f}ml ext={v['ext']:.0f}{flag}")
    log(f"DONE  ({time.time()-_t0:.1f}s)   flagged: {bad}")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
