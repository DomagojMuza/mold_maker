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
import sys, os, time, math, itertools
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
    MODEL_OFFSET_X: float = 0.0        # mm, slides the master in X before anything else runs
    MODEL_OFFSET_Y: float = 0.0        # mm, slides the master in Y
    SPLIT_X: Optional[float] = None    # parting-plane pivot X (world mm); None -> model bbox centre
    SPLIT_Y: Optional[float] = None    # parting-plane pivot Y
    SPLIT_ANGLE_X: float = 0.0         # degrees, normal direction of the X-like plane (xlo/xhi); independent of Y
    SPLIT_ANGLE_Y: float = 90.0        # degrees, normal direction of the Y-like plane (ylo/yhi, and the only
                                       # plane used for --two). Default 90 = perpendicular to X, i.e. old behaviour.
    EXTRA_PLANES: List[Tuple[float, float, float]] = field(default_factory=list)
    # each (pivot_x, pivot_y, angle_deg) is one MORE full-height parting plane
    # beyond the base X/Y pair, with its OWN pivot -- for shapes (swept wings,
    # a protruding limb) where 2 planes crossing at one point can't give every
    # part a clean pull direction. Every extra plane doubles the candidate
    # piece count (one more sequential ISECT per piece); combos that don't
    # intersect the shell are dropped automatically. Registration pins on
    # extra planes use simple self-parity keying (independent of the other
    # planes) with an extra keep_largest() pass to drop a pin ball that landed
    # outside that particular cut -- less refined than the base X/Y pin
    # scheme, but robust for a pivot that isn't centred on the whole shell.

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

def rot_box(pivot_x, pivot_y, angle_deg, local_center, size):
    """A box built at `local_center` (offset from the origin, full `size`), then
    the WHOLE local frame is rotated by angle_deg around Z and moved so the local
    origin lands on (pivot_x, pivot_y). Since local_center encodes the box's
    offset from the origin, this rotates the box about the pivot, not its own
    centre -- exactly what a pair of parting planes crossing at a point need."""
    b = box(local_center[0], local_center[1], local_center[2], size[0], size[1], size[2])
    if angle_deg:
        R = mr.Matrix3f.rotation(V(0, 0, 1), math.radians(angle_deg))
        b.transform(mr.AffineXf3f(R, V(pivot_x, pivot_y, 0)))
    else:
        b.transform(mr.AffineXf3f.translation(V(pivot_x, pivot_y, 0)))
    return b

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

def local_radius(mesh, cx, cy, z, span, n=12):
    """Min surface distance from (cx,cy) at height z, sampled over n outward
    directions -- a cheap proxy for 'how wide is the model here'. Used to
    stop a wide pour post from overhanging a spot too thin to support it."""
    best = None
    for i in range(n):
        th = 2 * math.pi * i / n
        dx, dy = math.cos(th), math.sin(th)
        h = ray(mesh, (cx + span * dx, cy + span * dy, z), (-dx, -dy, 0))
        if h is not None:
            d = math.hypot(h[0] - cx, h[1] - cy)
            if best is None or d < best:
                best = d
    return best


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
    if cfg.MODEL_OFFSET_X or cfg.MODEL_OFFSET_Y:      # slide the master before anything reads its bbox
        master.transform(mr.AffineXf3f.translation(V(cfg.MODEL_OFFSET_X, cfg.MODEL_OFFSET_Y, 0.0)))
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
    angle_x, angle_y = cfg.SPLIT_ANGLE_X, cfg.SPLIT_ANGLE_Y   # each plane's own normal direction
    # full plane list: base X[,Y] pair (shared split_x/split_y pivot, as before)
    # + any extra planes, each with its OWN pivot/angle -- see Config.EXTRA_PLANES.
    planes = []
    if cfg.FOUR_PIECE:
        planes.append(("x", split_x, split_y, angle_x))
    planes.append(("y", split_x, split_y, angle_y))
    for i, (px, py, ang) in enumerate(cfg.EXTRA_PLANES):
        planes.append((f"e{i}", float(px), float(py), float(ang)))
    vs = cfg.VOXEL if cfg.VOXEL else min(max(diag / 200.0, 0.30), 1.5)
    log(f"master {nfaces(master):,} tris  bbox {tuple(round(s,1) for s in size)}  "
        f"voxel {vs:.3f}  split ({split_x:.1f},{split_y:.1f})  angles ({angle_x:.0f},{angle_y:.0f})deg  "
        f"planes {len(planes)}")

    # -- heal the source ONCE (SDF round-trip). Raw STLs have self-intersections
    # / non-manifold edges that make later slab clips and booleans fail.
    master = offset(master, 0.0, vs, raw=True)
    log(f"healed master {nfaces(master):,} tris")

    outer  = offset(master, cfg.SHELL_OFFSET, vs, raw=True)
    cavity = offset(master, cfg.CORE_OFFSET,  vs, raw=True)            # silicone-gap solid
    log(f"outer {nfaces(outer):,}  cavity {nfaces(cavity):,}")

    # -- flange bands sit ON the parting planes through the split pivot, not the
    # bbox centre, so the cut lands in flange meat not thin shell. Each plane
    # gets its own band, independently angled: a band thin along its own normal
    # and wide (spanning `span`) along the perpendicular is built by rotating a
    # (FLANGE_THICK, span, zh) box so its local x-axis (the thin one) lands on
    # that plane's own normal direction -- no shared frame between the two.
    def plane_band(px, py, angle_deg, zc, zh):
        return rot_box(px, py, angle_deg, (0, 0, zc), (cfg.FLANGE_THICK, span, zh))

    (_, _, ozlo), (_, _, ozhi) = bbox(outer)
    if cfg.FLANGE_THICK > 0:
        fblob = offset(outer, cfg.FLANGE_REACH, vs)
        zc = (ozlo - 10.0 + ozhi) / 2
        zh = ozhi - (ozlo - 10.0)
        merged = 0
        for label, px, py, ang in planes:
            try:                                   # one bad plane's flange must not sink the whole build
                band = plane_band(px, py, ang, zc, zh)
                outer = boolop(outer, boolop(fblob, band, ISECT), UNION)
                merged += 1
            except RuntimeError as e:
                log(f"  flange band '{label}' skipped: {e}")
        log(f"flange merged ({merged}/{len(planes)} plane(s))   outer {nfaces(outer):,}")

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
    pbore = max(0.1, cfg.POUR_BORE)                 # no upper clamp - bore can exceed the post
    if abs(pbore - cfg.POUR_R) < 0.3:              # bore == post wall -> coincident faces; nudge clear
        pbore = cfg.POUR_R + 0.3
    mouth = max(0.1, cfg.POUR_R - 1.0)              # funnel / counterbore mouth radius
    fbot = res_top - min(cfg.FUNNEL_H, cfg.POUR_RES_H - 1.0)

    # -- a post wider than the model actually is up here (thin spike/ridge +
    # a big requested hole) has no real material to bond to, and a full-width
    # bore dropped straight down can drill clean through the model's side
    # wall instead of just opening into the cavity from above. Measure the
    # local shell width at the apex and, if the post would exceed it, grow
    # BOTH the post and the bore from that local width up to the full
    # POUR_R/pbore with a matching cone taper -- same idea as the existing
    # top-of-bore funnel, mirrored at the base. The two ramps are linear so
    # their wall thickness only needs checking at the endpoints (its minimum
    # can't dip below either one): base_bore is kept >= MIN_WALL inside
    # base_r, and the top end inherits whatever gap POUR_R vs pbore already
    # has (including the near-equal nudge above, unaffected by this).
    MIN_WALL = 1.5
    local_r = local_radius(outer, sp[0], sp[1], sp[2], span)
    post_z0, bore_z0 = sp[2] + 0.5, sp[2] - 3.0
    tapered = False
    if local_r is not None and local_r < cfg.POUR_R - MIN_WALL:
        tapered = True
        # floor base_r/base_bore comfortably above the voxel size -- hugging an
        # exact local_r that's itself thinner than ~2 voxels (a knife-thin tip)
        # builds geometry the boolean engine can't reliably represent, so a
        # small overhang at the very base is the safer trade there
        base_r = max(local_r - MIN_WALL, vs * 3.0, 2.0)
        base_bore = max(min(pbore, base_r - MIN_WALL), vs * 1.5, 0.3)
        # cap the taper to the room actually available between the apex and
        # where the funnel/counterbore starts (fbot) -- a big POUR_R needing
        # a tall taper on a short POUR_RES_H can otherwise push taper_top
        # ABOVE fbot, which hands the final straight bore cut a NEGATIVE
        # height cylinder (z0 > z1); MeshLib doesn't error on that, it just
        # builds inverted/garbage geometry that erases the whole mesh on
        # subtraction. Always leave at least 1mm of straight section.
        max_taper_h = max(fbot - 1.0 - post_z0, 1.0)
        taper_h = min(max(cfg.POUR_R - base_r, 4.0), max_taper_h)
        taper_top = post_z0 + taper_h
        if taper_h < cfg.POUR_R - base_r:
            log(f"  pour taper compressed to {taper_h:.1f}mm (POUR_RES_H too short for a "
                f"smooth {cfg.POUR_R - base_r:.1f}mm taper here) - increase Pour post height for a smoother transition")
        OVERLAP = 0.5   # the straight post/bore below start a hair INSIDE the cone's
                         # top instead of exactly at it -- an exact shared seam at the
                         # same radius is a coincident-face crash (same issue as the
                         # lip-tunnel fix), a slice of real volumetric overlap isn't
        body = boolop(body, cone(sp[0], sp[1], post_z0, taper_top + OVERLAP, base_r, cfg.POUR_R), UNION)
        body = boolop(body, cone(sp[0], sp[1], bore_z0, taper_top + OVERLAP, base_bore, pbore), DIFF)
        post_z0 = bore_z0 = taper_top - OVERLAP
        log(f"  pour taper: local width {local_r:.1f}mm < post {cfg.POUR_R:.1f}mm -> "
            f"cone {base_r:.1f}->{cfg.POUR_R:.1f}mm over {taper_h:.1f}mm")

    body = boolop(body, prism(sp[0], sp[1], post_z0, res_top, cfg.POUR_R, shp), UNION)
    if shp == "square":            # square counterbore mouth instead of a cone
        body = boolop(body, box(sp[0], sp[1], (fbot + res_top + 1) / 2,
                                2*mouth, 2*mouth, res_top + 1 - fbot), DIFF)
    else:
        body = boolop(body, cone(sp[0], sp[1], fbot, res_top + 1.0, pbore, mouth), DIFF)
    body = boolop(body, prism(sp[0], sp[1], bore_z0, fbot + 0.5, pbore, shp), DIFF)   # bore
    if tapered:
        # the deliberate cone/cylinder overlap above (needed to dodge a
        # coincident-face crash) can leave a few tiny spurious handles at
        # this scale -- same SDF-reheal trick already used for the open-top
        # self-intersection fix elsewhere in this file. At an extreme taper
        # (a big POUR_R on a very thin apex) the SDF round-trip can itself
        # misbehave and gut the mesh instead of cleaning it up, so the
        # result is sanity-checked against the pre-reheal body and only
        # kept if it isn't drastically smaller -- a body with 0 (or a
        # handful of) faces here silently propagates into "empty" flange/
        # split failures much further downstream, which is far worse than
        # just skipping this cleanup pass.
        pre_faces = nfaces(body)
        try:
            healed = keep_largest(offset(body, 0.0, vs))
            if nfaces(healed) >= pre_faces * 0.5:
                body = healed
            else:
                log(f"  pour reheal shrank the shell ({pre_faces:,} -> {nfaces(healed):,} faces), keeping pre-reheal body")
        except RuntimeError as e:
            log(f"  pour reheal failed, keeping pre-reheal body: {e}")
    log(f"pour {shp} @ ({sp[0]:.0f},{sp[1]:.0f})  R{cfg.POUR_R}/bore{pbore}  top z={res_top:.1f}")

    return dict(master=master, body=body, cavity=cavity, sp=sp, res_top=res_top,
                cx=cx, cy=cy, cutz=cutz, span=span, maxdim=maxdim, vs=vs,
                mnz=mnz, mxz=mxz, size=size, split_x=split_x, split_y=split_y,
                angle_x=angle_x, angle_y=angle_y, planes=planes)


def finish_mould(cfg: Config, st, vents=None, feet=None):
    """Phase B - air vents, stab feet, clamping lip, cradle, split + pins.
    `vents` / `feet` are [(x,y,z), ...] picked on the SHELL (or None -> auto).
    Returns dict(pieces=[(tag, Mesh)], cradle=Mesh|None)."""
    master, body, cavity = st["master"], st["body"], st["cavity"]
    sp, res_top = st["sp"], st["res_top"]
    cx, cy, cutz, span = st["cx"], st["cy"], st["cutz"], st["span"]
    maxdim, vs, mnz, mxz, size = st["maxdim"], st["vs"], st["mnz"], st["mxz"], st["size"]
    split_x, split_y = st["split_x"], st["split_y"]
    angle_x, angle_y = st["angle_x"], st["angle_y"]
    planes = st["planes"]

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

    # Each plane keeps its OWN normal direction (angle_x, angle_y) -- they no
    # longer have to stay perpendicular. A plane's "band direction" (the line
    # pins run along, and the direction that stays exactly ON that plane) is
    # always 90deg from its own normal, independent of the other plane.
    def hat(deg):
        r = math.radians(deg)
        return (math.cos(r), math.sin(r))

    def edge_along(px, py, angle_deg, plus, pz):    # scan the given plane's own band line for the shell edge
        h = hat(angle_deg + 90)
        sgn = 1 if plus else -1
        ox, oy = px + sgn * span * h[0], py + sgn * span * h[1]
        hit = ray(body, (ox, oy, pz), (-sgn * h[0], -sgn * h[1], 0))
        if hit is None:
            return None
        d = (hit[0] - px) * h[0] + (hit[1] - py) * h[1] - sgn * inset   # distance from pivot along h
        return (px + d * h[0], py + d * h[1], pz)

    def key(piece, pt, male):
        if male:
            return boolop(piece, ball(*pt, cfg.PIN_RADIUS), UNION)
        return boolop(piece, ball(*pt, cfg.PIN_RADIUS + cfg.PIN_CLEAR), DIFF)

    def half_box(px, py, angle_deg, lo):
        c = -span / 2 if lo else span / 2
        return rot_box(px, py, angle_deg, (c, 0, cutz), (span, 2 * span, 2 * span))

    # N-plane split: `planes` = [("x",...), ("y",...), ("e0",...), ...] (see
    # build_shell). Each plane contributes ONE more sequential half-space
    # ISECT per piece -> up to 2**N candidate quadrants; ones that don't
    # actually intersect the shell (a real possibility once extra planes have
    # their own off-centre pivot) are silently dropped. The first n_base
    # planes (x[,y]) keep the EXACT original cross-wired pin scheme; any
    # extra planes beyond that get simple self-parity pins on their own band,
    # independent of the other planes, with a keep_largest() safety pass
    # afterwards to drop a pin ball that landed outside this particular cut.
    n_base = 2 if cfg.FOUR_PIECE else 1
    N = len(planes)
    pieces = []
    for combo in itertools.product((1, 0), repeat=N):    # 1 = lo side, 0 = hi side
        tag = "_".join(f"{planes[i][0]}{'l' if combo[i] else 'h'}" for i in range(N))
        try:
            p = body
            for i in range(N):
                _, px, py, ang = planes[i]
                p = boolop(p, half_box(px, py, ang, combo[i]), ISECT)
            if nfaces(p) < 20:
                log(f"  combo {tag} empty, skipped")
                continue
            p = keep_largest(p)
        except RuntimeError as e:
            log(f"  combo {tag} failed, skipped: {e}")
            continue

        if cfg.ADD_PINS:
            if cfg.FOUR_PIECE:
                xlo, ylo = combo[0], combo[1]
                _, xpx, xpy, xang = planes[0]
                _, ypx, ypy, yang = planes[1]
                for pz in pin_zs:
                    pu = edge_along(xpx, xpy, xang, not xlo, pz)
                    if pu is not None:
                        p = key(p, pu, male=not ylo)
                    pv = edge_along(ypx, ypy, yang, not ylo, pz)
                    if pv is not None:
                        p = key(p, pv, male=not xlo)
            else:
                ylo = combo[0]
                _, ypx, ypy, yang = planes[0]
                for pz in pin_zs:
                    for plus in (False, True):
                        pu = edge_along(ypx, ypy, yang, plus, pz)
                        if pu is not None:
                            p = key(p, pu, male=not ylo)
            for i in range(n_base, N):                    # extra planes: self-parity, both band ends
                lo = combo[i]
                _, px, py, ang = planes[i]
                for pz in pin_zs:
                    for plus in (False, True):
                        pu = edge_along(px, py, ang, plus, pz)
                        if pu is not None:
                            p = key(p, pu, male=not lo)
            if N > n_base:
                p = keep_largest(p)   # drop a pin ball that landed outside this cut

        # Final size gate on the piece as it will actually be exported: a pin ball
        # (getLargestComponent picks by FACE COUNT, not physical size) can outrank
        # a real but low-poly shell sliver at the step above, so this has to be
        # checked here -- after pins -- not on the pre-pin candidate.
        (pnx, pny, pnz), (pxx, pxy, pxz) = bbox(p)
        pext = max(pxx - pnx, pxy - pny, pxz - pnz)
        min_ext = max(10.0, 0.05 * maxdim)
        if nfaces(p) < 50 or pext < min_ext:
            log(f"  combo {tag} sliver ({nfaces(p)} faces, ext={pext:.1f}mm), skipped")
            continue

        pieces.append((tag, p))
    log(f"split -> {len(pieces)}/{2**N} candidate pieces" + ("  + pins" if cfg.ADD_PINS else ""))

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
                         "[--split-x N] [--split-y N] [--split-angle-x DEG] [--split-angle-y DEG] "
                         "[--offset-x N] [--offset-y N] [--pour-shape round|square] "
                         "[--extra-plane X,Y,DEG ...]")
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
        elif a == "--split-angle-x": cfg.SPLIT_ANGLE_X = float(args[i+1]); i += 1
        elif a == "--split-angle-y": cfg.SPLIT_ANGLE_Y = float(args[i+1]); i += 1
        elif a == "--offset-x": cfg.MODEL_OFFSET_X = float(args[i+1]); i += 1
        elif a == "--offset-y": cfg.MODEL_OFFSET_Y = float(args[i+1]); i += 1
        elif a == "--pour-shape": cfg.POUR_SHAPE = args[i+1]; i += 1        # round | square
        elif a == "--extra-plane":
            x, y, ang = (float(v) for v in args[i+1].split(","))
            cfg.EXTRA_PLANES.append((x, y, ang)); i += 1
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
