#!/usr/bin/env python3
"""
make_mold.py  --  automated cast-mould generator (Blender headless)

Matrix (glove) mould: a rigid shell that the base MODEL sits inside; silicone is
poured into the gap (= CORE_OFFSET) between model and shell. A separate cradle
locates both and seals the open bottom.

Pipeline (no clicking):
  1. import master mesh (STL)
  2. decimate to target poly budget (skips if already low)
  3. clean / make manifold via voxel remesh
  4. build OUTER solid  = model offset OUT by SHELL_OFFSET   (the shell wall)
  5. build FLANGE blob  = model offset OUT by SHELL_OFFSET+FLANGE_REACH
        -> intersect a thin vertical slab (FLANGE_THICK) on the parting plane(s)
  6. shell body = OUTER (union flange band(s)) MINUS the NEGATIVE (model+gap)
        -> the silicone gap (= model + CORE_OFFSET) is carved out
  7. flat-bottom cut: bottom stays OPEN (the cradle is the floor)
  8. sprue funnel on top (top pour mode)
  9. cradle STL: centre pocket = model, outer recess = shell, ridge = gap seal
 10. split body by vertical plane(s) + ball registration keys:
        1 flange -> 2 pieces   |   2 flanges @90deg -> 4 pieces
 11. export shell pieces + cradle as STL

Run:
  blender --background --python make_mold.py -- input.stl [outdir] [--four] [--voxel 0.6] [--target 6000]

Notes:
  * split planes are VERTICAL (contain Z), centred on the bounding box.
  * --four  = two flanges 90deg apart -> 4-piece mould (default).
  * offsets are TRUE constant-distance offsets (stepped dilation + voxel remesh,
    i.e. discrete Minkowski sum with a sphere), same idea as Meshmixer "Make Solid".
"""

import bpy, bmesh, sys, os
from mathutils import Vector

# ----------------------------------------------------------------------------
# CONFIG (defaults -- override some via CLI)
# ----------------------------------------------------------------------------
SHELL_OFFSET  = 4.2    # mm, mould wall thickness (model offset OUT -> shell)
CORE_OFFSET   = 3.0    # mm, cavity clearance: cavity = model grown by this
                       #   (0 = exact model -> solid cast; >0 = looser cavity)
FLANGE_REACH  = 10.0   # mm, how far flange lip sticks out past the shell
FLANGE_THICK  = 6.0    # mm, thickness of the flange slab (clamp meat)
BASE_CUT      = None   # mm up from the lowest point to slice a FLAT bottom
                       #   (pour mouth + stand). None -> auto = 8% of height.
DECIMATE_TGT  = 6000   # target triangle count (decimate only if above)
VOXEL_DETAIL  = 220    # remesh divisions across bbox diagonal (higher = finer/slower)
FOUR_PIECE    = True   # split into 4 (two planes @90deg) vs 2 (one plane)
ADD_FLANGE    = True   # add flange lip(s) on the parting plane(s). False = bare split
ADD_PINS      = True   # ball registration keys on the flange (needs flange ON)
PIN_RADIUS    = 2.0    # mm, ball key radius (must be < FLANGE_THICK/2 to stay backed)
PIN_CLEAR     = 0.25   # mm, socket oversize so the male ball drops in
PIN_COUNT     = None   # pins per interface up the height. None -> auto (1 per ~40mm)
VOXEL_SIZE    = None   # set via --voxel to override auto value (mm)

# pour inlet  (bottom is always OPEN -- the cradle is the floor)
POUR_MODE     = "top"  # "top" = add a sprue funnel to pour from the top
                       # "bottom" = no sprue, pour through the open bottom
SPRUE_TOP_R   = 7.0    # mm, funnel mouth radius (top of sprue)
SPRUE_THROAT_R= 2.5    # mm, funnel throat radius (where it enters the cavity)

# base cradle: locates the base MODEL (centre pocket) AND the mould shell (outer
# recess); the thin ridge between them seals the silicone gap at the bottom.
ADD_CRADLE    = True
CRADLE_WALL   = 4.0    # mm, wall thickness around the shell recess
CRADLE_RECESS = 2.0    # mm, how deep model + shell seat into the cradle
                       #   (just enough to locate; deeper = more model lost in base)
CRADLE_FLOOR  = 3.0    # mm, tray floor thickness
CRADLE_TOL    = 0.5    # mm, pocket oversize (fit + base-material expansion)

# ----------------------------------------------------------------------------
# small helpers
# ----------------------------------------------------------------------------
def log(msg): print(f"[mould] {msg}")

def activate(obj):
    try: bpy.ops.object.mode_set(mode='OBJECT')
    except Exception: pass
    for o in bpy.context.scene.objects: o.select_set(False)
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj

def apply_mod(obj, name):
    activate(obj)
    bpy.ops.object.modifier_apply(modifier=name)

def dup(obj, name):
    n = obj.copy(); n.data = obj.data.copy(); n.name = name
    bpy.context.collection.objects.link(n)
    return n

def rm(obj):
    if obj and obj.name in bpy.data.objects:
        bpy.data.objects.remove(obj, do_unlink=True)

def world_bbox(obj):
    cs = [obj.matrix_world @ Vector(c) for c in obj.bound_box]
    mn = Vector((min(c.x for c in cs), min(c.y for c in cs), min(c.z for c in cs)))
    mx = Vector((max(c.x for c in cs), max(c.y for c in cs), max(c.z for c in cs)))
    return mn, mx

def offset_inplace(obj, dist):
    """Move every vertex along its normal by dist (mm)."""
    me = obj.data
    bm = bmesh.new(); bm.from_mesh(me)
    bm.normal_update()
    for v in bm.verts:
        v.co += v.normal * dist
    bm.to_mesh(me); bm.free()

def voxel_remesh(obj, vs):
    activate(obj)
    obj.data.remesh_voxel_size = vs
    obj.data.remesh_voxel_adaptivity = 0.0
    bpy.ops.object.voxel_remesh()

def offset_solid(base, dist, vs, name):
    """Constant-distance offset via stepped dilation: displace a little, remesh,
    repeat. Small steps avoid the self-intersection tangle that wrecks a single
    big vertex-normal push, so it stays robust at any offset distance."""
    o = dup(base, name)
    step_max = max(1.5 * vs, 1.5)             # mm of displacement per step
    n = int(abs(dist) / step_max) + 1
    d = dist / n
    for _ in range(n):
        offset_inplace(o, d)
        voxel_remesh(o, vs)                   # heal before the next push
    return o

def boolean(a, b, op, keep_b=False):
    """a = a <op> b   (op: 'UNION' | 'DIFFERENCE' | 'INTERSECT')."""
    m = a.modifiers.new("bool", 'BOOLEAN')
    m.operation = op
    m.solver = 'EXACT'
    m.object = b
    apply_mod(a, m.name)
    if not keep_b:
        rm(b)

def make_box(name, center, dims):
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=center)
    c = bpy.context.active_object
    c.name = name
    c.scale = (dims[0], dims[1], dims[2])
    activate(c)
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    return c

def make_ball(name, center, r):
    bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=3, radius=r, location=center)
    c = bpy.context.active_object
    c.name = name
    activate(c)
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    return c

def make_cone(name, center, r_bot, r_top, depth):
    bpy.ops.mesh.primitive_cone_add(radius1=r_bot, radius2=r_top, depth=depth,
                                    vertices=48, location=center)
    c = bpy.context.active_object
    c.name = name
    activate(c)
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    return c

def make_cyl(name, center, r, depth):
    bpy.ops.mesh.primitive_cylinder_add(radius=r, depth=depth, vertices=32, location=center)
    c = bpy.context.active_object
    c.name = name
    activate(c)
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    return c

def ray_edge(obj, origin, direction):
    """world-space hit point where a ray first meets obj, or None."""
    mw = obj.matrix_world; invw = mw.inverted()
    ol = invw @ Vector(origin)
    dl = (invw.to_3x3() @ Vector(direction)).normalized()
    res = obj.ray_cast(ol, dl)
    return (mw @ res[1]) if res[0] else None

def mesh_volume(obj):
    bm = bmesh.new(); bm.from_mesh(obj.data); v = bm.calc_volume(); bm.free()
    return abs(v)

def export_stl(obj, path):
    activate(obj)
    try:
        bpy.ops.wm.stl_export(filepath=path, export_selected_objects=True)   # Blender 4.x
    except AttributeError:
        bpy.ops.export_mesh.stl(filepath=path, use_selection=True)           # legacy
    log(f"wrote {path}")

# ----------------------------------------------------------------------------
# argument parsing (after the "--")
# ----------------------------------------------------------------------------
argv = sys.argv[sys.argv.index("--")+1:] if "--" in sys.argv else []
if not argv:
    raise SystemExit("usage: blender -b --python make_mold.py -- input.stl [outdir] [--four] [--voxel N] [--target N]")

in_path = os.path.abspath(argv[0])
outdir  = os.path.dirname(in_path)
i = 1
while i < len(argv):
    a = argv[i]
    if a == "--four":   FOUR_PIECE = True
    elif a == "--two":  FOUR_PIECE = False
    elif a == "--voxel":  VOXEL_SIZE = float(argv[i+1]); i += 1
    elif a == "--target": DECIMATE_TGT = int(argv[i+1]); i += 1
    elif a == "--base":   BASE_CUT = float(argv[i+1]); i += 1
    elif a == "--nobase": BASE_CUT = -1   # disable bottom cut
    elif a == "--core":   CORE_OFFSET = float(argv[i+1]); i += 1   # cavity clearance
    elif a == "--shell":  SHELL_OFFSET = float(argv[i+1]); i += 1
    elif a == "--flange":   ADD_FLANGE = True
    elif a == "--noflange": ADD_FLANGE = False
    elif a == "--pins":     ADD_PINS = True
    elif a == "--nopins":   ADD_PINS = False
    elif a == "--pin":      PIN_RADIUS = float(argv[i+1]); i += 1
    elif a == "--pins-n":   PIN_COUNT = int(argv[i+1]); i += 1
    elif a == "--pour":     POUR_MODE = argv[i+1]; i += 1   # "top" | "bottom"
    elif a == "--nocradle": ADD_CRADLE = False
    elif not a.startswith("--"): outdir = os.path.abspath(a)
    i += 1
os.makedirs(outdir, exist_ok=True)
stem = os.path.splitext(os.path.basename(in_path))[0]

# ----------------------------------------------------------------------------
# 0. fresh scene + import
# ----------------------------------------------------------------------------
bpy.ops.wm.read_factory_settings(use_empty=True)
log(f"import {in_path}")
try:
    bpy.ops.wm.stl_import(filepath=in_path)         # Blender 4.x
except AttributeError:
    bpy.ops.import_mesh.stl(filepath=in_path)       # legacy

# join everything that came in, bake transforms
meshes = [o for o in bpy.context.scene.objects if o.type == 'MESH']
activate(meshes[0])
for o in meshes: o.select_set(True)
if len(meshes) > 1:
    bpy.ops.object.join()
master = bpy.context.active_object
master.name = "master"
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)

# ----------------------------------------------------------------------------
# 1. decimate to poly budget
# ----------------------------------------------------------------------------
ntri = len(master.data.polygons)
if ntri > DECIMATE_TGT:
    m = master.modifiers.new("dec", 'DECIMATE')
    m.ratio = DECIMATE_TGT / ntri
    apply_mod(master, m.name)
    log(f"decimate {ntri} -> {len(master.data.polygons)} tris")
else:
    log(f"skip decimate ({ntri} tris already <= {DECIMATE_TGT})")

# ----------------------------------------------------------------------------
# 2. geometry params from bbox
# ----------------------------------------------------------------------------
mn, mx = world_bbox(master)
center = (mn + mx) * 0.5
size   = mx - mn
maxdim = max(size.x, size.y, size.z)
diag   = size.length
span   = maxdim * 4.0                       # big enough to cover everything
vs     = VOXEL_SIZE if VOXEL_SIZE else max(diag / VOXEL_DETAIL, 0.25)
log(f"bbox {tuple(round(v,1) for v in size)}  voxel={vs:.3f}mm")

cutz = None   # flat-bottom cut height, decided after the cavity is built (pour mode)

def cut_bottom_box(obj):
    """Flat bottom via boolean intersect with a half-space box (caps the ring
    cross-section of the hollow mould body and gives a flat foot + pour mouth)."""
    if cutz is None:
        return
    h = (mx.z + maxdim) - cutz                          # tight box just above the part
    keep = make_box(obj.name + "_keep",
                    Vector((center.x, center.y, cutz + h / 2)),
                    (4 * maxdim, 4 * maxdim, h))
    boolean(obj, keep, 'INTERSECT')

# heal the master itself so booleans are robust
voxel_remesh(master, vs)

# ----------------------------------------------------------------------------
# 3. OUTER shell solid + optional FLANGE band(s)
# ----------------------------------------------------------------------------
outer = offset_solid(master, SHELL_OFFSET, vs, "outer")

if ADD_FLANGE:
    flangeblob = offset_solid(master, SHELL_OFFSET + FLANGE_REACH, vs, "flangeblob")

    def flange_band(normal_axis):
        """slab thin along normal_axis, centred on parting plane, clipped to flangeblob."""
        if normal_axis == 'Y':
            dims = (span, FLANGE_THICK, span)
        else:  # 'X'
            dims = (FLANGE_THICK, span, span)
        slab = make_box(f"slab_{normal_axis}", center, dims)
        band = dup(flangeblob, f"band_{normal_axis}")
        boolean(band, slab, 'INTERSECT')        # band = flangeblob ∩ slab
        return band

    boolean(outer, flange_band('Y'), 'UNION')
    if FOUR_PIECE:
        boolean(outer, flange_band('X'), 'UNION')
    rm(flangeblob)
    log("flange: ON")
else:
    log("flange: OFF (bare split)")

# ----------------------------------------------------------------------------
# 4. carve the pour cavity (NEGATIVE cuts the cavity out of the shell)
# ----------------------------------------------------------------------------
negative = offset_solid(master, CORE_OFFSET, vs, "negative") if CORE_OFFSET > 1e-6 \
           else dup(master, "negative")
neg_mn, neg_mx = world_bbox(negative)
cavity_vol = mesh_volume(negative)
log(f"cavity volume ~ {cavity_vol/1000:.1f} cm3 -> ~{cavity_vol/1000:.1f} ml of cast material")
log(f"mould wall = {SHELL_OFFSET}mm  (cavity clearance {CORE_OFFSET}mm)")

# sample cavity high points (ray straight down onto the negative) for sprue+vents
high_pts = []
if POUR_MODE == "top":
    g = 18
    for ix in range(g):
        for iy in range(g):
            x = neg_mn.x + (neg_mx.x - neg_mn.x) * (ix + 0.5) / g
            y = neg_mn.y + (neg_mx.y - neg_mn.y) * (iy + 0.5) / g
            hit = ray_edge(negative, (x, y, neg_mx.z + span), (0, 0, -1))
            if hit:
                high_pts.append(hit)
    high_pts.sort(key=lambda p: -p.z)

boolean(outer, negative, 'DIFFERENCE')         # negative consumed
body = outer
body.name = "mould_body"

# ----------------------------------------------------------------------------
# flat-bottom cut: bottom is ALWAYS open (the cradle is the floor). Cut at the
# model bottom so the cavity opens there and the model can seat in the cradle.
# ----------------------------------------------------------------------------
if BASE_CUT is not None and BASE_CUT < 0:
    cutz = None                                 # --nobase: no cut
else:
    cutz = mn.z + (BASE_CUT or 0.0)
if cutz is not None:
    log(f"flat bottom cut (open) @ z={cutz:.2f}")
cut_bottom_box(body)

# ----------------------------------------------------------------------------
# 5. pour inlet: sprue funnel -- top mode only
# ----------------------------------------------------------------------------
if POUR_MODE == "top" and high_pts:
    top_z = world_bbox(body)[1].z + 1.0
    sp = high_pts[0]                            # cavity apex -> sprue
    throat_z = sp.z - 1.5                       # punch slightly into the cavity
    boolean(body, make_cone("sprue",
                            Vector((sp.x, sp.y, (top_z + throat_z) / 2)),
                            SPRUE_THROAT_R, SPRUE_TOP_R, top_z - throat_z), 'DIFFERENCE')
    log(f"sprue @ ({sp.x:.0f},{sp.y:.0f},{sp.z:.0f})")

# ----------------------------------------------------------------------------
# 6. base cradle (separate STL): centre pocket = base MODEL, outer recess =
#    mould SHELL. The ridge between them (= the silicone gap) seals the bottom.
#    Both pockets oversized by CRADLE_TOL for fit + base-material expansion.
# ----------------------------------------------------------------------------
if ADD_CRADLE and cutz is not None:
    shell_cut = offset_solid(body,   CRADLE_TOL, vs, "cr_shell")   # mould footprint + tol
    model_cut = offset_solid(master, CRADLE_TOL, vs, "cr_model")   # base model + tol
    smn, smx = world_bbox(shell_cut)
    rim_z, bot_z = cutz + CRADLE_RECESS, cutz - CRADLE_FLOOR
    plate = make_box("cradle",
                     Vector((center.x, center.y, (rim_z + bot_z) / 2)),
                     ((smx.x - smn.x) + 2 * CRADLE_WALL,
                      (smx.y - smn.y) + 2 * CRADLE_WALL, rim_z - bot_z))
    boolean(plate, shell_cut, 'DIFFERENCE')     # outer recess -> locates the shell
    boolean(plate, model_cut, 'DIFFERENCE')     # centre pocket -> locates the model
    export_stl(plate, os.path.join(outdir, f"{stem}_cradle.stl"))
    rm(plate)
    log(f"cradle: model pocket + shell recess (tol {CRADLE_TOL}mm)")

# ----------------------------------------------------------------------------
# 7. split into pieces with vertical half-space boxes (+ ball keys)
# ----------------------------------------------------------------------------
def halfspace(name, axis, keep_low):
    """box occupying the keep side of the parting plane on the given axis."""
    c = list(center)
    d = list((2*span, 2*span, 2*span))
    ai = 0 if axis == 'X' else 1
    d[ai] = span
    c[ai] = (center[ai] - span/2) if keep_low else (center[ai] + span/2)
    return make_box(name, Vector(c), d)

def cut(piece, cutters):
    for cb in cutters:
        boolean(piece, cb, 'INTERSECT')
    return piece

# ball-key registration. Keys sit on the parting plane(s), seated just inside
# the ACTUAL flange edge (found by ray-cast, so they hug the silhouette and
# never float). Convention: +side of a plane = male ball, -side = socket.
keys_on = ADD_PINS and ADD_FLANGE
inset = PIN_RADIUS + 2.0                              # seat the ball this far in from the lip edge

# pin heights: spread N keys up the parting line (15%..85% of model height).
# N auto-scales at ~1 per 40mm of height.
n_pins = PIN_COUNT if PIN_COUNT else max(1, int(size.z / 40) + 1)
if n_pins == 1:
    pin_zs = [center.z]
else:
    zlo, zhi = mn.z + 0.15 * size.z, mn.z + 0.85 * size.z
    pin_zs = [zlo + (zhi - zlo) * k / (n_pins - 1) for k in range(n_pins)]
if keys_on:
    log(f"pins: up to {n_pins} per interface")

def lip(plus, along, pz):
    """find pin coord on axis `along` ('x'/'y') just inside the flange edge,
    at height pz, on the +side (plus=True) or -side. None if no flange there."""
    o = list(center); d = [0.0, 0.0, 0.0]; ax = 0 if along == 'x' else 1
    o[2] = pz
    o[ax] = (center[ax] + span) if plus else (center[ax] - span)
    d[ax] = -1.0 if plus else 1.0
    e = ray_edge(body, o, d)
    if e is None:
        return None
    return (e[ax] - inset) if plus else (e[ax] + inset)

def key(piece, point, male):
    """add a ball boss (male) or socket (female) to a piece at point."""
    if male:
        boolean(piece, make_ball(f"{piece.name}_m", point, PIN_RADIUS), 'UNION')
    else:
        boolean(piece, make_ball(f"{piece.name}_f", point, PIN_RADIUS + PIN_CLEAR), 'DIFFERENCE')

pieces = []
if FOUR_PIECE:
    combos = [("xl_yl", True,  True),
              ("xh_yl", False, True),
              ("xl_yh", True,  False),
              ("xh_yh", False, False)]
    for tag, xlo, ylo in combos:                       # xlo: on -X side, ylo: on -Y side
        p = dup(body, f"piece_{tag}")
        cut(p, [halfspace(f"hs1_{tag}", 'X', xlo),
                halfspace(f"hs2_{tag}", 'Y', ylo)])
        if keys_on:
            for pz in pin_zs:
                px = lip(not xlo, 'x', pz)             # Y-plane key on this piece's X side
                if px is not None:
                    key(p, Vector((px, center.y, pz)), male=not ylo)
                py = lip(not ylo, 'y', pz)             # X-plane key on this piece's Y side
                if py is not None:
                    key(p, Vector((center.x, py, pz)), male=not xlo)
        pieces.append((tag, p))
    rm(body)
else:
    for tag, ylo in (("yl", True), ("yh", False)):
        p = dup(body, f"piece_{tag}")
        cut(p, [halfspace(f"hs_{tag}", 'Y', ylo)])
        if keys_on:
            for pz in pin_zs:
                for xplus in (False, True):            # keys at both -X and +X lips
                    px = lip(xplus, 'x', pz)
                    if px is not None:
                        key(p, Vector((px, center.y, pz)), male=not ylo)
        pieces.append((tag, p))
    rm(body)

for tag, p in pieces:
    export_stl(p, os.path.join(outdir, f"{stem}_mould_{tag}.stl"))

extras = []
if POUR_MODE == "top": extras.append("sprue")
if ADD_CRADLE: extras.append("cradle")
log(f"DONE -> {len(pieces)} mould pieces" + (f" + {', '.join(extras)}" if extras else "") + f" in {outdir}")
