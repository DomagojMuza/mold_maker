"""
make_mold_addon.py  --  Blender add-on for interactive mold generation.

Install:  Edit > Preferences > Add-ons > Install…  -> select this file -> enable
Panel:    3D Viewport > Sidebar (N) > Mold tab

Workflow:
  1. Open / import your master model into Blender.
  2. Select it.
  3. In the Mold panel: set shell/core offsets and other settings.
  4. Click "Pick Pour" -> left-click on the model surface to set pour location.
  5. Click "+ Add Vent" (repeat) to pick air-vent locations.
  6. Click "+ Add Foot" (repeat) to pick stabilization-foot locations.
     (If no vents/feet are picked, auto-placement runs as in the headless script.)
  7. Set output folder, then click "Generate Mold".

Generated pieces stay in the Blender scene AND are exported as STL files.
"""

bl_info = {
    "name":        "Mold Maker",
    "author":      "Domagoj Muza",
    "version":     (1, 0, 0),
    "blender":     (4, 5, 0),
    "location":    "View3D > Sidebar > Mold",
    "description": "Generate multi-piece casting molds with interactive placement",
    "category":    "Object",
}

import bpy, bmesh, math, os
from mathutils import Vector


# ─────────────────────────────────────────────────────────────────────────────
# Core pipeline  (mirrors make_mold.py — keep in sync)
# ─────────────────────────────────────────────────────────────────────────────

def log(msg):
    print(f"[mould] {msg}")


def activate(obj):
    try:
        bpy.ops.object.mode_set(mode='OBJECT')
    except Exception:
        pass
    for o in bpy.context.scene.objects:
        o.select_set(False)
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj


def apply_mod(obj, name):
    activate(obj)
    bpy.ops.object.modifier_apply(modifier=name)


def dup(obj, name):
    n = obj.copy()
    n.data = obj.data.copy()
    n.name = name
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
    me = obj.data
    bm = bmesh.new()
    bm.from_mesh(me)
    bm.normal_update()
    for v in bm.verts:
        v.co += v.normal * dist
    bm.to_mesh(me)
    bm.free()


def voxel_remesh(obj, vs):
    activate(obj)
    obj.data.remesh_voxel_size = vs
    obj.data.remesh_voxel_adaptivity = 0.0
    bpy.ops.object.voxel_remesh()


def offset_solid(base, dist, vs, name):
    o = dup(base, name)
    step_max = max(1.5 * vs, 1.5)
    n = int(abs(dist) / step_max) + 1
    d = dist / n
    for _ in range(n):
        offset_inplace(o, d)
        voxel_remesh(o, vs)
    return o


def boolean(a, b, op, keep_b=False, solver='EXACT'):
    m = a.modifiers.new("bool", 'BOOLEAN')
    m.operation = op
    m.solver = solver
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
    bpy.ops.mesh.primitive_cone_add(
        radius1=r_bot, radius2=r_top, depth=depth, vertices=48, location=center)
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
    depsgraph = bpy.context.evaluated_depsgraph_get()
    ev   = obj.evaluated_get(depsgraph)
    mw   = ev.matrix_world
    invw = mw.inverted()
    ol   = invw @ Vector(origin)
    dl   = (invw.to_3x3() @ Vector(direction)).normalized()
    res  = ev.ray_cast(ol, dl)
    return (mw @ res[1]) if res[0] else None


def mesh_volume(obj):
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    v = bm.calc_volume()
    bm.free()
    return abs(v)


def export_stl(obj, path):
    activate(obj)
    try:
        bpy.ops.wm.stl_export(filepath=path, export_selected_objects=True)
    except AttributeError:
        bpy.ops.export_mesh.stl(filepath=path, use_selection=True)
    log(f"wrote {path}")


# ─────────────────────────────────────────────────────────────────────────────
# Property group for a single picked point
# ─────────────────────────────────────────────────────────────────────────────

class MoldPoint(bpy.types.PropertyGroup):
    co: bpy.props.FloatVectorProperty(name="", subtype='XYZ', default=(0.0, 0.0, 0.0))


# ─────────────────────────────────────────────────────────────────────────────
# Visual marker helpers  (small colored spheres at picked locations)
# ─────────────────────────────────────────────────────────────────────────────

def _make_marker(name, location, color):
    bpy.ops.mesh.primitive_uv_sphere_add(radius=1.5, location=location,
                                          segments=8, ring_count=6)
    m = bpy.context.active_object
    m.name = name
    mat = bpy.data.materials.new(f"_mm_{name}")
    mat.diffuse_color = (*color, 1.0)
    mat.use_nodes = False
    m.data.materials.clear()
    m.data.materials.append(mat)
    return m


def _remove_markers(prefix):
    for obj in list(bpy.data.objects):
        if obj.name.startswith(prefix):
            bpy.data.objects.remove(obj, do_unlink=True)


def _refresh_markers(scene):
    """Rebuild all markers from scene property lists."""
    _remove_markers("mm_pour")
    _remove_markers("mm_vent")
    _remove_markers("mm_foot")
    if scene.mold_pour_set:
        _make_marker("mm_pour", scene.mold_pour_co, (1.0, 0.35, 0.0))
    for i, pt in enumerate(scene.mold_vent_points):
        _make_marker(f"mm_vent_{i}", pt.co, (1.0, 0.85, 0.0))
    for i, pt in enumerate(scene.mold_foot_points):
        _make_marker(f"mm_foot_{i}", pt.co, (0.25, 0.85, 0.25))


# ─────────────────────────────────────────────────────────────────────────────
# Ray from mouse — no bpy_extras (avoids API version differences)
# ─────────────────────────────────────────────────────────────────────────────

def _mouse_ray(region, rv3d, mouse_x, mouse_y):
    """World-space (origin, direction) from a 2D mouse position.
    Uses rv3d.perspective_matrix (view*proj) to avoid bpy_extras dependency."""
    from mathutils import Vector
    nx = (2.0 * mouse_x / region.width)  - 1.0
    ny = (2.0 * mouse_y / region.height) - 1.0
    inv = rv3d.perspective_matrix.inverted()

    def unproject(nz):
        v = inv @ Vector((nx, ny, nz, 1.0))
        w = v.w if abs(v.w) > 1e-8 else 1e-8
        return Vector((v.x / w, v.y / w, v.z / w))

    origin    = unproject(-1.0)
    direction = (unproject(1.0) - origin).normalized()
    return origin, direction


# ─────────────────────────────────────────────────────────────────────────────
# Modal pick operator  (shared for pour / vent / foot)
# ─────────────────────────────────────────────────────────────────────────────

class MOLD_OT_pick(bpy.types.Operator):
    """Click on the model surface to place a point.  ESC / RMB cancels."""
    bl_idname = "mold.pick"
    bl_label  = "Pick Point on Model"

    pick_type: bpy.props.StringProperty()   # "pour" | "vent" | "foot"

    def modal(self, context, event):
        context.area.header_text_set(
            f"Click surface to place {self.pick_type}   |   ESC / RMB = cancel")

        if event.type == 'LEFTMOUSE' and event.value == 'PRESS':
            region   = context.region
            rv3d     = context.region_data
            mx       = event.mouse_region_x
            my       = event.mouse_region_y
            ray_orig, ray_dir = _mouse_ray(region, rv3d, mx, my)

            obj = context.active_object
            if obj and obj.type == 'MESH':
                mw_inv  = obj.matrix_world.inverted()
                ro_l    = mw_inv @ ray_orig
                rd_l    = (mw_inv.to_3x3() @ ray_dir).normalized()
                hit, loc, _n, _i = obj.ray_cast(ro_l, rd_l)
                if hit:
                    world_loc = obj.matrix_world @ loc
                    self._store(context, world_loc)
                    context.area.header_text_set(None)
                    _refresh_markers(context.scene)
                    return {'FINISHED'}

            self.report({'WARNING'}, "No hit — click directly on the mesh surface")
            return {'RUNNING_MODAL'}

        elif event.type in {'RIGHTMOUSE', 'ESC'}:
            context.area.header_text_set(None)
            return {'CANCELLED'}

        return {'PASS_THROUGH'}

    def _store(self, context, loc):
        scene = context.scene
        if self.pick_type == "pour":
            scene.mold_pour_co  = loc
            scene.mold_pour_set = True
        elif self.pick_type == "vent":
            pt    = scene.mold_vent_points.add()
            pt.co = loc
        elif self.pick_type == "foot":
            pt    = scene.mold_foot_points.add()
            pt.co = loc

    def invoke(self, context, event):
        if context.area.type != 'VIEW_3D':
            self.report({'WARNING'}, "Must run from 3D Viewport")
            return {'CANCELLED'}
        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}


# ─────────────────────────────────────────────────────────────────────────────
# Remove a single point from a collection by index
# ─────────────────────────────────────────────────────────────────────────────

class MOLD_OT_remove_point(bpy.types.Operator):
    """Remove a single picked point."""
    bl_idname = "mold.remove_point"
    bl_label  = "Remove Point"

    point_type:  bpy.props.StringProperty()   # "vent" | "foot"
    point_index: bpy.props.IntProperty()

    def execute(self, context):
        scene = context.scene
        col   = scene.mold_vent_points if self.point_type == "vent" else scene.mold_foot_points
        if 0 <= self.point_index < len(col):
            col.remove(self.point_index)
        _refresh_markers(scene)
        return {'FINISHED'}


# ─────────────────────────────────────────────────────────────────────────────
# Clear operators
# ─────────────────────────────────────────────────────────────────────────────

class MOLD_OT_clear_pour(bpy.types.Operator):
    bl_idname = "mold.clear_pour"
    bl_label  = "Clear Pour Location"

    def execute(self, ctx):
        ctx.scene.mold_pour_set = False
        _remove_markers("mm_pour")
        return {'FINISHED'}


class MOLD_OT_clear_vents(bpy.types.Operator):
    bl_idname = "mold.clear_vents"
    bl_label  = "Clear All Vents"

    def execute(self, ctx):
        ctx.scene.mold_vent_points.clear()
        _remove_markers("mm_vent")
        return {'FINISHED'}


class MOLD_OT_clear_feet(bpy.types.Operator):
    bl_idname = "mold.clear_feet"
    bl_label  = "Clear All Feet"

    def execute(self, ctx):
        ctx.scene.mold_foot_points.clear()
        _remove_markers("mm_foot")
        return {'FINISHED'}


# ─────────────────────────────────────────────────────────────────────────────
# Generate operator  (main pipeline)
# ─────────────────────────────────────────────────────────────────────────────

class MOLD_OT_generate(bpy.types.Operator):
    """Run the mold generation pipeline on the active mesh object."""
    bl_idname  = "mold.generate"
    bl_label   = "Generate Mold"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        scene      = context.scene
        master_src = context.active_object
        if not master_src or master_src.type != 'MESH':
            self.report({'ERROR'}, "Select the master mesh first")
            return {'CANCELLED'}

        # ── read settings ──
        SHELL_OFFSET  = scene.mold_shell_offset
        CORE_OFFSET   = scene.mold_core_offset
        FLANGE_REACH  = scene.mold_flange_reach
        FLANGE_THICK  = scene.mold_flange_thick
        FOUR_PIECE    = scene.mold_four_piece
        ADD_FLANGE    = scene.mold_add_flange
        ADD_PINS      = scene.mold_add_pins
        PIN_RADIUS    = scene.mold_pin_radius
        PIN_CLEAR     = 0.25
        PIN_COUNT     = None
        VOXEL_DETAIL  = 220
        POUR_R        = scene.mold_pour_r
        POUR_BORE     = scene.mold_pour_bore
        POUR_RES_H    = scene.mold_pour_res_h
        FUNNEL_H      = 6.0
        ADD_VENTS     = scene.mold_add_vents
        VENT_BORE     = scene.mold_vent_bore
        VENT_POST_R   = 2.5
        VENT_MIN_SEP  = 25.0
        VENT_MAX_N    = 3
        ADD_STAB      = scene.mold_add_stab
        STAB_R        = scene.mold_stab_r
        STAB_TIP_R    = 2.0
        STAB_SHAPE    = scene.mold_stab_shape
        STAB_N        = scene.mold_stab_n if scene.mold_stab_n > 0 else None
        ADD_CRADLE    = scene.mold_add_cradle
        CRADLE_WALL   = 4.0
        CRADLE_RECESS = 2.0
        CRADLE_FLOOR  = 3.0
        CRADLE_TOL    = 0.5

        outdir = bpy.path.abspath(scene.mold_output_dir or "//")
        os.makedirs(outdir, exist_ok=True)
        stem = master_src.name.replace(" ", "_")

        # ── 0. duplicate master; bake world matrix into mesh data ──
        # Read matrix_world from SOURCE before duplicating — the copy's matrix_world
        # is stale until the depsgraph runs, so reading from master_src is correct.
        from mathutils import Matrix
        src_wm = master_src.matrix_world.copy()
        master = dup(master_src, "mould_master_work")
        bm_tmp = bmesh.new()
        bm_tmp.from_mesh(master.data)
        bm_tmp.transform(src_wm)       # bake actual world transform into vertex data
        bm_tmp.to_mesh(master.data)
        bm_tmp.free()
        master.matrix_world = Matrix.Identity(4)
        master.data.update()           # force mesh bounds recalc
        bpy.context.view_layer.update()  # flush depsgraph so bound_box is fresh

        # ── 1. geometry params (mirrors headless script exactly) ──
        mn, mx   = world_bbox(master)
        center   = (mn + mx) * 0.5
        size     = mx - mn
        maxdim   = max(size.x, size.y, size.z)
        diag     = size.length
        span     = maxdim * 4.0

        wall_thick = SHELL_OFFSET - CORE_OFFSET
        vs = min(max(diag / VOXEL_DETAIL, 0.1), 0.5)
        if wall_thick > 0:
            vs = min(vs, wall_thick / 4)
        log(f"bbox {tuple(round(v,1) for v in size)}  wall={wall_thick:.2f}mm  voxel={vs:.3f}mm")

        cutz = mn.z   # flat-bottom cut height

        voxel_remesh(master, vs)

        # ── 2. outer shell ──
        WALL_OUT = SHELL_OFFSET
        outer = offset_solid(master, WALL_OUT, vs, "outer")

        # ── 3. flange (matches headless exactly) ──
        if ADD_FLANGE:
            flangeblob = offset_solid(master, WALL_OUT + FLANGE_REACH, vs, "flangeblob")
            flange_top = world_bbox(outer)[1].z
            z_lo = mn.z - WALL_OUT - 10.0
            z_c  = (z_lo + flange_top) / 2
            z_h  = flange_top - z_lo

            def flange_band(normal_axis):
                dims = (span, FLANGE_THICK, z_h) if normal_axis == 'Y' \
                       else (FLANGE_THICK, span, z_h)
                slab = make_box(f"slab_{normal_axis}",
                                Vector((center.x, center.y, z_c)), dims)
                band = dup(flangeblob, f"band_{normal_axis}")
                boolean(band, slab, 'INTERSECT')
                return band

            boolean(outer, flange_band('Y'), 'UNION')
            if FOUR_PIECE:
                boolean(outer, flange_band('X'), 'UNION')
            rm(flangeblob)
            log("flange: ON")

        # ── 4. negative (silicone gap) ──
        negative = offset_solid(master, CORE_OFFSET, vs, "negative") \
                   if CORE_OFFSET > 1e-6 else dup(master, "negative")
        neg_mn, neg_mx = world_bbox(negative)
        gap_vol  = mesh_volume(negative) - mesh_volume(master)
        log(f"silicone needed ~ {gap_vol/1000:.1f} ml")

        # sample cavity high points for auto pour / vent placement
        high_pts = []
        g = 18
        for ix in range(g):
            for iy in range(g):
                x   = neg_mn.x + (neg_mx.x - neg_mn.x) * (ix + 0.5) / g
                y   = neg_mn.y + (neg_mx.y - neg_mn.y) * (iy + 0.5) / g
                hit = ray_edge(negative, (x, y, neg_mx.z + span), (0, 0, -1))
                if hit:
                    high_pts.append(hit)
        high_pts.sort(key=lambda p: -p.z)

        cavity_cutter = dup(negative, "cavity_cutter")
        boolean(outer, negative, 'DIFFERENCE')
        body = outer
        body.name = "mould_body"

        # ── 5. flat-bottom cut (open bottom, matches headless) ──
        h    = (mx.z + maxdim) - cutz
        keep = make_box("body_keep",
                        Vector((center.x, center.y, cutz + h / 2)),
                        (4 * maxdim, 4 * maxdim, h))
        boolean(body, keep, 'INTERSECT')

        # ── 5. pour inlet ──
        res_top = None
        if scene.mold_pour_set:
            sp_pick = Vector(scene.mold_pour_co)
            # pick is on model surface; snap to cavity surface at same XY
            sp_snapped = ray_edge(cavity_cutter, (sp_pick.x, sp_pick.y, neg_mx.z + span), (0, 0, -1))
            sp = sp_snapped if sp_snapped else sp_pick
        elif high_pts:
            sp = high_pts[0]
        else:
            sp = None

        if sp is not None:
            res_top   = sp.z + POUR_RES_H
            post_bot  = sp.z + 0.5
            boolean(body, make_cyl("res",
                                   Vector((sp.x, sp.y, (res_top + post_bot) / 2)),
                                   POUR_R, res_top - post_bot), 'UNION')
            funnel_top_r = min(POUR_R - 1.0, POUR_R - 1.0)
            funnel_h     = min(FUNNEL_H, POUR_RES_H - 1.0)
            funnel_top_z = res_top + 1.0
            funnel_bot_z = res_top - funnel_h
            boolean(body, make_cone("funnel",
                                    Vector((sp.x, sp.y, (funnel_top_z + funnel_bot_z) / 2)),
                                    POUR_BORE, funnel_top_r, funnel_top_z - funnel_bot_z),
                    'DIFFERENCE')
            bore_bot = sp.z - 3.0
            bore_top = funnel_bot_z + 0.5
            boolean(body, make_cyl("bore",
                                   Vector((sp.x, sp.y, (bore_top + bore_bot) / 2)),
                                   POUR_BORE, bore_top - bore_bot), 'DIFFERENCE')
            log(f"pour @ ({sp.x:.0f},{sp.y:.0f})  R{POUR_R}/bore{POUR_BORE}/h{POUR_RES_H}")

        # ── 6. air vents ──
        if ADD_VENTS and res_top is not None:
            # manual locations if set, else auto from high_pts
            if len(scene.mold_vent_points) > 0:
                vent_pts = []
                for pt in scene.mold_vent_points:
                    pt_v = Vector(pt.co)
                    hit = ray_edge(cavity_cutter, (pt_v.x, pt_v.y, neg_mx.z + span), (0, 0, -1))
                    vent_pts.append(hit if hit else pt_v)
            else:
                vent_pts = []
                for vp in high_pts:
                    if (vp.xy - sp.xy).length < VENT_MIN_SEP:
                        continue
                    if any((vp.xy - q.xy).length < VENT_MIN_SEP for q in vent_pts):
                        continue
                    vent_pts.append(vp)
                    if len(vent_pts) >= VENT_MAX_N:
                        break

            for vp in vent_pts:
                vpost_bot = vp.z + 0.5
                vpost = make_cyl("ventpost",
                                 Vector((vp.x, vp.y, (res_top + vpost_bot) / 2)),
                                 VENT_POST_R, res_top - vpost_bot)
                boolean(vpost, dup(cavity_cutter, "vccopy"), 'DIFFERENCE', solver='MANIFOLD')
                boolean(body, vpost, 'UNION', solver='MANIFOLD')
                vbore_bot = vp.z - 2.0
                boolean(body,
                        make_cyl("ventbore",
                                 Vector((vp.x, vp.y, (res_top + 1 + vbore_bot) / 2)),
                                 VENT_BORE, (res_top + 1) - vbore_bot),
                        'DIFFERENCE', solver='MANIFOLD')
            if vent_pts:
                voxel_remesh(body, vs)
                log(f"air vents: {len(vent_pts)} @ R{VENT_POST_R}/bore R{VENT_BORE}mm")

        # ── 7. stabilization feet ──
        feet_placed = False
        if ADD_STAB and res_top is not None:
            bpy.context.view_layer.update()
            foot_top_z = res_top + 1.0

            if len(scene.mold_foot_points) > 0:
                # manual: use picked XY, ray down to find shell top z
                for pt in scene.mold_foot_points:
                    fp  = Vector(pt.co)
                    hit = ray_edge(body, (fp.x, fp.y, world_bbox(body)[1].z + 20), (0, 0, -1))
                    base_z = (hit.z - 5.0) if hit else (fp.z - 5.0)
                    foot_h = foot_top_z - base_z
                    if foot_h < 1.0:
                        continue
                    mid_z = (foot_top_z + base_z) / 2
                    foot  = (make_cone("stab", Vector((fp.x, fp.y, mid_z)), STAB_R, STAB_TIP_R, foot_h)
                             if STAB_SHAPE == "cone"
                             else make_cyl("stab", Vector((fp.x, fp.y, mid_z)), STAB_R, foot_h))
                    boolean(foot, dup(cavity_cutter, "fcutter"), 'DIFFERENCE', solver='MANIFOLD')
                    boolean(body, foot, 'UNION', solver='MANIFOLD')
                    feet_placed = True
                log(f"feet: {len(scene.mold_foot_points)} (manual)")
            else:
                # auto: radial rays around bbox perimeter
                mn_b, mx_b = world_bbox(body)
                cx_b, cy_b = (mn_b.x + mx_b.x) / 2, (mn_b.y + mx_b.y) / 2
                rx_b, ry_b = (mx_b.x - mn_b.x) / 2, (mx_b.y - mn_b.y) / 2
                perimeter  = 4 * (rx_b + ry_b)
                n_feet     = max(4, STAB_N if STAB_N else int(perimeter / 80))
                angle_step = 2 * math.pi / n_feet
                angle_off  = angle_step / 2
                mid_z_ray  = mn_b.z + (mx.z - mn_b.z) * 0.3
                placed_xy  = []
                for i in range(n_feet):
                    theta    = angle_step * i + angle_off
                    dx, dy   = math.cos(theta), math.sin(theta)
                    horiz    = ray_edge(body, (cx_b, cy_b, mid_z_ray), (dx, dy, 0))
                    if not horiz:
                        continue
                    fx, fy   = horiz.x - dx * (STAB_R * 2.0), horiz.y - dy * (STAB_R * 2.0)
                    hit      = ray_edge(body, (fx, fy, mx_b.z + 20), (0, 0, -1))
                    if not hit:
                        continue
                    base_z   = hit.z - 5.0
                    foot_h   = foot_top_z - base_z
                    if foot_h < 1.0:
                        continue
                    mid_z    = (foot_top_z + base_z) / 2
                    foot     = (make_cone("stab", Vector((fx, fy, mid_z)), STAB_R, STAB_TIP_R, foot_h)
                                if STAB_SHAPE == "cone"
                                else make_cyl("stab", Vector((fx, fy, mid_z)), STAB_R, foot_h))
                    boolean(foot, dup(cavity_cutter, "fcutter"), 'DIFFERENCE', solver='MANIFOLD')
                    boolean(body, foot, 'UNION', solver='MANIFOLD')
                    placed_xy.append((fx, fy))
                if placed_xy:
                    feet_placed = True
                log(f"stab feet: {len(placed_xy)}/{n_feet} placed (auto)")

        rm(cavity_cutter)
        if feet_placed:
            voxel_remesh(body, vs)

        # ── 8. cradle ──
        if ADD_CRADLE:
            shell_cut = offset_solid(body,   CRADLE_TOL, vs, "cr_shell")
            model_cut = offset_solid(master, CRADLE_TOL, vs, "cr_model")
            smn, smx  = world_bbox(shell_cut)
            rim_z     = cutz + CRADLE_RECESS
            bot_z     = cutz - CRADLE_FLOOR
            plate     = make_box("cradle",
                                 Vector((center.x, center.y, (rim_z + bot_z) / 2)),
                                 ((smx.x - smn.x) + 2 * CRADLE_WALL,
                                  (smx.y - smn.y) + 2 * CRADLE_WALL,
                                  rim_z - bot_z))
            boolean(plate, shell_cut, 'DIFFERENCE')
            boolean(plate, model_cut, 'DIFFERENCE')
            cradle_path = os.path.join(outdir, f"{stem}_cradle.stl")
            export_stl(plate, cradle_path)
            plate.name = f"{stem}_cradle"
            _assign_color(plate, (0.3, 0.6, 1.0))
            log("cradle done")

        # ── 9. registration pins ──
        keys_on = ADD_PINS and ADD_FLANGE
        inset   = PIN_RADIUS + 2.0
        n_pins  = PIN_COUNT if PIN_COUNT else max(1, int(size.z / 40) + 1)
        if n_pins == 1:
            pin_zs = [center.z]
        else:
            zlo    = mn.z + 0.15 * size.z
            zhi    = mn.z + 0.85 * size.z
            pin_zs = [zlo + (zhi - zlo) * k / (n_pins - 1) for k in range(n_pins)]

        def lip(plus, along, pz):
            o = list(center); d = [0.0, 0.0, 0.0]
            ax = 0 if along == 'x' else 1
            o[2] = pz
            o[ax] = (center[ax] + span) if plus else (center[ax] - span)
            d[ax] = -1.0 if plus else 1.0
            e = ray_edge(body, o, d)
            if e is None:
                return None
            return (e[ax] - inset) if plus else (e[ax] + inset)

        def key(piece, point, male):
            if male:
                boolean(piece, make_ball(f"{piece.name}_m", point, PIN_RADIUS), 'UNION')
            else:
                boolean(piece, make_ball(f"{piece.name}_f", point, PIN_RADIUS + PIN_CLEAR),
                        'DIFFERENCE')

        # ── 10. split ──
        def halfspace(name, axis, keep_low):
            c  = list(center)
            d  = [2 * span, 2 * span, 2 * span]
            ai = 0 if axis == 'X' else 1
            d[ai] = span
            c[ai] = (center[ai] - span / 2) if keep_low else (center[ai] + span / 2)
            return make_box(name, Vector(c), d)

        def cut(piece, cutters):
            for cb in cutters:
                boolean(piece, cb, 'INTERSECT', solver='MANIFOLD')
            return piece

        pieces = []
        PIECE_COLORS = [
            (1.0, 0.4, 0.4),
            (0.4, 1.0, 0.5),
            (0.4, 0.6, 1.0),
            (1.0, 0.9, 0.3),
        ]

        if FOUR_PIECE:
            combos = [
                ("xl_yl", True,  True),
                ("xh_yl", False, True),
                ("xl_yh", True,  False),
                ("xh_yh", False, False),
            ]
            for ci, (tag, xlo, ylo) in enumerate(combos):
                p = dup(body, f"piece_{tag}")
                cut(p, [halfspace(f"hs1_{tag}", 'X', xlo),
                         halfspace(f"hs2_{tag}", 'Y', ylo)])
                if keys_on:
                    for pz in pin_zs:
                        px = lip(not xlo, 'x', pz)
                        if px is not None:
                            key(p, Vector((px, center.y, pz)), male=not ylo)
                        py = lip(not ylo, 'y', pz)
                        if py is not None:
                            key(p, Vector((center.x, py, pz)), male=not xlo)
                path = os.path.join(outdir, f"{stem}_mould_{tag}.stl")
                export_stl(p, path)
                p.name = f"{stem}_mould_{tag}"
                _assign_color(p, PIECE_COLORS[ci % len(PIECE_COLORS)])
                pieces.append(p)
        else:
            for ci, (tag, ylo) in enumerate([("yl", True), ("yh", False)]):
                p = dup(body, f"piece_{tag}")
                cut(p, [halfspace(f"hs_{tag}", 'Y', ylo)])
                if keys_on:
                    for pz in pin_zs:
                        for xplus in (False, True):
                            px = lip(xplus, 'x', pz)
                            if px is not None:
                                key(p, Vector((px, center.y, pz)), male=not ylo)
                path = os.path.join(outdir, f"{stem}_mould_{tag}.stl")
                export_stl(p, path)
                p.name = f"{stem}_mould_{tag}"
                _assign_color(p, PIECE_COLORS[ci % len(PIECE_COLORS)])
                pieces.append(p)

        rm(body)
        rm(master)

        log(f"DONE -> {len(pieces)} pieces + cradle in {outdir}")
        self.report({'INFO'}, f"Mold done — {len(pieces)} pieces → {outdir}")
        return {'FINISHED'}


def _assign_color(obj, rgb):
    mat = bpy.data.materials.new(obj.name + "_mat")
    mat.diffuse_color = (*rgb, 1.0)
    mat.use_nodes = False
    obj.data.materials.clear()
    obj.data.materials.append(mat)


# ─────────────────────────────────────────────────────────────────────────────
# Panel
# ─────────────────────────────────────────────────────────────────────────────

class MOLD_PT_main(bpy.types.Panel):
    bl_label       = "Mold Maker"
    bl_idname      = "MOLD_PT_main"
    bl_space_type  = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category    = 'Mold'

    def draw(self, context):
        layout = self.layout
        scene  = context.scene

        # master object
        box = layout.box()
        box.label(text="Master mesh", icon='OBJECT_DATA')
        if context.active_object and context.active_object.type == 'MESH':
            box.label(text=f"  {context.active_object.name}", icon='CHECKMARK')
        else:
            box.label(text="  (select a mesh)", icon='ERROR')

        # output
        box = layout.box()
        box.label(text="Output folder")
        box.prop(scene, "mold_output_dir", text="")

        # shell settings
        box = layout.box()
        box.label(text="Shell", icon='MOD_SOLIDIFY')
        col = box.column(align=True)
        col.prop(scene, "mold_shell_offset")
        col.prop(scene, "mold_core_offset")
        col.prop(scene, "mold_four_piece")
        col.prop(scene, "mold_add_flange")
        row = col.row()
        row.enabled = scene.mold_add_flange
        row.prop(scene, "mold_flange_reach")
        row = col.row()
        row.enabled = scene.mold_add_flange
        row.prop(scene, "mold_flange_thick")
        col.prop(scene, "mold_add_pins")
        row = col.row()
        row.enabled = scene.mold_add_pins and scene.mold_add_flange
        row.prop(scene, "mold_pin_radius")
        col.prop(scene, "mold_add_cradle")

        # pour
        box = layout.box()
        box.label(text="Pour hole", icon='TRIA_DOWN')
        col = box.column(align=True)
        col.prop(scene, "mold_pour_r")
        col.prop(scene, "mold_pour_bore")
        col.prop(scene, "mold_pour_res_h")
        row = box.row(align=True)
        op = row.operator("mold.pick", text="Pick on model", icon='CURSOR')
        op.pick_type = "pour"
        row.operator("mold.clear_pour", text="", icon='X')
        if scene.mold_pour_set:
            c = scene.mold_pour_co
            box.label(text=f"  ({c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f})", icon='CHECKMARK')
        else:
            box.label(text="  auto (cavity apex)", icon='INFO')

        # vents
        box = layout.box()
        box.label(text=f"Air vents  [{len(scene.mold_vent_points)} picked]", icon='DRIVER_DISTANCE')
        col = box.column(align=True)
        col.prop(scene, "mold_add_vents")
        col.prop(scene, "mold_vent_bore")
        for i, pt in enumerate(scene.mold_vent_points):
            row = col.row(align=True)
            row.label(text=f"  {i+1}: ({pt.co[0]:.1f}, {pt.co[1]:.1f}, {pt.co[2]:.1f})")
            op = row.operator("mold.remove_point", text="", icon='X')
            op.point_type  = "vent"
            op.point_index = i
        row = box.row(align=True)
        op = row.operator("mold.pick", text="+ Add vent", icon='ADD')
        op.pick_type = "vent"
        row.operator("mold.clear_vents", text="Clear all", icon='X')
        if not scene.mold_vent_points:
            box.label(text="  auto-placed if none picked", icon='INFO')

        # feet
        box = layout.box()
        box.label(text=f"Stab feet  [{len(scene.mold_foot_points)} picked]", icon='GRIP')
        col = box.column(align=True)
        col.prop(scene, "mold_add_stab")
        col.prop(scene, "mold_stab_r")
        col.prop(scene, "mold_stab_shape")
        col.prop(scene, "mold_stab_n")
        for i, pt in enumerate(scene.mold_foot_points):
            row = col.row(align=True)
            row.label(text=f"  {i+1}: ({pt.co[0]:.1f}, {pt.co[1]:.1f}, {pt.co[2]:.1f})")
            op = row.operator("mold.remove_point", text="", icon='X')
            op.point_type  = "foot"
            op.point_index = i
        row = box.row(align=True)
        op = row.operator("mold.pick", text="+ Add foot", icon='ADD')
        op.pick_type = "foot"
        row.operator("mold.clear_feet", text="Clear all", icon='X')
        if not scene.mold_foot_points:
            box.label(text="  auto-placed if none picked", icon='INFO')

        # generate
        layout.separator()
        layout.operator("mold.generate", text="Generate Mold", icon='MOD_SOLIDIFY')


# ─────────────────────────────────────────────────────────────────────────────
# Registration
# ─────────────────────────────────────────────────────────────────────────────

classes = [
    MoldPoint,
    MOLD_OT_pick,
    MOLD_OT_remove_point,
    MOLD_OT_clear_pour,
    MOLD_OT_clear_vents,
    MOLD_OT_clear_feet,
    MOLD_OT_generate,
    MOLD_PT_main,
]


def register():
    for cls in classes:
        bpy.utils.register_class(cls)

    S = bpy.types.Scene
    S.mold_output_dir   = bpy.props.StringProperty(
        name="Output dir", subtype='DIR_PATH', default="//")
    S.mold_shell_offset = bpy.props.FloatProperty(
        name="Shell offset (mm)", default=4.2, min=0.5, max=30,
        description="Outer boundary from model surface.  Wall = shell - core")
    S.mold_core_offset  = bpy.props.FloatProperty(
        name="Core offset (mm)", default=3.0, min=0.0, max=20,
        description="Silicone gap around the model")
    S.mold_flange_reach = bpy.props.FloatProperty(
        name="Flange reach (mm)", default=10.0, min=2, max=40)
    S.mold_flange_thick = bpy.props.FloatProperty(
        name="Flange thick (mm)", default=6.0, min=2, max=20)
    S.mold_four_piece   = bpy.props.BoolProperty(
        name="4-piece (two parting planes)", default=True)
    S.mold_add_flange   = bpy.props.BoolProperty(name="Add flange", default=True)
    S.mold_add_pins     = bpy.props.BoolProperty(name="Registration pins", default=True)
    S.mold_pin_radius   = bpy.props.FloatProperty(
        name="Pin radius (mm)", default=2.0, min=0.5, max=8)
    S.mold_add_cradle   = bpy.props.BoolProperty(name="Add cradle", default=True)
    S.mold_pour_r       = bpy.props.FloatProperty(
        name="Pour post R (mm)", default=6.0, min=2, max=20)
    S.mold_pour_bore    = bpy.props.FloatProperty(
        name="Pour bore R (mm)", default=3.0, min=1, max=15)
    S.mold_pour_res_h   = bpy.props.FloatProperty(
        name="Reservoir height (mm)", default=12.0, min=3, max=40)
    S.mold_add_vents    = bpy.props.BoolProperty(name="Add air vents", default=True)
    S.mold_vent_bore    = bpy.props.FloatProperty(
        name="Vent bore R (mm)", default=1.0, min=0.3, max=5)
    S.mold_add_stab     = bpy.props.BoolProperty(name="Add stab feet", default=True)
    S.mold_stab_r       = bpy.props.FloatProperty(
        name="Foot base R (mm)", default=5.0, min=2, max=20)
    S.mold_stab_shape   = bpy.props.EnumProperty(
        name="Foot shape",
        items=[("cone", "Cone", "Tapered cone"), ("cyl", "Cylinder", "Flat cylinder")],
        default="cone")
    S.mold_stab_n       = bpy.props.IntProperty(
        name="Foot count (0=auto)", default=0, min=0, max=20)
    S.mold_pour_set     = bpy.props.BoolProperty(default=False)
    S.mold_pour_co      = bpy.props.FloatVectorProperty(subtype='XYZ', default=(0, 0, 0))
    S.mold_vent_points  = bpy.props.CollectionProperty(type=MoldPoint)
    S.mold_foot_points  = bpy.props.CollectionProperty(type=MoldPoint)


def unregister():
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)
    S = bpy.types.Scene
    for attr in [
        'mold_output_dir', 'mold_shell_offset', 'mold_core_offset',
        'mold_flange_reach', 'mold_flange_thick', 'mold_four_piece',
        'mold_add_flange', 'mold_add_pins', 'mold_pin_radius', 'mold_add_cradle',
        'mold_pour_r', 'mold_pour_bore', 'mold_pour_res_h',
        'mold_add_vents', 'mold_vent_bore',
        'mold_add_stab', 'mold_stab_r', 'mold_stab_shape', 'mold_stab_n',
        'mold_pour_set', 'mold_pour_co', 'mold_vent_points', 'mold_foot_points',
    ]:
        try:
            delattr(S, attr)
        except AttributeError:
            pass


if __name__ == "__main__":
    register()
