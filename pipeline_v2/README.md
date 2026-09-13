# pipeline_v2 - glove mould without Blender

MeshLib (`mrmeshpy`) + a pyvista picker, replacing the Blender `core/glove_mold.py`
path. Same geometry design; different primitives.

| | Blender (`make_mold.py`, voxel 0.4) | pipeline_v2 |
|---|---|---|
| gingerbread_7cm | 222 s | **7.7 s** |
| piece STL | 25 MB / ~500k f, **not watertight**, 8-10 components | **~1 MB / ~20k f, watertight, 1 component** |
| baby_jesus (Blender: unsolved) | empty cradle / sliver pieces | **all pieces + cradle valid** |

## Why it's fast + clean

- **offset** = `mrmeshpy.offsetMesh` (OpenVDB narrow-band SDF, dual marching cubes).
  One call per offset. The Blender version did stepped vertex-push + a full voxel
  remesh per sub-step, ~35 remeshes on ever-growing meshes, so triangle count
  compounded stage over stage.
- **boolean / split** = `mrmeshpy.boolean` (exact CSG) + a box per quadrant.
  Blender's halfspace-INTERSECT split silently produced cutter-box fragments on
  concave shapes (the "sequential INTERSECT" failure in ../CONTEXT.md).
- **voxel** auto-scales with model size (`diag/200`, clamped 0.30-1.5 mm) so cost
  is ~flat across models instead of exploding on big ones.
- **heal once**: the source STL is SDF-round-tripped at the start; every later op
  uses that clean copy (raw STLs have self-intersections that break slab clips).
- `HoleWindingRule` sign mode everywhere - ~3x faster than the default OpenVDB
  mode on these meshes and tolerant of messy input.

## Run

```bash
cd pipeline_v2
../offset_bench/.venv/Scripts/python make_mold2.py ../gingerbread_7cm.stl out
# flags: --two --novents --nofeet --nopins --nocradle --nolip --voxel N
```

Exits non-zero if any piece fails the sanity gate (not watertight / multi-component
/ bounding box ~4x the model = cutter-box garbage). A piece that carries the pour
or a vent legitimately has a channel through it -> `euler < 2`; that is expected,
not flagged.

### GUI - two phases, Qt side panel

```bash
pipeline_v2\run_gui.bat gingerbread_7cm.stl          # double-clickable; defaults to gingerbread
# or: ..\offset_bench\.venv\Scripts\python.exe gui.py ..\gingerbread_7cm.stl
```

3-D view on the left; a **Parameters dock on the right** - at the top **Model**
and **Output** rows each with a `Browse...` button (swap the input STL or the
output folder without restarting; a new model resets to phase 1), then a
scrollable form of spin boxes + check boxes for every knob of the current phase,
a pour-shape dropdown (phase 1), and the phase action buttons (Build shell /
Finish / Back / Restart). Everything you can't click on the model lives there.

**Mouse:** LEFT = rotate / pan / zoom (never places anything). **RIGHT-click the
surface = place a point** in the current mode (a right-*drag* is a zoom, ignored).

**Phase 1** (master shown): dock has **Model offset X** / **Model offset Y** at
the top (slides the master under a FIXED world-space split axis - an explicit
split pivot stays put, so sliding the model actually changes which part of it
falls on which side of the cut; surface picks - pour point, vents, feet - move
with the model since they're tied to a feature on it; an unset/auto split
pivot still tracks the model's own centre), then shell/core/flange/pour/voxel
spins + 4-piece check + pour shape, plus
**Split angle X** and **Split angle Y** - two fully independent spin boxes; the
two parting planes no longer have to stay perpendicular. In the view: `P` = pour
mode (translucent post+bore preview
follows the pour spins live); `S` = split mode, press `S` again to cycle which
parting plane a right-click (position) or `Left`/`Right` arrow key (angle, 5° at
a time) affects - **X, Y, then any extra planes** (2-piece: Y + extras only),
active plane is the brighter guide, un-set X/Y position = model centre. Guide
planes, flange bands and the split geometry itself all follow each plane's own
angle. **Build shell** / `G`.

**Extra split planes** - for a shape where 2 planes crossing at one point can't
give every part a clean pull direction (e.g. wings swept back at their own
angle on a figure): the **"Extra split planes"** box in the dock (`+ Add` / `N`,
`- Remove`, plus Pivot X/Y + Angle spin boxes for whichever plane is active)
adds one MORE independently-pivoted, independently-angled parting plane -
position it with a right-click same as X/Y, rotate with `Left`/`Right` while
it's active. Each extra plane doubles the candidate piece count (a 4-piece
split + 2 extra planes = up to 16 candidates); combos that don't actually
intersect the shell (e.g. "left of the sagittal AND right of the left-wing
plane") are dropped automatically and logged, not exported. Registration pins
on the base X/Y pair use the original cross-wired keying; pins on extra planes
use simpler self-parity keying (their own band, both ends) since an extra
plane's pivot isn't assumed to be centred on the whole shell the way X/Y's is.
CLI: repeatable `--extra-plane X,Y,DEG`.

**Phase 2** (built shell shown): dock has cradle/lip/pin/vent/foot spins + the
ADD_* checks. In the view: `V` vent, `F` foot - right-click places on the ACTUAL
shell. **Finish & export** button / `G`.

Result: **Back** (re-pick vents/feet, no rebuild) or **Restart**. `U` undo / `C`
clear the current phase's picks. Empty pick category = auto. STLs -> `<stem>_v2out/`.

**Build shell** and **Finish & export** run on a background thread - the 3D
view and the rest of the UI stay responsive (and keep repainting) during the
multi-second MeshLib run instead of freezing; the primary/back/restart/browse
buttons just disable until it's done. On Windows especially, a long fully
synchronous call used to leave the window looking blank/frozen until the OS
got around to repainting it - this is what fixed that.

The pipeline is split to match: `build_shell(cfg, master, pour=, split=)` ->
state, then `finish_mould(cfg, state, vents=, feet=)`. `generate()` chains both
for the CLI. CLI overrides: `--split X,Y` / `--split-x N` / `--split-y N` (world
mm; unset axis = model centre); `--split-angle-x DEG` / `--split-angle-y DEG`
(each plane's own normal direction, fully independent - defaults 0 / 90 =
perpendicular = the old axis-aligned X/Y behaviour); `--offset-x N` / `--offset-y N`
(slides the master in X/Y before anything else runs); `--pour-shape round|square`;
`--extra-plane X,Y,DEG` (repeatable - one more independently-pivoted parting
plane, see "Extra split planes" above).

## Verified (2026-09-09)

gingerbread_7cm 7.7 s · zenska 9.1 s · Tesla_FINAL 12.1 s · baby_jesus 10.1 s ·
cap_high_res 15.0 s - all: watertight pieces, 1 component, plausible volume, `flagged: 0`.

## Pour hole on a thin model top

A pour post/bore wider than the model actually is at the apex (a big requested
hole landing on a thin spike, horn, or ridge) used to drop a full-width post
straight onto whatever tiny sliver of material was there - an unsupported
overhang barely attached to the model, and a bore that could drill through the
side wall instead of just opening into the cavity. It now measures the local
shell width at the apex and, if the post/bore would exceed it, grows both from
that local width up to the full size with a cone taper - the pour post now has
a real base to stand on no matter how thin the model's top is. The taper is
capped to whatever vertical room the pour post height (`POUR_RES_H`) actually
gives it before the funnel/counterbore has to start; a very wide post on a
short post height compresses into a steeper taper rather than overshooting
into the funnel zone (a log line says so when it happens - raise `POUR_RES_H`
for a smoother transition instead). A piece whose pour post bridges over
several small bumps/serrations right at the apex (e.g. a rooster-comb-style
crest) can legitimately end up with a few small enclosed voids there and trip
the euler-based sanity check with a `<-- CHECK` flag despite being perfectly
valid (watertight, single component, correct volume) - more compressed tapers
(wide post, short post height) trip it harder; eyeball that piece rather than
treating the flag as a hard failure.

## Not done / known limits

- **feet** auto-placement is ported from the Blender logic as-is; spacing/height
  can look uneven. Manual foot picks work.
- **cradle** sometimes comes out `euler 0` (one micro-handle where flange recess
  meets shell recess). Watertight, solid floor, correct footprint - tolerated; an
  SDF re-heal is applied on export but doesn't always remove it.
- deep concavities between a figure's limbs bridge over (SHELL/CORE offsets close
  gaps < ~2x offset) - inherent to a glove mould at this size, same as Blender.
- unit scaling (cm / m sources) not wired - assumes 1 unit = 1 mm.
- tested on 5 STLs only (plus basilisk.stl for the N-plane split / pour-taper work).

## Deps

`meshlib`, `trimesh`, `numpy`, `pyvista`, `pyvistaqt`, `PyQt5` (see
`requirements.txt`). All in `../offset_bench/.venv`:
`..\offset_bench\.venv\Scripts\python.exe -m pip install -r requirements.txt`.
OpenVDB via pip does not exist for Windows - not needed, MeshLib bundles its own.
