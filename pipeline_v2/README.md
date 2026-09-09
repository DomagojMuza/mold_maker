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

**Phase 1** (master shown): dock has shell/core/flange/pour/voxel spins + 4-piece
check + pour shape. In the view: `P` = pour mode (translucent post+bore preview
follows the pour spins live); `S` = split mode, press `S` again to toggle which
parting plane a right-click moves - **X and Y independent** (2-piece: Y only),
active plane is the brighter guide, un-set = model centre. **Build shell** / `G`.

**Phase 2** (built shell shown): dock has cradle/lip/pin/vent/foot spins + the
ADD_* checks. In the view: `V` vent, `F` foot - right-click places on the ACTUAL
shell. **Finish & export** button / `G`.

Result: **Back** (re-pick vents/feet, no rebuild) or **Restart**. `U` undo / `C`
clear the current phase's picks. Empty pick category = auto. STLs -> `<stem>_v2out/`.

The pipeline is split to match: `build_shell(cfg, master, pour=, split=)` ->
state, then `finish_mould(cfg, state, vents=, feet=)`. `generate()` chains both
for the CLI. CLI overrides: `--split X,Y` / `--split-x N` / `--split-y N` (world
mm; unset axis = model centre); `--pour-shape round|square`.

## Verified (2026-09-09)

gingerbread_7cm 7.7 s · zenska 9.1 s · Tesla_FINAL 12.1 s · baby_jesus 10.1 s ·
cap_high_res 15.0 s - all: watertight pieces, 1 component, plausible volume, `flagged: 0`.

## Not done / known limits

- **feet** auto-placement is ported from the Blender logic as-is; spacing/height
  can look uneven. Manual foot picks work.
- **cradle** sometimes comes out `euler 0` (one micro-handle where flange recess
  meets shell recess). Watertight, solid floor, correct footprint - tolerated; an
  SDF re-heal is applied on export but doesn't always remove it.
- deep concavities between a figure's limbs bridge over (SHELL/CORE offsets close
  gaps < ~2x offset) - inherent to a glove mould at this size, same as Blender.
- unit scaling (cm / m sources) not wired - assumes 1 unit = 1 mm.
- GUI `generate` blocks the window for the ~10 s run (no worker thread yet).
- tested on 5 STLs only.

## Deps

`meshlib`, `trimesh`, `numpy`, `pyvista`, `pyvistaqt`, `PyQt5` (see
`requirements.txt`). All in `../offset_bench/.venv`:
`..\offset_bench\.venv\Scripts\python.exe -m pip install -r requirements.txt`.
OpenVDB via pip does not exist for Windows - not needed, MeshLib bundles its own.
