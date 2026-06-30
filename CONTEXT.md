# mold_maker — project context / handoff

Automated cast-mould generator. One Blender-headless Python script eats an STL of
a master model and spits out a multi-piece rigid **matrix (glove) mould** plus a
**base cradle**, with no manual clicking. Replaces the old manual flow
(Meshmixer "make solid" + offsets + CAD flange).

- **Repo:** https://github.com/DomagojMuza/mold_maker  (branch `main`)
- **Main file:** `make_mold.py`
- **Test STLs** (`bun.stl`, `pink_fat_cap.stl`) are **git-ignored** (`*.stl`), so they are
  NOT in the repo — copy them over manually, or just point the script at any STL.

## How to run

```bash
blender --background --python make_mold.py -- input.stl [outdir] [flags]
```

On this Windows PC Blender lives at:
`C:\Program Files\Blender Foundation\Blender 4.3\blender.exe`

Example used during development:
```powershell
& "C:\Program Files\Blender Foundation\Blender 4.3\blender.exe" --background --python make_mold.py -- bun.stl
```

Outputs (next to input, or into `outdir`):
`<stem>_mould_xl_yl.stl … xh_yh.stl` (the shell pieces) + `<stem>_cradle.stl`.

## The casting workflow (the mental model — important)

It's a **matrix / glove mould** for casting **silicone**:

```
        pour silicone in the gap ↓ (top sprue)
   ████        ████
   ████ ░░░░░░ ████     ████ = rigid 3D-printed shell (open bottom)
   ████ ░┌──┐░ ████     ░    = silicone, fills the GAP (= CORE_OFFSET)
   ████ ░│MD│░ ████     MD   = base model, sits in the cradle centre pocket
   ╞════╪═╪══╪═╪════╡
   └─ cradle: centre pocket (model) + outer recess (shell) + ridge (gap seal)
```

- Base model drops into the cradle **centre pocket**.
- Rigid shell pieces drop into the cradle **outer recess** around it.
- Silicone is poured (top sprue) into the **gap** between model and shell.
- The cradle floor + the ridge between the two pockets seal the bottom.
- Mould bottom is **OPEN** — the cradle is the floor, not the shell.

## Pipeline (what the script does)

1. import STL, join, bake transforms
2. decimate to `DECIMATE_TGT` tris (skip if already low)
3. voxel-remesh to clean / make manifold
4. OUTER = model offset OUT by `SHELL_OFFSET` (the shell wall)
5. FLANGE = model offset OUT by `SHELL_OFFSET+FLANGE_REACH`, intersected with a thin
   vertical slab (`FLANGE_THICK`) on each parting plane → flange band(s)
6. shell body = OUTER (∪ flange bands) − NEGATIVE, where NEGATIVE = model + `CORE_OFFSET`
   (this carves the silicone gap; the negative is the cavity cutter, NOT a separate core)
7. flat-bottom cut — **stays open** (cradle is the floor)
8. sprue funnel on top (`POUR_MODE="top"`), auto-placed at the cavity apex via ray-cast
9. cradle STL: centre pocket (model) + outer recess (shell), oversized by `CRADLE_TOL`
10. split into 2 (one flange) or 4 (two flanges @90°) pieces, with **ball registration
    keys** on the parting faces — keys ray-cast the flange edge so they hug the silhouette
    and never float; multiple keys up tall parts (`PIN_COUNT`, auto ~1 per 40mm)
11. export pieces + cradle; print **silicone volume needed** (the gap volume, in ml)

## Config knobs (top of make_mold.py)

| name | meaning |
|---|---|
| `SHELL_OFFSET` (4.2) | shell wall thickness |
| `CORE_OFFSET` (3.0) | silicone gap = how much bigger the cavity is than the model |
| `FLANGE_REACH` (10) / `FLANGE_THICK` (6) | flange lip stick-out / plate thickness |
| `BASE_CUT` (None) | mm up from model bottom for the flat cut; None = at model bottom; <0 disables |
| `DECIMATE_TGT` (6000) / `VOXEL_DETAIL` (220) / `VOXEL_SIZE` | poly budget / remesh fineness |
| `FOUR_PIECE` (True) | 4-piece (two planes) vs 2-piece |
| `ADD_FLANGE` (True) / `ADD_PINS` (True) | flange lips / ball keys |
| `PIN_RADIUS` (2) / `PIN_CLEAR` (0.25) / `PIN_COUNT` (None=auto) | ball key size / fit / count up height |
| `POUR_MODE` ("top") | "top"=sprue funnel, "bottom"=pour through open bottom |
| `SPRUE_TOP_R` (7) / `SPRUE_THROAT_R` (2.5) | funnel mouth / throat |
| `ADD_CRADLE` (True), `CRADLE_WALL` (4), `CRADLE_RECESS` (2), `CRADLE_FLOOR` (3), `CRADLE_TOL` (0.5) | cradle |

CLI flags: `--four/--two`, `--noflange`, `--nopins`, `--pins-n N`, `--pin R`,
`--core N`, `--shell N`, `--base N`, `--nobase`, `--pour top|bottom`, `--nocradle`,
`--voxel N`, `--target N`.

## Key technical lessons (don't re-learn the hard way)

- **Offsets must be constant-distance (Minkowski-sum-with-sphere)**, NOT scale. Implemented
  as **stepped dilation**: displace verts along normals by a small step, voxel-remesh, repeat.
  A single big vertex-normal push self-intersects → remesh returns garbage. `offset_solid()` does this.
- **Shell offset = `CORE_OFFSET + SHELL_OFFSET`** (var `WALL_OUT`). The wall must be measured
  from the CAVITY surface (model+gap), not the model — else the wall comes out only
  `SHELL_OFFSET − CORE_OFFSET` thin and collapses.
- **Voxel size is CAPPED at 0.5mm** (`vs = min(diag/VOXEL_DETAIL, 0.5)`). This is critical:
  coarser voxels (e.g. 0.84mm auto for a ~185mm-tall model) make the **EXACT booleans silently
  return empty** — the cavity carve collapses the body to ~40 tris and all 4 pieces come out
  empty (the "irregular shapes break" bug). 0.5mm works, 0.6mm already fails. Big models are
  therefore heavier (tesla bust → ~480k-tri body, 11MB pieces) but correct.
- **`make_box` bakes ALL transforms (incl. location).** Leaving location un-applied makes the
  EXACT solver silently return empty when the box is large / far from origin.
- `boolean()` takes a `solver=` arg ('EXACT' default, 'FAST' available). FAST is more tolerant
  of messy meshes but leaves non-manifold output, so we keep EXACT everywhere + the voxel cap.
- **Cradle ridge = the silicone gap seal.** Width ≈ `CORE_OFFSET − 2·CRADLE_TOL`; keep
  `CORE_OFFSET > 2·CRADLE_TOL` or the ridge vanishes.
- Render-to-PNG with `BLENDER_WORKBENCH` + MATCAP was the verification method (scratchpad scripts).
  Verified on `bun.stl` (small, symmetric) and `tesla.stl` (tall irregular bust).

## Pour reservoir (replaced the old cone sprue)

At the cavity apex (highest point, found by ray-cast), a solid **cylinder post** is unioned on
top of the shell as an **overflow reservoir**, then a narrower **bore** is drilled down through
it into the cavity so it never plugs the hollow. `POUR_R` (post radius), `POUR_BORE` (channel),
`POUR_RES_H` (height above roof). `--pour bottom` skips it.

The **flange is capped in Z at the shell top** (`flange_top = outer top`); otherwise the flange
(built from a bigger offset `+FLANGE_REACH`) towers above the shell and blocks the pour mouth
at the apex. The lip still sticks out radially for clamping — it just doesn't rise above.

## Current status

All committed up to the matrix-mould rework + handoff. This session's changes (commit them):
- silicone-volume report (gap volume in ml)
- pour **reservoir** cylinder (replaced cone sprue)
- **wall offset fix** (`WALL_OUT = CORE_OFFSET + SHELL_OFFSET`)
- **voxel cap at 0.5mm** → fixes empty pieces on big / irregular models

DISCARDED earlier (too much work for little): rotational lock peg + embossed labels.

## Ideas / backlog (not built)

1. silicone-volume report — **DONE**
2. irregular-shape pieces breaking — **FIXED (voxel cap)**
3. rotational lock peg — *discarded*
4. part labels / FRONT arrow — *discarded*
5. auto-orient model (flattest face down) before moulding
6. auto parting axis (split along widest girth instead of world X/Y) to ease release
7. config presets (JSON profiles) instead of editing the script
8. manifold-cleanup pass to guarantee watertight pieces
9. adaptive voxel: only go fine where needed (speed up big models)
10. Blender addon GUI (sliders + button) instead of CLI

## Environment notes

- Blender 4.3.2. Script targets 4.x API (`wm.stl_import` / `wm.stl_export`) with legacy fallback.
- Git push uses Git Credential Manager: `git config --global credential.helper manager`
  (VS Code's GIT_ASKPASS was broken — "Permission denied" — bypassed by GCM).
