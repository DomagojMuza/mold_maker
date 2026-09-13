# svg_extrude - SVG outline -> solid, one side offset

`svg_to_3d.py` turns a filled SVG shape into a watertight prism, then offsets
**one side** of it - either the outline on one half (in-plane) or one flat cap
(along Z).

Same stack as `../pipeline_v2` (trimesh + meshlib venv) plus `shapely` /
`svgpathtools` / `mapbox_earcut` for the 2-D side.

## Run

### GUI

```bash
svg_extrude\run_gui.bat logo.svg          # double-clickable; SVG arg optional
# or: ..\offset_bench\.venv\Scripts\python.exe gui.py ..\logo.svg
```

3-D view on the left (LEFT mouse = orbit / pan / zoom). The right dock is
everything else: **SVG** + **Output** rows with `Browse...`, then live spin boxes
- **Resize (x)**, **Extrude height**, **Inflate outline** (all-sides offset),
**Widen cap X/Y** (the taper), **Move top cap Z** (the flat-cap offset) - plus
the `widen face` / `face` combos, a
checkable **Negative mould** group (wall / floor / clearance / freeboard), and an
**Advanced** group (curve density, simplify, flip-Y). The mesh rebuilds on every
change (~5-65 ms), the status line turns green/red on watertight, **Export STL**
(or `Ctrl+S`) writes to the output folder (`_mould` suffix when the mould is on).
It never blocks - no worker thread needed because the geometry is that cheap (see
**Speed**). `--grow` (one-sided in-plane offset) is CLI-only.

### CLI

```bash
cd svg_extrude
../offset_bench/.venv/Scripts/python.exe svg_to_3d.py logo.svg out.stl \
    --height 10 --grow 4 --grow-axis x --grow-side max
```

Both share `svg_to_3d.build(svg, height=, scale=, widen=, offset=, grow=, ...)`.

### `--inflate` - offset the whole outline (all sides, both axes)

Straight 2-D offset of the outline - `+X`, `-X`, `+Y`, `-Y` all move out by N mm,
holes shrink by N. `-N` offsets inward. This is the "make the shape bigger
everywhere" operation.

| flag | default | meaning |
|---|---|---|
| `--inflate N` | 0 | offset the whole outline out N mm (`-N` = in) |
| `--inflate-join mitre\|round\|bevel` | round | corner style of the offset outline |

Pieces closer than `2 * inflate` merge (physical - their offsets overlap).
Runs before `--widen` / `--offset` / `--mold`, so those act on the inflated shape.

### `--grow` - offset ONE side of the outline (in-plane)

Splits the outline at a parting line and dilates **only one half** outward; the
other half is untouched, so the parting line gets a step.

| flag | default | meaning |
|---|---|---|
| `--grow N` | 0 | dilate one half of the outline by N mm (`-N` carves that half in) |
| `--grow-axis x\|y` | x | axis the parting line is perpendicular to |
| `--grow-side max\|min` | max | which half moves: `max` = +axis side, `min` = -axis side |
| `--grow-at N` | bbox centre | world-mm position of the parting line |
| `--grow-join mitre\|round\|bevel` | mitre | corner style of the grown edge - use `round`/`bevel` for large `--grow`, `mitre` spikes at sharp corners |

### `--widen` - taper: offset one cap's outline in X/Y

Takes the top (or bottom) cap and slides its **outline** outward by N mm - the
walls become a straight slope from the SVG outline at one cap to SVG+N at the
other. This is the "take the top face and widen it in X and Y" operation.

| flag | default | meaning |
|---|---|---|
| `--widen N` | 0 | move **every edge** of one cap out N mm (`-N` = that cap smaller) |
| `--widen-face top\|bottom` | top | which cap gets the wider outline |
| `--widen-steps N` | 10 | stacked slabs across the taper - higher = smoother wall, slower |

This is the Blender "extrude Z, select the top face, *Offset Edges → Move*" flow:
a true uniform in-plane offset of one cap's outline. The cross-section at height
fraction `t` (from the un-widened cap) is `outline.buffer(N * t)`; it is built as
`--widen-steps` stacked extruded slabs `boolean.union`-ed, so **+X, -X, +Y, -Y
all move out exactly N** and the un-widened cap is the plain extrude - every
bottom vertex stays on the SVG outline (measured: 0.00 mm off). Holes shrink by
N. Pieces closer than `2 N` fuse where their drafts meet (physical). The wall is
a fine staircase - raise `--widen-steps` to smooth it. Cost: ~50-100 ms for one
outline, ~0.5 s for the 16-glyph logo.

### `--offset` - offset one flat cap along Z

| flag | default | meaning |
|---|---|---|
| `--offset N` | 0 | move one cap by N mm along Z (`+` thicker, `-` carves) |
| `--face top\|bottom` | top | which cap moves |

On a plain uniform extrude this is just "make the slab taller/shorter" - both
caps stay flat, nothing one-sided to see. Use `--grow` (silhouette) or `--widen`
(taper) for a visible change.

### `--mold` - negative mould (subtract the model from a tray)

Presses the finished model (after `--widen` / `--offset`) into a tray and
subtracts it: a solid base with a model-shaped **pocket**, its mouth flush with
the base top, wrapped by a `--mold-wall`-thick rim that stands `--mold-freeboard`
mm above the model. `--widen` on the model becomes draft on the pocket walls, so
the cast part lifts straight out. Result bottom sits at z = 0.

| flag | default | meaning |
|---|---|---|
| `--mold` | off | build the negative mould instead of the bare model |
| `--mold-wall N` | 4 | perimeter rim thickness, mm |
| `--mold-floor N` | 4 | solid base under the deepest point of the pocket, mm |
| `--mold-clearance N` | 3 | gap from the model bbox to the wall inner face, mm |
| `--mold-freeboard N` | 3 | how far the walls stand above the model top, mm |

Footprint = model bbox + 2 x (`clearance` + `wall`). Total height = `floor` +
model height + `freeboard`. The tray + subtraction are `trimesh.boolean`
(manifold3d); adds ~30 ms.

### general

| flag | default | meaning |
|---|---|---|
| `--height N` | 10 | extrusion depth, mm |
| `--scale N` | 1 | multiply SVG coords, height, `--grow`, `--grow-at` |
| `--heal` | off | MeshLib repair of self-intersections / degenerate faces (see **Mesh integrity**) |
| `--no-check` | off | skip the MeshLib defect scan of the result |
| `--density N` | 1 | curve sample points per mm |
| `--simplify N` | 0.05 | shapely simplify tolerance, mm (`0` = keep every sample) |
| `--no-flip-y` | off | keep SVG's native Y-down orientation |

Output: `.stl` / `.obj` / `.ply` / `.3mf`. Exit non-zero if the result is not
watertight, has self-intersecting faces, or `--offset` would collapse the slab.

## Pipeline

1. **parse** - `svgpathtools.Document`: group/element transforms baked in,
   `<rect> <circle> <ellipse> <line> <polyline> <polygon>` -> paths, curves
   sampled at `--density` pts/mm.
2. **nest** - rings sorted by area; parent = smallest ring that contains it;
   even depth = solid, odd = hole. (Not point-in-count - a big ring's interior
   point can fall inside a small ring it encloses and be misread as a hole.)
3. **grow one side** (`--grow`) - `shapely`: `mp.buffer(N)` dilates the whole
   outline, intersect that with the +half (or -half) plane, union with the
   untouched other half of the original.
4. **build the solid** - `trimesh.creation.extrude_polygon` per shape (`z` in
   `[0, height]`, vertical walls). With `--widen`, each exterior vertex is then
   slid along its 2-D outward normal by `widen * z/height` toward `--widen-face`.
5. **offset one cap** (`--offset`) - every vertex on the global top (or bottom)
   plane is translated in Z. Walls stretch to follow; mesh stays watertight.

## Speed

The geometry is trivial - measured on this repo's venv, warm process, per-phase:

| SVG | contours | parse | polygons | build (plain / widen) | export |
|---|---|---|---|---|---|
| test (3 shapes) | 3 | 4 ms | 1 ms | 4 ms / 4 ms | <1 ms |
| logo (16 glyphs) | 22 | 13 ms | 3 ms | 9 ms / 28 ms | <1 ms |

So a full rebuild is **~10-45 ms**. The ~0.8 s you see on a cold `python
svg_to_3d.py ...` run is almost entirely the numpy/trimesh/shapely/svgpathtools
import; once a process is warm (the GUI) every knob is real-time and no
background thread is needed.

## Mesh integrity

Every `build()` runs a MeshLib scan and reports `self_intersections` /
`degenerate_faces` in `info` (CLI prints them and exits non-zero on any;
the GUI status line goes red with the count).

| stage | manifold / watertight | self-intersections |
|---|---|---|
| plain extrude, `--offset` | yes | none |
| `--widen` (any shape, any amount, incl. serif text) | yes | none for one outline; a few on the 16-glyph logo at large `widen` - `--heal` clears them |
| `--mold` (incl. multi-piece serif text + `--widen`) | yes | none |
| `--grow` on a multi-piece SVG | yes | a few where the grown halves of nearby pieces meet |

When `--widen` drafts a multi-piece SVG the neighbouring pieces flare into each
other; `negative_mold` `boolean.union`s the pieces into clean single geometry
**before** the subtraction, so the overlaps never reach the CSG. (Where two
letters are closer than `2 * widen` their draft cones genuinely merge - that is
physical, not a defect.)

`--heal` / the GUI **Heal** checkbox is a safety valve for anything the checks
still flag: MeshLib's local **Relax** pass nudges only the offending vertices
(no detail deleted, no remesh) then pins the base plane flat. It is a no-op on a
clean mesh, which is now the normal case.

## Limits

- **Filled closed paths only.** Convert strokes and text to fills first
  (Inkscape: *Path > Stroke to Path*, *Object to Path*).
- **`--grow` fuses nearby pieces** on the grown side - two glyphs closer than
  `2 * grow` merge into one blob. Fine for a single-outline shape (a mould
  half); messy for multi-letter logos.
- `--grow` leaves a **step at the parting line** (the grown half is wider in the
  cross-axis right at the split). That is the intended "one side is oversized"
  behaviour, not a blend.
- `mitre` join spikes at sharp corners for large `--grow`; switch to `round`.
- `--widen` self-intersection on serif text - see **Mesh integrity** above.
- Unit scaling from the SVG header (`mm`, `in`, dpi) is not read - use `--scale`.

## Deps

Added to `../offset_bench/.venv` on top of the pipeline_v2 set:

```bash
../offset_bench/.venv/Scripts/python.exe -m pip install shapely svgpathtools mapbox_earcut
```
