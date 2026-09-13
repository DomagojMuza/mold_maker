#!/usr/bin/env python3
"""gui.py - live SVG -> solid with a Qt side panel.

    ..\offset_bench\.venv\Scripts\python.exe gui.py [logo.svg]

3-D view on the left (LEFT mouse = orbit / pan / zoom). Everything else is the
dock on the right: pick the SVG and output folder, then drag the spin boxes -
Resize, Extrude height, Widen (taper one cap in X/Y), Move top cap (Z), and the
one-sided Grow. The mesh rebuilds on every change (a few ms), stats update live,
red = not watertight. Export writes an STL to the output folder.
"""
import os
import sys

os.environ.setdefault("QT_API", "pyqt5")

import numpy as np
import pyvista as pv
from PyQt5 import QtCore, QtGui, QtWidgets
from pyvistaqt import QtInteractor

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import svg_to_3d as S

MESH_COLOR = "#c8a978"

# attr, label, lo, hi, step, decimals, tooltip
FIELDS = [
    ("scale",   "Resize  (x)",           0.05,  50.0, 0.05, 2, "multiply SVG size and height"),
    ("height",  "Extrude height (mm)",    0.10, 500.0, 0.5,  2, "prism depth before any taper"),
    ("inflate", "Inflate outline (mm)", -50.0,  50.0, 0.5,  2, "offset the whole outline out on every side (- = in); holes shrink"),
    ("widen",   "Widen cap X/Y (mm)",  -50.0,  50.0, 0.5,  2, "taper: average edge of one cap moves out this much (- = smaller)"),
    ("offset",  "Move top cap Z (mm)", -50.0,  50.0, 0.5,  2, "shift the top (or bottom) cap along Z"),
]
MOLD = [
    ("mold_wall",      "Wall thickness (mm)",  0.5, 50.0, 0.5, 2, "perimeter rim thickness"),
    ("mold_floor",     "Floor thickness (mm)", 0.5, 50.0, 0.5, 2, "solid base under the deepest point of the cavity"),
    ("mold_clearance", "Clearance (mm)",       0.0, 50.0, 0.5, 2, "gap between the model bbox and the wall inner face"),
    ("mold_freeboard", "Freeboard (mm)",       0.0, 50.0, 0.5, 2, "how far the walls stand above the model top"),
]
ADV = [
    ("widen_steps", "Widen slabs (wall smoothness)", 2, 40, 1, 0),
    ("density",     "Curve density (pt/mm)",       0.1, 20.0, 0.1, 2),
    ("simplify",    "Simplify tol (mm)",           0.0,  5.0, 0.01, 2),
]


class Win(QtWidgets.QMainWindow):
    def __init__(self, svg=None, outdir=None):
        super().__init__()
        self.setWindowTitle("svg_extrude")
        self.resize(1180, 760)

        self.p = {"scale": 1.0, "height": 10.0, "inflate": 0.0, "widen": 0.0,
                  "widen_steps": 10, "offset": 0.0, "density": 1.0, "simplify": 0.05,
                  "widen_face": "top", "face": "top", "flip_y": True, "heal": False,
                  "mold": False, "mold_wall": 4.0, "mold_floor": 4.0,
                  "mold_clearance": 3.0, "mold_freeboard": 3.0}
        self.src = svg
        self.outdir = outdir or (os.path.dirname(svg) if svg else os.getcwd())
        self.mesh = None

        self.view = QtInteractor(self)
        self.setCentralWidget(self.view)
        self.view.add_axes()

        self._build_dock()

        self._timer = QtCore.QTimer(self, singleShot=True, interval=50)
        self._timer.timeout.connect(self._rebuild)
        QtWidgets.QShortcut(QtGui.QKeySequence("Ctrl+S"), self, self._export)

        if self.src:
            self._rebuild(reset_cam=True)
        self._refresh_paths()

    # ---------------------------------------------------------------- dock
    def _build_dock(self):
        dock = QtWidgets.QDockWidget("Parameters", self)
        dock.setFeatures(QtWidgets.QDockWidget.NoDockWidgetFeatures)
        dock.setMinimumWidth(320)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, dock)
        host = QtWidgets.QWidget()
        v = QtWidgets.QVBoxLayout(host)

        paths = QtWidgets.QGridLayout()
        paths.addWidget(QtWidgets.QLabel("SVG"), 0, 0)
        self.svg_lbl = QtWidgets.QLabel("-")
        paths.addWidget(self.svg_lbl, 0, 1)
        b1 = QtWidgets.QPushButton("Browse...")
        b1.clicked.connect(self._browse_svg)
        paths.addWidget(b1, 0, 2)
        paths.addWidget(QtWidgets.QLabel("Output"), 1, 0)
        self.out_lbl = QtWidgets.QLabel("-")
        paths.addWidget(self.out_lbl, 1, 1)
        b2 = QtWidgets.QPushButton("Browse...")
        b2.clicked.connect(self._browse_out)
        paths.addWidget(b2, 1, 2)
        paths.setColumnStretch(1, 1)
        v.addLayout(paths)

        self.status = QtWidgets.QLabel()
        self.status.setWordWrap(True)
        self.status.setStyleSheet("padding:4px;")
        v.addWidget(self.status)

        form = QtWidgets.QFormLayout()
        self.spins = {}
        for attr, label, lo, hi, step, dec, tip in FIELDS:
            sb = QtWidgets.QDoubleSpinBox()
            sb.setRange(lo, hi)
            sb.setSingleStep(step)
            sb.setDecimals(dec)
            sb.setValue(self.p[attr])
            sb.setToolTip(tip)
            sb.valueChanged.connect(lambda val, a=attr: self._set(a, val))
            self.spins[attr] = sb
            form.addRow(label, sb)
        v.addLayout(form)

        combos = QtWidgets.QFormLayout()
        self.combos = {}
        for attr, opts in [("widen_face", ["top", "bottom"]),
                           ("face", ["top", "bottom"])]:
            cb = QtWidgets.QComboBox()
            cb.addItems(opts)
            cb.setCurrentText(self.p[attr])
            cb.currentTextChanged.connect(lambda txt, a=attr: self._set(a, txt))
            self.combos[attr] = cb
            combos.addRow(attr.replace("_", " "), cb)
        v.addLayout(combos)

        self.mold_box = QtWidgets.QGroupBox("Negative mould  (subtract model from a tray)")
        self.mold_box.setCheckable(True)
        self.mold_box.setChecked(self.p["mold"])
        self.mold_box.toggled.connect(self._toggle_mold)
        mf = QtWidgets.QFormLayout(self.mold_box)
        for attr, label, lo, hi, step, dec, tip in MOLD:
            sb = QtWidgets.QDoubleSpinBox()
            sb.setRange(lo, hi)
            sb.setSingleStep(step)
            sb.setDecimals(dec)
            sb.setValue(self.p[attr])
            sb.setToolTip(tip)
            sb.valueChanged.connect(lambda val, a=attr: self._set(a, val))
            self.spins[attr] = sb
            mf.addRow(label, sb)
        v.addWidget(self.mold_box)

        adv = QtWidgets.QGroupBox("Advanced")
        adv.setCheckable(True)
        adv.setChecked(False)
        af = QtWidgets.QFormLayout(adv)
        for attr, label, lo, hi, step, dec in ADV:
            sb = QtWidgets.QDoubleSpinBox()
            sb.setRange(lo, hi)
            sb.setSingleStep(step)
            sb.setDecimals(dec)
            sb.setValue(self.p[attr])
            sb.valueChanged.connect(lambda val, a=attr: self._set(a, val))
            self.spins[attr] = sb
            af.addRow(label, sb)
        fy = QtWidgets.QCheckBox("flip Y (SVG is Y-down)")
        fy.setChecked(True)
        fy.toggled.connect(lambda s: self._set("flip_y", s))
        af.addRow(fy)
        v.addWidget(adv)

        self.heal_cb = QtWidgets.QCheckBox("Heal self-intersections (MeshLib)")
        self.heal_cb.setToolTip("repair the self-crossing walls a sharp-corner "
                                "outline + large Widen can produce")
        self.heal_cb.toggled.connect(lambda s: self._set("heal", s))
        v.addWidget(self.heal_cb)

        self.edges = QtWidgets.QCheckBox("show mesh edges")
        self.edges.toggled.connect(lambda _s: self._draw())
        v.addWidget(self.edges)

        v.addStretch(1)
        exp = QtWidgets.QPushButton("Export STL   (Ctrl+S)")
        exp.clicked.connect(self._export)
        v.addWidget(exp)

        dock.setWidget(host)

    # ---------------------------------------------------------------- state
    def _set(self, attr, val):
        self.p[attr] = val
        self._timer.start()

    def _toggle_mold(self, on):
        self.p["mold"] = bool(on)
        self._timer.start()

    def _refresh_paths(self):
        self.svg_lbl.setText(os.path.basename(self.src) if self.src else "-")
        self.svg_lbl.setToolTip(self.src or "")
        self.out_lbl.setText(self.outdir)
        self.out_lbl.setToolTip(self.outdir)

    def _browse_svg(self):
        fn, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Choose SVG", os.path.dirname(self.src or "") or os.getcwd(),
            "SVG (*.svg);;All files (*)")
        if fn:
            self.src = fn
            self._refresh_paths()
            self._rebuild(reset_cam=True)

    def _browse_out(self):
        d = QtWidgets.QFileDialog.getExistingDirectory(self, "Output folder", self.outdir)
        if d:
            self.outdir = d
            self._refresh_paths()

    # ---------------------------------------------------------------- build
    def _rebuild(self, reset_cam=False):
        if not self.src:
            return
        try:
            mesh, info = S.build(
                self.src, height=self.p["height"], scale=self.p["scale"],
                inflate=self.p["inflate"],
                widen=self.p["widen"], widen_face=self.p["widen_face"],
                widen_steps=int(self.p["widen_steps"]),
                offset=self.p["offset"], face=self.p["face"],
                mold=self.p["mold"], mold_wall=self.p["mold_wall"],
                mold_floor=self.p["mold_floor"], mold_clearance=self.p["mold_clearance"],
                mold_freeboard=self.p["mold_freeboard"], heal=self.p["heal"],
                density=self.p["density"], simplify=self.p["simplify"],
                flip_y=self.p["flip_y"])
        except SystemExit as e:
            self.status.setText(str(e))
            self.status.setStyleSheet("padding:4px; color:#c00; font-weight:bold;")
            return
        except Exception as e:  # noqa: BLE001 - surface anything to the panel
            self.status.setText(f"{type(e).__name__}: {e}")
            self.status.setStyleSheet("padding:4px; color:#c00; font-weight:bold;")
            return

        self.mesh = mesh
        self._info = info
        wt = info["watertight"]
        sx = info.get("self_intersections", 0)
        kind = "mould" if info.get("mold") else f"{info['components']} solid(s)"
        flags = []
        if not wt:
            flags.append("NOT watertight")
        if sx:
            flags.append(f"{sx} self-intersections"
                         + ("" if self.p["heal"] else " - tick Heal"))
        tail = " | ".join(flags) if flags else "clean"
        self.status.setText(
            f"{info['faces']} tris | {kind} | "
            f"bbox {info['bbox'][0]:.1f} x {info['bbox'][1]:.1f} x {info['bbox'][2]:.1f} mm | {tail}")
        self.status.setStyleSheet(
            "padding:4px; font-weight:bold; " + ("color:#080;" if not flags else "color:#c00;"))
        self._draw(reset_cam)

    def _draw(self, reset_cam=False):
        if self.mesh is None:
            return
        self.view.remove_actor("solid")
        self.view.add_mesh(pv.wrap(self.mesh), name="solid", color=MESH_COLOR,
                           show_edges=self.edges.isChecked(), edge_color="dimgray",
                           smooth_shading=False)
        if reset_cam:
            self.view.reset_camera()
        self.view.render()

    # ---------------------------------------------------------------- export
    def _export(self):
        if self.mesh is None:
            return
        stem = os.path.splitext(os.path.basename(self.src))[0]
        suffix = "_mould" if self._info.get("mold") else "_extrude"
        default = os.path.join(self.outdir, f"{stem}{suffix}.stl")
        fn, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export mesh", default,
            "STL (*.stl);;OBJ (*.obj);;PLY (*.ply);;3MF (*.3mf)")
        if not fn:
            return
        self.mesh.export(fn)
        note = "" if self._info["watertight"] else "  (WARNING: not watertight)"
        self.status.setText(f"wrote {fn}{note}")


def main():
    ap_svg = sys.argv[1] if len(sys.argv) > 1 else None
    if ap_svg is None:
        here = os.path.dirname(os.path.abspath(__file__))
        cand = os.path.join(os.path.dirname(here), "logo.svg")
        ap_svg = cand if os.path.exists(cand) else None
    app = QtWidgets.QApplication(sys.argv)
    w = Win(ap_svg)
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
