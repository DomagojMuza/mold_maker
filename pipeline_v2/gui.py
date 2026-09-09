#!/usr/bin/env python3
"""
gui.py - two-phase mould picker for make_mold2, with a Qt side panel.

    <venv>/python gui.py <master.stl> [--two] [--nofeet] [--nopins] [--novents]
                          [--nocradle] [--nolip] [--voxel N]

3-D window - LEFT mouse = rotate / pan / zoom; RIGHT-click the surface to place a
point in the current mode. The right dock has every other knob (spin boxes +
check boxes) plus the phase action buttons.

Keys (focus the 3-D view):
    P  pour mode         S  split mode (press again: X<->Y plane)
    V  vent (phase 2)    F  foot (phase 2)
    U  undo current      C  clear current phase's picks
    G / Enter  = the dock's primary action (Build shell / Finish)

Phase 1 sets the shell + pour + parting planes -> Build shell.
Phase 2 places vents/feet on the real shell + tunes cradle/lip/pins -> Finish.
Any pick category left empty auto-places.  Output -> <stem>_v2out/.
"""
import os, sys, io, time
os.environ.setdefault("QT_API", "pyqt5")

import numpy as np
import pyvista as pv
import vtk
from PyQt5 import QtWidgets, QtCore
from pyvistaqt import QtInteractor

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_mold2 as mm
import meshlib.mrmeshpy as mr

COL = {"pour": "#ff7722", "split": "#cc33cc", "vent": "#ffcc00", "foot": "#33cc55"}
PIECE_COL = ["#dd8866", "#88cc77", "#7799dd", "#e0cc66"]

# (attr, label, lo, hi, step, decimals)
FIELDS = {
    "pick1": [("SHELL_OFFSET", "Shell offset (mm)", 0.5, 20, 0.1, 1),
              ("CORE_OFFSET", "Core / silicone gap", 0.0, 15, 0.1, 1),
              ("FLANGE_REACH", "Flange reach", 0.0, 40, 0.5, 1),
              ("FLANGE_THICK", "Flange thickness", 0.0, 25, 0.5, 1),
              ("POUR_R", "Pour post R", 1.0, 20, 0.5, 1),
              ("POUR_BORE", "Pour bore R", 0.3, 15, 0.1, 1),
              ("POUR_RES_H", "Pour post height", 2.0, 50, 1.0, 1),
              ("VOXEL", "Voxel  (0 = auto)", 0.0, 2.0, 0.05, 2)],
    "pick2": [("CRADLE_MARGIN", "Cradle margin", 2, 50, 0.5, 1),
              ("CRADLE_RECESS", "Cradle recess", 0.5, 15, 0.5, 1),
              ("CRADLE_FLOOR", "Cradle floor", 1, 20, 0.5, 1),
              ("CRADLE_TOL", "Cradle fit tol", 0.0, 3, 0.05, 2),
              ("LIP_THICK", "Lip thickness", 0.5, 15, 0.5, 1),
              ("LIP_INSET", "Lip inset", 0.0, 25, 0.5, 1),
              ("PIN_RADIUS", "Pin radius", 0.5, 10, 0.25, 2),
              ("PIN_CLEAR", "Pin clearance", 0.0, 2, 0.05, 2),
              ("VENT_BORE", "Vent bore R", 0.2, 6, 0.1, 1),
              ("VENT_POST_R", "Vent post R", 0.5, 10, 0.25, 2),
              ("VENT_MIN_SEP", "Vent min separation", 5, 80, 1, 0),
              ("STAB_R", "Foot base R", 1, 25, 0.5, 1),
              ("STAB_TIP_R", "Foot tip R", 0.5, 15, 0.5, 1)],
}
CHECKS = {
    "pick1": [("FOUR_PIECE", "4-piece split")],
    "pick2": [("ADD_VENTS", "Air vents"), ("ADD_FEET", "Stab feet"),
              ("ADD_CRADLE", "Cradle"), ("ADD_LIP", "Clamping lip"),
              ("ADD_PINS", "Registration pins")],
}


def to_pv(mesh):
    import trimesh
    b = io.BytesIO()
    mr.saveMesh(mesh, "*.stl", b)
    b.seek(0)
    return pv.wrap(trimesh.load(b, file_type="stl", force="mesh"))


def build_cfg(argv):
    cfg = mm.Config()
    i = 0
    while i < len(argv):
        a = argv[i]
        if   a == "--two":      cfg.FOUR_PIECE = False
        elif a == "--novents":  cfg.ADD_VENTS = False
        elif a == "--nofeet":   cfg.ADD_FEET = False
        elif a == "--nopins":   cfg.ADD_PINS = False
        elif a == "--nocradle": cfg.ADD_CRADLE = False
        elif a == "--nolip":    cfg.ADD_LIP = False
        elif a == "--voxel":    cfg.VOXEL = float(argv[i + 1]); i += 1
        i += 1
    return cfg


class Main(QtWidgets.QMainWindow):
    def __init__(self, src, cfg):
        super().__init__()
        self.cfg = cfg
        self.resize(1280, 860)

        self.plotter = QtInteractor(self)
        self.setCentralWidget(self.plotter.interactor)
        self.plotter.set_background("white")

        self.phase = "pick1"
        self.mode = "pour"
        self.picks = {"pour": [], "vent": [], "foot": []}
        self.split_x = self.split_y = None
        self.split_axis = "x"
        self.state = None
        self.dyn = []          # scene meshes
        self.markers = []      # pick markers + split guides + pour preview

        # RIGHT-click places a point on the model surface; LEFT mouse stays pure
        # camera (rotate / pan). A right-DRAG is a zoom, so only a click that
        # didn't move counts as a placement.
        self._picker = vtk.vtkCellPicker()
        self._picker.SetTolerance(0.005)
        self._rmb_xy = None
        try:
            self.plotter.iren.add_observer("RightButtonPressEvent", self._rmb_down)
            self.plotter.iren.add_observer("RightButtonReleaseEvent", self._rmb_up)
        except Exception:
            self.plotter.enable_surface_point_picking(
                callback=self._on_pick, show_point=False, left_clicking=True, show_message=False)

        for k, fn in {"p": lambda: self._set_mode("pour"), "v": lambda: self._set_mode("vent"),
                      "f": lambda: self._set_mode("foot"), "s": self._split_key,
                      "u": self._undo, "c": self._clear,
                      "g": self._primary, "Return": self._primary}.items():
            self.plotter.add_key_event(k, fn)

        self._build_dock()
        self._load_model(src)

    # ------------------------------------------------------ model / output
    @staticmethod
    def _elide(s, n=36):
        return s if len(s) <= n else "..." + s[-(n - 3):]

    def _load_model(self, path):
        self.src = os.path.abspath(path)
        self.stem = os.path.splitext(os.path.basename(self.src))[0]
        self.outdir = os.path.join(os.path.dirname(self.src), f"{self.stem}_v2out")
        self.setWindowTitle(f"mold_maker v2 - {self.stem}")
        self.master_pv = pv.read(self.src)
        b = self.master_pv.bounds
        self.center = np.array([(b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2])
        self.diag = self.master_pv.length
        self.state = None
        self.picks = {"pour": [], "vent": [], "foot": []}
        self.split_x = self.split_y = None
        self.split_axis = "x" if self.cfg.FOUR_PIECE else "y"
        self.btn_primary.setEnabled(True)
        self._refresh_paths()
        self._enter_pick1()

    def _refresh_paths(self):
        self.model_lbl.setText(self._elide(self.src)); self.model_lbl.setToolTip(self.src)
        self.out_lbl.setText(self._elide(self.outdir)); self.out_lbl.setToolTip(self.outdir)

    def _browse_model(self):
        fn, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Choose master STL", os.path.dirname(getattr(self, "src", "") or ""),
            "STL files (*.stl);;All files (*)")
        if fn:
            try:
                self._load_model(fn)
            except Exception as e:
                import traceback; traceback.print_exc()
                self.status.setText(f"LOAD FAILED:\n{e}")

    def _browse_output(self):
        d = QtWidgets.QFileDialog.getExistingDirectory(self, "Output folder", self.outdir)
        if d:
            self.outdir = d
            self._refresh_paths()

    # ---------------------------------------------------------------- dock
    def _build_dock(self):
        self.dock = QtWidgets.QDockWidget("Parameters", self)
        self.dock.setFeatures(QtWidgets.QDockWidget.NoDockWidgetFeatures)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, self.dock)

        outer = QtWidgets.QWidget()
        ov = QtWidgets.QVBoxLayout(outer)

        paths = QtWidgets.QGridLayout()
        paths.addWidget(QtWidgets.QLabel("Model"), 0, 0)
        self.model_lbl = QtWidgets.QLabel("-"); paths.addWidget(self.model_lbl, 0, 1)
        mb = QtWidgets.QPushButton("Browse..."); mb.clicked.connect(self._browse_model)
        paths.addWidget(mb, 0, 2)
        paths.addWidget(QtWidgets.QLabel("Output"), 1, 0)
        self.out_lbl = QtWidgets.QLabel("-"); paths.addWidget(self.out_lbl, 1, 1)
        ob = QtWidgets.QPushButton("Browse..."); ob.clicked.connect(self._browse_output)
        paths.addWidget(ob, 1, 2)
        paths.setColumnStretch(1, 1)
        ov.addLayout(paths)

        self.status = QtWidgets.QLabel()
        self.status.setWordWrap(True)
        self.status.setStyleSheet("font-weight:bold; padding:4px;")
        ov.addWidget(self.status)

        # pour shape (phase 1 only; hidden in phase 2)
        self.shape_row = QtWidgets.QWidget()
        sr = QtWidgets.QHBoxLayout(self.shape_row); sr.setContentsMargins(0, 0, 0, 0)
        sr.addWidget(QtWidgets.QLabel("Pour shape"))
        self.shape_combo = QtWidgets.QComboBox()
        self.shape_combo.addItems(["round", "square"])
        self.shape_combo.setCurrentText(self.cfg.POUR_SHAPE)
        self.shape_combo.currentTextChanged.connect(self._on_shape)
        sr.addWidget(self.shape_combo, 1)
        ov.addWidget(self.shape_row)

        scroll = QtWidgets.QScrollArea(); scroll.setWidgetResizable(True)
        self.form_host = QtWidgets.QWidget()
        self.form = QtWidgets.QFormLayout(self.form_host)
        scroll.setWidget(self.form_host)
        ov.addWidget(scroll, 1)

        row = QtWidgets.QHBoxLayout()
        self.btn_primary = QtWidgets.QPushButton()
        self.btn_primary.clicked.connect(self._primary)
        self.btn_back = QtWidgets.QPushButton("< Back")
        self.btn_back.clicked.connect(self._back)
        self.btn_restart = QtWidgets.QPushButton("Restart")
        self.btn_restart.clicked.connect(self._restart)
        row.addWidget(self.btn_primary, 2); row.addWidget(self.btn_back); row.addWidget(self.btn_restart)
        ov.addLayout(row)

        self.dock.setWidget(outer)
        self.dock.setMinimumWidth(300)

    def _populate_form(self):
        while self.form.rowCount():
            self.form.removeRow(0)
        self._spins = {}
        for attr, label, lo, hi, step, dec in FIELDS.get(self.phase, []):
            sb = QtWidgets.QDoubleSpinBox()
            sb.setRange(lo, hi); sb.setSingleStep(step); sb.setDecimals(dec)
            cur = getattr(self.cfg, attr)
            sb.setValue(0.0 if cur is None else float(cur))
            sb.valueChanged.connect(lambda v, a=attr: self._set_param(a, v))
            self._spins[attr] = sb
            self.form.addRow(label, sb)
        for attr, label in CHECKS.get(self.phase, []):
            cb = QtWidgets.QCheckBox()
            cb.setChecked(bool(getattr(self.cfg, attr)))
            cb.toggled.connect(lambda s, a=attr: self._set_param(a, s))
            self.form.addRow(label, cb)
        self.shape_row.setVisible(self.phase == "pick1")

    def _set_param(self, attr, v):
        if attr == "VOXEL":
            self.cfg.VOXEL = None if float(v) < 0.15 else float(v)
        elif isinstance(getattr(self.cfg, attr), bool):
            setattr(self.cfg, attr, bool(v))
        else:
            setattr(self.cfg, attr, float(v))
        if attr in ("POUR_R", "POUR_BORE", "POUR_RES_H"):
            self._redraw_markers()
        self._status()

    def _on_shape(self, txt):
        self.cfg.POUR_SHAPE = txt
        self._redraw_markers(); self._status()

    # ---------------------------------------------------------- phases
    def _clear_scene(self):
        for a in self.dyn:
            self.plotter.remove_actor(a)
        self.dyn = []

    def _clear_markers(self):
        for a in self.markers:
            self.plotter.remove_actor(a)
        self.markers = []

    def _enter_pick1(self):
        self.phase = "pick1"; self.mode = "pour"
        self._clear_scene(); self._clear_markers()
        self.dyn.append(self.plotter.add_mesh(self.master_pv, color="#c9c9c9",
                                              smooth_shading=True, name="master"))
        self._populate_form()
        self.btn_primary.setText("Build shell  >")
        self.btn_back.setEnabled(False)
        self._redraw_markers(); self._status(); self.plotter.reset_camera()

    def _enter_pick2(self):
        self.phase = "pick2"; self.mode = "vent"
        self._clear_scene(); self._clear_markers()
        self.dyn.append(self.plotter.add_mesh(to_pv(self.state["body"]), color="#cdb79e",
                                              smooth_shading=True, name="shell"))
        self._populate_form()
        self.btn_primary.setText("Finish & export  >")
        self.btn_back.setEnabled(False)
        self._redraw_markers(); self._status(); self.plotter.reset_camera()

    def _enter_result(self, paths, cradle_path):
        self.phase = "result"
        self._clear_scene(); self._clear_markers()
        for i, (_, fp) in enumerate(paths):
            self.dyn.append(self.plotter.add_mesh(pv.read(fp), color=PIECE_COL[i % 4],
                                                  smooth_shading=True, name=f"res{i}"))
        if cradle_path:
            self.dyn.append(self.plotter.add_mesh(pv.read(cradle_path), color="#bfbfbf",
                                                  opacity=0.45, name="rescradle"))
        while self.form.rowCount():
            self.form.removeRow(0)
        self.shape_row.setVisible(False)
        self.btn_primary.setText("(done)")
        self.btn_primary.setEnabled(False)
        self.btn_back.setEnabled(True)
        self.status.setText(f"DONE - {len(paths)} pieces + cradle\n{self.outdir}")
        self.plotter.reset_camera()

    # ---------------------------------------------------------- picking
    def _set_mode(self, m):
        if (self.phase == "pick1" and m in ("pour", "split")) or \
           (self.phase == "pick2" and m in ("vent", "foot")):
            self.mode = m; self._status()

    def _split_key(self):
        if self.phase != "pick1":
            return
        if self.mode != "split":
            self.mode = "split"
        elif self.cfg.FOUR_PIECE:
            self.split_axis = "y" if self.split_axis == "x" else "x"
        self._redraw_markers(); self._status()

    def _rmb_down(self, *a):
        self._rmb_xy = self.plotter.iren.get_event_position()

    def _rmb_up(self, *a):
        start, self._rmb_xy = self._rmb_xy, None
        if start is None:
            return
        x, y = self.plotter.iren.get_event_position()
        if (x - start[0]) ** 2 + (y - start[1]) ** 2 > 25:   # dragged -> zoom, not a placement
            return
        self._picker.Pick(x, y, 0, self.plotter.renderer)
        if self._picker.GetActor() is not None:
            self._on_pick(np.asarray(self._picker.GetPickPosition(), float))

    def _on_pick(self, point, *a):
        if self.phase == "result" or point is None:
            return
        xyz = np.asarray(point, float).ravel()[:3]
        if self.mode == "split":
            if self.split_axis == "x":
                self.split_x = float(xyz[0])
            else:
                self.split_y = float(xyz[1])
        elif self.mode == "pour":
            self.picks["pour"] = [xyz]
        else:
            self.picks[self.mode].append(xyz)
        self._redraw_markers(); self._status()

    def _undo(self):
        if self.mode == "split":
            if self.split_axis == "x":
                self.split_x = None
            else:
                self.split_y = None
        elif self.mode == "pour":
            self.picks["pour"] = []
        elif self.picks.get(self.mode):
            self.picks[self.mode].pop()
        self._redraw_markers(); self._status()

    def _clear(self):
        if self.phase == "pick1":
            self.picks["pour"] = []; self.split_x = self.split_y = None
        elif self.phase == "pick2":
            self.picks["vent"] = []; self.picks["foot"] = []
        self._redraw_markers(); self._status()

    # ---------------------------------------------------------- drawing
    def _split_xy(self):
        return (self.split_x if self.split_x is not None else float(self.center[0]),
                self.split_y if self.split_y is not None else float(self.center[1]))

    def _redraw_markers(self):
        self._clear_markers()
        r = max(self.diag * 0.012, 0.6)
        for cat in ("pour", "vent", "foot"):
            for q in self.picks[cat]:
                self.markers.append(self.plotter.add_mesh(pv.Sphere(radius=r, center=q), reset_camera=False, pickable=False,
                                                          color=COL[cat], name=f"mk{cat}{len(self.markers)}"))
        if self.phase == "pick1":
            self._draw_split_guides()
            self._draw_pour_preview()

    def _draw_split_guides(self):
        sx, sy = self._split_xy()
        b = self.master_pv.bounds; pad = self.diag
        ax, ay = self.mode == "split" and self.split_axis == "x", self.mode == "split" and self.split_axis == "y"
        xz = pv.Plane(center=(sx, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2), direction=(1, 0, 0),
                      i_size=b[5] - b[4] + pad, j_size=b[3] - b[2] + pad)
        yz = pv.Plane(center=((b[0] + b[1]) / 2, sy, (b[4] + b[5]) / 2), direction=(0, 1, 0),
                      i_size=b[1] - b[0] + pad, j_size=b[5] - b[4] + pad)
        if self.cfg.FOUR_PIECE:
            self.markers.append(self.plotter.add_mesh(xz, color=COL["split"], reset_camera=False, pickable=False,
                                opacity=0.32 if ax else 0.13, name="gx"))
        self.markers.append(self.plotter.add_mesh(yz, color=COL["split"], reset_camera=False, pickable=False,
                            opacity=0.32 if ay else 0.13, name="gy"))

    def _draw_pour_preview(self):
        c = self.cfg
        b = self.master_pv.bounds
        if self.picks["pour"]:
            x, y = float(self.picks["pour"][0][0]), float(self.picks["pour"][0][1])
        else:
            x, y = float(self.center[0]), float(self.center[1])
        z0, z1 = b[5] - 2.0, b[5] + c.POUR_RES_H
        zc = (z0 + z1) / 2
        bore = min(c.POUR_BORE, c.POUR_R - 1.5)
        if c.POUR_SHAPE == "square":
            post = pv.Cube(center=(x, y, zc), x_length=2 * c.POUR_R, y_length=2 * c.POUR_R, z_length=z1 - z0)
            hole = pv.Cube(center=(x, y, zc), x_length=2 * bore, y_length=2 * bore, z_length=z1 - z0 + 2)
        else:
            post = pv.Cylinder(center=(x, y, zc), direction=(0, 0, 1), radius=c.POUR_R, height=z1 - z0)
            hole = pv.Cylinder(center=(x, y, zc), direction=(0, 0, 1), radius=bore, height=z1 - z0 + 2)
        self.markers.append(self.plotter.add_mesh(post, color="#ff7722", opacity=0.25, reset_camera=False, pickable=False, name="pp"))
        self.markers.append(self.plotter.add_mesh(hole, color="#cc3300", opacity=0.55, reset_camera=False, pickable=False, name="pb"))

    def _status(self):
        if self.phase == "pick1":
            fx = "auto" if self.split_x is None else f"{self.split_x:.0f}"
            fy = "auto" if self.split_y is None else f"{self.split_y:.0f}"
            ax = f"  [moving {self.split_axis.upper()}]" if self.mode == "split" else ""
            self.status.setText(f"PHASE 1  mode [{self.mode}]{ax}\n"
                                f"pour {'set' if self.picks['pour'] else 'auto'} - "
                                f"split X:{fx}  Y:{fy}\nP pour  S split  -  RIGHT-click to place")
        elif self.phase == "pick2":
            self.status.setText(f"PHASE 2  mode [{self.mode}]\n"
                                f"vents {len(self.picks['vent']) or 'auto'}   "
                                f"feet {len(self.picks['foot']) or 'auto'}\n"
                                f"V vent  F foot  -  RIGHT-click the shell to place")

    # ---------------------------------------------------------- actions
    def _primary(self):
        if self.phase == "pick1":
            self._build()
        elif self.phase == "pick2":
            self._finish()

    def _busy(self, msg):
        self.status.setText(msg)
        QtWidgets.QApplication.processEvents()

    def _build(self):
        self._busy("building shell...  (see console)")
        pour = [tuple(q) for q in self.picks["pour"]] or None
        split = (self.split_x, self.split_y)
        try:
            master = mr.loadMesh(self.src)
            mm._t0 = time.time()
            self.state = mm.build_shell(self.cfg, master, pour=pour, split=split)
        except Exception as e:
            import traceback; traceback.print_exc()
            self.status.setText(f"BUILD FAILED:\n{e}")
            return
        self._enter_pick2()

    def _finish(self):
        self._busy("finishing...  (see console)")
        vents = [tuple(q) for q in self.picks["vent"]] or None
        feet = [tuple(q) for q in self.picks["foot"]] or None
        os.makedirs(self.outdir, exist_ok=True)
        try:
            mm._t0 = time.time()
            res = mm.finish_mould(self.cfg, self.state, vents=vents, feet=feet)
        except Exception as e:
            import traceback; traceback.print_exc()
            self.status.setText(f"FINISH FAILED:\n{e}")
            return
        paths = []
        for tag, msh in res["pieces"]:
            fp = os.path.join(self.outdir, f"{self.stem}_mould_{tag}.stl")
            mm.export(msh, fp, self.cfg.EXPORT_MAX_ERR)
            paths.append((tag, fp))
        cradle_path = None
        if res["cradle"] is not None:
            cradle_path = os.path.join(self.outdir, f"{self.stem}_cradle.stl")
            mm.export(res["cradle"], cradle_path, self.cfg.EXPORT_MAX_ERR,
                      voxel=self.cfg.VOXEL, reheal=True)
        self._enter_result(paths, cradle_path)

    def _back(self):
        if self.phase == "result" and self.state is not None:
            self.btn_primary.setEnabled(True)
            self._enter_pick2()

    def _restart(self):
        self.state = None
        for k in self.picks:
            self.picks[k] = []
        self.split_x = self.split_y = None
        self.btn_primary.setEnabled(True)
        self._enter_pick1()


def main():
    args = sys.argv[1:]
    if not args or args[0].startswith("--"):
        raise SystemExit(__doc__)
    if not os.path.isfile(args[0]):
        raise SystemExit(f"[gui] no such file: {args[0]}")
    app = QtWidgets.QApplication(sys.argv)
    try:
        w = Main(args[0], build_cfg(args[1:]))
    except Exception:
        import traceback
        traceback.print_exc()
        raise SystemExit("[gui] failed to start - traceback above")
    w.show()
    print("[gui] window open. Close it or Ctrl+C to exit.", flush=True)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
