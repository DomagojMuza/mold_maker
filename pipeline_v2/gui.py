#!/usr/bin/env python3
"""
gui.py - two-phase mould picker for make_mold2, with a Qt side panel.

    <venv>/python gui.py <master.stl> [--two] [--nofeet] [--nopins] [--novents]
                          [--nocradle] [--nolip] [--voxel N]

3-D window - LEFT mouse = rotate / pan / zoom; RIGHT-click the surface to place a
point in the current mode. The right dock has every other knob (spin boxes +
check boxes) plus the phase action buttons.

Keys (focus the 3-D view):
    P  pour mode         S  split mode (press again: cycles X / Y / extra planes)
    N  add an extra split plane (own pivot + angle - for wings/limbs the
       base X/Y pair can't give a clean pull direction)
    V  vent (phase 2)    F  foot (phase 2)
    U  undo current      C  clear current phase's picks
    G / Enter  = the dock's primary action (Build shell / Finish)

Phase 1 sets the shell + pour + parting planes -> Build shell.
Phase 2 places vents/feet on the real shell + tunes cradle/lip/pins -> Finish.
Any pick category left empty auto-places.  Output -> <stem>_v2out/.
"""
import os, sys, io, time, math
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
    "pick1": [("MODEL_OFFSET_X", "Model offset X", -1000, 1000, 1.0, 1),
              ("MODEL_OFFSET_Y", "Model offset Y", -1000, 1000, 1.0, 1),
              ("SHELL_OFFSET", "Shell offset (mm)", 0.5, 20, 0.1, 1),
              ("CORE_OFFSET", "Core / silicone gap", 0.0, 15, 0.1, 1),
              ("FLANGE_REACH", "Flange reach", 0.0, 40, 0.5, 1),
              ("FLANGE_THICK", "Flange thickness", 0.0, 25, 0.5, 1),
              ("POUR_R", "Pour post R", 0.1, 1000, 0.5, 1),
              ("POUR_BORE", "Pour bore R", 0.1, 1000, 0.1, 1),
              ("POUR_RES_H", "Pour post height", 0.0, 1000, 1.0, 1),
              ("VOXEL", "Voxel  (0 = auto)", 0.0, 2.0, 0.05, 2),
              ("SPLIT_ANGLE_X", "Split angle X (deg)", -180, 180, 5, 0),
              ("SPLIT_ANGLE_Y", "Split angle Y (deg)", -180, 180, 5, 0)],
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


class _Worker(QtCore.QObject):
    """Runs a slow callable (build_shell / finish_mould, several seconds of
    pure MeshLib work, no Qt/GUI touched) on a worker QThread so the Qt event
    loop keeps pumping and the render window stays alive/repainting instead
    of going blank while the main thread is blocked - the long-standing
    'GUI generate blocks the window' gap, which on Windows can leave the 3D
    view showing nothing at all until the OS decides to repaint it."""
    done = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)

    def __init__(self, fn):
        super().__init__()
        self.fn = fn

    def run(self):
        try:
            result = self.fn()
        except Exception as e:
            import traceback; traceback.print_exc()
            self.failed.emit(str(e))
        else:
            self.done.emit(result)


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
        self.extra_planes = []   # [{"x":.., "y":.., "angle":..}, ...] -- see _add_plane
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
                      "n": self._add_plane,
                      "u": self._undo, "c": self._clear,
                      "Left": lambda: self._nudge_angle(-5), "Right": lambda: self._nudge_angle(5),
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
        self._master_base = pv.read(self.src)          # never moved; offset copies are derived from this
        self.master_pv = self._master_base
        self.diag = self.master_pv.length
        self.cfg.MODEL_OFFSET_X = self.cfg.MODEL_OFFSET_Y = 0.0
        b = self.master_pv.bounds
        self.center = np.array([(b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2])
        self.state = None
        self.picks = {"pour": [], "vent": [], "foot": []}
        self.split_x = self.split_y = None
        self.split_axis = "x" if self.cfg.FOUR_PIECE else "y"
        self.extra_planes = []
        self.btn_primary.setEnabled(True)
        self._refresh_paths()
        self._enter_pick1()

    def _apply_model_offset(self, new_ox, new_oy):
        """Slide the model under a FIXED world-space split axis. Surface
        picks (pour/vent/foot) move with the model since they're tied to a
        feature on it; the split pivot (once explicitly set) stays put in
        world space so sliding the model actually changes which part of it
        falls on which side of the cut. An unset (auto) split pivot tracks
        the model's own centre, which itself moves with the model."""
        dx, dy = new_ox - self.cfg.MODEL_OFFSET_X, new_oy - self.cfg.MODEL_OFFSET_Y
        self.cfg.MODEL_OFFSET_X, self.cfg.MODEL_OFFSET_Y = new_ox, new_oy
        if dx == 0 and dy == 0:
            return
        self.master_pv = self._master_base.translate((new_ox, new_oy, 0.0), inplace=False)
        b = self.master_pv.bounds
        self.center = np.array([(b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2])
        shift = np.array([dx, dy, 0.0])
        for cat in self.picks:
            self.picks[cat] = [q + shift for q in self.picks[cat]]
        if self.phase == "pick1":
            self._clear_scene()
            self.dyn.append(self.plotter.add_mesh(self.master_pv, color="#c9c9c9",
                                                  smooth_shading=True, name="master"))
            self._redraw_markers()

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
        self.btn_browse_model = QtWidgets.QPushButton("Browse...")
        self.btn_browse_model.clicked.connect(self._browse_model)
        paths.addWidget(self.btn_browse_model, 0, 2)
        paths.addWidget(QtWidgets.QLabel("Output"), 1, 0)
        self.out_lbl = QtWidgets.QLabel("-"); paths.addWidget(self.out_lbl, 1, 1)
        self.btn_browse_output = QtWidgets.QPushButton("Browse...")
        self.btn_browse_output.clicked.connect(self._browse_output)
        paths.addWidget(self.btn_browse_output, 1, 2)
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

        # extra split planes (phase 1 only) - own pivot + angle each, for
        # limbs/wings the base X/Y pair can't give a clean pull direction
        self.planes_box = QtWidgets.QGroupBox("Extra split planes (wings, limbs...)")
        pv_lay = QtWidgets.QVBoxLayout(self.planes_box)
        self.planes_list = QtWidgets.QListWidget()
        self.planes_list.setMaximumHeight(70)
        self.planes_list.currentRowChanged.connect(self._on_plane_row)
        pv_lay.addWidget(self.planes_list)
        prow = QtWidgets.QHBoxLayout()
        addp = QtWidgets.QPushButton("+ Add (N)"); addp.clicked.connect(self._add_plane)
        remp = QtWidgets.QPushButton("- Remove"); remp.clicked.connect(self._remove_active_plane)
        prow.addWidget(addp); prow.addWidget(remp)
        pv_lay.addLayout(prow)
        pform = QtWidgets.QFormLayout()
        self.plane_x_sb = QtWidgets.QDoubleSpinBox(); self.plane_x_sb.setRange(-100000, 100000); self.plane_x_sb.setDecimals(1)
        self.plane_y_sb = QtWidgets.QDoubleSpinBox(); self.plane_y_sb.setRange(-100000, 100000); self.plane_y_sb.setDecimals(1)
        self.plane_a_sb = QtWidgets.QDoubleSpinBox(); self.plane_a_sb.setRange(-1000, 1000); self.plane_a_sb.setDecimals(1)
        self.plane_a_sb.setSingleStep(5)
        for sb, key in ((self.plane_x_sb, "x"), (self.plane_y_sb, "y"), (self.plane_a_sb, "angle")):
            sb.valueChanged.connect(lambda v, k=key: self._on_plane_spin(k, v))
        pform.addRow("Pivot X", self.plane_x_sb)
        pform.addRow("Pivot Y", self.plane_y_sb)
        pform.addRow("Angle (deg)", self.plane_a_sb)
        pv_lay.addLayout(pform)
        ov.addWidget(self.planes_box)

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
        self.planes_box.setVisible(self.phase == "pick1")
        if self.phase == "pick1":
            self._refresh_planes_list()

    def _set_param(self, attr, v):
        if attr == "MODEL_OFFSET_X":
            self._apply_model_offset(float(v), self.cfg.MODEL_OFFSET_Y); self._status(); return
        if attr == "MODEL_OFFSET_Y":
            self._apply_model_offset(self.cfg.MODEL_OFFSET_X, float(v)); self._status(); return
        if attr == "VOXEL":
            self.cfg.VOXEL = None if float(v) < 0.15 else float(v)
        elif isinstance(getattr(self.cfg, attr), bool):
            setattr(self.cfg, attr, bool(v))
        else:
            setattr(self.cfg, attr, float(v))
        if attr in ("POUR_R", "POUR_BORE", "POUR_RES_H", "SPLIT_ANGLE_X", "SPLIT_ANGLE_Y"):
            self._redraw_markers()
        self._status()

    def _nudge_angle(self, delta):
        """Left/Right arrow keys: rotate whichever plane is currently active in
        split mode (S cycles x/y/extras) by 5 degrees, independent of the
        others. No-op outside split mode - there'd be no way to say which plane."""
        if self.phase != "pick1" or self.mode != "split":
            return
        if self.split_axis in ("x", "y"):
            attr = "SPLIT_ANGLE_X" if self.split_axis == "x" else "SPLIT_ANGLE_Y"
            sb = self._spins[attr]
            sb.setValue(sb.value() + delta)   # fires _set_param via valueChanged
        else:
            idx = int(self.split_axis[1:])
            self.extra_planes[idx]["angle"] += delta
            self._refresh_planes_list()
            self._redraw_markers(); self._status()

    def _refresh_planes_list(self):
        self.planes_list.blockSignals(True)
        self.planes_list.clear()
        for i, pl in enumerate(self.extra_planes):
            self.planes_list.addItem(f"e{i}: ({pl['x']:.0f},{pl['y']:.0f})  {pl['angle']:.0f}deg")
        if self.split_axis.startswith("e"):
            row = int(self.split_axis[1:])
            if row < self.planes_list.count():
                self.planes_list.setCurrentRow(row)
        self.planes_list.blockSignals(False)
        self._sync_plane_spins()

    def _sync_plane_spins(self):
        idx = int(self.split_axis[1:]) if self.split_axis.startswith("e") else None
        enabled = idx is not None and idx < len(self.extra_planes)
        for sb in (self.plane_x_sb, self.plane_y_sb, self.plane_a_sb):
            sb.blockSignals(True)
        if enabled:
            pl = self.extra_planes[idx]
            self.plane_x_sb.setValue(pl["x"]); self.plane_y_sb.setValue(pl["y"]); self.plane_a_sb.setValue(pl["angle"])
        for sb in (self.plane_x_sb, self.plane_y_sb, self.plane_a_sb):
            sb.setEnabled(enabled)
            sb.blockSignals(False)

    def _on_plane_row(self, row):
        if row < 0 or row >= len(self.extra_planes):
            return
        self.mode = "split"
        self.split_axis = f"e{row}"
        self._sync_plane_spins()
        self._redraw_markers(); self._status()

    def _on_plane_spin(self, key, v):
        if not self.split_axis.startswith("e"):
            return
        idx = int(self.split_axis[1:])
        if idx >= len(self.extra_planes):
            return
        self.extra_planes[idx][key] = float(v)
        item = self.planes_list.item(idx) if idx < self.planes_list.count() else None
        if item is not None:
            pl = self.extra_planes[idx]
            self.planes_list.blockSignals(True)
            item.setText(f"e{idx}: ({pl['x']:.0f},{pl['y']:.0f})  {pl['angle']:.0f}deg")
            self.planes_list.blockSignals(False)
        self._redraw_markers(); self._status()

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

    def _plane_axes(self):
        """Ordered list of cyclable split-plane keys: base x[,y] + any extras."""
        return (["x"] if self.cfg.FOUR_PIECE else []) + ["y"] + \
               [f"e{i}" for i in range(len(self.extra_planes))]

    def _split_key(self):
        if self.phase != "pick1":
            return
        if self.mode != "split":
            self.mode = "split"
        else:
            axes = self._plane_axes()
            idx = axes.index(self.split_axis) if self.split_axis in axes else -1
            self.split_axis = axes[(idx + 1) % len(axes)]
        self._sync_plane_spins()
        self._redraw_markers(); self._status()

    def _add_plane(self):
        """N: add one more independently-angled parting plane (its own pivot,
        starts at model centre / 0deg) for shapes 2 planes can't handle alone
        (e.g. a wing that sweeps off at its own angle)."""
        if self.phase != "pick1":
            return
        self.extra_planes.append({"x": float(self.center[0]), "y": float(self.center[1]), "angle": 0.0})
        self.mode = "split"
        self.split_axis = f"e{len(self.extra_planes) - 1}"
        self._refresh_planes_list()
        self._redraw_markers(); self._status()

    def _remove_active_plane(self):
        if self.phase != "pick1" or not self.split_axis.startswith("e"):
            return
        idx = int(self.split_axis[1:])
        if 0 <= idx < len(self.extra_planes):
            del self.extra_planes[idx]
        self.split_axis = "y"
        self._refresh_planes_list()
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
            elif self.split_axis == "y":
                self.split_y = float(xyz[1])
            else:
                idx = int(self.split_axis[1:])
                self.extra_planes[idx]["x"] = float(xyz[0])
                self.extra_planes[idx]["y"] = float(xyz[1])
                self._refresh_planes_list()
        elif self.mode == "pour":
            self.picks["pour"] = [xyz]
        else:
            self.picks[self.mode].append(xyz)
        self._redraw_markers(); self._status()

    def _undo(self):
        if self.mode == "split":
            if self.split_axis == "x":
                self.split_x = None
            elif self.split_axis == "y":
                self.split_y = None
            else:
                idx = int(self.split_axis[1:])
                self.extra_planes[idx]["x"] = float(self.center[0])
                self.extra_planes[idx]["y"] = float(self.center[1])
                self._refresh_planes_list()
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
        b = self.master_pv.bounds
        zc = (b[4] + b[5]) / 2
        zsize = (b[5] - b[4]) + self.diag                # Z extent isn't affected by rotation
        wsize = 2 * self.diag + 10                        # generous, orientation-independent

        def plane_mesh(px, py, angle_deg):
            r = math.radians(angle_deg)
            return pv.Plane(center=(px, py, zc), direction=(math.cos(r), math.sin(r), 0.0),
                            i_size=zsize, j_size=wsize)

        ax = self.mode == "split" and self.split_axis == "x"
        ay = self.mode == "split" and self.split_axis == "y"
        if self.cfg.FOUR_PIECE:
            self.markers.append(self.plotter.add_mesh(plane_mesh(sx, sy, self.cfg.SPLIT_ANGLE_X),
                                color=COL["split"], reset_camera=False, pickable=False,
                                opacity=0.32 if ax else 0.13, name="gx"))
        self.markers.append(self.plotter.add_mesh(plane_mesh(sx, sy, self.cfg.SPLIT_ANGLE_Y),
                            color=COL["split"], reset_camera=False, pickable=False,
                            opacity=0.32 if ay else 0.13, name="gy"))
        for i, pl in enumerate(self.extra_planes):        # wing/limb planes: own pivot, own colour
            active = self.mode == "split" and self.split_axis == f"e{i}"
            self.markers.append(self.plotter.add_mesh(plane_mesh(pl["x"], pl["y"], pl["angle"]),
                                color="#3399ff", reset_camera=False, pickable=False,
                                opacity=0.32 if active else 0.13, name=f"ge{i}"))

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
            mo = (f"  model offset:({self.cfg.MODEL_OFFSET_X:.0f},{self.cfg.MODEL_OFFSET_Y:.0f})"
                  if (self.cfg.MODEL_OFFSET_X or self.cfg.MODEL_OFFSET_Y) else "")
            ep = f"  extra planes: {len(self.extra_planes)}" if self.extra_planes else ""
            self.status.setText(f"PHASE 1  mode [{self.mode}]{ax}{mo}{ep}\n"
                                f"pour {'set' if self.picks['pour'] else 'auto'} - "
                                f"split X:{fx}  Y:{fy}\n"
                                f"angle X:{self.cfg.SPLIT_ANGLE_X:.0f}deg  "
                                f"Y:{self.cfg.SPLIT_ANGLE_Y:.0f}deg (independent)\n"
                                f"P pour  S split (cycles x/y/extras)  N add plane\n"
                                f"Left/Right = rotate active plane  -  RIGHT-click to place")
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

    def _run_async(self, fn, on_done, fail_prefix):
        """Run `fn` (no args, returns a picklable-free plain result - just
        MeshLib objects/dicts, no Qt) on a worker thread; `on_done(result)`
        runs back on the main/GUI thread once it finishes. Keeps the primary
        button disabled (and the Qt event loop alive) for the duration.

        The completion signals connect to bound methods of `self` (a QObject
        that lives on the main thread), not plain closures - PyQt can only
        infer 'queue this back onto the receiver's own thread' from a real
        QObject receiver, so a bound method is what actually keeps the
        MeshLib result handling (which touches VTK/Qt widgets) off the
        worker thread. A closure here would run cross-thread instead."""
        self.btn_primary.setEnabled(False)
        self.btn_back.setEnabled(False)
        self.btn_restart.setEnabled(False)
        self.btn_browse_model.setEnabled(False)
        self._bg_on_done, self._bg_fail_prefix = on_done, fail_prefix
        thread = QtCore.QThread(self)
        worker = _Worker(fn)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.done.connect(self._on_bg_done)
        worker.failed.connect(self._on_bg_failed)
        worker.done.connect(thread.quit)
        worker.failed.connect(thread.quit)
        thread.finished.connect(thread.deleteLater)
        # keep references alive for the thread's lifetime (Qt won't GC a
        # thread/worker still running just because this function returned)
        self._bg_thread, self._bg_worker = thread, worker
        thread.start()

    def _bg_ui_reset(self):
        self.btn_restart.setEnabled(True)
        self.btn_browse_model.setEnabled(True)
        if self.phase != "result":
            self.btn_primary.setEnabled(True)

    def _on_bg_done(self, result):
        self._bg_ui_reset()
        self._bg_on_done(result)

    def _on_bg_failed(self, msg):
        self._bg_ui_reset()
        self.status.setText(f"{self._bg_fail_prefix}:\n{msg}")

    def _build(self):
        self._busy("building shell...  (running in background)")
        pour = [tuple(q) for q in self.picks["pour"]] or None
        split = (self.split_x, self.split_y)
        self.cfg.EXTRA_PLANES = [(pl["x"], pl["y"], pl["angle"]) for pl in self.extra_planes]
        src, cfg = self.src, self.cfg

        def work():
            master = mr.loadMesh(src)
            mm._t0 = time.time()
            return mm.build_shell(cfg, master, pour=pour, split=split)

        def done(state):
            self.state = state
            self._enter_pick2()

        self._run_async(work, done, "BUILD FAILED")

    def _finish(self):
        self._busy("finishing...  (running in background)")
        vents = [tuple(q) for q in self.picks["vent"]] or None
        feet = [tuple(q) for q in self.picks["foot"]] or None
        os.makedirs(self.outdir, exist_ok=True)
        cfg, state, outdir, stem = self.cfg, self.state, self.outdir, self.stem

        def work():
            mm._t0 = time.time()
            res = mm.finish_mould(cfg, state, vents=vents, feet=feet)
            paths = []
            for tag, msh in res["pieces"]:
                fp = os.path.join(outdir, f"{stem}_mould_{tag}.stl")
                mm.export(msh, fp, cfg.EXPORT_MAX_ERR)
                paths.append((tag, fp))
            cradle_path = None
            if res["cradle"] is not None:
                cradle_path = os.path.join(outdir, f"{stem}_cradle.stl")
                mm.export(res["cradle"], cradle_path, cfg.EXPORT_MAX_ERR,
                          voxel=cfg.VOXEL, reheal=True)
            return paths, cradle_path

        def done(result):
            paths, cradle_path = result
            self._enter_result(paths, cradle_path)

        self._run_async(work, done, "FINISH FAILED")

    def _back(self):
        if self.phase == "result" and self.state is not None:
            self.btn_primary.setEnabled(True)
            self._enter_pick2()

    def _restart(self):
        self.state = None
        for k in self.picks:
            self.picks[k] = []
        self.split_x = self.split_y = None
        self.extra_planes = []
        self.master_pv = self._master_base
        self.cfg.MODEL_OFFSET_X = self.cfg.MODEL_OFFSET_Y = 0.0
        b = self.master_pv.bounds
        self.center = np.array([(b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2])
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
