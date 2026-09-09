"""Quick env check for pipeline_v2. Run with the venv python:
    ../offset_bench/.venv/Scripts/python.exe check_env.py
"""
import sys
print("python :", sys.executable)
print("version:", sys.version.split()[0])
for mod in ("meshlib", "trimesh", "numpy", "pyvista", "vtk"):
    try:
        m = __import__(mod)
        print(f"  OK   {mod:9} {getattr(m, '__version__', '?')}")
    except Exception as e:
        print(f"  MISS {mod:9} {type(e).__name__}: {e}")

try:
    import os
    print("PYVISTA_OFF_SCREEN env:", os.environ.get("PYVISTA_OFF_SCREEN"))
    import pyvista as pv
    print("pv.OFF_SCREEN        :", pv.OFF_SCREEN)
    print("theme.notebook      :", pv.global_theme.notebook)
    p = pv.Plotter(off_screen=True)
    p.add_mesh(pv.Sphere())
    p.screenshot("_env_ok.png")
    p.close()
    print("offscreen render    : OK  (_env_ok.png written)")
except Exception as e:
    import traceback; traceback.print_exc()
