@echo off
rem  double-click to launch the svg_extrude GUI (optionally drop an .svg on it)
"%~dp0..\offset_bench\.venv\Scripts\python.exe" "%~dp0gui.py" %*
if errorlevel 1 pause
