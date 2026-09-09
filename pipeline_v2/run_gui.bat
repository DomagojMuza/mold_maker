@echo off
rem  Double-click this, or:  run_gui.bat <master.stl> [flags]
rem  With no STL arg it defaults to ..\gingerbread_7cm.stl
setlocal
set PY=%~dp0..\offset_bench\.venv\Scripts\python.exe
set GUI=%~dp0gui.py

if not exist "%PY%" (
  echo [run_gui] venv python not found at:
  echo    %PY%
  echo Build it:  cd %~dp0..\offset_bench ^&^& python -m venv .venv ^&^& .venv\Scripts\python.exe -m pip install -r %~dp0requirements.txt
  pause & exit /b 1
)

cd /d "%~dp0.."
if "%~1"=="" (
  "%PY%" "%GUI%" gingerbread_7cm.stl
) else (
  "%PY%" "%GUI%" %*
)
echo.
echo [run_gui] exited. Window closed or error above.
pause
