@echo off
setlocal
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
    echo Missing .venv Python.
    echo Create it first, then install: pip install -r requirements-ppi.txt
    exit /b 1
)

".venv\Scripts\python.exe" "ppi_viewer.py"
