@echo off
rem Double-clic pour lancer LidarScope (visualisation en direct du lidar LD06)
cd /d "%~dp0"
".venv\Scripts\python.exe" -m lidar.scope
if errorlevel 1 pause
