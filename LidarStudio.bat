@echo off
rem Double-clic pour lancer LidarStudio (cartographie pas à pas) avec les scans réels de la salle
cd /d "%~dp0"
".venv\Scripts\python.exe" -m lidar.studio "data\salle\session_salle_2026-11-02.lidar.json"
if errorlevel 1 pause
