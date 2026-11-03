@echo off
rem LidarLab : l'application unique du TP (récolter, nettoyer, recaler, cartes, localiser, exporter)
cd /d "%~dp0"
".venv\Scripts\python.exe" -m lidar.app %*
if errorlevel 1 pause
