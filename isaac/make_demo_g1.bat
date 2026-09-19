@echo off
rem ============================================================
rem make_demo_g1.bat - generate 6s demo trajectory (squat+wave+twist)
rem   output: D:\BlenderPro\G1\g1_demo.npy/.csv/_columns.json
rem   all args pass through to make_demo_trajectory.py
rem   (--meta / --out / --fps / --duration)
rem ============================================================
setlocal
if not defined ISAACLAB_BAT set "ISAACLAB_BAT=C:\isaac-lab\isaaclab.bat"
call "%ISAACLAB_BAT%" -p "%~dp0make_demo_trajectory.py" %*
