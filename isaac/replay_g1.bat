@echo off
rem ============================================================
rem replay_g1.bat - G1 trajectory replay shortcut
rem   replay_g1.bat <npy> [--usd robot.usd] [--loop] [--speed 0.5] [--physics]
rem   replay_g1.bat D:\BlenderPro\G1\myanim.npy --loop
rem   replay_g1.bat D:\BlenderPro\G1\myanim.npy --usd D:\BlenderPro\G1\g1_dfq.usd --loop
rem If Isaac Lab lives elsewhere: set ISAACLAB_BAT=C:\other\isaaclab.bat
rem ============================================================
setlocal
if not defined ISAACLAB_BAT set "ISAACLAB_BAT=C:\isaac-lab\isaaclab.bat"
if "%~1"=="" (
  echo Usage: replay_g1.bat ^<npy_path^> [--usd path] [--loop] [--speed x] [--physics]
  echo   e.g.  replay_g1.bat D:\BlenderPro\G1\myanim.npy --loop
  exit /b 1
)
call "%ISAACLAB_BAT%" -p "%~dp0replay_trajectory_isaaclab.py" --npy %*
