@echo off
rem ============================================================
rem replay_latest_g1.bat - ONE-CLICK replay of your newest export
rem   Picks the newest .npy OR .csv in D:\BlenderPro\G1 (by mtime)
rem   and replays it, auto-using g1_dfq.usd (53 joints) when found.
rem   Double-click it, or pass extra args, e.g.:
rem     replay_latest_g1.bat --speed 0.5
rem   To replay a specific file use replay_g1.bat instead.
rem ============================================================
setlocal
if not defined ISAACLAB_BAT set "ISAACLAB_BAT=C:\isaac-lab\isaaclab.bat"
call "%ISAACLAB_BAT%" -p "%~dp0replay_trajectory_isaaclab.py" --latest --loop %*
