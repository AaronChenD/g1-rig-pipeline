@echo off
rem ============================================================
rem convert_urdf_g1.bat - URDF to USD via Isaac Lab UrdfConverter
rem   (no GUI; use this instead of File^>Import which can fail
rem    with cryptic errors on complex URDFs)
rem   output: D:\BlenderPro\G1\g1_dfq.usd (default)
rem   args pass through: --urdf path --out path --stiffness N --damping N
rem ============================================================
setlocal
if not defined ISAACLAB_BAT set "ISAACLAB_BAT=C:\isaac-lab\isaaclab.bat"
call "%ISAACLAB_BAT%" -p "%~dp0convert_urdf_usd.py" %*
