@echo off
REM ============================================================================
REM  convert_g1.bat — 一键把 G1 的 URDF 转成 Maya/Houdini/UE 各自可用的 USD
REM                    (Blender 4.4+/5.x)
REM
REM  用法:
REM     convert_g1.bat <URDF路径> <输出目录> [blender.exe路径]
REM
REM  示例:
REM     convert_g1.bat D:\BlenderPro\G1\unitree_ros\robots\g1_description\g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf D:\BlenderPro\G1
REM     convert_g1.bat my_g1.urdf out\ "C:\Program Files\Blender Foundation\Blender 5.0\blender.exe"
REM
REM  产出 (输出目录下, 一次全出, 各 DCC 免手工换单位/轴向):
REM     <名称>.blend               Blender 工程 (骨骼+网格+材质)
REM     <名称>.usda                Maya 用    — cm + Y-up, 面向 +Z (Maya 标准)
REM     <名称>_houdini.usda        Houdini 用 — m + Y-up (原生米制, 不用开 Convert Units)
REM     <名称>_ue.usda             UE 用      — cm + Z-up, 面向 +X (UE 惯例, 免转换)
REM     <名称>_skeleton_meta.json  关节轴/限位元数据 (做重定向时用)
REM     <名称>_preview.png         预览渲染图
REM ============================================================================
setlocal

set "URDF=%~1"
set "OUTDIR=%~2"
set "BLENDER=%~3"

if "%URDF%"=="" (
    echo 用法: convert_g1.bat ^<URDF路径^> ^<输出目录^> [blender.exe路径]
    exit /b 1
)
if "%OUTDIR%"=="" (
    echo 用法: convert_g1.bat ^<URDF路径^> ^<输出目录^> [blender.exe路径]
    exit /b 1
)
if not exist "%URDF%" (
    echo [错误] 找不到 URDF: %URDF%
    exit /b 1
)

if "%BLENDER%"=="" (
    where blender >nul 2>nul
    if errorlevel 1 (
        echo [错误] 未找到 blender。请把 Blender 加入 PATH, 或把第 3 个参数
        echo        指定为 blender.exe 的完整路径, 例如:
        echo        "C:\Program Files\Blender Foundation\Blender 5.0\blender.exe"
        exit /b 1
    )
    set "BLENDER=blender"
) else (
    if not exist "%BLENDER%" (
        echo [错误] 未找到 blender: %BLENDER%
        exit /b 1
    )
)

if not exist "%OUTDIR%" mkdir "%OUTDIR%"

for %%I in ("%URDF%") do set "NAME=%%~nI"

echo [运行] Blender 后台转换: %URDF%
"%BLENDER%" --background --python "%~dp0blender_import_urdf.py" -- "%URDF%" ^
    --blend "%OUTDIR%\%NAME%.blend" ^
    --usd "%OUTDIR%\%NAME%.usda" ^
    --meta "%OUTDIR%\%NAME%_skeleton_meta.json" ^
    --render "%OUTDIR%\%NAME%_preview.png"
if errorlevel 1 (
    echo [错误] 转换失败, 请把上面的报错信息反馈。
    exit /b 1
)

echo.
echo ============================================================
echo [完成] 输出到 %OUTDIR%
echo   Maya    : File ^> Import... 选 %NAME%.usda
echo             (或用 scripts\maya_import_g1.py 脚本导入并自动检查)
echo   Houdini : File LOP / SOP 直接用 %NAME%_houdini.usda (米制, 1:1, 免转换)
echo   UE      : Content Browser 拖入 %NAME%_ue.usda (cm, Z-up, 免转换)
echo ============================================================
endlocal
