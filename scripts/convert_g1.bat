@echo off
REM ============================================================================
REM  convert_g1.bat — 一键把 G1 的 URDF 转成 Maya/Houdini 可用的 USD (Blender 4.4+/5.x)
REM
REM  用法:
REM     convert_g1.bat <URDF路径> <输出目录> [blender.exe路径] [单位: cm|m]
REM
REM  示例:
REM     convert_g1.bat D:\BlenderPro\G1\unitree_ros\robots\g1_description\g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf D:\BlenderPro\G1
REM     convert_g1.bat my_g1.urdf out\ "C:\Program Files\Blender Foundation\Blender 5.0\blender.exe"
REM     convert_g1.bat my_g1.urdf out\ "" m        &:: Houdini 用米制版 (默认 cm 给 Maya)
REM
REM  产出 (输出目录下):
REM     <名称>.blend              Blender 工程 (骨骼+网格+材质)
REM     <名称>.usda               USD (Y-up, UsdSkel 蒙皮骨骼; cm=厘米给 Maya, m=米给 Houdini)
REM     <名称>_skeleton_meta.json 关节轴/限位元数据 (做重定向时用)
REM     <名称>.png                预览渲染图
REM     (单位选 m 时文件名自动加 _m 后缀, 不会覆盖 Maya 用的厘米版)
REM ============================================================================
setlocal

set "URDF=%~1"
set "OUTDIR=%~2"
set "BLENDER=%~3"
set "UNITS=%~4"

if "%URDF%"=="" (
    echo 用法: convert_g1.bat ^<URDF路径^> ^<输出目录^> [blender.exe路径] [cm^|m]
    exit /b 1
)
if "%OUTDIR%"=="" (
    echo 用法: convert_g1.bat ^<URDF路径^> ^<输出目录^> [blender.exe路径] [cm^|m]
    exit /b 1
)
if not exist "%URDF%" (
    echo [错误] 找不到 URDF: %URDF%
    exit /b 1
)
if "%UNITS%"=="" set "UNITS=cm"
if /i not "%UNITS%"=="cm" if /i not "%UNITS%"=="m" (
    echo [错误] 单位参数只能是 cm 或 m (收到: %UNITS%)
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
set "SUFFIX="
if /i "%UNITS%"=="m" set "SUFFIX=_m"

echo [运行] Blender 后台转换: %URDF%  (单位: %UNITS%)
"%BLENDER%" --background --python "%~dp0blender_import_urdf.py" -- "%URDF%" ^
    --blend "%OUTDIR%\%NAME%%SUFFIX%.blend" ^
    --usd "%OUTDIR%\%NAME%%SUFFIX%.usda" ^
    --meta "%OUTDIR%\%NAME%%SUFFIX%_skeleton_meta.json" ^
    --render "%OUTDIR%\%NAME%%SUFFIX%_preview.png" ^
    --usd-units %UNITS%
if errorlevel 1 (
    echo [错误] 转换失败, 请把上面的报错信息反馈。
    exit /b 1
)

echo.
echo ============================================================
echo [完成] 输出到 %OUTDIR%
if /i "%UNITS%"=="m" (
echo   Houdini: /stage 里放 File LOP 直接选 %NAME%%SUFFIX%.usda (米制, 1:1)
echo   SOP/KineFX: USD Character Import 节点记得开 Convert Units
) else (
echo   Maya 导入: File ^> Import... 选择 %NAME%.usda
echo   (或用 scripts\maya_import_g1.py 脚本导入并自动检查)
)
echo ============================================================
endlocal
