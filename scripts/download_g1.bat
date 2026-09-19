@echo off
REM ============================================================================
REM  download_g1.bat — 一键下载宇树 unitree_ros 的 G1 机器人描述文件 (URDF+STL)
REM
REM  用法:
REM     download_g1.bat                 下载到当前目录下的 unitree_ros\
REM     download_g1.bat D:\robots       下载到 D:\robots\unitree_ros\
REM     download_g1.bat D:\robots all   额外包含 G1-D(轮式) 和独立灵巧手描述
REM
REM  说明:
REM     完整 unitree_ros 约 315 MB, 本脚本用 sparse-checkout 只取 G1 部分
REM     (g1_description 约 111 MB), 下载快、不占多余空间。
REM ============================================================================
setlocal enabledelayedexpansion

set "DEST=%~1"
set "EXTRA=%~2"
if "%DEST%"=="" set "DEST=%CD%\unitree_ros"

where git >nul 2>nul
if errorlevel 1 (
    echo [错误] 未检测到 git。请先安装 Git for Windows:
    echo        https://git-scm.com/download/win
    exit /b 1
)

if exist "%DEST%\robots\g1_description" (
    echo [跳过] %DEST%\robots\g1_description 已存在。
    echo        如需重新下载, 请先删除 %DEST%
    exit /b 0
)

echo [1/2] 克隆 unitree_ros (sparse 模式, 只取文件清单)...
git clone --depth 1 --filter=blob:none --sparse https://github.com/unitreerobotics/unitree_ros.git "%DEST%"
if errorlevel 1 (
    echo [错误] git clone 失败, 请检查网络。
    exit /b 1
)

echo [2/2] 拉取 G1 描述文件 (URDF + STL 网格)...
pushd "%DEST%"
if /i "%EXTRA%"=="all" (
    git sparse-checkout set robots/g1_description robots/g1_d_description robots/dexterous_hand_description robots/g1_with_brainco_hand
) else (
    git sparse-checkout set robots/g1_description
)
set "SPARSE_RC=%errorlevel%"
popd
if not "%SPARSE_RC%"=="0" (
    echo [错误] sparse-checkout 失败。
    exit /b 1
)

echo.
echo ============================================================
echo [完成] G1 文件已下载到: %DEST%\robots\g1_description
echo.
echo   推荐 (含 5 指灵巧手 Inspire, 53 自由度):
echo     g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf
echo   三指手版本 (43 自由度):
echo     g1_29dof_with_hand_rev_1_0.urdf
echo   纯身体无手 (29 自由度):
echo     g1_29dof_rev_1_0.urdf
echo ============================================================
endlocal
