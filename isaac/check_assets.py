# -*- coding: utf-8 -*-
# check_assets.py — 诊断 Isaac Lab 云端资产可达性 (bipeds.py 崩溃排查)
# ============================================================================
# 背景: Isaac Sim 5.x 的内置资产 (地面 / 机器人 USD) 默认按需从 NVIDIA 云端
#       (Carb 设置 /persistent/isaac/asset_root/cloud) 下载。首次使用时若
#       该地址为空或网络不可达, spawn_ground_plane 会拿到空引用 -> 找不到
#       "Plane" 子 prim -> bind_physics_material(None) 崩溃:
#       "Stage.GetPrimAtPath(Stage, NoneType) did not match C++ signature"
#
# 用法:
#   isaaclab.bat -p check_assets.py
#   isaaclab.bat -p check_assets.py --headless
#
# 输出解读:
#   asset_root/cloud = None/""        -> 云端根路径没配置 (见打印的修复提示)
#   状态 0 (不存在/不可达)            -> 网络问题: 换网/代理后重试, 或走
#                                        --usd 本地机器人路线 (见 isaac/README)
#   状态 2 (服务器上存在)             -> 资产可达, bipeds 重试应能跑 (首次慢,
#                                        在下载)
# ============================================================================

from isaaclab.app import AppLauncher

import argparse

parser = argparse.ArgumentParser(description="Check Isaac cloud asset reachability")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(headless=args_cli.headless)
simulation_app = app_launcher.app

from isaaclab.utils.assets import (
    ISAAC_NUCLEUS_DIR,
    ISAACLAB_NUCLEUS_DIR,
    NUCLEUS_ASSET_ROOT_DIR,
    check_file_path,
)

print("=" * 60)
print("asset_root/cloud =", NUCLEUS_ASSET_ROOT_DIR)
print("ISAAC_NUCLEUS_DIR   =", ISAAC_NUCLEUS_DIR)
print("ISAACLAB_NUCLEUS_DIR=", ISAACLAB_NUCLEUS_DIR)
print("=" * 60)

if not NUCLEUS_ASSET_ROOT_DIR:
    print("[问题] 云端资产根路径为空! 修复 (任选其一):")
    print("  1) 联网重开一次 Isaac Sim UI, 让默认云端地址写回设置;")
    print("  2) 手动设置 (以 Kit 内 Python 或注册表方式):")
    print('     carb.settings.get_settings().set(')
    print('         "/persistent/isaac/asset_root/cloud",')
    print('         "http://omniverse-content-production.s3-us-west-2.amazonaws.com")')

targets = [
    ("地面 (GroundPlaneCfg)", f"{ISAAC_NUCLEUS_DIR}/Environments/Grid/default_environment.usd"),
    ("内置 G1", f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/G1/g1.usd"),
]
names = {0: "0 = 不可达/不存在", 1: "1 = 本地存在", 2: "2 = 云端可达"}
ok = True
for tag, p in targets:
    st = check_file_path(p)
    print("[资产] %-24s %s" % (tag, names.get(st, st)))
    if st == 0:
        ok = False
        print("       ", p)
print("=" * 60)
if ok:
    print("[结论] 资产可达 -> bipeds.py / 回放脚本可正常用云端资产 (首次会下载, 慢)")
else:
    print("[结论] 有资产不可达 -> ① 检查网络/代理后重跑本脚本; ② 或跳过云端依赖:")
    print("       用 Isaac Sim UI 的 URDF Importer 把本地 URDF 转成 USD,")
    print("       回放时 --usd 指定它 (replay 脚本已带本地地面兜底, 无需云端)。")

simulation_app.close()
