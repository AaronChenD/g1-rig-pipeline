# -*- coding: utf-8 -*-
# convert_urdf_usd.py — URDF -> USD 命令行转换 (Isaac Lab 2.3 UrdfConverter)
# ============================================================================
# 为什么不用 GUI (File > Import): 5.x 的通用导入器对复杂 URDF 会报
#   'NoneType' object has no attribute 'name' 且不显示真实原因;
# 本脚本走 Isaac Lab 的 UrdfConverter, 无 GUI, 出错有完整 traceback。
#
# 用法 (默认参数 = DFQ 灵巧手 G1):
#   isaaclab.bat -p convert_urdf_usd.py
#   isaaclab.bat -p convert_urdf_usd.py --urdf D:\...\x.urdf --out D:\...\x.usd
#   (可选: --stiffness 100 --damping 5  调物理模式的 PD 增益)
#
# 转换后自动打开 USD 验证: 打印关节数与名字 (应含 53 个 URDF 关节名,
# 与回放脚本 --usd 配合即 53/53 全匹配, 手指手腕全动)。
# ============================================================================

import argparse
import os
import sys

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="URDF -> USD converter (G1 pipeline)")
parser.add_argument("--urdf", default=r"D:\BlenderPro\G1\unitree_ros\robots\g1_description"
                                      r"\g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf")
parser.add_argument("--out", default=r"D:\BlenderPro\G1\g1_dfq.usd")
parser.add_argument("--stiffness", type=float, default=100.0, help="关节驱动刚度 (Nm/rad)")
parser.add_argument("--damping", type=float, default=5.0, help="关节驱动阻尼 (Nm/(rad/s))")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

# ---- 两种目录布局自动互备 (与 isaac/ 其他脚本一致) ----
urdf = args_cli.urdf
if not os.path.isfile(urdf):
    d, b = os.path.split(urdf)
    deep = os.path.join("unitree_ros", "robots", "g1_description")
    for cand in (os.path.join(d, deep, b) if not d.endswith("g1_description") else
                 os.path.join(d[: -len(deep) - 1], b),):
        if os.path.isfile(cand):
            print("[convert] note: %s 不存在, 改用 %s" % (urdf, cand))
            urdf = cand
            break
if not os.path.isfile(urdf):
    print("[convert][ERROR] 找不到 URDF: %s (也试过另一布局)" % args_cli.urdf)
    sys.exit(1)

from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

out_usd = os.path.abspath(args_cli.out)
cfg = UrdfConverterCfg(
    asset_path=os.path.abspath(urdf),
    usd_dir=os.path.dirname(out_usd) or ".",
    usd_file_name=os.path.basename(out_usd),
    force_usd_conversion=True,          # 重跑即重新生成
    fix_base=False,                     # 浮动基座 (回放需要移动根)
    merge_fixed_joints=False,           # 保留完整 link 树, 名字与 meta 一一对应
    joint_drive=UrdfConverterCfg.JointDriveCfg(
        target_type="position",
        drive_type="force",
        gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(
            stiffness=args_cli.stiffness, damping=args_cli.damping),
    ),
)
print("[convert] URDF -> USD: %s" % out_usd)
converter = UrdfConverter(cfg)
print("[convert] 转换完成: %s" % converter.usd_path)

# ---- 验证: 打开 USD 数关节 ----
from pxr import Usd, UsdPhysics

stage = Usd.Stage.Open(converter.usd_path)
names = []
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.RevoluteJointAPI) or prim.IsA(UsdPhysics.RevoluteJoint):
        names.append(prim.GetName())
print("[convert] USD 内旋转关节 %d 个" % len(names))
print("[convert] %s" % ", ".join(sorted(names)))

simulation_app.close()
print("[convert] 下一步: replay_g1.bat <npy> --usd \"%s\" --loop" % out_usd)
