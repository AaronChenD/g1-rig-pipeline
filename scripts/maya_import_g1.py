#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
maya_import_g1.py — 在 Maya 2025 中导入 g1-rig-pipeline 生成的 USD 骨骼文件
================================================================================

用法 (Maya 2025):
  1. 打开 Script Editor (窗口 > 常规编辑器 > 脚本编辑器)
  2. 切到 Python 标签, 把本文件全部内容粘贴进去
  3. 修改下面的 USD_FILE 路径, Ctrl+Enter (小键盘) 运行

期望结果 (以 g1_29dof_rev_1_0_with_inspire_hand_DFQ 为例):
  - 59 个 Maya joint (骨骼名 = URDF link 名, 例如 pelvis / left_shoulder_pitch_link)
  - 59 个 skinned mesh + skinCluster (每个 mesh 100% 绑定到自己的骨骼)
  - 机器人总高约 132 cm (Maya 默认 cm 单位下)
  - 站立方向: Y-up, 面朝 +Z (Maya 角色标准朝向)

备注:
  - USD 由 Blender 以 metersPerUnit=0.01 (厘米) 导出, 与 Maya 默认单位一致, 无需缩放。
  - 如果你导出时用了 --usd-units m (米制 USD), 请先在
    Windows > Settings/Preferences > Preferences > Settings > Working Units: Linear
    设为 meter 再导入, 或导入后调用本脚本末尾的 scale_rig(100)。

maya_import_g1.py — Import a g1-rig-pipeline USD skeleton into Maya 2025.
Run inside Maya's Script Editor (Python tab); edit USD_FILE first.
"""

import maya.cmds as cmds
import maya.mel as mel

# ---------------------------------------------------------------------------
# 配置 CONFIG
# ---------------------------------------------------------------------------
USD_FILE = r"C:\g1_rig\g1_29dof_rev_1_0_with_inspire_hand_DFQ.usda"   # <-- 改成你的路径


def load_usd_plugin():
    """Make sure the bundled maya-usd plugin is loaded (it ships with Maya 2022+)."""
    try:
        if not cmds.pluginInfo("mayaUsdPlugin", query=True, loaded=True):
            cmds.loadPlugin("mayaUsdPlugin")
        return True
    except Exception as e:
        print("[ERROR] 无法加载 mayaUsdPlugin: %s" % e)
        print("        请确认 Maya 2025 安装完整 (maya-usd 随 Maya 内置)。")
        return False


def import_usd(path):
    """Native USD import: UsdSkel -> Maya joints + skinCluster."""
    before = set(cmds.ls(long=True))
    try:
        # mayaUSDImport = File > Import (原生数据) 的脚本入口
        cmds.mayaUSDImport(file=path, primPath="/")
    except Exception as e:
        print("[ERROR] mayaUSDImport 失败: %s" % e)
        print("        备选: 菜单 File > Import..., 文件类型选 USD, 直接导入。")
        return False
    new = [o for o in cmds.ls(long=True) if o not in before]
    print("[OK] 导入完成, 新增顶层节点 %d 个" % len([o for o in new if '|' in o and o.count('|') == 1]))
    return True


def report():
    """Print a verification report of the imported rig."""
    joints = cmds.ls(type="joint") or []
    meshes = cmds.ls(type="mesh") or []
    skins = cmds.ls(type="skinCluster") or []
    print("=" * 60)
    print("场景检查 / Rig report")
    print("  joints      : %d" % len(joints))
    print("  meshes      : %d" % len(meshes))
    print("  skinClusters: %d" % len(skins))
    if joints:
        bbox = cmds.exactWorldBoundingBox(joints)
        h = bbox[4] - bbox[1]
        print("  骨骼包围盒高度: %.1f cm  (G1 整机约 132 cm)" % h)
        print("  根骨骼示例    : %s" % [j for j in joints if 'pelvis' in j or 'base' in j][:3])
    if joints and not skins:
        print("-" * 60)
        print("[注意] 有骨骼但没有 skinCluster。两种处理方式:")
        print("  A) 选中 USD 导入的骨骼根, 在 Outliner 右键 > Edit As Maya Data,")
        print("     然后手动 Smooth Bind (Rigid Bind 也行, 本绑定是刚体 100% 权重)。")
        print("  B) 改用 FBX 路线: Blender 里 File > Export > FBX (勾选 Armature/Skin),")
        print("     Maya 直接 File > Import FBX, skinCluster 会完整保留 (最稳)。")
    print("=" * 60)


def scale_rig(factor=100.0):
    """把整个刚体绑定机器人按 factor 缩放 (例如米制 USD 导入后 ×100 变 cm)。
    只缩放 joint — 网格由 skinCluster 驱动会跟着走, 不会出现双重变换。
    Scale the whole rigid-bound robot by `factor` (joints only; skinned geo follows)."""
    joints = cmds.ls(type="joint") or []
    if not joints:
        print("[ERROR] 场景中没有 joint")
        return
    cmds.select(joints)
    cmds.scale(factor, factor, factor, relative=True, objectScalePivot=True)
    # 物理数值恢复 1:1, 只保留变换结果
    cmds.makeIdentity(apply=True, jointOrient=False, scale=True, translate=False, rotate=False)
    print("[OK] 已缩放 ×%g" % factor)


def main():
    print("导入 USD: %s" % USD_FILE)
    if not load_usd_plugin():
        return
    if not import_usd(USD_FILE):
        return
    report()


main()
