#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
URDF -> Blender (armature + skinned meshes) -> USD, for the Unitree G1 (or any URDF robot).

Tested with Blender 4.2 LTS / 4.5 LTS / 5.0 (bpy 5.0.1). No ROS, no extra pip packages.

HOW TO USE
==========
A) GUI (recommended for inspection):
   1. Open Blender 5.x  ->  "Scripting" workspace tab
   2. Open this file, edit the CONFIG block below
   3. Click "Run Script" (▶).  Results appear in the 3D viewport / System console.
   (不会重置你当前打开的文件; 重复运行会自动清理上次生成的同名内容)

B) Command line, with the blender binary:
   blender --background --python blender_import_urdf.py -- ^
       "unitree_ros/robots/g1_description/g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf" ^
       --blend "out/g1.blend" --usd "out/g1.usda" --meta "out/g1.json" --render "out/g1.png"
   (注意 -- 分隔符不能少; 直接 `blender 脚本.py` 启动也能跑, 但推荐上面这种标准形式)

USD 输出 (默认一次导出三份, 按 DCC 各取所需, 无需再手工换单位/轴向):
   <name>.usda          Maya    — cm + Y-up, 面向 +Z (Maya 标准, 主文件名不变)
   <name>_houdini.usda  Houdini — m  + Y-up, 数据烘成 Y-up/SkelRoot 无变换
                          (USD Character Import 的 模型/骨骼/动画 三个输出方向全一致)
   <name>_ue.usda       UE      — cm + Z-up, 面向 +X (UE 的 Z-up/厘米/前向惯例, 免转换)
   (--usd-variants maya 可只导主文件; 后缀自动跟随 .usda/.usdc/.usdz 扩展名)

绑定姿势与摆位 (默认, 与官方一致):
   URDF 官方零位 (手臂自然下垂) + 双脚贴地 (脚底抬到世界 Z=0, pelvis 在 Z≈0.79 m;
   --no-ground 可退回 URDF 原生 pelvis 原点)。动捕重定向偏好 T-Pose 起手的话加
   --pose tpose (双肩 roll ±90° 外展, 偏移量记录在 meta JSON 的 pose.joint_offsets_deg)。

The generated USD contains a UsdSkel skeleton + skinned meshes (exported in
centimeters by default so it drops into Maya's default cm scene at true size).
A sidecar .json stores every joint's axis/limits for retargeting.
"""

import bpy
import sys
import os
import json
import math
import struct
import argparse
import re
import time
from mathutils import Matrix, Vector

import xml.etree.ElementTree as ET

# ----------------------------------------------------------------------------
# CONFIG - used when running from the Blender GUI (Scripting tab)
# ----------------------------------------------------------------------------
CONFIG = {
    "urdf":   r"D:\BlenderPro\G1\g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf",
    "blend":  r"",    # optional: save .blend next to the URDF if left empty
    "usd":    r"",    # optional: export USD next to the URDF if left empty
    "meta":   r"",    # optional: joint metadata .json path
    "render": r"",    # optional: preview .png path
    "scale":  1.0,    # overall scale (1.0 = meters, the URDF unit)
    "usd_units": "cm",  # 主 USD (Maya) 的单位: "cm" = Maya 友好 (默认), "m" = 物理米
    "usd_up": "Y",      # 主 USD (Maya) 的 up 轴: "Y" (Maya 标准) 或 "Z" (URDF 原生)
    "usd_variants": "maya,houdini,ue",  # 自动导出的 DCC 变体 (逗号分隔): maya / houdini / ue
    "skip_links": "force_sensor|imu|d435|mid360",  # regex; links to drop (noise)
    "face_maya": True,  # rotate rig so it faces +Z in Maya (Blender -Y forward)
    "pose": "zero",    # 绑定姿势: "zero" = URDF 官方零位 (默认, 手臂下垂),
                        #            "tpose" = 肩 roll ±90° 水平外展 (动捕重定向可选起手)
    "ground": True,     # 双脚贴地 (脚底抬到世界 Z=0; False = pelvis 在原点, URDF 原生)
    "auto_smooth": True,
    "auto_uv": True,          # STL 没有 UV, 自动 Smart UV Project (想自己贴图必须有 UV)
    "hik": True,              # 补 HIK (MotionBuilder/Maya HumanIK) 虚拟骨: 颈椎+双脚尖 (零权重)
    "nice_materials": True,   # 官方无贴图; 用白壳/深灰金属预设替代 URDF 的两个纯色
}

# ---------------------------------------------------------------------------
# GUI 安全报错退出
# 在 Blender GUI 里运行时, SystemExit 会把整个 Blender 直接关闭 (表现为"闪退"),
# 绝不能在 GUI 路径上用。GUI: 弹窗 + 控制台打印 + RuntimeError (由 Blender 捕获);
# 命令行 (带 '--' 参数): 才用 SystemExit。
_GUI = False


def _die(msg):
    print("[g1-rig] ERROR: " + msg)
    if _GUI:
        try:
            import bpy
            import textwrap

            def _draw(self, _ctx):
                for line in textwrap.wrap(msg, 56)[:12]:
                    self.layout.label(text=line)

            bpy.context.window_manager.popup_menu(_draw, title="g1-rig-pipeline", icon='ERROR')
        except Exception:
            pass
        raise RuntimeError(msg)      # GUI: 被文本编辑器捕获, 只报错不退出
    raise SystemExit(msg)            # 命令行: 正常非零退出


# ----------------------------------------------------------------------------
# URDF parsing
# ----------------------------------------------------------------------------
def _origin(el):
    """<origin xyz rpy> -> (4x4 Matrix, xyz, rpy).  URDF fixed-axis RPY: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    xyz = (0.0, 0.0, 0.0)
    rpy = (0.0, 0.0, 0.0)
    if el is not None:
        o = el.find("origin")
        if o is not None:
            if o.get("xyz"):
                xyz = tuple(float(v) for v in o.get("xyz").split())
            if o.get("rpy"):
                rpy = tuple(float(v) for v in o.get("rpy").split())
    r, p, y = rpy
    R = (Matrix.Rotation(y, 4, "Z") @ Matrix.Rotation(p, 4, "Y") @ Matrix.Rotation(r, 4, "X"))
    M = Matrix.Translation(Vector(xyz)) @ R
    return M, xyz, rpy


def _floats(s, default):
    if s is None:
        return tuple(default)
    return tuple(float(v) for v in s.split())


class Urdf:
    def __init__(self, path):
        self.path = os.path.abspath(path)
        self.dir = os.path.dirname(self.path)
        tree = ET.parse(self.path)
        self.root = tree.getroot()
        self.name = self.root.get("name", os.path.splitext(os.path.basename(self.path))[0])
        self.links = {l.get("name"): l for l in self.root.findall("link")}
        self.joints = list(self.root.findall("joint"))
        self.joint_by_child = {j.find("child").get("link"): j for j in self.joints}
        self.children = {n: [] for n in self.links}
        for j in self.joints:
            self.children[j.find("parent").get("link")].append(j)
        roots = [n for n in self.links if n not in self.joint_by_child]
        if not roots:
            raise ValueError("URDF has no root link (cyclic?)")
        self.root_link = roots[0]
        # world transform of every link at zero pose
        self.world = {}
        self._compute_world(self.root_link, Matrix.Identity(4))
        # global material definitions
        self.materials = {}
        for m in self.root.findall("material"):
            c = m.find("color")
            if c is not None:
                self.materials[m.get("name")] = _floats(c.get("rgba"), (0.8, 0.8, 0.8, 1.0))

    def _compute_world(self, link, parent_world):
        self.world[link] = parent_world
        for j in self.children.get(link, []):
            M, _, _ = _origin(j)
            self._compute_world(j.find("child").get("link"), parent_world @ M)

    def apply_tpose(self, angle_deg=90.0):
        """把双肩 roll 关节各转 ±90° (左 + / 右 -), 手臂水平外展成 T-Pose。

        直接改写 self.world (各 link 的零位世界变换) —— 之后骨骼/网格/蒙皮全部
        直接按 T-Pose 构建, 绑定姿势 (bind pose) 即 T-Pose, 无需任何事后 posing。
        返回 {骨名: 角度deg} 供 meta 记录 (机器人真值回放时做补偿用)。"""
        offsets = {}
        for side, sign in (("left", 1.0), ("right", -1.0)):
            link = "%s_shoulder_roll_link" % side
            j = self.joint_by_child.get(link)
            if link not in self.world or j is None or j.find("axis") is None:
                continue
            axis = (self.world[link].to_3x3()
                    @ Vector(_floats(j.find("axis").get("xyz"), (1, 0, 0)))).normalized()
            pivot = self.world[link].translation
            R = (Matrix.Translation(pivot)
                 @ Matrix.Rotation(sign * math.radians(angle_deg), 4, axis)
                 @ Matrix.Translation(-pivot))
            stack = [link]
            while stack:                      # 该关节子树 (上臂→手→手指) 全部跟随
                l = stack.pop()
                self.world[l] = R @ self.world[l]
                stack += [j2.find("child").get("link") for j2 in self.children.get(l, [])]
            offsets[link] = sign * angle_deg
        return offsets

    @staticmethod
    def joint_parent_link(j):
        return j.find("parent").get("link")

    @staticmethod
    def joint_child_link(j):
        return j.find("child").get("link")


def mesh_path(urdf, filename):
    """Resolve a URDF mesh filename ('meshes/x.STL' / 'package://pkg/...')."""
    fn = (filename or "").replace("\\", "/")
    if fn.startswith("package://"):
        fn = fn[len("package://"):].split("/", 1)[-1]
        print("  [warn] package:// path '%s' resolved relative to the URDF folder" % filename)
    if os.path.isabs(fn):
        return fn
    return os.path.normpath(os.path.join(urdf.dir, fn))


def stl_bbox_raw(path):
    """Read min/max of an STL (binary or ascii) without Blender. Returns (min,max) or None."""
    try:
        with open(path, "rb") as f:
            head = f.read(5)
            f.seek(0)
            verts = []
            if head == b"solid" and b"facet" in f.read(2048):
                f.seek(0)
                for line in f:
                    if line.startswith(b"vertex"):
                        verts.append([float(x) for x in line.split()[1:4]])
            else:
                f.seek(80)
                n = struct.unpack("<I", f.read(4))[0]
                for _ in range(n):
                    v = struct.unpack("<12f", f.read(50)[:48])
                    for k in range(3):
                        verts.append(v[3 + k * 3: 6 + k * 3])
        if not verts:
            return None
        cols = list(zip(*verts))
        return (min(cols[0]), min(cols[1]), min(cols[2])), (max(cols[0]), max(cols[1]), max(cols[2]))
    except Exception:
        return None


# ----------------------------------------------------------------------------
# Link selection
# ----------------------------------------------------------------------------
def has_visual(link_el):
    return any(v.find("geometry/mesh") is not None
               for v in link_el.findall("visual"))


def select_links(urdf, skip_re):
    """Keep links that carry visual meshes, plus every ancestor needed to keep the
    skeleton tree connected.  'skip_re' filters noisy links (sensors, logos...)."""
    skip = re.compile(skip_re) if skip_re else None
    keep = set()
    for name, l in urdf.links.items():
        if skip and skip.search(name):
            continue
        if has_visual(l):
            keep.add(name)
    # add ancestors so every kept link has a kept parent chain to the root
    changed = True
    while changed:
        changed = False
        for name in list(keep):
            if name == urdf.root_link:
                continue
            par = urdf.joint_parent_link(urdf.joint_by_child[name])
            if par not in keep:
                keep.add(par)
                changed = True
    # drop the URDF 'world' pseudo-root if it snuck in
    if "world" in keep and not has_visual(urdf.links.get("world", ET.Element("link"))):
        keep.discard("world")
    # set 的迭代顺序受 PYTHONHASHSEED 影响 -> 骨骼创建序/USD 关节序在两次运行间会不同;
    # 排序后保证可复现 (骨架拓扑与数据不变, 只是顺序确定)
    return sorted(keep)


def kept_subtree_size(urdf, name, keep):
    n = 0
    stack, seen = [name], set()
    while stack:
        cur = stack.pop()
        for j in urdf.children.get(cur, []):
            ch = urdf.joint_child_link(j)
            if ch in seen:
                continue
            seen.add(ch)
            if ch in keep:
                n += 1
            stack.append(ch)
    return n


def mesh_world_bbox(urdf, link_name):
    """World-space bbox (min, max) of a link's visual meshes, or None."""
    l = urdf.links[link_name]
    got = False
    mn = Vector((1e9, 1e9, 1e9))
    mx = Vector((-1e9, -1e9, -1e9))
    for vis in l.findall("visual"):
        m = vis.find("geometry/mesh") if vis.find("geometry") is not None else None
        if m is None:
            continue
        bb = stl_bbox_raw(mesh_path(urdf, m.get("filename")))
        if bb is None:
            continue
        Mo, _, _ = _origin(vis)
        W = urdf.world[link_name] @ Mo
        for x in (bb[0][0], bb[1][0]):
            for y in (bb[0][1], bb[1][1]):
                for z in (bb[0][2], bb[1][2]):
                    w = (W @ Vector((x, y, z, 1.0))).xyz
                    mn = Vector((min(mn.x, w.x), min(mn.y, w.y), min(mn.z, w.z)))
                    mx = Vector((max(mx.x, w.x), max(mx.y, w.y), max(mx.z, w.z)))
                    got = True
    return (mn, mx) if got else None


def bone_head_position(urdf, name, up):
    """骨骼 head 的世界位置 (URDF 系); up = 当前构建世界的"上"方向 (Z-up 构建=(0,0,1))。
    revolute/continuous/prismatic 一律用关节原点 (动画轴语义的唯一保证)。
    fixed 关节: 宇树部分 STL 是"全局装配坐标"风格 (head_link/logo_link 的
    关节原点被拉回骨盆), 原点落在 mesh bbox 之外 -> 改用几何锚点
    (bbox 沿"上"轴的底面中心, 即脖子根/零件根部), 让骨头回到零件上。fixed 无动画,
    挪动零风险; revolute 即使原点略偏也绝不能挪 (官方仿真轴就在那)。"""
    jpos = urdf.world[name].translation
    j = urdf.joint_by_child.get(name)
    jtype = j.get("type") if j is not None else "root"
    if jtype == "fixed":
        bb = mesh_world_bbox(urdf, name)
        if bb is not None:
            mn, mx = bb
            pad = 0.03
            outside = any(jpos[i] < mn[i] - pad or jpos[i] > mx[i] + pad for i in range(3))
            if outside:
                center = (mn + mx) * 0.5
                anchor = center - up * ((mx - mn) * 0.5).dot(up)   # 底面中心 (沿 up 轴)
                return anchor, True
    return jpos, False


def visual_center_world(urdf, link_name):
    """World-space center of a link's visual meshes (from raw STL headers)."""
    acc, n = Vector((0.0, 0.0, 0.0)), 0
    l = urdf.links[link_name]
    for vis in l.findall("visual"):
        m = vis.find("geometry/mesh") if vis.find("geometry") is not None else None
        if m is None:
            continue
        p = mesh_path(urdf, m.get("filename"))
        bb = stl_bbox_raw(p)
        if bb is None:
            continue
        Mo, _, _ = _origin(vis)
        W = urdf.world[link_name] @ Mo
        center = Vector(((bb[0][0] + bb[1][0]) / 2, (bb[0][1] + bb[1][1]) / 2, (bb[0][2] + bb[1][2]) / 2, 1.0))
        acc += (W @ center).xyz
        n += 1
    return (acc / n) if n else None


# ----------------------------------------------------------------------------
# Blender scene building
# ----------------------------------------------------------------------------
def cleanup_previous(robot_name):
    """GUI 模式下只清理本脚本上次运行生成的内容 (同名 collection / 骨架 / 网格),
    绝不动用户自己场景里的其他东西。"""
    victims = []
    coll = bpy.data.collections.get(robot_name)
    mesh_coll = bpy.data.collections.get(robot_name + "_meshes")
    for c in (coll, mesh_coll):
        if c:
            victims.extend(list(c.objects))
    arm = bpy.data.objects.get(robot_name + "_skeleton")
    if arm and arm not in victims:
        victims.append(arm)
    for o in victims:
        data = o.data
        try:
            bpy.data.objects.remove(o, do_unlink=True)
            if data is not None and data.users == 0:
                if isinstance(data, bpy.types.Mesh):
                    bpy.data.meshes.remove(data)
                elif isinstance(data, bpy.types.Armature):
                    bpy.data.armatures.remove(data)
        except Exception:
            pass
    for c in (coll, mesh_coll):
        if c:
            try:
                bpy.data.collections.remove(c)
            except Exception:
                pass


def fresh_scene(robot_name):
    if bpy.app.background:
        # 后台/批处理模式: 从空文件开始, 保证脚本可重复执行
        bpy.ops.wm.read_factory_settings(use_empty=True)
    else:
        # GUI 模式: 绝不重置用户当前打开的文件 (read_factory_settings 在 GUI /
        # 启动脚本阶段执行会破坏上下文, 导致后续 "Context missing active object"),
        # 只清理本脚本自己上次生成的同名内容
        cleanup_previous(robot_name)
    coll = bpy.data.collections.new(robot_name)
    bpy.context.scene.collection.children.link(coll)
    meshes_coll = bpy.data.collections.new(robot_name + "_meshes")
    coll.children.link(meshes_coll)
    return coll, meshes_coll


# "Nicer" shading presets keyed by URDF material name.
# 官方 URDF 只有两个纯色 (white 0.7 / dark 0.2) 且没有任何贴图;
# 这里把它们调成接近真机观感的材质 (哑光白壳 / 深灰金属), 纯着色器参数、不依赖贴图.
_NICE_MATERIALS = {
    "white": dict(base=(0.78, 0.79, 0.81), roughness=0.42, metallic=0.0),
    "dark":  dict(base=(0.085, 0.09, 0.10), roughness=0.30, metallic=0.85),
}


def _new_material(mat_name):
    """Create a node-based material (separate function so tests can simulate
    non-English Blender locales, where default node names are localized)."""
    mat = bpy.data.materials.new(mat_name)
    mat.use_nodes = True
    return mat


def urdf_material(urdf, vis_el, mesh_obj, nice=True):
    """Apply the <visual><material> color to a mesh (Principled BSDF).
    nice=True 时用 _NICE_MATERIALS 的白壳/深灰金属预设 (官方无贴图, 观感更接近真机);
    nice=False 时严格按 URDF 的 RGBA 纯色 + 默认粗糙度.

    Locale-proof: 节点按类型查找 (中文版 Blender 里节点名是 "原理化BSDF",
    按 "Principled BSDF" 字符串查找会静默失败); 并强制把所有节点名改写成
    ASCII —— 否则 USD 里会出现非 ASCII 的 Shader prim 名, Maya (Windows,
    GBK locale) 解析直接报 Ill-formed SdfPath 导致整个文件导入失败."""
    mat_el = vis_el.find("material")
    if mat_el is None:
        return
    rgba = None
    c = mat_el.find("color")
    if c is not None:
        rgba = _floats(c.get("rgba"), None)
    elif mat_el.get("name") in urdf.materials:
        rgba = urdf.materials[mat_el.get("name")]
    if not rgba:
        return
    mat_name = "urdf_" + (mat_el.get("name") or "mat")
    mat = bpy.data.materials.get(mat_name)
    if mat is None:
        mat = _new_material(mat_name)
        nt = mat.node_tree
        # 1) 全部节点名强制 ASCII (USD prim 名必须是 ASCII 标识符)
        for n in nt.nodes:
            if any(ord(ch) > 127 for ch in n.name):
                if n.type == "BSDF_PRINCIPLED":
                    n.name = "Principled_BSDF"
                elif n.type == "OUTPUT_MATERIAL":
                    n.name = "Material_Output"
                else:
                    n.name = "Node"
            n.label = n.name
        # 2) 按类型找 BSDF 节点 (不依赖界面语言)
        bsdf = next((n for n in nt.nodes if n.type == "BSDF_PRINCIPLED"), None)
        if bsdf is not None:
            bsdf.name = "Principled_BSDF"   # 确保英文名也统一成无空格 ASCII
            bsdf.label = "Principled_BSDF"
            preset = _NICE_MATERIALS.get(mat_el.get("name")) if nice else None
            if preset:
                bsdf.inputs["Base Color"].default_value = (*preset["base"], 1.0)
                bsdf.inputs["Roughness"].default_value = preset["roughness"]
                bsdf.inputs["Metallic"].default_value = preset["metallic"]
            else:
                bsdf.inputs["Base Color"].default_value = (rgba[0], rgba[1], rgba[2], 1.0)
                bsdf.inputs["Roughness"].default_value = 0.55
    if mesh_obj.data.materials:
        mesh_obj.data.materials[0] = mat
    else:
        mesh_obj.data.materials.append(mat)


def import_mesh_file(path):
    """Import an STL/OBJ mesh with raw coordinates (no axis conversion), return new objects."""
    ext = os.path.splitext(path)[1].lower()
    before = set(bpy.data.objects)
    if ext == ".stl":
        bpy.ops.wm.stl_import(filepath=path)
    elif ext == ".obj":
        bpy.ops.wm.obj_import(filepath=path)
    elif ext in (".dae",):
        print("  [warn] %s: Collada import was removed in Blender 4.5+/5.x - mesh skipped" % path)
        return []
    elif ext in (".usd", ".usda", ".usdc", ".usdz"):
        print("  [warn] %s: USD mesh references are not inlined - mesh skipped" % path)
        return []
    else:
        print("  [warn] unsupported mesh format: %s" % path)
        return []
    return [o for o in bpy.data.objects if o not in before]


def _activate_object(obj):
    """把 obj 设为上下文的活动对象, 并校验赋值真的生效。
    (在 GUI / `blender 脚本.py` 启动阶段, 直接赋值偶尔会静默失效,
    这是 mode_set 报 "Context missing active object" 的根源。)"""
    ctx = bpy.context
    try:
        ctx.view_layer.objects.active = obj
        if ctx.view_layer.objects.active == obj:
            return True
    except Exception:
        pass
    try:
        ctx.view_layer.update()          # 同步一次视图层再试
        ctx.view_layer.objects.active = obj
        if ctx.view_layer.objects.active == obj:
            return True
    except Exception:
        pass
    try:
        for vl in ctx.scene.view_layers:  # 逐个 view layer 都设一遍
            vl.objects.active = obj
        if ctx.view_layer.objects.active == obj:
            return True
    except Exception:
        pass
    return False


def _mode_set(obj, mode):
    """带上下文兜底的 mode_set; 失败时给出中文指引。"""
    if not _activate_object(obj):
        raise RuntimeError(
            "无法把骨架设为活动对象 (Blender 上下文异常)。\n"
            "  推荐用法 A: 打开 Blender 界面 -> 顶部 Scripting 标签 -> Open 打开本脚本 -> Run Script\n"
            "  推荐用法 B: blender --background --python blender_import_urdf.py -- <urdf路径> --usd 输出.usda"
        )
    try:
        for o in list(bpy.context.selected_objects):
            o.select_set(False)
    except Exception:
        pass
    try:
        obj.select_set(True)
    except Exception:
        pass
    try:
        bpy.ops.object.mode_set(mode=mode)
        return
    except RuntimeError:
        pass
    # 兜底: 显式上下文覆盖 (正常情况走不到这里)
    with bpy.context.temp_override(object=obj, active_object=obj, selected_objects=[obj]):
        try:
            bpy.ops.object.mode_set(mode=mode)
            return
        except RuntimeError:
            pass
    raise RuntimeError(
        "无法进入 %s 模式 (Context missing active object)。\n"
        "  推荐用法 A: 打开 Blender 界面 -> 顶部 Scripting 标签 -> Open 打开本脚本 -> Run Script\n"
        "  推荐用法 B: blender --background --python blender_import_urdf.py -- <urdf路径> --usd 输出.usda" % mode
    )


HIK_MAPPING = [
    # (HIK 槽位, G1 骨骼名) — 15 个必需节点 + 常用可选节点
    ("Reference", "pelvis"),
    ("Hips", "pelvis"),
    ("Spine", "waist_yaw_link"),
    ("Spine1", "waist_roll_link"),
    ("Spine2", "torso_link"),          # HIK 假设手臂连在最后一个 spine 节点 (正好是 torso)
    ("Neck", "neck_link"),             # 虚拟骨 (--hik 自动添加)
    ("Head", "head_link"),
    ("LeftArm", "left_shoulder_pitch_link"),
    ("LeftForeArm", "left_elbow_link"),
    ("LeftHand", "left_wrist_roll_link"),
    ("RightArm", "right_shoulder_pitch_link"),
    ("RightForeArm", "right_elbow_link"),
    ("RightHand", "right_wrist_roll_link"),
    ("LeftUpLeg", "left_hip_pitch_link"),
    ("LeftLeg", "left_knee_link"),
    ("LeftFoot", "left_ankle_pitch_link"),
    ("LeftToeBase", "left_toe_link"),  # 虚拟骨 (--hik 自动添加)
    ("RightUpLeg", "right_hip_pitch_link"),
    ("RightLeg", "right_knee_link"),
    ("RightFoot", "right_ankle_pitch_link"),
    ("RightToeBase", "right_toe_link"),
    # 手指 (可选; Inspire 灵巧手正好能填满)
    ("LeftHandThumb1", "L_thumb_proximal_base"),
    ("LeftHandThumb2", "L_thumb_proximal"),
    ("LeftHandThumb3", "L_thumb_intermediate"),
    ("LeftHandIndex1", "L_index_proximal"),
    ("LeftHandIndex2", "L_index_intermediate"),
    ("LeftHandMiddle1", "L_middle_proximal"),
    ("LeftHandMiddle2", "L_middle_intermediate"),
    ("LeftHandRing1", "L_ring_proximal"),
    ("LeftHandRing2", "L_ring_intermediate"),
    ("LeftHandPinky1", "L_pinky_proximal"),
    ("LeftHandPinky2", "L_pinky_intermediate"),
    ("RightHandThumb1", "R_thumb_proximal_base"),
    ("RightHandThumb2", "R_thumb_proximal"),
    ("RightHandThumb3", "R_thumb_intermediate"),
    ("RightHandIndex1", "R_index_proximal"),
    ("RightHandIndex2", "R_index_intermediate"),
    ("RightHandMiddle1", "R_middle_proximal"),
    ("RightHandMiddle2", "R_middle_intermediate"),
    ("RightHandRing1", "R_ring_proximal"),
    ("RightHandRing2", "R_ring_intermediate"),
    ("RightHandPinky1", "R_pinky_proximal"),
    ("RightHandPinky2", "R_pinky_intermediate"),
]


def add_hik_helper_bones(urdf, arm_data, scale, up, fwd):
    """为 MotionBuilder/Maya HumanIK 补 3 根零权重虚拟骨: 颈椎 + 双脚尖.
    G1 的 15 个 HIK 必需节点都有真实骨骼, 但 Neck 能让头颈重定向更平滑,
    ToeBase 能让 HIK 的脚部地板接触 (floor contact) / foot roll 生效.
    虚拟骨不绑任何网格顶点 -> 之后删除它们对模型零影响.
    up/fwd = 当前构建世界的"上/前"方向 (由根 link 的 world 矩阵推导, Z-up/Y-up 通用)."""
    eb = arm_data.edit_bones
    helpers = []
    # --- Neck: 插在 torso 与 head 之间 (head 骨头已按几何锚点放在脖子根) ---
    head_bone = eb.get("head_link")
    torso_bone = eb.get("torso_link")
    if head_bone is not None and torso_bone is not None:
        neck = eb.new("neck_link")
        p1 = head_bone.head.copy()                    # 脖子根 (几何锚点后的正确位置)
        d = (p1 - torso_bone.head)
        if d.length < 1e-6:
            d = up.copy()
        neck.tail = p1
        neck.head = p1 - d.normalized() * 0.07 * scale
        neck.parent = torso_bone
        neck.align_roll(up)
        head_bone.parent = neck
        helpers.append("neck_link")
    # --- Toes: 挂在 ankle_roll 下, 指向脚尖 (URDF 前进方向 = fwd) ---
    for side in ("left", "right"):
        ankle = eb.get(side + "_ankle_roll_link")
        if ankle is None:
            continue
        toe = eb.new(side + "_toe_link")
        down = -up
        toe.head = ankle.head + fwd * 0.06 + down * 0.02
        toe.tail = ankle.head + fwd * 0.16 + down * 0.02
        toe.parent = ankle
        toe.align_roll(up)
        helpers.append(side + "_toe_link")
    return helpers


def build_armature(urdf, keep, scale, robot_name, hik=True):
    arm_data = bpy.data.armatures.new(robot_name + "_skeleton")
    arm_obj = bpy.data.objects.new(robot_name + "_skeleton", arm_data)
    bpy.context.scene.collection.objects.link(arm_obj)
    arm_obj.show_in_front = True

    # 当前构建世界的"上/前"方向: 由根 link 的 world 矩阵推导。
    # 常规 Z-up 构建 = (0,0,1)/(1,0,0); Houdini 的 Y-up 烘焙构建 = (0,1,0)/(0,0,1)。
    Rw = urdf.world[urdf.root_link].to_3x3()
    up = (Rw @ Vector((0.0, 0.0, 1.0))).normalized()
    fwd = (Rw @ Vector((1.0, 0.0, 0.0))).normalized()

    # ---- bone heads -----------------------------------------------------------
    # 第一遍: 每根骨头的 head 位置 (revolute=关节原点; fixed+全局式STL=几何锚点)
    heads = {}
    geo_anchored = set()
    for name in keep:
        h, geo = bone_head_position(urdf, name, up)
        heads[name] = h * scale
        if geo:
            geo_anchored.add(name)
    if geo_anchored:
        print("  [fix] 几何锚点骨骼 (官方STL为全局坐标, 关节原点不在零件上): %s"
              % ", ".join(sorted(geo_anchored)))

    # ---- bone tails -----------------------------------------------------------
    tails = {}
    for name in keep:
        head = heads[name]
        kids = [j for j in urdf.children.get(name, []) if urdf.joint_child_link(j) in keep]
        if len(kids) == 1:
            tail = heads[urdf.joint_child_link(kids[0])]
        elif len(kids) == 0:
            tail = None
        else:
            # 多子: 优先"最向上"的子 (脊柱观感直), 再往自身几何中心微调
            kid_heads = [heads[urdf.joint_child_link(j)] for j in kids]
            main = max(kid_heads, key=lambda p: p.dot(up))
            vc = visual_center_world(urdf, name)
            tail = main.lerp(vc * scale, 0.25) if vc is not None else main
        min_len = 0.06 * scale
        if tail is None or (tail - head).length < 1e-6:
            tail = head + (urdf.world[name].to_3x3() @ Vector((0.0, 0.0, 0.1))) * scale
        elif (tail - head).length < min_len:
            tail = head + (tail - head).normalized() * min_len
        tails[name] = (head, tail)

    # ---- create bones --------------------------------------------------------
    _mode_set(arm_obj, "EDIT")
    eb = arm_data.edit_bones
    for name in keep:
        b = eb.new(name)
        b.head, b.tail = tails[name]
        # roll: bone local Z aligned with the link frame's Z axis (world space)
        b.align_roll((urdf.world[name].to_3x3() @ Vector((0.0, 0.0, 1.0))).normalized())
    for name in keep:
        if name == urdf.root_link:
            continue
        j = urdf.joint_by_child.get(name)
        par = urdf.joint_parent_link(j) if j is not None else None
        if par in eb:
            eb[name].parent = eb[par]
    hik_helpers = []
    if hik:
        hik_helpers = add_hik_helper_bones(urdf, arm_data, scale, up, fwd)
    _mode_set(arm_obj, "OBJECT")

    # ---- joint metadata as bone custom properties ----------------------------
    for name in keep:
        bone = arm_data.bones.get(name)
        j = urdf.joint_by_child.get(name)
        if bone is None or j is None:
            continue
        ax, lim = j.find("axis"), j.find("limit")
        bone["urdf_joint_name"] = j.get("name", "")
        bone["urdf_joint_type"] = j.get("type", "")
        if ax is not None:
            bone["urdf_axis"] = _floats(ax.get("xyz"), (0, 0, 1))
        if lim is not None:
            bone["urdf_limit_lower"] = float(lim.get("lower", "0"))
            bone["urdf_limit_upper"] = float(lim.get("upper", "0"))
            bone["urdf_limit_effort"] = float(lim.get("effort", "0"))
            bone["urdf_limit_velocity"] = float(lim.get("velocity", "0"))
    # HIK 虚拟骨标记 (导出/清理时按此识别)
    for hname in hik_helpers:
        bone = arm_data.bones.get(hname)
        if bone is not None:
            bone["hik_helper"] = True
    return arm_obj, arm_data, hik_helpers


def _ctx_override(**overrides):
    """temp_override() context manager (Blender 3.2+)."""
    return bpy.context.temp_override(**overrides)


def auto_uv(objects):
    """自动展 UV: STL 网格天生没有 UV, 不展的话在 Maya / Substance 里没法贴图.
    实现为"三面投影 (按面法线主方向分 6 个桶) + 自动图集": 每个零件占图集的一格,
    格内再按投影方向分 3x2 小格, 互不重叠; 全部零件统一纹素密度.
    (纯数据 API, 不依赖 bpy.ops, GUI / --background / bpy 模块结果一致)"""
    import numpy as np
    if not objects:
        return
    meshes = [o for o in objects if o.data and o.data.polygons]
    if not meshes:
        return
    # 统一纹素密度: 每个零件按其世界空间包围盒对角线归一化
    diags = {}
    for o in meshes:
        bb = [o.matrix_world @ Vector(c) for c in o.bound_box]
        diags[o.name] = max(max((bb[i] - bb[j]).length for i in range(8) for j in range(8)), 1e-6)
    n = len(meshes)
    cols = max(int(math.ceil(math.sqrt(n * 1.6))), 1)
    rows = max(int(math.ceil(n / cols)), 1)
    for idx, o in enumerate(meshes):
        me = o.data
        nverts = len(me.vertices)
        co = np.empty(nverts * 3, dtype=np.float64)
        me.vertices.foreach_get("co", co)
        co = co.reshape(-1, 3)
        npolys = len(me.polygons)
        pn = np.empty(npolys * 3, dtype=np.float64)
        me.polygons.foreach_get("normal", pn)
        pn = pn.reshape(-1, 3)
        totals = np.empty(npolys, dtype=np.int64)
        me.polygons.foreach_get("loop_total", totals)
        poly_of_loop = np.repeat(np.arange(npolys), totals)
        lvi = np.empty(len(me.loops), dtype=np.int64)
        me.loops.foreach_get("vertex_index", lvi)
        an = np.abs(pn)
        axis = np.argmax(an[poly_of_loop], axis=1)              # 0/1/2 主方向
        sign = pn[poly_of_loop][np.arange(len(poly_of_loop)), axis] < 0
        bucket = axis * 2 + sign.astype(int)                    # 0..5
        vco = co[lvi]                                           # 每个循环的顶点坐标
        # 三面投影 (带镜像修正, 让相对面纹理方向一致)
        uv = np.empty((len(lvi), 2), dtype=np.float64)
        for b in range(6):
            m = bucket == b
            if not m.any():
                continue
            x, y, z = vco[m, 0], vco[m, 1], vco[m, 2]
            if b == 0:   uv[m, 0], uv[m, 1] = -y, z
            elif b == 1: uv[m, 0], uv[m, 1] = y, z
            elif b == 2: uv[m, 0], uv[m, 1] = x, z
            elif b == 3: uv[m, 0], uv[m, 1] = -x, z
            elif b == 4: uv[m, 0], uv[m, 1] = x, -y
            else:        uv[m, 0], uv[m, 1] = x, y
        # 归一化到零件自己的图集格 (cell), 格内 3x2 子格按投影方向
        cell_w, cell_h = 1.0 / cols, 1.0 / rows
        cx, cy = idx % cols, idx // cols
        scale = 1.0 / diags[o.name]
        for b in range(6):
            m = bucket == b
            if not m.any():
                continue
            u, v = uv[m, 0] * scale, uv[m, 1] * scale
            u = u - u.min(); v = v - v.min()
            span_u, span_v = max(u.max(), 1e-9), max(v.max(), 1e-9)
            s = min((cell_w / 3.0 * 0.94) / span_u, (cell_h / 2.0 * 0.94) / span_v)
            uv[m, 0] = cx * cell_w + (b % 3) * (cell_w / 3.0) + u * s + cell_w * 0.003
            uv[m, 1] = cy * cell_h + (b // 3) * (cell_h / 2.0) + v * s + cell_h * 0.006
        if me.uv_layers.active is None:
            me.uv_layers.new(name="UVMap")
        me.uv_layers.active.data.foreach_set("uv", uv.reshape(-1))
        me.update()


def skin_meshes(urdf, keep, scale, arm_obj, meshes_coll, auto_smooth, nice_materials=True, do_uv=True):
    """Import each kept link's visual meshes and bind them rigidly (weight 1.0)
    to the link's bone -> a proper UsdSkel setup that survives the USD trip."""
    created = []
    for name in sorted(keep):
        l = urdf.links[name]
        for vi, vis in enumerate(l.findall("visual")):
            g = vis.find("geometry")
            m = g.find("mesh") if g is not None else None
            if m is None:
                continue
            p = mesh_path(urdf, m.get("filename"))
            if not os.path.isfile(p):
                print("  [warn] mesh not found: %s" % p)
                continue
            for o in import_mesh_file(p):
                Mo, _, _ = _origin(vis)
                W = urdf.world[name] @ Mo
                if scale != 1.0:
                    W = Matrix.Scale(scale, 4) @ W
                o.name = "%s_visual%d" % (name, vi)
                o.matrix_world = W
                # Parent to the armature. The mesh basis already holds the full
                # URDF transform, so matrix_parent_inverse must stay identity
                # (the armature object carries the Z-up -> Y-up rotation).
                o.parent = arm_obj
                # rigid skinning: 100% weight on this link's bone
                vg = o.vertex_groups.new(name=name)
                idx = list(range(len(o.data.vertices)))
                if idx:
                    vg.add(idx, 1.0, "REPLACE")
                mod = o.modifiers.new("Armature", "ARMATURE")
                mod.object = arm_obj
                urdf_material(urdf, vis, o, nice=nice_materials)
                for c in list(bpy.data.collections):
                    try:
                        c.objects.unlink(o)
                    except RuntimeError:
                        pass
                meshes_coll.objects.link(o)
                created.append(o)
    if do_uv and created:
        auto_uv(created)
    if auto_smooth and created:
        for o in created:
            o.select_set(True)
        try:
            with _ctx_override(selected_objects=created, active_object=created[0]):
                try:
                    bpy.ops.object.shade_auto_smooth(angle=math.radians(40))
                except TypeError:
                    bpy.ops.object.shade_auto_smooth(use_auto_smooth=True, angle=math.radians(40))
        except Exception as e:
            print("  [info] auto smooth skipped (%s)" % e)
        for o in created:
            o.select_set(False)
    return created


def apply_maya_facing(arm_obj):
    """Blender's world is Z-up (like URDF), so no up-axis rotation is needed in the scene.
    Rotate -90 deg about Z so the robot faces Blender's -Y (forward) -> after the USD
    Y-up conversion it faces +Z, the standard character-forward axis in Maya."""
    arm_obj.rotation_euler = (0.0, 0.0, -math.pi / 2)


def apply_ground_offset(arm_obj, meshes):
    """URDF 的原点在 pelvis —— 直接导入机器人会"站"在地下 (G1 脚底在 Z=-0.79m)。
    这里把整个骨架抬起, 让最低的网格点 (脚底/轮子) 正好落在世界 Z=0 (双足贴地站立)。
    返回抬升量 (米); pelvis 世界高度 = 抬升量。"""
    bpy.context.view_layer.update()
    zmin = None
    for o in meshes:
        mw = o.matrix_world
        for v in o.data.vertices:          # 逐顶点精确求最低点 (bound_box 在网格带旋转时会偏)
            z = (mw @ v.co).z
            if zmin is None or z < zmin:
                zmin = z
    if zmin is None:
        return 0.0
    off = -zmin
    arm_obj.location.z += off
    bpy.context.view_layer.update()
    return off


# ----------------------------------------------------------------------------
# USD export
# ----------------------------------------------------------------------------
def export_usd(path, units="cm", up_axis="Y", convert_orientation=None):
    meters_per_unit = 0.01 if units == "cm" else 1.0
    # Blender 是 Z-up; 常规 Y-up 导出靠导出器转换 (把 Z→Y 旋转放在 SkelRoot 变换上)。
    # Houdini 的 USD Character Import 不应用根变换 -> 该路径用 convert_orientation=False,
    # Y-up 旋转提前烘进数据本身 (见 export_houdini_usd)。
    if convert_orientation is None:
        convert_orientation = (up_axis == "Y")
    kwargs = dict(
        filepath=path,
        export_animation=False,
        export_meshes=True,
        export_materials=True,
        generate_preview_surface=True,
        generate_materialx_network=False,
        export_normals=True,
        export_uvmaps=True,
        export_armatures=True,
        only_deform_bones=False,
        export_shapekeys=False,
        export_hair=False,
        export_curves=False,
        export_points=False,
        export_volumes=False,
        export_lights=False,
        export_cameras=False,
        use_instancing=False,
        export_custom_properties=True,
        author_blender_name=True,
        convert_orientation=convert_orientation,
        export_global_forward_selection="NEGATIVE_Z",
        export_global_up_selection=up_axis,
        convert_scene_units="CUSTOM",
        meters_per_unit=meters_per_unit,
        triangulate_meshes=False,
    )
    valid = set(bpy.ops.wm.usd_export.get_rna_type().properties.keys())
    kwargs = {k: v for k, v in kwargs.items() if k in valid}
    bpy.ops.wm.usd_export(**kwargs)


# ----------------------------------------------------------------------------
# USD 变体: 一次导入自动导出多份, 各 DCC 免手工换单位/轴向
#   maya     主文件 (路径 = --usd), cm + Y-up, 面向 +Z —— 与旧版行为完全一致
#   houdini  _houdini 后缀, m + Y-up —— 数据直接烘成 Y-up、SkelRoot 无任何变换
#            (Houdini 的 USD Character Import 对 模型/骨骼 输出不应用根变换,
#             只有动画输出应用; 常规 Y-up 导出的根旋转会让前两者躺倒)
#   ue       _ue 后缀, cm + Z-up + URDF 原生朝向 (+X 前) —— 命中 UE 的 Z-up/厘米/前向惯例,
#            USD Stage 演员和 Content Browser 直接导入两条路都不需要任何转换
# ----------------------------------------------------------------------------
USD_VARIANTS = {
    "maya":    {"suffix": "",         "units": "cm", "up": "Y"},
    "houdini": {"suffix": "_houdini", "units": "m",  "up": "Y", "baked": True},
    "ue":      {"suffix": "_ue",      "units": "cm", "up": "Z"},
}


def usd_variant_path(usd_path, suffix):
    """g1.usda -> g1_houdini.usda (保留原扩展名; .usdc/.usdz 同样适用)"""
    if not suffix:
        return usd_path
    root, ext = os.path.splitext(usd_path)
    return root + suffix + ext


def plan_usd_variants(usd_path, cfg):
    """把 usd_variants 配置解析成导出计划: [{name, path, units, up, facing, baked}]"""
    if not usd_path:
        return []
    names = [v.strip().lower() for v in str(cfg.get("usd_variants", "maya,houdini,ue")).split(",")
             if v.strip()]
    bad = [v for v in names if v not in USD_VARIANTS]
    if bad:
        _die("未知 USD 变体: %s (可用: maya, houdini, ue)" % ", ".join(bad))
    face_maya = cfg.get("face_maya", True)
    plan = []
    for n in names:
        spec = USD_VARIANTS[n]
        if n == "maya":
            # 主文件: 单位/朝向仍跟随 --usd-units / --usd-up (兼容旧用法)
            path, units, up = usd_path, cfg["usd_units"], cfg.get("usd_up", "Y")
        else:
            path, units, up = usd_variant_path(usd_path, spec["suffix"]), spec["units"], spec["up"]
        if spec.get("baked"):
            # 烘焙式变体固定 .usda 文本格式: 导出后要修补 upAxis 元数据,
            # 而 Blender 自带 Python 没有 pxr 包, 只能文本替换 (二进制改不了)
            root, _ext = os.path.splitext(path)
            path = root + ".usda"
            plan.append({"name": n, "path": path, "units": units, "up": up,
                         "facing": "baked", "baked": True})
            continue
        # Y-up 变体用 Maya 朝向 (+Z 前); Z-up (UE) 变体保持 URDF 原生朝向 (+X 前)
        facing = "native" if (up == "Z" or not face_maya) else "maya"
        plan.append({"name": n, "path": path, "units": units, "up": up,
                     "facing": facing, "baked": False})
    return plan


def patch_usda_upaxis_y(path):
    """把 .usda 文本里的 upAxis = "Z" 修补为 "Y"。
    用于 Houdini 烘焙导出: 数据已经烘成 Y-up, 但 Blender 在 convert_orientation=False
    时固定写 upAxis="Z"。Blender 自带 Python 没有 pxr 包, 只能做文本替换。"""
    try:
        with open(path, "r", encoding="utf-8") as f:
            text = f.read()
        if 'upAxis = "Y"' in text[:2000]:
            return True
        new = text.replace('upAxis = "Z"', 'upAxis = "Y"', 1)
        if new == text:
            print("  [warn] %s: 未找到 upAxis 元数据, 请检查文件头" % path)
            return False
        with open(path, "w", encoding="utf-8") as f:
            f.write(new)
        return True
    except Exception as e:
        print("  [warn] upAxis 修补失败 (%s): %s" % (path, e))
        return False


def patch_usda_rest_to_bind(path):
    """把 .usda 里 Skeleton 的 restTransforms 数组替换为 bindTransforms 数组。

    Blender 的 USD 导出器把 restTransforms 写成了父级相对的局部变换, 而 USD 规范
    要求它与 bindTransforms 同为骨架空间绝对变换。Houdini 的 USD Character Import
    对 模型/骨骼 两个输出按规范消费 rest (动画输出用 bind) —— 局部值被当绝对值,
    骨架全部缩回原点附近 (表现为 pelvis 在原点 + 差 90° 朝向)。绑定姿势导出的
    文件 rest == bind 本就是应有状态, 这里直接对齐。仅文本替换, 无需 pxr。"""
    import re
    try:
        with open(path, "r", encoding="utf-8") as f:
            text = f.read()
        mb = re.search(r"^[ \t]*uniform matrix4d\[\] bindTransforms = (\[.*\])[ \t]*$", text, re.M)
        mr = re.search(r"^[ \t]*uniform matrix4d\[\] restTransforms = (\[.*\])[ \t]*$", text, re.M)
        if not mb or not mr:
            print("  [warn] %s: 未找到 bind/restTransforms 行, 跳过 rest=bind 补丁" % path)
            return False
        if mb.group(1) == mr.group(1):
            return True    # 本来就一致
        new = text[:mr.start(1)] + mb.group(1) + text[mr.end(1):]
        with open(path, "w", encoding="utf-8") as f:
            f.write(new)
        return True
    except Exception as e:
        print("  [warn] rest=bind 补丁失败 (%s): %s" % (path, e))
        return False


def patch_usda_add_animation(path, pose_name="zero"):
    """给 .usda 的 Skeleton 加动画绑定: SkelAnimation "bind_pose" prim + skel:animationSource 关系。

    Houdini 的 USD Character Import 没有 animationSource 时会警告
    "does not have an animation binding" (动画输出回退用 bind)。这里把绑定姿势
    本身写成一个静态 SkelAnimation (joints + 局部平移/旋转/缩放, 由 bindTransforms
    反推: local(i) = bind(i) * bind(parent)^-1), 并在 Skeleton 上挂
    rel skel:animationSource —— 输出3 变为数据驱动, 三输出信息完全一致。
    纯文本操作 + 自校验 (重建链路与 bind 比对, 误差大则放弃不写)。"""
    import re as _re
    try:
        with open(path, "r", encoding="utf-8") as f:
            text = f.read()
        mj = _re.search(r"^[ \t]*uniform token\[\] joints = (\[.*\])[ \t]*$", text, _re.M)
        mb = _re.search(r"^[ \t]*uniform matrix4d\[\] bindTransforms = (\[.*\])[ \t]*$", text, _re.M)
        msr = _re.search(r'^[ \t]*def SkelRoot "([^"]+)"', text, _re.M)
        if not (mj and mb and msr):
            print("  [warn] %s: 未找到 joints/bindTransforms/SkelRoot, 跳过动画绑定" % path)
            return False
        joints_text = mj.group(1)
        joints = [t.strip().strip('"') for t in joints_text.strip("[]").split(", ") if t.strip()]
        # 解析 bindTransforms 的每个 4x4 (pxr 行向量约定: 平移在最后一行)
        mats = []
        for m in _re.finditer(r"\(\s*\(\s*([^()]*?)\s*\)\s*,\s*\(\s*([^()]*?)\s*\)\s*,\s*\(\s*([^()]*?)\s*\)\s*,\s*\(\s*([^()]*?)\s*\)\s*\)", mb.group(1)):
            rows = [[float(v) for v in g.split(",")] for g in m.groups()]
            if all(len(r) == 4 for r in rows):
                mats.append(rows)
        if len(mats) != len(joints):
            print("  [warn] %s: bind 矩阵数 %d != 关节数 %d, 跳过动画绑定" % (path, len(mats), len(joints)))
            return False

        def mul(A, B):
            return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]

        def inv(M):
            n = 4
            A = [row[:] + [1.0 if i == j else 0.0 for j in range(n)] for i, row in enumerate(M)]
            for c in range(n):
                p = max(range(c, n), key=lambda r: abs(A[r][c]))
                if abs(A[p][c]) < 1e-12:
                    return None
                A[c], A[p] = A[p], A[c]
                pv = A[c][c]
                A[c] = [v / pv for v in A[c]]
                for r in range(n):
                    if r != c and A[r][c] != 0.0:
                        f = A[r][c]
                        A[r] = [a - f * b for a, b in zip(A[r], A[c])]
            return [row[n:] for row in A]

        def quat_from_mat3(m):
            # m: 列向量约定 3x3 (输入前先转置); 返回 (w, x, y, z)
            tr = m[0][0] + m[1][1] + m[2][2]
            if tr > 0:
                s = (tr + 1.0) ** 0.5 * 2
                return (0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s)
            if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
                s = (1.0 + m[0][0] - m[1][1] - m[2][2]) ** 0.5 * 2
                return ((m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s)
            if m[1][1] > m[2][2]:
                s = (1.0 + m[1][1] - m[0][0] - m[2][2]) ** 0.5 * 2
                return ((m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s)
            s = (1.0 + m[2][2] - m[0][0] - m[1][1]) ** 0.5 * 2
            return ((m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s)

        index = {p: i for i, p in enumerate(joints)}
        locs, quats, trans = [], [], []
        for i, p in enumerate(joints):
            parent = p.rsplit("/", 1)[0] if "/" in p else None
            if parent in index:
                Pinv = inv(mats[index[parent]])
                if Pinv is None:
                    print("  [warn] %s: 关节 %s 父矩阵不可逆, 跳过动画绑定" % (path, p))
                    return False
                L = mul(mats[i], Pinv)
            else:
                L = mats[i]
            locs.append(L)
            trans.append((L[3][0], L[3][1], L[3][2]))
            quats.append(quat_from_mat3([[L[0][0], L[1][0], L[2][0]],
                                         [L[0][1], L[1][1], L[2][1]],
                                         [L[0][2], L[1][2], L[2][2]]]))
        # 自校验: 局部链路重新累乘应还原 bind (平移误差 < 1mm)
        world = {}
        def wmat(i):
            if i in world:
                return world[i]
            p = joints[i].rsplit("/", 1)[0] if "/" in joints[i] else None
            W = locs[i] if p not in index else mul(locs[i], wmat(index[p]))   # 行向量: local * parent
            world[i] = W
            return W
        err = max(abs(wmat(i)[3][k] - mats[i][3][k]) for i in range(len(joints)) for k in range(3))
        if err > 1e-3:
            print("  [warn] %s: 动画自校验误差 %.6f 过大, 跳过动画绑定" % (path, err))
            return False

        skelroot = msr.group(1)
        anim_path = "/root/%s/bind_pose" % skelroot
        # 两个 UsdSkelAnimation 的硬性要求 (实测 pxr/Houdini 的 AnimQuery 否则拒绝求值):
        # 1) 属性必须写成 timeSamples —— 无采样的默认值会被忽略;
        # 2) scales 的 schema 类型是 half3[] 而非 float3[] —— 类型不符时整个动画无效。
        block = ("\n        def SkelAnimation \"bind_pose\"\n"
                 "        {\n"
                 "            uniform token[] joints = %s\n"
                 "            quatf[] rotations.timeSamples = {\n"
                 "                0: [%s],\n"
                 "            }\n"
                 "            half3[] scales.timeSamples = {\n"
                 "                0: [%s],\n"
                 "            }\n"
                 "            float3[] translations.timeSamples = {\n"
                 "                0: [%s],\n"
                 "            }\n"
                 "        }\n") % (
            joints_text,
            ", ".join("(%.9f, %.9f, %.9f, %.9f)" % q for q in quats),
            ", ".join("(1, 1, 1)" for _ in joints),
            ", ".join("(%.9f, %.9f, %.9f)" % t for t in trans))
        # 1) SkelAnimation 放进 SkelRoot (插到 SkelRoot 内第一个 def Xform 之前)
        mxf = _re.search(r"\n(        def Xform \")", text)
        if not mxf:
            print("  [warn] %s: 未找到插入点 (def Xform), 跳过动画绑定" % path)
            return False
        text = text[:mxf.start()] + "\n" + block + text[mxf.start():]
        # 2) Skeleton 上挂 animationSource 关系 (插到 joints 属性行之前)
        mjl = _re.search(r"(\n)([ \t]*)uniform token\[\] joints = \[", text)
        if not mjl:
            print("  [warn] %s: 未找到 joints 属性行, 跳过 animationSource" % path)
            return False
        text = text[:mjl.start()] + "\n%srel skel:animationSource = <%s>" % (mjl.group(2), anim_path) + text[mjl.start():]
        # 3) 版本标记 (用户端 Ctrl+F 自检; pose= 告诉你这份文件是哪个绑定姿势)
        text = text.replace("#usda 1.0\n",
                            "#usda 1.0\n# g1-rig-pipeline houdini variant v3: baked Y-up, SkelRoot identity,"
                            " rest=bind, animation binding, pose=%s\n" % pose_name, 1)
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        return True
    except Exception as e:
        print("  [warn] 动画绑定补丁失败 (%s): %s" % (path, e))
        return False


def export_houdini_usd(path, urdf, keep, cfg, lift):
    """Houdini 专用导出: 数据直接烘成 Y-up, SkelRoot 不带任何变换。

    背景: Blender 常规 Y-up 导出把 Z→Y 转换旋转放在 SkelRoot 的 xform 上,
    而 Houdini 的 USD Character Import 节点对 模型/骨骼 两个输出不应用根变换
    (只有动画输出应用) —— 结果模型和骨骼躺倒、动画却正常。
    做法: 把 Y-up 旋转 (含贴地抬升) 直接烘进 urdf.world 后重建场景再导出,
    根变换恒等, 三个输出全部一致。导出后把场景恢复成常规 Z-up (GUI 视口正常)。"""
    t0 = time.time()
    # (x,y,z)_Z-up -> (y,z,x)_Y-up: 上(+Z)->+Y, 前(+X)->+Z, 右手系保持
    R = Matrix(((0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0))).to_4x4()
    if lift:
        R = Matrix.Translation(Vector((0.0, lift, 0.0))) @ R
    saved = {l: m.copy() for l, m in urdf.world.items()}
    for l in urdf.world:
        urdf.world[l] = R @ urdf.world[l]

    _coll, meshes_coll = fresh_scene(urdf.name)
    arm_h, _arm_data, _hik = build_armature(urdf, keep, cfg["scale"], urdf.name,
                                            cfg.get("hik", True))
    skin_meshes(urdf, keep, cfg["scale"], arm_h, meshes_coll, cfg["auto_smooth"],
                cfg.get("nice_materials", True), cfg.get("auto_uv", True))
    # armature 对象保持恒等变换 (朝向已烘进 world, 不调 apply_maya_facing)
    export_usd(path, "m", "Y", convert_orientation=False)
    patch_usda_upaxis_y(path)
    patch_usda_rest_to_bind(path)
    patch_usda_add_animation(path, cfg.get("pose", "zero"))

    # 恢复常规 Z-up 场景 (GUI 用户看到的仍是标准结果; .blend 早已保存, 不受影响)
    urdf.world.clear()
    urdf.world.update(saved)
    _coll, meshes_coll = fresh_scene(urdf.name)
    arm_obj, _ad, _h = build_armature(urdf, keep, cfg["scale"], urdf.name,
                                      cfg.get("hik", True))
    meshes = skin_meshes(urdf, keep, cfg["scale"], arm_obj, meshes_coll, cfg["auto_smooth"],
                         cfg.get("nice_materials", True), cfg.get("auto_uv", True))
    if cfg.get("face_maya", True):
        apply_maya_facing(arm_obj)
    if cfg.get("ground", True):
        apply_ground_offset(arm_obj, meshes)
    bpy.context.view_layer.update()
    print("USD houdini: %s (units=m, up=Y, SkelRoot 恒等 + rest=bind + 动画绑定, 重建 %.0fs)"
          % (path, time.time() - t0))
    return arm_obj, meshes


def gui_popup(title, lines):
    """GUI 模式弹窗提示输出文件位置 (GUI 里 print 藏在系统控制台, 用户看不到).
    后台/批处理模式严禁调用 (headless 下会崩溃), 用 bpy.app.background 严格守卫."""
    if bpy.app.background:
        return
    try:
        ctx = bpy.context
        if not getattr(ctx, "window", None):
            return
        def draw(self, _ctx):
            for l in lines:
                self.layout.label(text=l)
        ctx.window_manager.popup_menu(draw, title=title, icon='INFO')
    except Exception:
        pass


# ----------------------------------------------------------------------------
# Metadata json (joint axes / limits, for retargeting & robotics)
# ----------------------------------------------------------------------------
def _root_bind16(arm_data, root_name):
    """根骨的绑定矩阵 (armature 空间, 行主序行向量)。isaac/ 导出脚本用它把
    骨骼系根位姿换算回 URDF link 系 (绑定时 link 系 = 单位阵)。"""
    try:
        b = arm_data.bones.get(root_name)
        if b is None:
            return None
        M = b.matrix_local.transposed()
        return [round(float(v), 9) for row in M for v in row]
    except Exception:
        return None


def write_meta(urdf, keep, path, usd_units, hik_helpers=None, usd_files=None,
               pose_name="zero", pose_offsets=None, ground=None, arm_data=None):
    if usd_files:
        units_str = "URDF native: meters, Z-up.  USD files: " + "; ".join(
            "%s=%s,%s-up" % (v["name"], v["units"], v["up"]) for v in usd_files)
    else:
        units_str = "URDF native: meters, Z-up.  USD export: %s, Y-up." % (
            "centimeters" if usd_units == "cm" else "meters")
    data = {
        "robot": urdf.name,
        "source_urdf": os.path.basename(urdf.path),
        "generator": "g1-rig-pipeline / blender_import_urdf.py",
        "units": units_str,
        "root_link": urdf.root_link,
        "root_bind16": (_root_bind16(arm_data, urdf.root_link) if arm_data is not None else None),
        "num_links_total": len(urdf.links),
        "num_joints_total": len(urdf.joints),
        "num_joints_movable": sum(1 for j in urdf.joints
                                  if j.get("type") in ("revolute", "continuous", "prismatic")),
        "bones": sorted(keep),
        "joints": [],
    }
    if usd_files:
        data["usd_files"] = {
            v["name"]: {
                "file": os.path.basename(v["path"]),
                "units": v["units"],
                "up": v["up"],
                "facing": "+X (URDF/UE 原生)" if v["facing"] == "native" else "+Z (DCC 前向)",
                "target_dcc": {"maya": "Maya", "houdini": "Houdini", "ue": "Unreal Engine"}[v["name"]],
            }
            for v in usd_files
        }
    data["pose"] = {
        "name": pose_name,
        "description": ("T-Pose: 双肩 roll ±90° 水平外展 (动捕重定向标准起手)"
                        if pose_name == "tpose" else "URDF 官方零位 (手臂自然下垂)"),
        "joint_offsets_deg": dict(pose_offsets or {}),
        "note": ("绑定姿势即 T-Pose。机器人真值回放补偿: urdf关节角 = 骨骼局部旋转 + 上述偏移"
                 if pose_offsets else "骨骼局部旋转 = urdf 关节角 (无偏移)"),
    }
    if ground is not None:
        data["ground"] = ground
    if hik_helpers:
        valid = set(keep) | set(hik_helpers)
        data["hik"] = {
            "note": "MotionBuilder/Maya HumanIK 骨骼映射表; helpers 为零权重虚拟骨, 可随时删除 (骨骼上 hik_helper=True)",
            "helpers": sorted(hik_helpers),
            "required_15_filled": True,
            "mapping": {slot: bone for slot, bone in HIK_MAPPING if bone in valid},
        }
    for name in sorted(keep):
        j = urdf.joint_by_child.get(name)
        if j is None:
            continue
        M, xyz, rpy = _origin(j)
        entry = {
            "bone": name,
            "joint": j.get("name"),
            "type": j.get("type"),
            "parent_bone": urdf.joint_parent_link(j),
            "origin_xyz_m": [round(v, 6) for v in xyz],
            "origin_rpy_rad": [round(v, 6) for v in rpy],
        }
        ax = j.find("axis")
        if ax is not None:
            entry["axis_in_child_frame"] = [round(v, 6) for v in _floats(ax.get("xyz"), (0, 0, 1))]
            # 绑定姿势下关节轴的世界方向 (Z-up, URDF 原生). 消费端置换 (x,y,z)->(y,z,x)
            # 即得 Y-up. 用于 DOF 驱动/提取 (isaac/ 导出脚本).
            awv = None
            try:
                aw = urdf.world.get(name)
                if aw is not None:
                    awv = (aw.to_3x3()
                           @ Vector(_floats(ax.get("xyz"), (0, 0, 1)))).normalized()
                    entry["axis_world_at_bind"] = [round(v, 6) for v in awv]
            except Exception:
                pass
            # Isaac 导出脚本 (isaac/) 用的两份数据. FBX/USD 往返都保留局部矩阵,
            # 以下数值在 Blender/Maya/MotionBuilder 三个软件里通用:
            #   bind_local16      绑定局部矩阵, 行主序 16 元组, 行向量约定, 相对父骨
            #   axis_parent_local 关节轴在父骨绑定坐标系下的单位向量
            try:
                if arm_data is not None and awv is not None:
                    b = arm_data.bones.get(name)
                    par_b = arm_data.bones.get(entry["parent_bone"])
                    if b is not None and par_b is not None:
                        L = (par_b.matrix_local.inverted() @ b.matrix_local).transposed()
                        entry["bind_local16"] = [round(float(v), 9)
                                                 for row in L for v in row]
                        a3 = (par_b.matrix_local.to_3x3().inverted() @ awv)
                        entry["axis_parent_local"] = [round(float(v), 9) for v in a3]
            except Exception:
                pass
        entry["bind_offset_deg"] = float((pose_offsets or {}).get(name, 0.0))
        lim = j.find("limit")
        if lim is not None:
            entry["limits"] = {
                "lower_rad": float(lim.get("lower", 0)),
                "upper_rad": float(lim.get("upper", 0)),
                "effort_nm": float(lim.get("effort", 0)),
                "velocity_rad_s": float(lim.get("velocity", 0)),
            }
        data["joints"].append(entry)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


# ----------------------------------------------------------------------------
# Preview render (Cycles CPU - safe in headless mode)
# ----------------------------------------------------------------------------
def render_preview(path, arm_obj, scale):
    scene = bpy.context.scene
    try:
        scene.render.engine = "CYCLES"
        scene.cycles.device = "CPU"
        scene.cycles.samples = 16
    except Exception:
        pass
    scene.render.resolution_x = 1000
    scene.render.resolution_y = 1300
    scene.render.image_settings.file_format = "PNG"
    scene.render.filepath = path
    h = 1.4 * scale                       # robot height, approx (Z is up in Blender)
    cam_data = bpy.data.cameras.new("preview_cam")
    cam = bpy.data.objects.new("preview_cam", cam_data)
    scene.collection.objects.link(cam)
    scene.camera = cam
    cam.location = (1.6 * h, -1.9 * h, 0.85 * h)    # 3/4 front view (robot faces -Y)
    cam.rotation_euler = (Vector((0, 0, 0.6 * h)) - cam.location).to_track_quat("-Z", "Y").to_euler()
    cam_data.lens = 45
    sun_data = bpy.data.lights.new("preview_sun", "SUN")
    sun_data.energy = 3.0
    sun = bpy.data.objects.new("preview_sun", sun_data)
    sun.rotation_euler = (math.radians(55), math.radians(-10), math.radians(25))
    scene.collection.objects.link(sun)
    # background/bpy-module sessions do not auto-flush transforms: force an update
    # so the camera/light matrices are current before rendering
    bpy.context.view_layer.update()
    bpy.context.evaluated_depsgraph_get()
    bpy.ops.render.render(write_still=True)


# ----------------------------------------------------------------------------
# CLI plumbing
# ----------------------------------------------------------------------------
def get_args():
    """Args after '--' when launched via `blender --python x.py -- ...`,
    or the normal argv when run with pip's bpy module."""
    argv = sys.argv
    if "--" in argv:
        argv = argv[argv.index("--") + 1:]
        if not argv:
            return None
    else:
        base = os.path.basename(argv[0]).lower()
        if base in ("blender", "blender.exe", "blender-bin", "blender-launcher", "blender-launcher.exe"):
            return None  # launched from the Blender GUI (no '--' args): use CONFIG
        if len(argv) < 2:
            return None
        argv = argv[1:]
    p = argparse.ArgumentParser(description="URDF -> Blender -> USD (g1-rig-pipeline)")
    p.add_argument("urdf")
    p.add_argument("--blend")
    p.add_argument("--usd")
    p.add_argument("--meta")
    p.add_argument("--render")
    p.add_argument("--scale", type=float, default=1.0)
    p.add_argument("--usd-units", choices=["cm", "m"], default="cm")
    p.add_argument("--usd-up", choices=["Y", "Z"], default="Y")
    p.add_argument("--usd-variants", default="maya,houdini,ue",
                   help="逗号分隔的自动导出变体: maya,houdini,ue (默认全导; 只导主文件用 maya)")
    p.add_argument("--skip-links", default="force_sensor|imu|d435|mid360")
    p.add_argument("--keep-urdf-orientation", action="store_true",
                   help="do not rotate the rig to face Maya's +Z (keeps URDF +X facing)")
    p.add_argument("--pose", choices=["tpose", "zero"], default="zero",
                   help="绑定姿势: zero = URDF 官方零位 (默认, 手臂下垂); tpose = T-Pose 肩外展 (动捕重定向可选)")
    p.add_argument("--no-ground", action="store_true",
                   help="不抬到脚底贴地 (保持 pelvis 在世界原点, URDF 原生行为)")
    p.add_argument("--no-smooth", action="store_true")
    p.add_argument("--no-uv", action="store_true", help="跳过自动展 UV (STL 默认无 UV)")
    p.add_argument("--no-hik", action="store_true",
                   help="不加 HIK 虚拟骨 (颈椎/脚尖; MotionBuilder 重定向用, 零权重可随时删)")
    p.add_argument("--flat-colors", action="store_true",
                   help="严格用 URDF 的两个纯色 (0.7 白 / 0.2 深灰), 不用美化材质预设")
    return p.parse_args(argv)


def main():
    global _GUI
    args = get_args()
    _GUI = args is None   # True = 从 Blender GUI 运行 (此时禁止 SystemExit)
    print("g1-rig-pipeline blender_import_urdf.py  (houdini usd variant v3; pose=%s)"
          % getattr(args, "pose", "?"))
    if args:
        cfg = {
            "urdf": args.urdf, "blend": args.blend or "", "usd": args.usd or "",
            "meta": args.meta or "", "render": args.render or "", "scale": args.scale,
            "usd_units": args.usd_units, "usd_up": args.usd_up, "skip_links": args.skip_links,
            "usd_variants": args.usd_variants,
            "face_maya": not args.keep_urdf_orientation, "auto_smooth": not args.no_smooth,
            "pose": args.pose, "ground": not args.no_ground,
            "auto_uv": not args.no_uv, "nice_materials": not args.flat_colors,
            "hik": not args.no_hik,
        }
    else:
        cfg = dict(CONFIG)

    t0 = time.time()
    urdf_path = os.path.abspath(cfg["urdf"])
    if not os.path.isfile(urdf_path):
        # 兼容旧目录布局: D:\...\G1\<urdf> 不在时, 试 D:\...\G1\unitree_ros\robots\g1_description\<urdf>
        cand = os.path.join(os.path.dirname(urdf_path), "unitree_ros", "robots",
                            "g1_description", os.path.basename(urdf_path))
        if os.path.isfile(cand):
            print("[g1-rig] note: %s 不存在, 改用旧布局路径 %s" % (urdf_path, cand))
            urdf_path = cand
        else:
            _die("URDF not found: %s\n(也试过旧布局: %s)\n"
                 "请在脚本头部 CONFIG['urdf'] 填入实际 URDF 路径" % (urdf_path, cand))
    out_dir = os.path.dirname(urdf_path)
    base = os.path.splitext(os.path.basename(urdf_path))[0]
    blend_path = cfg["blend"] or os.path.join(out_dir, base + ".blend")
    usd_path = cfg["usd"] or os.path.join(out_dir, base + ".usda")
    meta_path = cfg["meta"] or os.path.join(out_dir, base + "_skeleton_meta.json")
    render_path = cfg["render"]

    print("=" * 72)
    print("URDF   :", urdf_path)
    print("Blender:", bpy.app.version_string)

    urdf = Urdf(urdf_path)
    pose_offsets = {}
    if cfg.get("pose", "zero") == "tpose":
        pose_offsets = urdf.apply_tpose()
        if pose_offsets:
            print("Pose   : T-Pose (肩 roll 外展: %s)"
                  % ", ".join("%s%+.0f°" % (k.replace("_shoulder_roll_link", ""), v)
                              for k, v in sorted(pose_offsets.items())))
        else:
            print("Pose   : [提示] 未找到 shoulder_roll 关节, 保持 URDF 零位")
    else:
        print("Pose   : URDF 官方零位 (手臂下垂; 动捕偏好 T-Pose 起手可用 --pose tpose)")
    keep = select_links(urdf, cfg["skip_links"])
    n_movable = sum(1 for j in urdf.joints if j.get("type") in ("revolute", "continuous", "prismatic"))
    print("Robot  : %s | links %d | joints %d (movable %d) | skeleton bones %d"
          % (urdf.name, len(urdf.links), len(urdf.joints), n_movable, len(keep)))
    if urdf.name != base:
        print("  [note] robot name from URDF: %r" % urdf.name)

    _coll, meshes_coll = fresh_scene(urdf.name)
    arm_obj, arm_data, hik_helpers = build_armature(urdf, keep, cfg["scale"], urdf.name,
                                                    cfg.get("hik", True))
    meshes = skin_meshes(urdf, keep, cfg["scale"], arm_obj, meshes_coll, cfg["auto_smooth"],
                         cfg.get("nice_materials", True), cfg.get("auto_uv", True))
    if cfg.get("face_maya", True):
        apply_maya_facing(arm_obj)
    ground_info = None
    ground_lift = 0.0
    if cfg.get("ground", True):
        off = apply_ground_offset(arm_obj, meshes)
        ground_lift = off
        ground_info = {
            "feet_at_world_z": 0.0,
            "pelvis_height_m": round(off, 4),
            "lift_offset_m": round(off, 4),
            "note": "脚底(或轮子)贴世界 Z=0; pelvis 抬到 Z=%.3f m (URDF 原点在 pelvis, 原生脚底在 -0.79m)" % off,
        }
        print("Ground : 脚底贴地 (骨架抬升 %.3f m, pelvis 在 Z=%.3f m)" % (off, off))
    print("Built  : %d bones, %d mesh objects" % (len(arm_data.bones), len(meshes)))
    if hik_helpers:
        print(" HIK   : + %d helper bones (%s) - 零权重, 重定向用, 可删"
              % (len(hik_helpers), ", ".join(hik_helpers)))
    n_uv = sum(1 for o in meshes if o.data.uv_layers.active is not None)
    n_mat = len({o.data.materials[0].name for o in meshes if o.data.materials})
    print(" Mats  : %d materials (URDF 无贴图, %s) | UV: %d/%d meshes unwrapped"
          % (n_mat,
             "美化预设" if cfg.get("nice_materials", True) else "URDF 纯色",
             n_uv, len(meshes)))

    usd_plan = plan_usd_variants(usd_path, cfg)

    if meta_path:
        write_meta(urdf, keep, meta_path, cfg["usd_units"], hik_helpers, usd_files=usd_plan,
                   pose_name=cfg.get("pose", "zero"), pose_offsets=pose_offsets,
                   ground=ground_info, arm_data=arm_data)
        print("Meta   :", meta_path)
    if blend_path:
        if cfg["blend"] or bpy.app.background:
            # 显式指定了路径, 或后台批处理模式 -> 自动保存
            bpy.ops.wm.save_as_mainfile(filepath=blend_path)
            print("Blend  :", blend_path)
        else:
            # GUI 模式且未显式指定: 不往 URDF 所在目录偷偷写 12MB 文件
            print("[提示] GUI 模式默认不自动保存 .blend; 如需保存请在 CONFIG['blend'] 填路径,")
            print("       或在 Blender 里 File > Save As 手动保存 (默认建议路径: %s)" % blend_path)
    for v in usd_plan:
        if v.get("baked"):
            continue    # Houdini 烘焙式变体在渲染预览之后单独导出 (要重建场景)
        if v["facing"] == "native":
            arm_obj.rotation_euler = (0.0, 0.0, 0.0)   # URDF 原生: Z-up, +X 前 (UE 惯例)
        else:
            apply_maya_facing(arm_obj)                 # Blender -Y 前 -> USD +Z (Maya/Houdini)
        export_usd(v["path"], v["units"], v["up"])
        note = ", URDF 原生朝向 +X 前" if v["facing"] == "native" else ""
        print("USD %-8s: %s (units=%s, up=%s%s)" % (v["name"], v["path"], v["units"], v["up"], note))
    if usd_plan and cfg.get("face_maya", True):
        apply_maya_facing(arm_obj)   # 恢复场景朝向 (blend 已保存, 不影响已导出的文件)
    if render_path:
        render_preview(render_path, arm_obj, cfg["scale"])
        print("Render :", render_path)
    for v in usd_plan:               # Houdini 烘焙式导出 (重建 Y-up 场景, 导出后恢复 Z-up)
        if v.get("baked"):
            export_houdini_usd(v["path"], urdf, keep, cfg, ground_lift)

    print("Done in %.1fs" % (time.time() - t0))
    print("=" * 72)

    # GUI 模式: 弹窗告诉用户文件在哪 (print 只进系统控制台, 容易看不到)
    _usd_label = {"maya": "USD Maya (cm,Y-up)", "houdini": "USD Houdini (m,Y-up)",
                  "ue": "USD UE (cm,Z-up)"}
    popup_lines = [(_usd_label[v["name"]] + ": %s") % v["path"] for v in usd_plan]
    popup_lines += [
        "JSON (关节元数据): %s" % meta_path,
        "Blend (可选): %s" % (blend_path if (cfg["blend"] or bpy.app.background) else "未自动保存 (File > Save As 手动保存)"),
        "",
        "绑定姿势: %s | 摆位: %s" % (
            "T-Pose (重定向标准)" if cfg.get("pose") == "tpose" else "URDF 零位",
            "双脚贴地 (pelvis Z≈0.79m)" if cfg.get("ground", True) else "pelvis 在原点 (URDF 原生)"),
        "Maya: File > Import 选 USD / maya_import_g1.py",
        "Houdini/UE: 直接用对应后缀的 _houdini / _ue 文件, 免换单位轴向",
    ]
    gui_popup("G1 导入完成 - 输出文件位置", popup_lines)


if __name__ == "__main__":
    main()
