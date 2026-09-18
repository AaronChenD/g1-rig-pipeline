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
    "urdf":   r"D:\BlenderPro\G1\unitree_ros\robots\g1_description\g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf",
    "blend":  r"",    # optional: save .blend next to the URDF if left empty
    "usd":    r"",    # optional: export USD next to the URDF if left empty
    "meta":   r"",    # optional: joint metadata .json path
    "render": r"",    # optional: preview .png path
    "scale":  1.0,    # overall scale (1.0 = meters, the URDF unit)
    "usd_units": "cm",  # "cm" = Maya-friendly (default), "m" = physical meters
    "usd_up": "Y",      # USD up axis: "Y" (Maya standard) or "Z" (keep URDF native)
    "skip_links": "force_sensor|imu|d435|mid360",  # regex; links to drop (noise)
    "face_maya": True,  # rotate rig so it faces +Z in Maya (Blender -Y forward)
    "auto_smooth": True,
    "auto_uv": True,          # STL 没有 UV, 自动 Smart UV Project (想自己贴图必须有 UV)
    "hik": True,              # 补 HIK (MotionBuilder/Maya HumanIK) 虚拟骨: 颈椎+双脚尖 (零权重)
    "nice_materials": True,   # 官方无贴图; 用白壳/深灰金属预设替代 URDF 的两个纯色
}

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
    return keep


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


def add_hik_helper_bones(urdf, arm_data, scale):
    """为 MotionBuilder/Maya HumanIK 补 3 根零权重虚拟骨: 颈椎 + 双脚尖.
    G1 的 15 个 HIK 必需节点都有真实骨骼, 但 Neck 能让头颈重定向更平滑,
    ToeBase 能让 HIK 的脚部地板接触 (floor contact) / foot roll 生效.
    虚拟骨不绑任何网格顶点 -> 之后删除它们对模型零影响."""
    eb = arm_data.edit_bones
    helpers = []
    # --- Neck: 插在 torso_link 与 head_link 之间 ---
    head_bone = eb.get("head_link")
    torso_bone = eb.get("torso_link")
    if head_bone is not None and torso_bone is not None:
        neck = eb.new("neck_link")
        p0, p1 = torso_bone.head.copy(), head_bone.head.copy()
        neck.head = p0.lerp(p1, 0.55)
        neck.tail = p1
        neck.parent = torso_bone
        neck.align_roll(Vector((0.0, 0.0, 1.0)))
        head_bone.parent = neck
        helpers.append("neck_link")
    # --- Toes: 挂在 ankle_roll 下, 指向脚尖 (URDF 前进方向 = +X) ---
    for side in ("left", "right"):
        ankle = eb.get(side + "_ankle_roll_link")
        if ankle is None:
            continue
        toe = eb.new(side + "_toe_link")
        fwd = Vector((1.0, 0.0, 0.0)) * scale
        down = Vector((0.0, 0.0, -1.0)) * scale
        toe.head = ankle.head + fwd * 0.06 + down * 0.02
        toe.tail = ankle.head + fwd * 0.16 + down * 0.02
        toe.parent = ankle
        toe.align_roll(Vector((0.0, 0.0, 1.0)))
        helpers.append(side + "_toe_link")
    return helpers


def build_armature(urdf, keep, scale, robot_name, hik=True):
    arm_data = bpy.data.armatures.new(robot_name + "_skeleton")
    arm_obj = bpy.data.objects.new(robot_name + "_skeleton", arm_data)
    bpy.context.scene.collection.objects.link(arm_obj)
    arm_obj.show_in_front = True

    # ---- bone heads / tails -------------------------------------------------
    tails = {}
    for name in keep:
        head = urdf.world[name].translation
        kids = [j for j in urdf.children.get(name, []) if urdf.joint_child_link(j) in keep]
        if len(kids) == 1:
            tail = urdf.world[urdf.joint_child_link(kids[0])].translation
        elif len(kids) == 0:
            tail = None
        else:
            # multi-child: point toward the child with the biggest kept subtree,
            # blended toward the link's own visual mass (keeps the spine connected)
            kids_sorted = sorted(kids, key=lambda j: -kept_subtree_size(urdf, urdf.joint_child_link(j), keep))
            main = urdf.world[urdf.joint_child_link(kids_sorted[0])].translation
            vc = visual_center_world(urdf, name)
            tail = main.lerp(vc, 0.35) if vc is not None else main
        head = head * scale
        tail = tail * scale if tail is not None else None
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
        hik_helpers = add_hik_helper_bones(urdf, arm_data, scale)
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


# ----------------------------------------------------------------------------
# USD export
# ----------------------------------------------------------------------------
def export_usd(path, units="cm", up_axis="Y"):
    meters_per_unit = 0.01 if units == "cm" else 1.0
    convert_orientation = (up_axis == "Y")   # Blender is Z-up; USD/Maya default is Y-up
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
def write_meta(urdf, keep, path, usd_units, hik_helpers=None):
    data = {
        "robot": urdf.name,
        "source_urdf": os.path.basename(urdf.path),
        "generator": "g1-rig-pipeline / blender_import_urdf.py",
        "units": "URDF native: meters, Z-up.  USD export: %s, Y-up." % ("centimeters" if usd_units == "cm" else "meters"),
        "root_link": urdf.root_link,
        "num_links_total": len(urdf.links),
        "num_joints_total": len(urdf.joints),
        "num_joints_movable": sum(1 for j in urdf.joints
                                  if j.get("type") in ("revolute", "continuous", "prismatic")),
        "bones": sorted(keep),
        "joints": [],
    }
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
    p.add_argument("--skip-links", default="force_sensor|imu|d435|mid360")
    p.add_argument("--keep-urdf-orientation", action="store_true",
                   help="do not rotate the rig to face Maya's +Z (keeps URDF +X facing)")
    p.add_argument("--no-smooth", action="store_true")
    p.add_argument("--no-uv", action="store_true", help="跳过自动展 UV (STL 默认无 UV)")
    p.add_argument("--no-hik", action="store_true",
                   help="不加 HIK 虚拟骨 (颈椎/脚尖; MotionBuilder 重定向用, 零权重可随时删)")
    p.add_argument("--flat-colors", action="store_true",
                   help="严格用 URDF 的两个纯色 (0.7 白 / 0.2 深灰), 不用美化材质预设")
    return p.parse_args(argv)


def main():
    args = get_args()
    if args:
        cfg = {
            "urdf": args.urdf, "blend": args.blend or "", "usd": args.usd or "",
            "meta": args.meta or "", "render": args.render or "", "scale": args.scale,
            "usd_units": args.usd_units, "usd_up": args.usd_up, "skip_links": args.skip_links,
            "face_maya": not args.keep_urdf_orientation, "auto_smooth": not args.no_smooth,
            "auto_uv": not args.no_uv, "nice_materials": not args.flat_colors,
            "hik": not args.no_hik,
        }
    else:
        cfg = dict(CONFIG)

    t0 = time.time()
    urdf_path = os.path.abspath(cfg["urdf"])
    if not os.path.isfile(urdf_path):
        raise SystemExit("URDF not found: %s  (edit CONFIG at the top of the script, or pass a path)" % urdf_path)
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

    if meta_path:
        write_meta(urdf, keep, meta_path, cfg["usd_units"], hik_helpers)
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
    if usd_path:
        export_usd(usd_path, cfg["usd_units"], cfg.get("usd_up", "Y"))
        print("USD    : %s (units=%s, up=%s)" % (usd_path, cfg["usd_units"], cfg.get("usd_up", "Y")))
    if render_path:
        render_preview(render_path, arm_obj, cfg["scale"])
        print("Render :", render_path)

    print("Done in %.1fs" % (time.time() - t0))
    print("=" * 72)

    # GUI 模式: 弹窗告诉用户文件在哪 (print 只进系统控制台, 容易看不到)
    gui_popup("G1 导入完成 - 输出文件位置", [
        "USD  (给 Maya): %s" % usd_path,
        "JSON (关节元数据): %s" % meta_path,
        "Blend (可选): %s" % (blend_path if (cfg["blend"] or bpy.app.background) else "未自动保存 (File > Save As 手动保存)"),
        "",
        "Maya 导入: File > Import 选 USD, 或用 maya_import_g1.py",
    ])


if __name__ == "__main__":
    main()
