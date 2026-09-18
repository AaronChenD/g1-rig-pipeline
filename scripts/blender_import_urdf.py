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
    "urdf":   r"C:\unitree_ros\robots\g1_description\g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf",
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


def urdf_material(urdf, vis_el, mesh_obj):
    """Apply the <visual><material> color to a mesh (Principled BSDF)."""
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
        mat = bpy.data.materials.new(mat_name)
        mat.use_nodes = True
        bsdf = mat.node_tree.nodes.get("Principled BSDF")
        if bsdf:
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


def build_armature(urdf, keep, scale, robot_name):
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
    return arm_obj, arm_data


def _ctx_override(**overrides):
    """temp_override() context manager (Blender 3.2+)."""
    return bpy.context.temp_override(**overrides)


def skin_meshes(urdf, keep, scale, arm_obj, meshes_coll, auto_smooth):
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
                urdf_material(urdf, vis, o)
                for c in list(bpy.data.collections):
                    try:
                        c.objects.unlink(o)
                    except RuntimeError:
                        pass
                meshes_coll.objects.link(o)
                created.append(o)
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


# ----------------------------------------------------------------------------
# Metadata json (joint axes / limits, for retargeting & robotics)
# ----------------------------------------------------------------------------
def write_meta(urdf, keep, path, usd_units):
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
    return p.parse_args(argv)


def main():
    args = get_args()
    if args:
        cfg = {
            "urdf": args.urdf, "blend": args.blend or "", "usd": args.usd or "",
            "meta": args.meta or "", "render": args.render or "", "scale": args.scale,
            "usd_units": args.usd_units, "usd_up": args.usd_up, "skip_links": args.skip_links,
            "face_maya": not args.keep_urdf_orientation, "auto_smooth": not args.no_smooth,
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
    arm_obj, arm_data = build_armature(urdf, keep, cfg["scale"], urdf.name)
    meshes = skin_meshes(urdf, keep, cfg["scale"], arm_obj, meshes_coll, cfg["auto_smooth"])
    if cfg.get("face_maya", True):
        apply_maya_facing(arm_obj)
    print("Built  : %d bones, %d mesh objects" % (len(arm_data.bones), len(meshes)))

    if meta_path:
        write_meta(urdf, keep, meta_path, cfg["usd_units"])
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


if __name__ == "__main__":
    main()
