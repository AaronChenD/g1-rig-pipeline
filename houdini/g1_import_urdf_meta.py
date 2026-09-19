# g1_import_urdf_meta.py — Houdini Python SOP
# ============================================================================
# 作用: 读取 g1-rig-pipeline 的 _skeleton_meta.json, 把每个 URDF 关节的
#       轴向 / 限位 / 偏移盖章到 KineFX 骨架点上, 供下游 wrangle / 提取用.
#
# 用法:
#   1) SOP 网络里放一个 Python SOP, 把本文件内容粘进代码框
#      (或 Python Source 选 File 指向本文件);
#   2) 改下面 META_PATH 为你的 meta json 路径;
#   3) 输入接绑定姿势的骨架 (USD Character Import 的骨骼输出即可,
#      刚导入时就是绑定姿势);
#   4) Cook 后每个可动关节点上会多出:
#        v@urdf_axis     关节轴 (父骨绑定坐标系, 单位向量)
#        f@urdf_lo_deg   URDF 下限 (度; continuous 关节 = -9999)
#        f@urdf_hi_deg   URDF 上限 (度; continuous 关节 = +9999)
#        f@dof_offset    绑定姿势相对 URDF 零位的偏移 (度; 零位文件=0,
#                        --pose tpose 的肩 roll = 干90)
#        i@urdf_movable  1=可动关节 (53 个), 0=固定/结构骨
#        s@urdf_joint    URDF 关节名
#        f[]@bind_local16  盖章时的绑定 localtransform (16 浮点, 行主序,
#                          供 g1_extract_dof.py 反解 DOF 用)
#
# 注: 若输入是已摆姿势的骨架, 请在上游加一个 Rig Stash SOP (在绑定帧处
#     生成 rest 属性) 再接本节点 —— 本脚本会优先用 rest 属性算轴向.
# ============================================================================

node = hou.pwd()
geo = node.geometry()

import json

# ------------------------- 仅这一行需要改 -------------------------
META_PATH = r"D:\BlenderPro\G1\g1_29dof_rev_1_0_with_inspire_hand_DFQ_skeleton_meta.json"
# ------------------------------------------------------------------

with open(META_PATH, "r", encoding="utf-8") as f:
    meta = json.load(f)

mjoints = {e["bone"]: e for e in meta.get("joints", [])}
pose_offsets = meta.get("pose", {}).get("joint_offsets_deg", {}) or {}
RAD2DEG = 57.2957795130823

# ---- 收集骨架点 (点 name 可能是全路径, 取最后一段匹配) ----------------
pts = {}
for p in geo.points():
    nm = str(p.attribValue("name"))
    pts[nm.split("/")[-1]] = p
if not pts:
    raise hou.NodeError("输入骨架上没有 name 点属性 —— 请接 KineFX 骨架 (USD Character Import 输出2)")

# ---- 建属性 -----------------------------------------------------------
geo.addAttrib(hou.attribType.Point, "urdf_axis", (0.0, 0.0, 0.0))
geo.addAttrib(hou.attribType.Point, "urdf_lo_deg", 0.0)
geo.addAttrib(hou.attribType.Point, "urdf_hi_deg", 0.0)
geo.addAttrib(hou.attribType.Point, "dof_offset", 0.0)
geo.addAttrib(hou.attribType.Point, "urdf_movable", 0)
geo.addAttrib(hou.attribType.Point, "urdf_joint", "")
geo.addAttrib(hou.attribType.Point, "bind_local16", (0.0,) * 16)

# ---- 世界旋转: 优先 rest 属性, 否则用当前 localtransform 链乘 ----------
#     (localtransform 为行向量约定 4x4; 旋转 = 左上 3x3)
has_rest = "rest" in [a.name() for a in geo.pointAttribs()]

def mat3_of_local(p):
    v = p.attribValue("localtransform")     # 16 元组, 行主序
    return hou.Matrix3(((v[0], v[1], v[2]), (v[4], v[5], v[6]), (v[8], v[9], v[10])))

def rest_mat3(p):
    v = p.attribValue("rest")               # matrix3 属性 -> 9 元组
    return hou.Matrix3(((v[0], v[1], v[2]), (v[3], v[4], v[5]), (v[6], v[7], v[8])))

world_rot = {}   # bone -> 世界旋转 Matrix3 (绑定)
def wrot(name):
    if name in world_rot:
        return world_rot[name]
    e = mjoints.get(name)
    par = e["parent_bone"] if e else None
    if par and par in pts:
        # 行向量约定: 子帧世界 = local · 父帧世界 (local 在左!)
        M = mat3_of_local(pts[name]) * wrot(par)
    else:
        M = mat3_of_local(pts[name])
    world_rot[name] = M
    return M

for nm, p in pts.items():
    if has_rest:
        world_rot[nm] = rest_mat3(p)
    else:
        wrot(nm)     # 递归链乘 (要求输入在绑定姿势)

# ---- 盖章 -------------------------------------------------------------
n_axis = 0
for nm, p in pts.items():
    e = mjoints.get(nm)
    p.setAttribValue("urdf_movable", 0)
    if e is None:
        continue                                     # 虚拟骨 / 结构骨
    p.setAttribValue("urdf_joint", e.get("joint", ""))
    off = -float(pose_offsets.get(nm, 0.0))
    p.setAttribValue("dof_offset", off)
    lim = e.get("limits")
    if e.get("type") == "continuous" or not lim:
        p.setAttribValue("urdf_lo_deg", -9999.0)
        p.setAttribValue("urdf_hi_deg", 9999.0)
    else:
        p.setAttribValue("urdf_lo_deg", lim["lower_rad"] * RAD2DEG)
        p.setAttribValue("urdf_hi_deg", lim["upper_rad"] * RAD2DEG)
    aw = e.get("axis_world_at_bind")                 # Z-up 世界轴 (meta)
    par = e.get("parent_bone")
    if aw and par and par in world_rot:
        # Z-up -> Y-up 置换 (x,y,z)->(y,z,x), 再换到父骨绑定坐标系 (行向量 v*M)
        v = hou.Vector3((aw[1], aw[2], aw[0]))
        a_local = v * world_rot[par].inverted()
        p.setAttribValue("urdf_axis", (a_local[0], a_local[1], a_local[2]))
        p.setAttribValue("urdf_movable", 1)
        n_axis += 1
    # 绑定 local 快照 (行主序 16)
    lv = pts[nm].attribValue("localtransform")
    p.setAttribValue("bind_local16", tuple(float(x) for x in lv))

print("[g1_import_urdf_meta] 骨架点 %d | URDF 关节盖章 %d | rest属性: %s"
      % (len(pts), n_axis, "有" if has_rest else "无(用当前姿势当绑定)"))
if not has_rest:
    print("[g1_import_urdf_meta] 提示: 输入若已被 Rig Pose/动捕摆过姿势, 轴向会算错 —— "
          "在上游绑定帧处加 Rig Stash SOP 生成 rest 后再接本节点.")
