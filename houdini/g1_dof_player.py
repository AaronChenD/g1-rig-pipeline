# g1_dof_player.py — Houdini Python SOP (逐帧 DOF 播放器)
# ============================================================================
# 作用: 读机器人 DOF 数据 (CSV, 单位=度), 按当前帧给每个关节点写 f@dof,
#       配合下游 g1_dof_to_localtransform.vfl 驱动骨架.
#
# CSV 格式 (UTF-8, 逗号分隔):
#   第一行  = 关节名 (URDF joint 名如 left_elbow_joint, 或骨名
#             left_elbow_link, 两种都认; 可只写你关心的关节, 缺省=0)
#   之后每行 = 一帧的关节角 (度), 顺序与表头一致
#   示例:
#     frame,left_elbow_joint,left_hip_pitch_joint
#     1,0,0
#     2,15,-5
#     3,30,-10
#
# 用法: Python SOP 粘贴本文件, 改 CSV_PATH; 上游接 g1_import_urdf_meta.py.
#       播放时间线即逐帧取值 (hou.frame() 让节点随帧重算).
# ============================================================================

node = hou.pwd()
geo = node.geometry()

import csv

# ------------------------- 仅这两行需要改 -------------------------
CSV_PATH = r"D:\BlenderPro\G1\dof.csv"
DEGREES_INPUT = True      # False = CSV 里是弧度
FRAME_START = 1.0         # CSV 第一个数据行对应的 Houdini 帧
# ------------------------------------------------------------------

geo.addAttrib(hou.attribType.Point, "dof", 0.0)

pts = {}
for p in geo.points():
    nm = str(p.attribValue("name"))
    pts[nm.split("/")[-1]] = p
    pts.setdefault(nm, p)

# joint 名 / 骨名 -> 点 的映射 (借助上游盖的 urdf_joint 属性)
for p in geo.points():
    jn = str(p.attribValue("urdf_joint") or "")
    if jn:
        pts.setdefault(jn, p)

rows = []
with open(CSV_PATH, "r", encoding="utf-8-sig", newline="") as f:
    rd = csv.reader(f)
    header = next(rd)
    header = [h.strip() for h in header]
    for r in rd:
        if not r or all(not x.strip() for x in r):
            continue
        rows.append([float(x) for x in r])

if not rows:
    raise hou.NodeError("CSV 没有数据行: %s" % CSV_PATH)

frame = hou.frame()
row_i = int(round(frame - FRAME_START))
if row_i < 0:
    row_i = 0
if row_i >= len(rows):
    row_i = len(rows) - 1
vals = rows[row_i]

scale = 1.0 if DEGREES_INPUT else 57.2957795130823
hit = 0
for name, val in zip(header, vals):
    p = pts.get(name)
    if p is not None:
        p.setAttribValue("dof", val * scale)
        hit += 1
print("[g1_dof_player] 帧 %g -> CSV 行 %d/%d, 命中关节 %d/%d"
      % (frame, row_i + 1, len(rows), hit, len(header)))
