# g1_extract_dof.py — Houdini Python SOP (从骨架姿势反解 URDF DOF)
# ============================================================================
# 作用: 上游骨架被 Rig Pose / 动捕重定向 / FBIK 摆好姿势后, 反解每个关节的
#       URDF 关节角 (度), 写 f@dof_out / f@dof_clipped, 可选导出 CSV.
#       用于"动捕 -> G1 -> 机器人关节角"的回收链路.
#
# 节点链:  绑定骨架 -> g1_import_urdf_meta.py (盖 urdf_axis/bind_local16)
#          -> [Rig Pose / KineFX 动画 / FBIK ...]  (姿势来源)
#          -> [本 Python SOP]
#
# 原理 (与 g1_dof_to_localtransform.vfl 互逆):
#   X = bind_local⁻¹ · posed_local = R(â, θ)   (旋转部分)
#   θ  = 2·atan2( â·v, w )   (q=(w, v) 为 X 的四元数)
#   URDF 角 = θ + dof_offset, 再 clamp 到限位并记录越界.
#
# 输出:
#   f@dof_out      URDF 关节角 (度, 已 clamp)
#   f@dof_clipped  1 = 原始角超出 URDF 限位被压回 (清洗指标)
#   (可选) CSV: WRITE_CSV 填路径后, 每帧追加一行: frame,关节1,关节2,...
# ============================================================================

node = hou.pwd()
geo = node.geometry()

import math

WRITE_CSV = ""      # 例: r"D:\BlenderPro\G1\dof_out.csv"  (空 = 不写文件)
FRAME_START = 1.0   # CSV 第一帧对应的 Houdini 帧

geo.addAttrib(hou.attribType.Point, "dof_out", 0.0)
geo.addAttrib(hou.attribType.Point, "dof_clipped", 0)

def mat3_from_mat4_t16(v16):
    # v16 = 行主序 4x4 (hou 属性元组)
    return ((v16[0], v16[1], v16[2]),
            (v16[4], v16[5], v16[6]),
            (v16[8], v16[9], v16[10]))

def mat3_mul(A, B):
    return tuple(tuple(sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))

def mat3_t(A):     # 转置
    return tuple(tuple(A[j][i] for j in range(3)) for i in range(3))

def quat_from_mat3(M):
    # Shepperd 法 (列向量约定): 四元数 (w, x, y, z)
    # 注意: pxr/Houdini 的矩阵存储是行向量约定 (= 列约定的转置),
    # 调用处须先传入 mat3_t(M) 转置!
    tr = M[0][0] + M[1][1] + M[2][2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (M[2][1] - M[1][2]) / S
        y = (M[0][2] - M[2][0]) / S
        z = (M[1][0] - M[0][1]) / S
    elif M[0][0] > M[1][1] and M[0][0] > M[2][2]:
        S = math.sqrt(1.0 + M[0][0] - M[1][1] - M[2][2]) * 2
        w = (M[2][1] - M[1][2]) / S; x = 0.25 * S
        y = (M[0][1] + M[1][0]) / S; z = (M[0][2] + M[2][0]) / S
    elif M[1][1] > M[2][2]:
        S = math.sqrt(1.0 + M[1][1] - M[0][0] - M[2][2]) * 2
        w = (M[0][2] - M[2][0]) / S; x = (M[0][1] + M[1][0]) / S
        y = 0.25 * S; z = (M[1][2] + M[2][1]) / S
    else:
        S = math.sqrt(1.0 + M[2][2] - M[0][0] - M[1][1]) * 2
        w = (M[1][0] - M[0][1]) / S; x = (M[0][2] + M[2][0]) / S
        y = (M[1][2] + M[2][1]) / S; z = 0.25 * S
    n = math.sqrt(w*w + x*x + y*y + z*z)
    return (w/n, x/n, y/n, z/n)

out_rows = []
n_clip = 0
for p in geo.points():
    p.setAttribValue("dof_clipped", 0)
    p.setAttribValue("dof_out", 0.0)
    if not p.attribValue("urdf_movable"):
        continue
    bind16 = p.attribValue("bind_local16")            # 绑定 local (导入时快照)
    posed = p.attribValue("localtransform")           # 当前姿势 local
    B = mat3_from_mat4_t16(bind16)
    P = mat3_from_mat4_t16(posed)
    # 行向量约定: posed = bind · X  ->  X = bind⁻¹ · posed (标准矩阵积, B⁻¹ = Bᵀ)
    Xr = mat3_mul(mat3_t(B), P)
    # Xr 是行向量约定存储; quat_from_mat3 按列向量约定 -> 传转置
    w, qx, qy, qz = quat_from_mat3(mat3_t(Xr))
    ax = p.attribValue("urdf_axis")
    n = math.sqrt(ax[0]*ax[0] + ax[1]*ax[1] + ax[2]*ax[2])
    if n < 1e-9:
        continue
    dot = (qx*ax[0] + qy*ax[1] + qz*ax[2]) / n
    theta = math.degrees(2.0 * math.atan2(dot, w))    # 绕 â 的有符号角 (-180,180]
    q_urdf = theta + p.attribValue("dof_offset")
    lo = p.attribValue("urdf_lo_deg"); hi = p.attribValue("urdf_hi_deg")
    # 转角环绕: 限位范围超过 ±180° 的关节 (如某些 yaw/roll), θ 与 θ±360 是同一旋转,
    # 优先取落在限位区间内的候选
    if q_urdf < lo or q_urdf > hi:
        for cand in (q_urdf - 360.0, q_urdf + 360.0):
            if lo <= cand <= hi:
                q_urdf = cand
                break
    qc = min(max(q_urdf, lo), hi)
    if abs(qc - q_urdf) > 1e-6:
        p.setAttribValue("dof_clipped", 1)
        n_clip += 1
    p.setAttribValue("dof_out", qc)
    out_rows.append((str(p.attribValue("urdf_joint")), qc))

if WRITE_CSV:
    frame = hou.frame()
    if frame <= FRAME_START + 1e-6:
        mode = "w"        # 首帧重建文件 (重播时间线即可得到完整 CSV)
    else:
        mode = "a"
    with open(WRITE_CSV, mode, encoding="utf-8") as f:
        if mode == "w":
            f.write("frame," + ",".join(j for j, _ in out_rows) + "\n")
        f.write("%g," % frame + ",".join("%.4f" % v for _, v in out_rows) + "\n")

print("[g1_extract_dof] 关节 %d | 超限被压回 %d | 帧 %g"
      % (len(out_rows), n_clip, hou.frame()))
