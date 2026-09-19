# -*- coding: utf-8 -*-
# export_animation_maya.py — Maya 2025 骨骼动画 → Isaac CSV/NPY
# ============================================================================
# 作用: 把 G1 (FBX/USD 导入 Maya 的骨架) 的骨骼动画逐帧导出为 Isaac / pink-IK 可用:
#         根轨迹 (pelvis 位置 + wxyz 四元数, URDF Z-up, 米)
#         + 全部 53 个 URDF 关节角 (弧度, 已按限位 clamp)
#       输出: <out>.csv, <out>.npy, <out>_columns.json, <out>_summary.json
#
# 用法 (Maya Script Editor, Python 标签): 改 CONFIG 后整体运行。
#   骨架来源不限: Blender 导出的 FBX, 或 maya_import_g1.py 导入的 USD, 均可。
#
# 原理: 关节角 = 骨骼局部矩阵相对绑定局部矩阵的旋转在 URDF 关节轴上的投影。
#   绑定数据 (bind_local16 / axis_parent_local / root_bind16) 来自管线 meta json,
#   与 Maya 里的骨骼一一对应 (关节名/骨名匹配, 自动剥命名空间)。
# ============================================================================

import json
import math
import os

# ----------------------------- CONFIG ---------------------------------------
CONFIG = {
    "meta": r"D:\BlenderPro\G1\g1_29dof_rev_1_0_with_inspire_hand_DFQ_skeleton_meta.json",
    "out":  r"D:\BlenderPro\G1\g1_anim_from_maya",
    "root_bone": "pelvis",          # 骨架根 (meta root_link 的同名骨)
    "frame_start": None,            # None = 动画起始帧
    "frame_end": None,              # None = 动画结束帧
    "fps": None,                    # None = 按 Maya currentUnit 推断
}
# -----------------------------------------------------------------------------

# ==================== CORE v1 (与 Blender/MotionBuilder 版完全一致) ============
def _m3_from16(m):
    return ((m[0], m[1], m[2]), (m[4], m[5], m[6]), (m[8], m[9], m[10]))

def _m3_mul(A, B):
    return tuple(tuple(sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))

def _m3_T(A):
    return tuple(tuple(A[j][i] for j in range(3)) for i in range(3))

def _quat_from_m3(M):
    """Shepperd (列向量约定). 注意: 传入前需把行向量存储转置!"""
    tr = M[0][0] + M[1][1] + M[2][2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (M[2][1] - M[1][2]) / S; y = (M[0][2] - M[2][0]) / S; z = (M[1][0] - M[0][1]) / S
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
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return (w / n, x / n, y / n, z / n)

def _m4_mul(A, B):
    """扁平 16 元组 (行主序) 的 4x4 乘积。"""
    out = [0.0] * 16
    for i in range(4):
        for j in range(4):
            out[i * 4 + j] = sum(A[i * 4 + k] * B[k * 4 + j] for k in range(4))
    return tuple(out)

def _m4_inv_rigid(m):
    """刚体 4x4 逆 (行向量: [R;t]⁻¹ = [Rᵀ; -t·Rᵀ])。"""
    R = _m3_from16(m)
    Rt = _m3_T(R)
    t = (m[12], m[13], m[14])
    tr = tuple(-sum(t[i] * Rt[i][j] for i in range(3)) for j in range(3))
    out = [0.0] * 16
    for i in range(3):
        for j in range(3):
            out[i * 4 + j] = Rt[i][j]
    out[12], out[13], out[14] = tr
    out[15] = 1.0
    return tuple(out)

def root_link_matrix16(posed_root16, root_bind16):
    """骨骼系根位姿 -> URDF link 系: L = B0⁻¹ · B_posed。绑定时应为单位阵。"""
    return _m4_mul(_m4_inv_rigid(root_bind16), posed_root16)

def pos_quat_from16(m):
    pos = (m[12], m[13], m[14])
    w, x, y, z = _quat_from_m3(_m3_T(_m3_from16(m)))
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    return pos, (w, x, y, z)

def extract_dof(posed16, bind16, axis, lo_deg, hi_deg, bind_offset_deg):
    """posed/bind = 行向量 16 元组 -> (URDF角_度, 超限0/1, 残差_度)。"""
    B = _m3_from16(bind16); P = _m3_from16(posed16)
    X = _m3_mul(_m3_T(B), P)
    w, qx, qy, qz = _quat_from_m3(_m3_T(X))
    n = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
    if n < 1e-12:
        return (0.0, 0, 0.0)
    dot = (qx * axis[0] + qy * axis[1] + qz * axis[2]) / n
    theta = math.degrees(2.0 * math.atan2(dot, w)) + bind_offset_deg
    if theta < lo_deg or theta > hi_deg:
        for c in (theta - 360.0, theta + 360.0):
            if lo_deg <= c <= hi_deg:
                theta = c
                break
    clipped = 0
    if theta < lo_deg:
        theta, clipped = lo_deg, (1 if lo_deg - theta > 1e-3 else 0)
    elif theta > hi_deg:
        theta, clipped = hi_deg, (1 if theta - hi_deg > 1e-3 else 0)
    total = math.degrees(2.0 * math.acos(min(1.0, abs(w))))
    residual = max(0.0, total - abs(theta - bind_offset_deg))
    return (theta, clipped, residual)

# ========================== Maya 层 ==========================================

def _fps_from_maya(cmds):
    table = {"game": 15.0, "film": 24.0, "pal": 25.0, "ntsc": 30.0,
             "show": 48.0, "palf": 50.0, "ntscf": 60.0}
    u = cmds.currentUnit(query=True, time=True)
    if u in table:
        return table[u]
    try:
        return float(str(u).replace("fps", ""))
    except Exception:
        return 24.0


def _yupcm_world16_to_zupm(m, cmds):
    """Maya 世界矩阵 (Y-up, 场景单位) -> URDF Z-up 米。轴置换与管线一致:
    zup = (z_yup, x_yup, y_yup)。"""
    lin = cmds.currentUnit(query=True, linear=True)
    scale = {"mm": 0.001, "cm": 0.01, "m": 1.0, "in": 0.0254,
             "ft": 0.3048, "yd": 0.9144}.get(lin, 1.0)
    K = ((0, 1, 0), (0, 0, 1), (1, 0, 0))       # zup = yup_row · K
    Kinv = _m3_mul(K, K)                          # K⁻¹ = K² (K³=I)
    R = _m3_from16(m)
    Rz = _m3_mul(Kinv, _m3_mul(R, K))
    t = (m[12] * scale, m[13] * scale, m[14] * scale)
    tz = (t[2], t[0], t[1])
    out = [0.0] * 16
    for i in range(3):
        for j in range(3):
            out[i * 4 + j] = Rz[i][j]
    out[12], out[13], out[14] = tz
    out[15] = 1.0
    return tuple(out)


def run(cfg):
    import maya.cmds as cmds

    with open(cfg["meta"], "r", encoding="utf-8") as f:
        meta = json.load(f)
    joints = [e for e in meta["joints"]
              if "bind_local16" in e and "axis_parent_local" in e]
    root_bind16 = meta.get("root_bind16")
    if not joints or not root_bind16:
        raise RuntimeError("meta 缺 bind_local16/axis_parent_local/root_bind16 —— "
                           "请用 2025-09-19 之后的 blender_import_urdf.py 重新生成")
    jmap = {e["bone"]: e for e in joints}

    # ---- 收集关节, 剥命名空间 ----
    all_joints = cmds.ls(type="joint") or []
    by_name = {}
    for j in all_joints:
        nm = j.split("|")[-1].split(":")[-1]
        by_name.setdefault(nm, j)
    missing = [b for b in jmap if b not in by_name]
    if missing:
        print("[warn] %d 根骨没找到 (取短名匹配): %s%s"
              % (len(missing), ", ".join(missing[:5]), " ..." if len(missing) > 5 else ""))

    root_jnt = by_name.get(cfg["root_bone"])
    if root_jnt is None:
        raise RuntimeError("找不到根关节 %s" % cfg["root_bone"])

    # ---- 帧范围 ----
    f0 = int(cfg["frame_start"] if cfg["frame_start"] is not None
             else cmds.playbackOptions(q=True, min=True))
    f1 = int(cfg["frame_end"] if cfg["frame_end"] is not None
             else cmds.playbackOptions(q=True, max=True))
    fps = float(cfg["fps"] if cfg["fps"] else _fps_from_maya(cmds))

    rows, stats = [], {e["joint"]: {"clip": 0, "res_max": 0.0} for e in joints}
    for f in range(f0, f1 + 1):
        cmds.currentTime(f)
        row = {"frame": f, "time": (f - f0) / fps}
        # ---- 根轨迹 (pelvis 局部矩阵 -> URDF link 系) ----
        # 常规: pelvis 有父节点 (FBX 根变换等), .matrix 即姿势空间, 直接用。
        # 例外: pelvis 直接挂世界级, .matrix = Y-up cm 世界矩阵 -> 转 Z-up m。
        parent = cmds.listRelatives(root_jnt, parent=True)
        m = tuple(cmds.getAttr(root_jnt + ".matrix"))   # 局部 (含 jointOrient), 行主序
        if not parent:
            m = _yupcm_world16_to_zupm(m, cmds)
            print("[note] pelvis 无父节点, 已把世界矩阵 (Y-up cm) 转为 URDF (Z-up m)")
        link16 = root_link_matrix16(m, tuple(root_bind16))
        pos, quat = pos_quat_from16(link16)
        row["root_pos"] = pos
        row["root_quat_wxyz"] = quat
        # ---- 关节角 ----
        for bone, e in jmap.items():
            jnt = by_name.get(bone)
            if jnt is None:
                continue
            posed = tuple(cmds.getAttr(jnt + ".matrix"))
            lim = e.get("limits") or {}
            lo = math.degrees(lim.get("lower_rad", -9999.0)) if lim else -9999.0
            hi = math.degrees(lim.get("upper_rad", 9999.0)) if lim else 9999.0
            th, clipped, res = extract_dof(posed, tuple(e["bind_local16"]),
                                           e["axis_parent_local"], lo, hi,
                                           e.get("bind_offset_deg", 0.0))
            row[e["joint"]] = math.radians(th)
            if clipped:
                stats[e["joint"]]["clip"] += 1
            stats[e["joint"]]["res_max"] = max(stats[e["joint"]]["res_max"], res)
        rows.append(row)

    _write_outputs(cfg, meta, joints, rows, fps, stats)
    return rows


def _write_outputs(cfg, meta, joints, rows, fps, stats):
    jnames = [e["joint"] for e in joints]
    out = cfg["out"]
    with open(out + ".csv", "w", encoding="utf-8") as f:
        f.write("frame,time_s,root_x,root_y,root_z,root_qw,root_qx,root_qy,root_qz,"
                + ",".join(jnames) + "\n")
        for r in rows:
            f.write("%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                    % (r["frame"], r["time"], *r["root_pos"], *r["root_quat_wxyz"]))
            f.write(",".join("%.6f" % r[j] for j in jnames) + "\n")
    try:
        import numpy as np
        dt = np.dtype([("time", "<f8"), ("root_pos", "<f8", (3,)), ("root_quat_wxyz", "<f8", (4,))]
                      + [(jn, "<f8") for jn in jnames])
        arr = np.zeros(len(rows), dtype=dt)
        for i, r in enumerate(rows):
            arr[i]["time"] = r["time"]
            arr[i]["root_pos"] = r["root_pos"]
            arr[i]["root_quat_wxyz"] = r["root_quat_wxyz"]
            for jn in jnames:
                arr[i][jn] = r[jn]
        np.save(out + ".npy", arr)
    except ImportError:
        print("[warn] 无 numpy, 跳过 .npy")
    with open(out + "_columns.json", "w", encoding="utf-8") as f:
        json.dump({
            "fps": fps,
            "units": {"translation": "meters", "rotation": "radians",
                      "root_frame": "URDF Z-up (x=forward, y=left, z=up)",
                      "quat_order": "wxyz"},
            "root": meta["root_link"],
            "joint_names": jnames,
            "joint_limits_rad": {e["joint"]: [e.get("limits", {}).get("lower_rad", None),
                                              e.get("limits", {}).get("upper_rad", None)]
                                 for e in joints},
            "bind_pose": meta.get("pose", {}).get("name", "zero"),
        }, f, indent=2, ensure_ascii=False)
    n_clip = sum(s["clip"] for s in stats.values())
    res_max = max((s["res_max"] for s in stats.values()), default=0.0)
    with open(out + "_summary.json", "w", encoding="utf-8") as f:
        json.dump({"frames": len(rows), "joints": len(jnames),
                   "clipped_total": n_clip,
                   "clipped_per_joint": {k: v["clip"] for k, v in stats.items() if v["clip"]},
                   "residual_deg_max": round(res_max, 4)},
                  f, indent=2, ensure_ascii=False)
    print("[export] %d 帧 x %d 关节 -> %s.csv/.npy/_columns.json/_summary.json "
          "(越限 %d, 最大残差 %.2f°)"
          % (len(rows), len(jnames), out, n_clip, res_max))


run(CONFIG)
