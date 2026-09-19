# -*- coding: utf-8 -*-
# export_animation_blender.py — Blender 骨骼动画 → Isaac CSV/NPY
# ============================================================================
# 作用: 把 G1 (管线生成的 .blend) 的骨骼动画逐帧导出为 Isaac / pink-IK 可用的:
#         根轨迹 (pelvis 位置 + wxyz 四元数, URDF Z-up, 米)
#         + 全部 53 个 URDF 关节角 (弧度, 已按限位 clamp)
#       输出: <out>.csv, <out>.npy (结构化数组, 字段=关节名), <out>_columns.json,
#             <out>_summary.json (越限/残差统计)
#
# 用法 A (GUI):  Blender Scripting 标签打开本文件, 改 CONFIG, 点 Run。
# 用法 B (命令行):
#   blender --background D:\BlenderPro\G1\xxx.blend --python export_animation_blender.py -- ^
#       --meta D:\BlenderPro\G1\xxx_skeleton_meta.json --out D:\BlenderPro\G1\anim1
#
# 数据要求: meta json 需为 2025-09-19 之后版本 (含 bind_local16 / axis_parent_local)。
# 根轨迹默认取 pelvis 的 **姿势空间** (armature 局部) 矩阵 = URDF 语义 (Z-up 米,
# 不含物体级变换); 若你把根动画 K 在骨架物体上, 改 CONFIG['root_space']='world'。
# ============================================================================

import json
import math
import os
import sys

# ----------------------------- CONFIG ---------------------------------------
CONFIG = {
    "meta": r"D:\BlenderPro\G1\g1_29dof_rev_1_0_with_inspire_hand_DFQ_skeleton_meta.json",
    "out":  r"D:\BlenderPro\G1\g1_anim",          # 输出前缀 (自动加 .csv/.npy/...)
    "armature": "",                               # 空 = 场景里第一个 Armature
    "frame_start": None,                          # None = 场景起始帧
    "frame_end": None,                            # None = 场景结束帧
    "fps": None,                                  # None = 取场景 fps
    "root_space": "pose",                         # 'pose' (URDF 语义) | 'world'
}
# -----------------------------------------------------------------------------

# ==================== CORE v1 (与 Maya/MotionBuilder 版完全一致) ============
# 约定: 16 元组 = 行主序 4x4, 行向量 (v*M), 与 pxr/Maya MMatrix/MotionBuilder 一致。

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
    """骨骼系根位姿 -> URDF link 系: L = B0⁻¹ · B_posed (行向量)。
    绑定时输出单位阵 (根位置=URDF 原点, 根朝向=URDF 朝向)。"""
    return _m4_mul(_m4_inv_rigid(root_bind16), posed_root16)

def pos_quat_from16(m):
    """行向量 16 -> (pos3, quat wxyz)。"""
    pos = (m[12], m[13], m[14])
    w, x, y, z = _quat_from_m3(_m3_T(_m3_from16(m)))
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    return pos, (w, x, y, z)

def extract_dof(posed16, bind16, axis, lo_deg, hi_deg, bind_offset_deg):
    """posed/bind = 行向量 16 元组 -> (URDF角_度, 超限0/1, 残差_度)。
    残差 = 姿势里 URDF 单轴表达不了的旋转量 (动捕重定向后 >0, 管线原生动画 ≈0)。"""
    B = _m3_from16(bind16); P = _m3_from16(posed16)
    X = _m3_mul(_m3_T(B), P)                       # bind⁻¹ · posed
    w, qx, qy, qz = _quat_from_m3(_m3_T(X))        # 行向量存储 -> 传转置
    n = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
    if n < 1e-12:
        return (0.0, 0, 0.0)
    dot = (qx * axis[0] + qy * axis[1] + qz * axis[2]) / n
    theta = math.degrees(2.0 * math.atan2(dot, w)) + bind_offset_deg
    if theta < lo_deg or theta > hi_deg:           # ±360° 环绕歧义
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

# ========================== Blender 层 =======================================

def _row16_from_matrix(m_col):
    """mathutils Matrix (列向量) -> 行主序 16 元组 (行向量约定)。"""
    t = m_col.transposed()
    return [float(v) for row in t for v in row]


def run(cfg):
    import bpy  # 延迟 import, 便于其他软件参考 CORE 逻辑

    with open(cfg["meta"], "r", encoding="utf-8") as f:
        meta = json.load(f)
    joints = [e for e in meta["joints"]
              if "bind_local16" in e and "axis_parent_local" in e]
    if not joints:
        raise SystemExit("meta 里没有 bind_local16/axis_parent_local —— 请用 2025-09-19 "
                         "之后的 blender_import_urdf.py 重新生成 meta")
    jmap = {e["bone"]: e for e in joints}

    arm_obj = None
    for o in bpy.data.objects:
        if o.type == "ARMATURE":
            if not cfg["armature"] or o.name == cfg["armature"] or o.name.startswith(cfg["armature"]):
                arm_obj = o
                break
    if arm_obj is None:
        raise SystemExit("场景里找不到 Armature")

    scene = bpy.context.scene
    f0 = int(cfg["frame_start"] if cfg["frame_start"] is not None else scene.frame_start)
    f1 = int(cfg["frame_end"] if cfg["frame_end"] is not None else scene.frame_end)
    fps = float(cfg["fps"] if cfg["fps"] is not None else scene.render.fps)

    root_bone = meta["root_link"]
    root_bind16 = meta.get("root_bind16")
    if not root_bind16:
        raise SystemExit("meta 缺 root_bind16 —— 请用新版 blender_import_urdf.py 重新生成")
    pb_root = arm_obj.pose.bones.get(root_bone)
    if pb_root is None:
        raise SystemExit("骨架里没有根骨 %s" % root_bone)

    rows, stats = [], {e["joint"]: {"clip": 0, "res_max": 0.0} for e in joints}
    for f in range(f0, f1 + 1):
        scene.frame_set(f)
        bpy.context.view_layer.update()
        row = {"frame": f, "time": (f - f0) / fps}
        # ---- 根轨迹 (骨骼系 -> URDF link 系修正) ----
        M = pb_root.matrix                         # 姿势空间 (armature 局部)
        if cfg.get("root_space") == "world":
            M = arm_obj.matrix_world @ M
        link16 = root_link_matrix16(_row16_from_matrix(M), root_bind16)
        pos, quat = pos_quat_from16(link16)
        row["root_pos"] = pos
        row["root_quat_wxyz"] = quat
        # ---- 关节角 ----
        for pb in arm_obj.pose.bones:
            e = jmap.get(pb.name)
            if e is None:
                continue
            posed_col = pb.matrix if pb.parent is None else (pb.parent.matrix.inverted() @ pb.matrix)
            posed16 = _row16_from_matrix(posed_col)
            lim = e.get("limits") or {}
            lo = math.degrees(lim.get("lower_rad", -9999.0)) if lim else -9999.0
            hi = math.degrees(lim.get("upper_rad", 9999.0)) if lim else 9999.0
            th, clipped, res = extract_dof(posed16, e["bind_local16"],
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
    """三脚本共用的输出格式 (npy 部分在 Blender 里用 numpy)。"""
    jnames = [e["joint"] for e in joints]
    out = cfg["out"]
    # ---- CSV ----
    with open(out + ".csv", "w", encoding="utf-8") as f:
        f.write("frame,time_s,root_x,root_y,root_z,root_qw,root_qx,root_qy,root_qz,"
                + ",".join(jnames) + "\n")
        for r in rows:
            f.write("%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                    % (r["frame"], r["time"], *r["root_pos"], *r["root_quat_wxyz"]))
            f.write(",".join("%.6f" % r[j] for j in jnames) + "\n")
    # ---- NPY (结构化数组, 字段名=关节名) ----
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
        print("[warn] 无 numpy, 跳过 .npy (只写了 .csv)")
    # ---- columns.json ----
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
    # ---- summary.json ----
    n_clip = sum(s["clip"] for s in stats.values())
    res_max = max((s["res_max"] for s in stats.values()), default=0.0)
    with open(out + "_summary.json", "w", encoding="utf-8") as f:
        json.dump({"frames": len(rows), "joints": len(jnames),
                   "clipped_total": n_clip,
                   "clipped_per_joint": {k: v["clip"] for k, v in stats.items() if v["clip"]},
                   "residual_deg_max": round(res_max, 4),
                   "note": "residual = URDF 单轴无法表达的旋转量 (重定向动画会 >0)"},
                  f, indent=2, ensure_ascii=False)
    print("[export] %d 帧 x %d 关节 -> %s.csv/.npy/_columns.json/_summary.json "
          "(越限 %d, 最大残差 %.2f°)"
          % (len(rows), len(jnames), out, n_clip, res_max))


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    for i in range(0, len(argv) - 1, 2):
        k = argv[i].lstrip("-").replace("-", "_")
        if k in CONFIG and argv[i + 1]:
            CONFIG[k] = argv[i + 1]
    CONFIG["frame_start"] = int(CONFIG["frame_start"]) if CONFIG["frame_start"] else None
    CONFIG["frame_end"] = int(CONFIG["frame_end"]) if CONFIG["frame_end"] else None
    CONFIG["fps"] = float(CONFIG["fps"]) if CONFIG["fps"] else None
    run(CONFIG)


if __name__ == "__main__":
    main()
