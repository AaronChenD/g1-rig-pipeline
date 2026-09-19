# -*- coding: utf-8 -*-
# export_animation_motionbuilder.py — MotionBuilder 骨骼动画 → Isaac CSV/NPY
# ============================================================================
# 作用: 把 G1 (FBX 导入 MotionBuilder 的骨架, 如 HIK 重定向 + Plot 之后) 的
#       骨骼动画逐帧导出为 Isaac / pink-IK 可用:
#         根轨迹 (pelvis 位置 + wxyz 四元数, URDF Z-up, 米)
#         + 全部 53 个 URDF 关节角 (弧度, 已按限位 clamp)
#       输出: <out>.csv, <out>.npy, <out>_columns.json, <out>_summary.json
#
# 用法: MotionBuilder 里 Window > Python Editor (F11), 改 CONFIG 后整体运行。
#   建议在 Character > Plot Character > Skeleton 之后运行 (骨骼已逐帧烘焙)。
#
# 原理: 关节角 = 骨骼局部矩阵相对绑定局部矩阵的旋转在 URDF 关节轴上的投影。
#   局部矩阵 = 世界矩阵相对父骨换算 (乘法序自动检测); 绑定数据来自管线 meta json。
# ============================================================================

import json
import math
import os

# ----------------------------- CONFIG ---------------------------------------
CONFIG = {
    "meta": r"D:\BlenderPro\G1\g1_29dof_rev_1_0_with_inspire_hand_DFQ_skeleton_meta.json",
    "out":  r"D:\BlenderPro\G1\g1_anim_from_mb",
    "root_bone": "pelvis",
    "frame_start": None,            # None = 取播放范围
    "frame_end": None,
    "fps": None,                    # None = 按 PlayerControl 推断
}
# -----------------------------------------------------------------------------

# ==================== CORE v1 (与 Blender/Maya 版完全一致) ===================
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

# ========================== MotionBuilder 层 =================================

def _matrix16(model, lSys):
    from pyfbsdk import FBModelTransformationMatrix
    m = [0.0] * 16
    model.GetMatrix(m, FBModelTransformationMatrix.kModelTransformation, True)
    return tuple(m)


def _local16(model, parent, lSys, order):
    """子世界 -> 相对父局部。order='child_parent' 则 local = Wc·Wp⁻¹ (行向量)。"""
    Wc = _matrix16(model, lSys)
    if parent is None:
        return Wc
    Wp = _matrix16(parent, lSys)
    Wp_inv = _inv4(Wp)
    return _m4_mul(Wc, Wp_inv) if order == "child_parent" else _m4_mul(Wp_inv, Wc)


def _inv4(m):
    """一般 4x4 逆 (高斯消元, 处理含缩放的根节点)。"""
    n = 4
    A = [list(m[i * 4:(i + 1) * 4]) + [1.0 if i == j else 0.0 for j in range(n)]
         for i in range(n)]
    for col in range(n):
        piv = max(range(col, n), key=lambda r: abs(A[r][col]))
        if abs(A[piv][col]) < 1e-12:
            return tuple([1.0 if i == j else 0.0 for i in range(4) for j in range(4)])
        A[col], A[piv] = A[piv], A[col]
        p = A[col][col]
        A[col] = [v / p for v in A[col]]
        for r in range(n):
            if r != col and A[r][col] != 0:
                f = A[r][col]
                A[r] = [a - f * b for a, b in zip(A[r], A[col])]
    out = []
    for i in range(n):
        out += A[i][n:]          # 每行后 4 列 = 逆矩阵行
    return tuple(out)


def run(cfg):
    from pyfbsdk import FBSystem, FBPlayerControl, FBTime, FBModelSkeleton, \
        FBModelTransformationMatrix

    with open(cfg["meta"], "r", encoding="utf-8") as f:
        meta = json.load(f)
    joints = [e for e in meta["joints"]
              if "bind_local16" in e and "axis_parent_local" in e]
    root_bind16 = meta.get("root_bind16")
    if not joints or not root_bind16:
        raise RuntimeError("meta 缺 bind_local16/axis_parent_local/root_bind16 —— "
                           "请用 2025-09-19 之后的 blender_import_urdf.py 重新生成")
    jmap = {e["bone"]: e for e in joints}

    lSys = FBSystem()
    lPC = FBPlayerControl()

    # ---- 收集骨骼 (短名匹配, 自动剥命名空间) ----
    bones = {}
    for skel in lSys.Scene.ModelSkeletons:
        nm = skel.Name.split(":")[-1]
        bones.setdefault(nm, skel)
    missing = [b for b in jmap if b not in bones]
    if missing:
        print("[warn] %d 根骨没找到: %s%s"
              % (len(missing), ", ".join(missing[:5]), " ..." if len(missing) > 5 else ""))
    root_model = bones.get(cfg["root_bone"])
    if root_model is None:
        raise RuntimeError("找不到根骨 %s" % cfg["root_bone"])

    # ---- 帧范围 / fps ----
    f0 = int(cfg["frame_start"] if cfg["frame_start"] is not None
             else lPC.ZoomStart.GetFrame())
    f1 = int(cfg["frame_end"] if cfg["frame_end"] is not None
             else lPC.ZoomEnd.GetFrame())
    if cfg["fps"]:
        fps = float(cfg["fps"])
    else:
        rates = {FBTime.kFBCustomFrameRate: 30.0}  # 兜底
        try:
            fps = {"kFB60FPSTimeCode": 60.0}.get(str(lSys.Scene.PlayerSettings.Rate), None)
        except Exception:
            fps = None
        if fps is None:
            fps = 30.0
            print("[note] 未识别帧率, 按 %g fps (CONFIG['fps'] 可指定)" % fps)

    # ---- 乘法序自动检测: 用 pelvis 的一个子骨, 取平移小的那种序 ----
    child_of_root = None
    for b in jmap:
        if jmap[b]["parent_bone"] == cfg["root_bone"] and b in bones:
            child_of_root = b
            break
    order = "child_parent"
    if child_of_root:
        for cand in ("child_parent", "parent_child"):
            L = _local16(bones[child_of_root], root_model, lSys, cand)
            if abs(max(L[12], L[13], L[14])) < 2.0:     # 局部平移 = 骨长 (米级)
                order = cand
                break
    print("[info] 局部矩阵乘法序: %s" % order)

    rows, stats = [], {e["joint"]: {"clip": 0, "res_max": 0.0} for e in joints}
    for f in range(f0, f1 + 1):
        lSys.LocalTime = FBTime(0, 0, 0, f)
        lSys.Scene.Evaluate()
        row = {"frame": f, "time": (f - f0) / fps}
        # ---- 根轨迹 ----
        posed_root = _local16(root_model, root_model.Parent, lSys, order)
        link16 = root_link_matrix16(posed_root, tuple(root_bind16))
        pos, quat = pos_quat_from16(link16)
        row["root_pos"] = pos
        row["root_quat_wxyz"] = quat
        # ---- 关节角 ----
        for bone, e in jmap.items():
            model = bones.get(bone)
            if model is None:
                continue
            par = bones.get(e["parent_bone"])
            posed = _local16(model, par, lSys, order)
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
