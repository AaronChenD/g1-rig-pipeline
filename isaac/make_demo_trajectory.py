# -*- coding: utf-8 -*-
# make_demo_trajectory.py — 生成一段 G1 演示动作 (蹲起 + 右臂挥手 + 转腰)
# ============================================================================
# 作用: 不依赖任何 DCC, 直接用 meta json 生成一段 6 秒演示轨迹 (NPY/CSV),
#       用于在 Isaac Lab 里测试 replay_trajectory_isaaclab.py 回放链路。
#       动作设计: 周期蹲起 (髋 -0.35s / 膝 +0.7s / 踝 -0.35s + 根下沉, 符号
#       与 IsaacLab 官方 G1 初始膝姿一致) + 右臂侧举挥手 + 腰部左右摆。
#
# 用法 (任一 Python 3.8+ / numpy 均可; 也可用 isaaclab.bat -p):
#   python make_demo_trajectory.py
#   可选参数: --meta <路径> --out <前缀> --fps 30 --duration 6
#
# 输出: <out>.npy (与 export_animation_*.py 完全同格式), <out>.csv,
#       <out>_columns.json
# ============================================================================

import argparse
import json
import math
import os

CONFIG = {
    "meta": r"D:\BlenderPro\G1\g1_29dof_rev_1_0_with_inspire_hand_DFQ_skeleton_meta.json",
    "out":  r"D:\BlenderPro\G1\g1_demo",
    "fps": 30.0,
    "duration": 6.0,
}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--meta", default=CONFIG["meta"])
    p.add_argument("--out", default=CONFIG["out"])
    p.add_argument("--fps", type=float, default=CONFIG["fps"])
    p.add_argument("--duration", type=float, default=CONFIG["duration"])
    args, _ = p.parse_known_args()

    import numpy as np

    try:
        with open(args.meta, "r", encoding="utf-8") as f:
            meta = json.load(f)
    except FileNotFoundError:
        print("[demo][ERROR] 找不到 meta 文件: %s" % args.meta)
        print("  meta 是 Blender 里 blender_import_urdf.py 生成的, 写在 URDF 同目录")
        print("  (文件名 = <URDF名>_skeleton_meta.json)。两个办法:")
        print("  1) 搜索现有 meta (找到后用 --meta <路径> 指定, 文件名 Tab 补全):")
        print('       Get-ChildItem D:\\BlenderPro -Recurse -Filter *_meta.json')
        print("  2) 下载仓库示例 meta (与默认同名, 生成演示轨迹够用) 到默认路径:")
        print("     Invoke-WebRequest https://raw.githubusercontent.com/AaronChenD/g1-rig-pipeline/arena/01a0b00d-g1-rig-pipeline/docs/example/g1_29dof_rev_1_0_with_inspire_hand_DFQ_skeleton_meta.json -OutFile \"%s\"" % args.meta)
        print("     (上面 URL 从本控制台复制是纯文本, 安全; 从聊天窗口复制会被富文本损坏)")
        raise SystemExit(1)
    joints = [e for e in meta["joints"]
              if e.get("type") in ("revolute", "continuous") and e.get("limits")]
    jnames = [e["joint"] for e in joints]
    jmap = {e["joint"]: e for e in joints}
    print("[demo] 关节 %d 个 (来自 %s)" % (len(jnames), os.path.basename(args.meta)))

    # ---- 根高基线与蹲深 (按 meta 几何实算, 勿手拍) ----
    # 站立 pelvis 高度: ground.pelvis_height_m (脚底贴 Z=0 时)。
    # 蹲深 = 髋-0.35/膝+0.70/踝-0.35 姿势下腿的竖直缩短量:
    #   大腿段 (hip_roll+hip_yaw+knee 的 z 分量) 与小腿段 (ankle_pitch 的 z) 各倾 0.35rad。
    ground = meta.get("ground") or {}
    H = float(ground.get("pelvis_height_m", 0.79))
    drop = 0.038
    try:
        thigh_z = sum(abs(jmap[n]["origin_xyz_m"][2]) for n in
                      ("left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint"))
        shin_z = abs(jmap["left_ankle_pitch_joint"]["origin_xyz_m"][2])
        drop = (thigh_z + shin_z) * (1.0 - math.cos(0.35))
    except Exception:
        pass
    print("[demo] 根高 H=%.4f m, 蹲深=%.4f m" % (H, drop))

    n = int(args.duration * args.fps)
    t = np.arange(n) / args.fps

    def clamp(jn, v):
        lim = jmap[jn]["limits"]
        return min(max(v, lim["lower_rad"]), lim["upper_rad"])

    # ---- 动作曲线 ----
    s = 0.5 - 0.5 * np.cos(2 * math.pi * t / 3.0)        # 蹲起节拍 (3s 周期, 0..1)
    w = np.clip(t / 1.5, 0.0, 1.0)                       # 挥手臂 1.5s 内抬起
    wave = np.sin(2 * math.pi * t / 0.8)                 # 挥手频率
    twist = 0.2 * np.sin(2 * math.pi * t / 3.0)          # 转腰

    pose = {jn: np.zeros(n) for jn in jnames}
    for side in ("left", "right"):
        pose["%s_hip_pitch_joint" % side][:] = -0.35 * s
        pose["%s_knee_joint" % side][:] = 0.70 * s
        pose["%s_ankle_pitch_joint" % side][:] = -0.35 * s
    pose["waist_yaw_joint"][:] = twist
    # 右臂: 侧举 (roll 负 = 外展, 与 --pose tpose 实验一致) + 肘弯 + 腕摆
    if "right_shoulder_roll_joint" in pose:
        pose["right_shoulder_roll_joint"][:] = -(0.4 + 0.5 * w)
    pose["right_elbow_joint"][:] = 0.4 + 0.35 * wave
    pose["right_wrist_roll_joint"][:] = 0.7 * wave
    for jn in jnames:
        pose[jn][:] = [clamp(jn, v) for v in pose[jn]]

    # ---- 打包 (与 export_animation_*.py 同格式) ----
    dt = np.dtype([("time", "<f8"), ("root_pos", "<f8", (3,)), ("root_quat_wxyz", "<f8", (4,))]
                  + [(jn, "<f8") for jn in jnames])
    arr = np.zeros(n, dtype=dt)
    arr["time"] = t
    arr["root_pos"][:, 2] = H - drop * s               # 蹲下时根下沉 (与腿缩短量一致, 脚底贴地)
    arr["root_quat_wxyz"][:, 0] = 1.0
    for jn in jnames:
        arr[jn] = pose[jn]
    np.save(args.out + ".npy", arr)

    with open(args.out + ".csv", "w", encoding="utf-8") as f:
        f.write("frame,time_s,root_x,root_y,root_z,root_qw,root_qx,root_qy,root_qz,"
                + ",".join(jnames) + "\n")
        for i in range(n):
            f.write("%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                    % (i + 1, t[i], 0, 0, arr["root_pos"][i][2], 1, 0, 0, 0))
            f.write(",".join("%.6f" % arr[i][jn] for jn in jnames) + "\n")

    with open(args.out + "_columns.json", "w", encoding="utf-8") as f:
        json.dump({
            "fps": args.fps,
            "units": {"translation": "meters", "rotation": "radians",
                      "root_frame": "URDF Z-up (x=forward, y=left, z=up)",
                      "quat_order": "wxyz"},
            "root": meta.get("root_link", "pelvis"),
            "joint_names": jnames,
            "joint_limits_rad": {e["joint"]: [e["limits"]["lower_rad"], e["limits"]["upper_rad"]]
                                 for e in joints},
            "bind_pose": "demo",
            "demo": "squat (3s cycle) + right-arm wave + waist twist",
        }, f, indent=2, ensure_ascii=False)

    print("[demo] %d 帧 @ %g fps -> %s.npy/.csv/_columns.json" % (n, args.fps, args.out))
    print("[demo] 回放: isaaclab.bat -p replay_trajectory_isaaclab.py --npy %s.npy" % args.out)


if __name__ == "__main__":
    main()
