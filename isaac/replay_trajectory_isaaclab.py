# -*- coding: utf-8 -*-
# replay_trajectory_isaaclab.py — 在 Isaac Lab 里回放管线导出的骨骼动画
# ============================================================================
# 作用: 读取 isaac/export_animation_*.py 导出的 .npy/.csv 轨迹, 在 Isaac Lab
#       (Isaac Sim 5.1 + Lab 2.3, Windows/Linux 通用) 里驱动 G1 实时回放。
#       用途: 动捕重定向结果的 3D 视觉预览 / 给 pink-IK 调参考轨迹。
#
# 用法 (Windows, 在 C:\isaac-lab 目录下):
#   isaaclab.bat -p replay_trajectory_isaaclab.py --npy D:\BlenderPro\G1\g1_anim.npy
#   常用参数:
#     --usd <路径>      自定义机器人 USD (如你自己导入的 DFQ 灵巧手版 URDF)
#     --physics         物理模式 (重力+PD 目标, 机器人可能倒, 适合测试动力学)
#     --loop            循环播放
#     --speed 0.5       慢放 (0.5 = 一半速)
#     --headless --video 录像 (无窗口)
#
# 默认预览模式: 关重力 + 每帧写入关节状态/根位姿 = 精确运动学回放 (不需要平衡控制器)。
# 关节按名字匹配: 内置 G1 是 29dof 身体关节, 我们数据里的手指关节会自动跳过并提示;
# 想连手指一起回放, 用 --usd 指向你导入的 DFQ 版 USD (53 关节全匹配)。
#
# 注: 脚本按 Isaac Lab 2.3 API 编写; 个别接口名若有小版本差异, 按报错提示微调。
# ============================================================================

import argparse
import json
import math
import os

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Replay g1-rig-pipeline animation in Isaac Lab")
parser.add_argument("--npy", type=str, required=True, help="export_animation_*.py 输出的 .npy")
parser.add_argument("--usd", type=str, default="", help="自定义机器人 USD 路径 (默认用内置 G1)")
parser.add_argument("--physics", action="store_true", help="物理模式 (重力+PD 目标)")
parser.add_argument("--loop", action="store_true", help="循环播放")
parser.add_argument("--speed", type=float, default=1.0, help="回放倍速")
parser.add_argument("--fps", type=float, default=0.0, help="覆盖帧率 (默认读 _columns.json)")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(headless=args_cli.headless, video=args_cli.video if hasattr(args_cli, "video") else False)
simulation_app = app_launcher.app

# ---------------- Isaac Lab 层 (必须放在 AppLauncher 之后) --------------------
import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg

# InitState 配置类: Isaac Lab 2.3+ 是 ArticulationCfg 的嵌套类, 旧版为顶层导出
try:
    from isaaclab.assets import ArticulationInitStateCfg  # 旧版 (<=2.2)
except ImportError:
    ArticulationInitStateCfg = ArticulationCfg.InitialStateCfg  # v2.3+
from isaaclab.sim import SimulationContext


def load_motion(npy_path):
    import numpy as _np
    data = _np.load(npy_path)
    fps = 0.0
    cols_path = os.path.splitext(npy_path)[0] + "_columns.json"
    if os.path.isfile(cols_path):
        with open(cols_path, "r", encoding="utf-8") as f:
            fps = float(json.load(f).get("fps", 0.0))
    if fps <= 0 and len(data) > 1:
        fps = 1.0 / float(np.median(np.diff(data["time"])))
    return data, fps


def build_robot_cfg(usd_override):
    """内置 G1 (isaaclab_assets) 为基础; 允许覆盖 USD 与固定根。"""
    cfg = None
    try:
        from isaaclab_assets.robots.unitree import G1_CFG   # Isaac Lab 2.3 命名
        cfg = G1_CFG.copy()
    except Exception:
        try:
            from isaaclab_assets.robots.unitree import UNITREE_G1  # 旧版命名
            cfg = UNITREE_G1.copy()
        except Exception:
            cfg = None
    if cfg is None:
        # 兜底: 自己拼 (Isaac Sim 5.1 自带资产路径, 首次加载可能需联网缓存)
        try:
            from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
            usd = f"{ISAACLAB_NUCLEUS_DIR}/Robots/Unitree/G1/G1_with_hand/g1_29dof_with_hand_rev_1_0.usd"
        except Exception:
            raise RuntimeError("找不到内置 G1 资产配置 —— 请用 --usd 指定机器人 USD 路径")
        cfg = ArticulationCfg(
            prim_path="/World/G1",
            spawn=sim_utils.UsdFileCfg(usd_path=usd),
            init_state=ArticulationInitStateCfg(pos=[0.0, 0.0, 0.79]),
            actuators={},
        )
    # v2.3.0: prim path 必须是普通全局路径 (以 / 开头), {regex:...} 包裹语法已废除
    cfg.prim_path = "/World/G1"
    if usd_override:
        cfg.spawn.usd_path = usd_override
    cfg.init_state = ArticulationInitStateCfg(pos=[0.0, 0.0, 0.7923])
    return cfg


def main():
    data, fps = load_motion(args_cli.npy)
    if fps <= 0:
        fps = 30.0
    if args_cli.fps > 0:
        fps = args_cli.fps
    dur = float(data["time"][-1] - data["time"][0])
    t_data = data["time"] - data["time"][0]
    joint_fields = [n for n in data.dtype.names
                    if n not in ("time", "root_pos", "root_quat_wxyz")]
    print("[replay] 轨迹: %d 帧, %.2fs @ %g fps | 关节 %d | 根数据: %s"
          % (len(data), dur, fps, len(joint_fields),
             "有" if "root_pos" in data.dtype.names else "无"))

    # ---- 仿真上下文 ----
    gravity = (0.0, 0.0, -9.81) if args_cli.physics else (0.0, 0.0, 0.0)
    sim_dt = 1.0 / fps / max(args_cli.speed, 1e-3)
    sim = SimulationContext(sim_utils.SimulationCfg(
        dt=sim_dt, gravity=gravity, device=args_cli.device,
        # 30fps 时 dt=0.0333s > 官方推荐阈值, 开稳定化避免大步长物理问题
        physx=sim_utils.PhysxCfg(enable_stabilization=True),
    ))
    # ---- 机器人资产预检 (内置 G1 在云端; 不可达时给明确对策) ----
    robot_cfg = build_robot_cfg(args_cli.usd)
    usd_path = str(getattr(robot_cfg.spawn, "usd_path", "") or "")
    if usd_path and not os.path.isfile(usd_path):
        try:
            from isaaclab.utils.assets import check_file_path
            if check_file_path(usd_path) == 0:
                print("[replay][ERROR] 内置 G1 的云端资产不可达:\n        %s" % usd_path)
                print("        对策: ① 联网/代理后重试 (首次会下载并缓存, 之后离线可用);")
                print("              ② 或用 --usd 指向本地 USD (Isaac Sim URDF Importer")
                print("                 转换的 DFQ 版, 53 关节全匹配, 完全离线)。")
                simulation_app.close()
                raise SystemExit(1)
        except SystemExit:
            raise
        except Exception as e:
            print("[replay][note] 资产预检跳过 (%s)" % e)

    # ---- 地面: 官方 Grid USD; 云端不可达时用本地 Cuboid 兜底 (无需下载) ----
    try:
        ground = sim_utils.GroundPlaneCfg()
        ground.func("/World/GroundPlane", ground)
        print("[replay] 地面: 官方 Grid USD (云端)")
    except Exception as e:
        floor = sim_utils.CuboidCfg(
            size=(20.0, 20.0, 0.1),
            collision_props=sim_utils.CollisionCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.18, 0.22, 0.28)),
        )
        floor.func("/World/GroundCuboid", floor, translation=(0.0, 0.0, -0.05))
        print("[replay] 地面: 云端资产不可用 (%s) -> 已用本地 Cuboid 兜底" % type(e).__name__)

    light = sim_utils.DomeLightCfg(intensity=2000.0)
    light.func("/World/DomeLight", light)

    robot = Articulation(cfg=robot_cfg)
    sim.reset()
    robot.reset()

    # ---- 关节名匹配 (数据字段 -> 机器人关节索引) ----
    # 内置 G1 (g1.usd) 与 URDF 命名有差异, 已知别名 (无对应的仍跳过):
    #   URDF *_elbow_joint   <-> USD *_elbow_pitch_joint (USD 肘拆 pitch+roll, 取 pitch)
    #   URDF waist_yaw_joint <-> USD torso_joint
    # 内置版没有: 手腕 3x2、waist_roll/pitch、DFQ 手指 (走 --usd 本地转换路线可全对上)
    ALIAS = {
        "left_elbow_joint": "left_elbow_pitch_joint",
        "right_elbow_joint": "right_elbow_pitch_joint",
        "waist_yaw_joint": "torso_joint",
    }
    rjoints = list(robot.joint_names)
    idx_map, skipped, aliased = {}, [], []
    for jf in joint_fields:
        jn = ALIAS.get(jf, jf)
        if jn in rjoints:
            idx_map[jf] = rjoints.index(jn)
            if jn != jf:
                aliased.append("%s->%s" % (jf, jn))
        else:
            skipped.append(jf)
    if aliased:
        print("[replay] 别名映射: " + ", ".join(aliased))
    if skipped:
        print("[replay] 提示: %d 个数据关节目中没有对应关节 (如手指/手腕), 跳过: %s%s"
              % (len(skipped), ", ".join(skipped[:6]), " ..." if len(skipped) > 6 else ""))
        # 打印机器人实际关节名, 便于发现新的命名差异
        print("[replay] 机器人关节名 (%d): %s" % (len(rjoints), ", ".join(rjoints)))
    matched = sorted(idx_map.items(), key=lambda kv: kv[1])
    print("[replay] 机器人关节数 %d, 匹配 %d" % (len(rjoints), len(matched)))

    dev = robot.device
    n_j = len(rjoints)
    # 数据列 -> 数组
    ang = {jf: np.asarray(data[jf], dtype=float) for jf in joint_fields}
    root_pos = np.asarray(data["root_pos"], dtype=float) if "root_pos" in data.dtype.names else None
    root_quat = np.asarray(data["root_quat_wxyz"], dtype=float) if "root_quat_wxyz" in data.dtype.names else None

    def sample(t):
        t = float(np.clip(t, 0.0, t_data[-1]))
        i1 = int(np.searchsorted(t_data, t))
        i0 = max(i1 - 1, 0)
        i1 = min(i1, len(t_data) - 1)
        a = t_data[i1] - t_data[i0]
        w = 0.0 if a <= 0 else (t - t_data[i0]) / a
        q = {}
        for jf in joint_fields:
            q[jf] = ang[jf][i0] * (1 - w) + ang[jf][i1] * w
        rp = rq = None
        if root_pos is not None:
            rp = root_pos[i0] * (1 - w) + root_pos[i1] * w
            rq = root_quat[i0] * (1 - w) + root_quat[i1] * w
            nq = math.sqrt(float(np.sum(rq * rq)))
            rq = rq / nq
        return q, rp, rq

    # ---- 根轨迹语义: 数据是"绑定相对" (绑定时 = (0,0,0)+identity), ----
    # ---- 须叠加机器人初始世界摆放 T0: world = T0 ∘ L ----------------------
    try:
        _t0 = robot.data.root_link_pose_w[0].detach().cpu().numpy()
        p0 = [float(_t0[0]), float(_t0[1]), float(_t0[2])]
        q0 = [float(_t0[3]), float(_t0[4]), float(_t0[5]), float(_t0[6])]
    except Exception as e:
        print("[replay][note] 读初始根位姿失败 (%s), 用默认 0.7923" % e)
        p0, q0 = [0.0, 0.0, 0.7923], [1.0, 0.0, 0.0, 0.0]
    print("[replay] 根轨迹: 绑定相对 -> 世界 (叠加初始摆放 z=%.3f)" % p0[2])

    def _quat_mul(a, b):   # wxyz
        aw, ax, ay, az = a; bw, bx, by, bz = b
        return (aw*bw - ax*bx - ay*by - az*bz,
                aw*bx + ax*bw + ay*bz - az*by,
                aw*by - ax*bz + ay*bw + az*bx,
                aw*bz + ax*by - ay*bx + az*bw)

    def _quat_rot(q, v):   # v' = q v q*
        w, x, y, z = q
        tx, ty, tz = 2*(y*v[2] - z*v[1]), 2*(z*v[0] - x*v[2]), 2*(x*v[1] - y*v[0])
        return (v[0] + w*tx + (y*tz - z*ty),
                v[1] + w*ty + (z*tx - x*tz),
                v[2] + w*tz + (x*ty - y*tx))

    # ---- 写入 API (兼容 2.x 的两种命名) ----
    def write_root(p, quat_wxyz):
        # p/quat 是绑定相对量; 先合成到世界系, API 要 (N,7) 张量 [xyz, qwxyz]
        pw = _quat_rot(q0, p)
        pw = [pw[i] + p0[i] for i in range(3)]
        qw = _quat_mul(q0, quat_wxyz)
        pose = torch.tensor([pw + list(qw)], dtype=torch.float, device=dev)
        fn = getattr(robot, "write_root_link_pose_to_sim", None) \
            or getattr(robot, "write_root_pose_to_sim")
        fn(pose)

    # 初始帧
    q0, rp0, rq0 = sample(0.0)
    pos0 = torch.zeros((1, n_j), dtype=torch.float, device=dev)
    for jf, i in idx_map.items():
        pos0[0, i] = float(q0[jf])
    robot.write_joint_state_to_sim(pos0, torch.zeros_like(pos0))
    if not args_cli.physics and rp0 is not None:
        write_root(rp0, rq0)
    sim.set_camera_view(eye=[2.5, 2.5, 1.6], target=[0.0, 0.0, 0.9])

    print("[replay] 播放中... (关掉窗口或 Ctrl+C 结束)")
    t = 0.0
    while simulation_app.is_running():
        q, rp, rq = sample(t)
        if args_cli.physics:
            tgt = torch.zeros((1, n_j), dtype=torch.float, device=dev)
            for jf, i in idx_map.items():
                tgt[0, i] = float(q[jf])
            # 物理模式: 写 PD 目标 (经 actuator 产生力矩), 而非直接搬关节
            robot.set_joint_position_target(tgt)
            robot.write_data_to_sim()
        else:
            pos = torch.zeros((1, n_j), dtype=torch.float, device=dev)
            for jf, i in idx_map.items():
                pos[0, i] = float(q[jf])
            robot.write_joint_state_to_sim(pos, torch.zeros_like(pos))
            if rp is not None:
                write_root(rp, rq)
        sim.step()
        sim.render()
        t += sim_dt * args_cli.speed
        if t > dur:
            if args_cli.loop:
                t = 0.0
            else:
                print("[replay] 播放结束 (%.1fs)" % dur)
                break
    simulation_app.close()


if __name__ == "__main__":
    main()
