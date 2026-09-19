# Isaac 资产 — 骨骼动画导出 (CSV / NPY)

把 **Blender / Maya / MotionBuilder** 里的 G1 骨骼动画统一导出为 Isaac（或任何 URDF
工具链 / pink-IK）可直接消费的轨迹数据。三个脚本共用同一套经过端到端验证的数学
（见文末"验证"），输出格式完全一致：

| 文件 | 运行环境 | 动画来源 |
|---|---|---|
| `export_animation_blender.py` | Blender 5（GUI 或 `--background`） | .blend 里的 Action / 手 K / 重定向结果 |
| `export_animation_maya.py` | Maya 2025（Script Editor, Python 标签） | FBX/USD 导入的动画、HIK bake 结果 |
| `export_animation_motionbuilder.py` | MotionBuilder（Python Editor, F11） | HIK 重定向 + Plot 之后的烘焙动画 |
| `replay_trajectory_isaaclab.py` | Isaac Lab 2.3（`isaaclab.bat -p`） | 回放上面导出的 .npy（视觉预览 / pink-IK 参考） |

**前置条件**：`_skeleton_meta.json` 必须是 **2025-09-19 之后**管线生成的版本
（含 `bind_local16` / `axis_parent_local` / `root_bind16` 字段）。旧 meta 请重跑
`scripts/blender_import_urdf.py` 重新生成。

---

## 一、输出格式（三个脚本完全一致）

每次导出生成 4 个文件（`<out>` 为输出前缀）：

| 文件 | 内容 |
|---|---|
| `<out>.csv` | 人读版：`frame,time_s,root_x..root_qz,<53个URDF关节名>` |
| `<out>.npy` | numpy 结构化数组，**字段名=关节名**（见下方加载示例） |
| `<out>_columns.json` | 元信息：fps、单位约定、关节名列表、限位表 |
| `<out>_summary.json` | 质量统计：越限次数（逐关节）、最大残差角 |

**数据内容**（每帧一行）：

- **根轨迹**：pelvis（URDF 根 link）的**位置 (x,y,z) + 四元数 (w,x,y,z)**。
  坐标系 = **URDF 原生 Z-up、米**（x 前、y 左、z 上），绑定帧时 = (0,0,0)+(1,0,0,0)。
- **关节角**：53 个 URDF 关节，**弧度**，关节名与 Isaac 里的 articulation joint 名
  一致（即 URDF joint 名）；已按 URDF 限位 clamp，±180° 以上量程的关节自动处理
  ±360° 环绕歧义。

Isaac 侧加载示例：

```python
import numpy as np
motion = np.load("g1_anim.npy")
t   = motion["time"]                    # (T,) 秒
pos = motion["root_pos"]                # (T,3) 米, Z-up
rot = motion["root_quat_wxyz"]          # (T,4) [w,x,y,z]
q   = motion["left_elbow_joint"]        # (T,) 弧度 —— 字段名就是关节名
joint_names = list(motion.dtype.names)  # 关节顺序见 _columns.json
```

## 二、各软件用法

### Blender（`export_animation_blender.py`）

GUI：Scripting 标签打开，改文件头 `CONFIG`（meta/out/帧范围），Run。

命令行批处理：

```
blender --background D:\BlenderPro\G1\xxx.blend --python export_animation_blender.py -- ^
    --meta D:\BlenderPro\G1\xxx_skeleton_meta.json --out D:\BlenderPro\G1\anim1
```

根轨迹默认取 **armature 姿势空间**（URDF 语义，不受骨架物体变换影响）；若把根
动画 K 在骨架物体上，`CONFIG['root_space']='world'`。

### Maya（`export_animation_maya.py`）

Script Editor（Python 标签）粘贴全文，改 `CONFIG`，运行。骨架来源不限（Blender
FBX 或 `maya_import_g1.py` 的 USD 均可），逐帧读关节 `.matrix`（局部，含
jointOrient）。若 pelvis 直接挂在世界级（无父节点），脚本自动把世界矩阵从
Y-up/cm 换算成 URDF Z-up/m。

### MotionBuilder（`export_animation_motionbuilder.py`）

建议流程：**HIK 重定向 → Character > Plot Character > Skeleton**（骨骼逐帧烘焙）
→ Python Editor (F11) 运行本脚本。局部矩阵由世界矩阵换算（乘法序自动检测），
命名空间自动剥离。

## 三、60 秒自检（金标准）

任一软件里，把 `left_elbow_joint` 摆到 **+30°**（其余全零、骨架未平移）后导出，
CSV 第一行应满足：

| 字段 | 期望值 |
|---|---|
| `left_elbow_joint` | **0.523599** (rad) |
| 其余 52 个关节 | **0.000000** |
| `root_x..root_qz` | **0,0,0,1,0,0,0** |

再加三个独立验证姿势：`left_hip_pitch_joint = −20°` → −0.349066；
`right_knee_joint = +25°` → 0.436332；`R_pinky_proximal_joint = +20°` → 0.349066。

对不上 = 数据版本或节点链问题（最常见：meta 是旧版，或 Maya/MB 里骨架被套了
额外变换——看脚本打印的 `[note]`/`[warn]`）。

## 四、在 Isaac Lab 里回放（Windows 快速上手）

环境约定：`C:\isaac-lab`（Isaac Lab 2.3.0）+ `C:\isaac-sim`（Isaac Sim 5.1.0）——这正好是官方配对版本（Lab 2.3.0 基于 Sim 5.1 构建）。RTX 4080 + 580 驱动满足要求。

### 0. 心智模型（1 分钟）

- **Isaac Sim** = 引擎 + UI（你已经能打开的那个窗口）；
- **Isaac Lab** = 构建在 Sim 之上的 Python 机器人学习框架，一切通过命令行跑；
- `isaaclab.bat`（在 `C:\isaac-lab` 根目录）是你的总入口：
  - `isaaclab.bat -p 脚本.py` → 用 Isaac Sim 自带的 Python 跑脚本（**最常用**）
  - `isaaclab.bat -s` → 启动 Sim UI
  - `isaaclab.bat -n` → 从模板新建项目
  - `isaaclab.bat -i` → 安装依赖/学习框架

### 1. 验证安装（第一次必做）

```bat
cd C:\isaac-lab
isaaclab.bat -p scripts\tutorials\00_sim\create_empty.py
```

能弹出一个空场景窗口 = Lab 装好了。再跑个自带 G1 的演示：

```bat
isaaclab.bat -p scripts\demos\bipeds.py
```

### 2. 回放我们导出的动捕动画（`replay_trajectory_isaaclab.py`）

把本目录的 `replay_trajectory_isaaclab.py` 和导出的 `g1_anim.npy`（及 `_columns.json`）放好，然后：

```bat
cd C:\isaac-lab
isaaclab.bat -p D:\BlenderPro\G1\replay_trajectory_isaaclab.py --npy D:\BlenderPro\G1\g1_anim.npy
```

- **默认预览模式**：关重力、逐帧写关节状态+根位姿 → 精确运动学回放（不需要平衡控制器，动捕长什么样机器人就摆什么样）；
- `--physics`：物理模式（重力 + 关节目标），机器人可能会倒——那正是之后 pink-IK/RL 要解决的部分；
- `--loop` 循环、`--speed 0.5` 慢放、`--headless --video` 无窗口录像；
- **关节按名字匹配**：内置 G1 是 29dof 身体版，我们数据里的 22 个手指关节会自动跳过（有提示）；想连手指回放，用 Isaac Sim 的 URDF Importer 把 `g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf` 转成 USD，再 `--usd 转换结果.usd` → 53 关节全匹配（URDF 导入时关节名会保留）。

数据坐标系无需转换：我们的根轨迹是 URDF Z-up/米，Isaac Sim 世界同样是 Z-up/米。

### 3. 下一步学习路线

- 官方教程（就在本地仓库）：`C:\isaac-lab\scripts\tutorials\` 从 `00_sim` 往后按序看；
- RL 训练一个任务试试水：`isaaclab.bat -p scripts\reinforcement_learning\rsl_rl\train.py --task=Isaac-Ant-v0 --headless`；
- **pink-IK**：pink 是独立的 Python 库（基于 pinocchio，`isaaclab.bat -m pip install pink` 装），典型用法是离线或在控制循环里解 IK 生成关节目标——我们导出的根轨迹+关节角正是它的参考输入/初值来源；pink 解出的目标序列同样可以用回放脚本预览。

### 4. 完整链路（推荐工作流）

```
动捕源(BVH/FBX) ──► MotionBuilder: characterize_g1.py 角色化 + 重定向 + Plot
                        │
                        ▼
          isaac/export_animation_motionbuilder.py
                        │
                        ▼
              g1_anim.npy ──► replay_trajectory_isaaclab.py 视觉预览
                        │
                        ▼
              pink-IK / 模仿学习 / RL (Isaac Lab)
```

也可在 Maya（HIK bake 后）或 Blender（重定向/手 K）导出，殊途同归。

## 五、语义与边界（重要）

- **残差（residual）**：动捕重定向后的骨骼运动不一定能被 URDF 单轴关节精确表达。
  导出值 = 该关节在 URDF 轴上的最优投影角；`_summary.json` 里的
  `residual_deg_max` 是"表达不了的部分"的上界（管线原生 DOF 动画 ≈0°，
  重定向动画会有几度~几十度——这是数据本身的性质，不是导出错误）。Isaac 里
  回放这些轨迹时机器人会走"最接近"的可达姿态。
- **越限（clipped）**：投影角超出 URDF 限位时被压回边界并计数。
  `_summary.json` 的 `clipped_total` 大说明动捕动作超出机器人能力，可考虑
  在重定向阶段收敛幅度。
- **T-Pose 绑定文件**（`--pose tpose` 生成的）：肩部 +90° 偏移由 meta 的
  `bind_offset_deg` 自动补偿，输入输出都是 URDF 关节角语义。
- **HIK 虚拟骨**（neck/toe）与结构骨：不在 53 关节表里，不参与导出。
- 前提是骨架**层级未被重组**（53 个可动骨的直接父级与 URDF 一致——管线默认
  构建即满足；三个脚本对此都做过全量核对）。

## 六、验证方法（给 TD）

三个脚本共用同一 CORE（行向量矩阵、Shepperd 四元数、轴投影角 + 限位/环绕处理），
验证方式：

1. **Blender 实测**：对管线构建的 .blend 用 URDF 世界轴做 4 关节（分属臂/腿/手指
   不相交子树）单关节 FK 摆位，运行导出器：53 关节全部还原真值（误差 < 1e-4 rad），
   根轨迹 = (0,0,0)+(1,0,0,0)。
2. **Maya / MotionBuilder 模拟**：以 1 的真实数据构造仿真场景（含命名空间、
   FBX 根节点、世界矩阵链）驱动两个脚本的 App 层，结果与 Blender 逐位一致。
3. 骨架父级一致性：53 可动骨的实际父级 vs URDF 父级，全量核对 0 差异。
