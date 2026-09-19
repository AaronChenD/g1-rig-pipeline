# Houdini 资产 — G1 DOF 驱动 / 限位清洗 / 动捕回收

本文件夹存放 Houdini（KineFX，H20/21/22 通用）专用脚本，配合管线输出的
`<名称>_houdini.usda`（米制 Y-up，三输出已对齐）与 `<名称>_skeleton_meta.json` 使用：

| 文件 | 类型 | 作用 |
|---|---|---|
| `g1_import_urdf_meta.py` | Python SOP | 读 meta json，把每关节的 URDF 轴向/限位/偏移盖章到骨架点 |
| `g1_dof_player.py` | Python SOP | 读 DOF 的 CSV，按帧写 `f@dof`（度） |
| `g1_dof_to_localtransform.vfl` | Attribute Wrangle 代码 | 按 `f@dof` 驱动骨架，自动 clamp 到 URDF 限位 |
| `g1_extract_dof.py` | Python SOP | 反向：从摆好姿势的骨架解出 URDF 关节角（动捕→机器人回收） |

> 需要重新生成 meta/USD 的话跑仓库根目录的 `scripts/blender_import_urdf.py`
> （2025-09-19 之后的版本才带 `axis_world_at_bind` 字段）。

---

## 一、最小可用管线（DOF 播放）

```
USD Character Import (File: *_houdini.usda)
 ├─ 输出1 (Rest Geometry) ──────────────┐
 └─ 输出2 (Capture Pose 骨架) ─┐        │
                              ▼        │
              [Python SOP: g1_import_urdf_meta.py]   ← 改脚本里的 META_PATH
                              │        │
                              ▼        │
              [Python SOP: g1_dof_player.py]         ← 改 CSV_PATH
                              │        │
                              ▼        │
              [Attribute Wrangle] 粘贴 g1_dof_to_localtransform.vfl
                              │        │
                              ▼        ▼
                          [Bone Deform] ─→ 模型随关节动
```

播放时间线即逐帧驱动（`g1_dof_player.py` 内部调用 `hou.frame()`，节点自动随帧重算）。

### CSV 格式（`g1_dof_player.py`）

```csv
frame,left_elbow_joint,left_hip_pitch_joint,right_knee_joint
1,0,0,0
2,15,-5,25
3,30,-10,25
```

- 第一行 = 关节名（**URDF joint 名或骨名都认**）；只写关心的关节即可，没写的 = 0°；
- 数值单位**度**（`DEGREES_INPUT=False` 可切弧度）；
- 单位约定：整个 Houdini 侧统一用**度**，限位属性也是度。

## 二、60 秒自检（强烈建议先做）

装好节点链后，只给 `left_elbow_joint` 设 `f@dof = 30`（其余不动），在 wrangle 后面
接个 Null 查看点的世界坐标，应与下表**逐位吻合**（数据来自 pxr 对 URDF FK 的数值验证）：

| 测试 | 关节角 | 骨骼 | 期望世界坐标 (米, Y-up) |
|---|---|---|---|
| A | left_elbow = +30° | `left_wrist_roll_link` | **(0.1487, 0.8389, 0.0974)** |
| B | left_hip_pitch = −20° | `left_ankle_roll_link` | **(0.1185, 0.0749, 0.2237)** |
| C | right_knee = +25° | `right_ankle_pitch_link` | **(−0.1185, 0.0811, −0.1268)** |
| D | 全 0（绑定姿势） | `head_link` | **(0.0000, 1.1171, 0.0038)** |

对不上 = 节点链/数据版本有问题（最常见：meta json 是旧版没有 `axis_world_at_bind`，
或输入骨架不在绑定姿势——见下节"注意"）。

## 三、每个文件详解

### 1. `g1_import_urdf_meta.py`（Python SOP）

- 只需改 `META_PATH`；
- 盖章属性：`v@urdf_axis`（父骨绑定系下的关节轴）、`f@urdf_lo_deg/hi_deg`（URDF 限位，
  continuous 关节 = ±9999）、`f@dof_offset`（T-pose 文件的肩外展补偿，零位文件全 0）、
  `i@urdf_movable`、`s@urdf_joint`、`f[]@bind_local16`（绑定姿势快照，供提取用）；
- **注意**：轴向计算假设输入在**绑定姿势**（刚从 USD Character Import 出来就是）。
  如果上游已经摆过姿势，请在绑定帧处加 **Rig Stash SOP** 生成 `rest` 属性后再接本节点
  （脚本检测到 `rest` 会自动改用它）。

### 2. `g1_dof_to_localtransform.vfl`（Attribute Wrangle，Run Over: Points）

- 数学（对全 62 关节用 pxr 验证到 2.7e-5 m）：
  `posed_local = [ bind_local 的旋转 · R(urdf_axis, θ) | 平移不变 ]`，
  其中 θ = clamp(dof, URDF 下限, 上限) − 绑定姿势偏移；
  关节枢轴就是骨骼自身原点，所以平移天然不动——这也是公式能这么短的原因；
- 没写 `f@dof` 的关节 = 0°（保持绑定姿势）；`urdf_movable=0` 的骨（固定骨/结构骨/虚拟骨）不碰。

### 3. `g1_dof_player.py`（Python SOP）

见 CSV 格式。逐帧取行；帧超范围则钳到最后一行。

### 4. `g1_extract_dof.py`（Python SOP，反向回收）

- 节点链：绑定骨架 → `g1_import_urdf_meta.py` → **[Rig Pose / KineFX 动画 / FBIK /
  MotionBuilder 重定向结果]** → 本节点；
- 输出 `f@dof_out`（URDF 关节角，度，已 clamp）、`f@dof_clipped`（1 = 超出 URDF 限位被压回，
  清洗质量指标——理想动捕结果应几乎全 0）；
- `WRITE_CSV` 填路径后每帧追加一行（首帧重建文件，重播时间线即得完整 CSV）；
- 数学与正公式互逆，53 关节随机角往返测试误差 < 0.001°；限位超过 ±180° 的关节
  自动在 θ / θ±360° 里选落在限位内的候选（同一旋转的环绕歧义）。

## 四、限位与清洗（节点 vs VEX）

- **程序化限位**：wrangle 已内置 clamp（对 URDF 单轴精确成立，这是主路径）；
- **交互式限位**：`Configure Joint Limits` SOP（H20+）可批量设限、**视口里可视化越限关节**；
  `Rig Pose` 视口状态开 **Enforce Transform Limits**（快捷键 F）后手柄拖不过限位。
  注意它按关节欧拉轴记录限位，而 URDF 关节是任意单轴——想可视化的话参考
  `g1_import_urdf_meta.py` 控制台打印的每关节轴向，在节点里对准对应欧拉轴；
- **曲线清洗（CHOPs，全可视化）**：`File CHOP` 直接读 `.chan`/CSV 动画 →
  `Filter CHOP`（平滑）→ `Math CHOP`（限幅）→ `Lag`（迟滞）→ `Slope CHOP`
  （按 meta 的 `velocity_rad_s` 限速）→ export 回属性；
- **物理级**：`Ragdoll` 系（限位从 jointconfig 读），机器人 DOF 回收一般用不到。

## 五、已知边界

- **虚拟骨**（`neck_link`、左右 `toe_link`，`--hik` 默认添加）：不在 URDF 关节表里，
  DOF 驱动/提取都不碰它们（它们零蒙皮权重，跟随父骨即可）；
- **T-Pose 文件**（`--pose tpose` 生成）：肩 roll 的绑定姿势偏移由 `f@dof_offset`
  自动补偿，输入输出仍是 URDF 关节角语义；
- **连续关节**（continuous）：无 URDF 限位，按 ±9999° 处理（不 clamp）；
- 提取（`g1_extract_dof.py`）要求姿势骨架的 `localtransform` 相对**绑定姿势**是纯关节旋转
  ——普通 KineFX 动画/Rig Pose/重定向结果都满足；若你手动平移过骨骼（IK 拉伸等），
  平移部分会被忽略。

## 六、数学备注（给 TD）

- USD/pxr/Houdini 矩阵为**行向量约定**（平移在第 4 行）；`posed = bind · X`，
  `X = T(−t)·R(â,θ)·T(t)`，因 t 恰为骨骼原点局部平移，展开后平移抵消：
  `posed = [B·R(â,θ) | t]`；
- `â = R_parent_bind_world⁻¹ · P(axis_world_at_bind)`，其中 `P: (x,y,z)→(y,z,x)`
  是 Z-up→Y-up 的轴向置换（`_houdini.usda` 已烘 Y-up）；
- `.vfl` 里手写的轴角矩阵元素布局已对 pxr `Gf.Rotation` 逐元素校验（500 组随机轴角一致）；
  四元数反解注意存储为行向量约定（= 列约定的转置），`g1_extract_dof.py` 内已处理。
