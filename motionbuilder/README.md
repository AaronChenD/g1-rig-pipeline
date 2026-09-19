# MotionBuilder 资产 — G1 HIK 角色化与动捕重定向

本文件夹存放 MotionBuilder 专用的映射文案与脚本：

| 文件 | 作用 |
|---|---|
| `characterize_g1.py` | 一键 HIK 角色化脚本（Python Editor 粘贴运行，自动填槽） |
| `README.md` | 本文档：HIK 槽位映射表 + 完整重定向流程 + 手指/关节数对不上的处理 |

---

## 一、准备：把 G1 弄进 MotionBuilder

1. Blender 打开管线生成的 `.blend`；
2. `File > Export > FBX`，只勾选 Armature + 全部网格，默认参数即可（MB 的 FBX 导入器认 Blender 4/5 的 FBX）；
3. MotionBuilder `File > Open`（或 Merge）导入该 FBX。

> 绑定姿势说明：默认导出是 **URDF 官方零位（手臂自然下垂）**。HIK 允许任意 stance 起手
> （它会按锁定定义时的姿势计算站位偏移）。若你的动捕源都是 T 起手、想走常规流程，
> 用 `--pose tpose` 重跑管线再导一份 FBX 即可，两份互不影响。

## 二、HIK 槽位映射表（G1 → HumanIK）

**必需要 15 槽全填**（Neck / 手指 / 脚趾都是可选）。G1 的 15 个必需槽全部有真实骨骼：

| HIK 槽位 | G1 骨骼 | HIK 槽位 | G1 骨骼 |
|---|---|---|---|
| Hips | `pelvis` | LeftArm / RightArm | `left/right_shoulder_pitch_link` |
| Spine | `waist_yaw_link` | LeftForeArm / RightForeArm | `left/right_elbow_link` |
| Spine1（可选） | `waist_roll_link` | LeftHand / RightHand | `left/right_wrist_roll_link` |
| Spine2（可选） | `torso_link` | LeftUpLeg / RightUpLeg | `left/right_hip_pitch_link` |
| Neck（可选） | `neck_link`（虚拟骨） | LeftLeg / RightLeg | `left/right_knee_link` |
| Head | `head_link` | LeftFoot / RightFoot | `left/right_ankle_pitch_link` |
| LeftToeBase / RightToeBase（可选） | `left/right_toe_link`（虚拟骨） | 手指（可选 22 槽） | `L/R_thumb/index/middle/ring/pinky_*` |

- 管线默认 `--hik` 已自动加 3 根零权重虚拟骨（`neck_link`、左右 `toe_link`）供 Neck/ToeBase 槽使用——**它们不参与蒙皮，随时可删**；
- 映射表也写进了 meta json 的 `hik.mapping` 字段，脚本 `characterize_g1.py` 与本表一致。

## 三、角色化（两种方式）

**方式 A：脚本（推荐）** —— `Window > Python Editor (F11)`，粘贴 `characterize_g1.py` 全文运行。
脚本自动建 Character、填全部槽位（含手指）、`SetCharacterizeOn`，失败会打印
`GetCharacterizeError()` 与未命中槽位清单。

**方式 B：手动** —— 选中骨架 → Assets 浏览器 → Character Controls → Create → *Character (Skeleton)*，
在 Definition 页按上表逐槽把骨骼拖进去（右键槽位 → Assign Selected Bone），15 个必需槽全绿后点 **Lock**。

## 四、重定向动捕动画

1. **动捕源也 characterize**：BVH/FBX 导入后建第二个 Character（MB 对常见命名能自动识别大半，
   剩余手动补）；
2. **指定输入源**：Character Controls → Controls 页 → Input Source 下拉选源角色（实时预览），
   或用 Story 工具把源动画放进轨道对齐时间；
3. **微调**：HIK 会自动补偿源/目标的比例差异；接触不良时调 Character Controls 里的
   Pull / Reach / 手脚接触参数；
4. **Plot 烘焙**：`Character > Plot Character > Skeleton`——**不 Plot 直接导出的 FBX 没有动画**；
5. 导出 FBX 给下游（Houdini 清洗 / 机器人 DOF 回收，见 `../houdini/README.md`）。

## 五、“关节数对不上 / 手指怎么办”（重要心智模型）

**重定向 ≠ 复制。** HIK 从不要求两边骨骼数相等——源和目标各按槽位 characterize，
HIK 只按槽传动作，多余的骨骼它根本不碰。所以“对不上”不是错误，关键是想清楚**哪些槽映射、
多余 DOF 谁负责**：

| 情况 | 做法 |
|---|---|
| 动捕源没有手套（多数惯性服） | **手指槽干脆不填**。G1 的 22 个手指关节保持绑定姿势，之后用姿势库/手动 K/程序化抓握补 |
| 有手套（Manus 等） | 两侧都填手指槽；G1 每指 2 节 + 拇指 4 骨，和 HIK 每指 3 槽对不齐的部分**空着即可**（HIK 允许部分映射） |
| 手指“拧麻花/断裂” | 经典原因 = 两侧手指骨轴向不一致（X 轴应沿指骨指向指尖）。成熟解法：放弃 HIK 手指求解，改**旋转约束 + Snap**（记住 stance 偏移、只传旋转），最后 bake 手指 |
| G1 的“多余 DOF”（肩复合 roll/yaw、腕 pitch/yaw、腰 roll/pitch、全部手指） | **HIK 不会自动重定向**。没有源数据就保持默认；需要细节时用 Character Extension / 约束 / 手动 K 补 |

机器人真值回收时，这些多余通道在 Houdini 里按 URDF 限位 clamp 清洗
（`../houdini/README.md` 的提取与限位一节）。

## 六、常见问题

- **“骨骼不够”无法创建角色**：15 个必需槽没填满。G1 全部有真实骨，按上表检查；
  用 `characterize_g1.py` 的话看它打印的未命中清单。
- **想删虚拟骨**：MB 里选中删除（先把子级挂回父级，如删 neck 前把 head 挂回 torso）；
  或源头 `--no-hik` 重导“干净版”。留着也无害（零权重，meta 的 `hik.helpers` 有清单）。
- **重定向后脚滑**：Plot 前在 Character Controls 调整脚部 Reach/Pull，或后处理用
  Houdini 的 CHOP 滤波（见 houdini 文档）。
