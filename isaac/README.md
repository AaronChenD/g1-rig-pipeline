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
  坐标系 = **URDF 原生 Z-up、米**（x 前、y 左、z 上），**绑定相对**语义：
  绑定姿势（站立）时 = (0,0,0)+(1,0,0,0)，数值表示"相对绑定姿势的运动"。
  Isaac 回放脚本会自动把它叠加到机器人初始摆放上（世界位姿 = T₀ ∘ L，
  站立时 pelvis 世界 z≈0.79）；因此**不要**把站立高度 0.79 写进数据。
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

GUI（推荐）：打开你的动画 `.blend` → 顶部 **Scripting（脚本）** 工作区 →
文本编辑器里 **Open（打开）** 本脚本 → 改文件头 `CONFIG`（meta / out 前缀；
帧范围和 fps 留 None = 自动取场景设置）→ 点 **▶ Run Script（运行脚本）**。
运行日志在 **窗口 → 切换系统控制台**（Window → Toggle System Console）里看，
结束后回场景另存即可。

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
  - `.\isaaclab.bat -p 脚本.py` → 用 Isaac Sim 自带的 Python 跑脚本（**最常用**；PowerShell 必须带 `.\` 前缀）
  - `isaaclab.bat -s` → 启动 Sim UI
  - `isaaclab.bat -n` → 从模板新建项目
  - `isaaclab.bat -i` → 安装依赖/学习框架

### 1. Windows 两个坑（PowerShell 必读）

- **PowerShell 不执行当前目录的程序**，必须加 `\` 前缀：`.\isaaclab.bat ...`
  （用 cmd.exe 则不用前缀，直接 `isaaclab.bat` 即可）；
- **别从聊天窗口/网页直接复制命令**——富文本会把文件名变成
  `create_[empty.py](http://...)` 这种带链接的坏名字，请手动敲或用纯文本粘贴。
  **根治办法**：文件名不经过聊天窗口。**首选 git**（脚本只存在于仓库里,
  `git pull` 即更新, 见下方"目录约定"）；没有 git 时的备选——让 PowerShell
  从 GitHub API 拿文件名, 把本目录 (isaac/) 所有 `.py`+`.md` 下载到
  `D:\BlenderPro\G1\`（此块可整段粘贴, 无文件名字面量, 不会被富文本改坏）:

  ```powershell
  [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
  $dir  = "D:\BlenderPro\G1"
  $api  = "https://api.github.com/repos/AaronChenD/g1-rig-pipeline/contents/isaac?ref=arena/01a0b00d-g1-rig-pipeline"
  foreach ($f in (Invoke-RestMethod $api | Where-Object { $_.name -match '\.(py|md)$' })) {
      Invoke-WebRequest $f.download_url -OutFile (Join-Path $dir $f.name)
      "已下载 " + $f.name
  }
  ```

  之后运行长命令时, 文件名部分用 **Tab 补全**: 敲到 `...\\check_` 按 Tab,
  PowerShell 会自动补全成正确文件名, 不会被富文本污染。

### 1.5 批处理文件行尾坑（症状：一堆 '不是内部或外部命令' 的残片）

IsaacLab 官方仓库里的 `isaaclab.bat` 是 **LF（Unix）行尾**（v2.3.0 实测：673 个
换行全是 LF、0 个 CRLF，且 `.gitattributes` 没有 `*.bat` 的 eol 规则）。**无论
GitHub "Download ZIP"（打包的就是仓库原始字节，必然 LF）还是 `autocrlf=false` 的
git 克隆，本地 bat 都是 LF** → cmd.exe 解析 LF 批处理会错位，表现为满屏
`'ause' / 'xists' / 'hon_exe' 不是内部或外部命令` 之类的残片命令 +
`此时不应有 |`，最后 `[ERROR] Unable to find any Python executable`。

修复（PowerShell，一次性；**zip 安装与 git 克隆都适用**——直接转换行尾）：

```powershell
$p = "C:\isaac-lab\isaaclab.bat"
$t = [IO.File]::ReadAllText($p)
[IO.File]::WriteAllText($p, ($t -replace "`r?`n", "`r`n"), (New-Object System.Text.UTF8Encoding($false)))
```

（UTF-8 无 BOM 写回，不会引入新问题。）
要一并修目录下所有 .bat：

```powershell
Get-ChildItem "C:\isaac-lab" -Filter *.bat -Recurse | ForEach-Object {
    $t = [IO.File]::ReadAllText($_.FullName)
    [IO.File]::WriteAllText($_.FullName, ($t -replace "`r?`n", "`r`n"), (New-Object System.Text.UTF8Encoding($false)))
    Write-Host "已修" $_.Name
}
```

> git 克隆用户的替代法：`git config core.autocrlf true` 后
> `del isaaclab.bat; git checkout -- isaaclab.bat`（强制重新检出自动转 CRLF）。
> zip 安装没有 .git，只能用上面的直接转换。

验证 bat 修好了：`.\isaaclab.bat --help` 应打印 usage 帮助而不是报错。

### 1.2 bat 找不到 Isaac Sim / 用错 Python（症状同上：找不到 Python executable）

**Isaac Lab 2.3.0 的 `isaaclab.bat` 不读 `ISAACSIM_PATH` 环境变量**（已核对官方源码：
它只认 ① conda 环境 ② `C:\isaac-lab\_isaac_sim\python.bat` ③ 兜底抓 PATH 里的系统
Python——最后这条是灾难，会把包装进错误的 Python）。zip 安装没有 `_isaac_sim`，
所以必须手动建一个 junction（一次性，无需管理员）：

```powershell
cmd /c mklink /J "C:\isaac-lab\_isaac_sim" "C:\isaac-sim"   # 换成你实际的 Sim 目录名!
Test-Path C:\isaac-lab\_isaac_sim\python.bat    # 应输出 True
```

> **注意**：`mklink /J` 创建时**不校验目标是否存在**——路径写错（比如把
> `issac-sim` 写成 `isaac-sim`）照样报"创建的联接"，但 Test-Path 会是 False
> （悬空 junction）。排查真名：
> `Get-ChildItem C:\ -Directory | Where-Object Name -like "*saac*"`；
> zip 解压的 Sim 可能还嵌套一层（python.bat 不在根），用
> `Get-ChildItem C:\isaac-sim -Recurse -Filter python.bat -Depth 2` 找到
> 实际层级，junction 指向含 python.bat 的那层。改目标先
> `cmd /c rmdir "C:\isaac-lab\_isaac_sim"`（只删链接不删文件）。

> 判断是否中招：`-i` 安装日志里的 pip 报错路径出现
> `AppData\Local\Programs\Python\Python312`（系统 Python）而不是 isaac-sim 的
> 路径 = bat 用错了 Python。新版（3.x）的 bat 才读 `ISAACSIM_PATH`。

### 2. 验证安装（第一次必做）

> **zip 安装的用户**：zip 里只是源码，必须先把 Isaac Lab 装进 Sim 的 Python
> （需联网，会下载 PyTorch 等数 GB，耐心），否则任何脚本都会
> `ModuleNotFoundError: isaaclab`：
>
> ```powershell
> cd C:\isaac-lab
> .\isaaclab.bat -i
> ```
>
> **先做一步预修**：isaaclab 依赖的 `flatdict==4.0.1` 是 2021 年的老源码包，
> 其 setup.py 用的 `pkg_resources` 已被新版 setuptools 移除 → pip 构建必炸
> （`ModuleNotFoundError: No module named 'pkg_resources'`）。在 Sim 的 Python 里
> 预装一份旧 setuptools 再关掉构建隔离装 flatdict：
>
> ```powershell
> & "C:\isaac-lab\_isaac_sim\python.bat" -m pip install setuptools==80.9.0 wheel
> & "C:\isaac-lab\_isaac_sim\python.bat" -m pip install flatdict==4.0.1 --no-build-isolation
> ```
>
> **坑：别在 bat 命令里写 `"setuptools<81"` 这种带 `<` 的版本约束**——PowerShell 5.1
> 给 bat 传参时会丢掉手打的引号，cmd 收到裸的 `<81` 会当成"从文件 81 重定向输入"，
> 瞬间报"系统找不到指定的文件"且 pip 根本没跑。一律用 `==` 具体版本号。
>
> 之后 `-i` 看到 flatdict 已满足就会跳过构建。若还有其他包报同样的
> `pkg_resources` 错，同样套路处理。
>
> **`-i` 阶段两条已知无害提示**（上游 bat 自身的小毛病，非安装失败）：
> - `文件名、目录名或卷标语法不正确。`——bat 里 torch 版本探测那句 `for /f`
>   的嵌套引号在 cmd 下解析失败（v2.3.0 源码第 56 行）；
> - `[INFO] Found PyTorch version .`（空版本号）——上一条的后果：探测不到
>   Sim 自带 torch 的版本，于是 bat 卸掉重装 `torch==2.7.0+cu128`——这本来就是
>   Isaac Lab 2.3 的官方要求版本，最终状态正确，等它装完即可。
>
> 装完验收：
> `& "C:\isaac-lab\_isaac_sim\python.bat" -c "import torch; print(torch.__version__, torch.cuda.is_available())"`
> 应输出 `2.7.0+cu128 True`；`import isaaclab; print(isaaclab.__version__)` 应为 `0.47.2`。

```bat
cd C:\isaac-lab
.\isaaclab.bat -p scripts\tutorials\00_sim\create_empty.py
```

能弹出一个空场景窗口 = Lab 装好了。再跑个自带 G1 的演示：

```bat
.\isaaclab.bat -p scripts\demos\bipeds.py
```

### 3. 回放我们导出的动捕动画（`replay_trajectory_isaaclab.py`）

**目录约定（git 模式，脚本与数据分离）**：

| 位置 | 放什么 | 谁管理 |
|---|---|---|
| 仓库 clone（示例 `D:\BlenderPro\g1-rig-pipeline`） | **全部脚本**（isaac/、scripts/…） | `git pull` 更新 |
| `D:\BlenderPro\G1\` | **你的数据**：URDF、.blend、meta json、npy/csv 输出、g1_dfq.usd | 手动/脚本产出，不进 git |

脚本里的 `CONFIG` 默认都指向 `D:\BlenderPro\G1\` 的数据文件，仓库 clone 到
哪里都能直接跑。首次 clone（分支名照抄）：

```powershell
git clone -b arena/01a0b00d-g1-rig-pipeline https://github.com/AaronChenD/g1-rig-pipeline.git D:\BlenderPro\g1-rig-pipeline
```

更新只要 `git -C D:\BlenderPro\g1-rig-pipeline pull`（或 cd 进去 `git pull`）。
**不要**把脚本复制到 `G1\` 目录——那会产生无法同步的旧拷贝。

最省事——用仓库里的 `replay_g1.bat`（任何终端甚至资源管理器地址栏都能跑；
它自动调用 `C:\isaac-lab\isaaclab.bat`，装在别处就先 `set ISAACLAB_BAT=...`）：

```bat
D:\BlenderPro\g1-rig-pipeline\isaac\replay_g1.bat D:\BlenderPro\G1\myanim.npy --loop
D:\BlenderPro\g1-rig-pipeline\isaac\replay_g1.bat D:\BlenderPro\G1\myanim.npy --usd D:\BlenderPro\G1\g1_dfq.usd --loop
```

或者手动完整命令（`.bat` 内容就是这个）：

```bat
cd C:\isaac-lab
.\isaaclab.bat -p D:\BlenderPro\g1-rig-pipeline\isaac\replay_trajectory_isaaclab.py --npy D:\BlenderPro\G1\g1_anim.npy
```

- **默认预览模式**：关重力、逐帧写关节状态+根位姿 → 精确运动学回放（不需要平衡控制器，动捕长什么样机器人就摆什么样）；
- `--physics`：物理模式（重力 + 关节目标），机器人可能会倒——那正是之后 pink-IK/RL 要解决的部分；
- `--loop` 循环、`--speed 0.5` 慢放、`--headless --video` 无窗口录像；
- **关节按名字匹配**：内置 G1 是 29dof 身体版，我们数据里的 22 个手指关节会自动跳过（有提示）；想连手指回放，用 Isaac Sim 的 URDF Importer 把 `g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf` 转成 USD，再 `--usd 转换结果.usd` → 53 关节全匹配（URDF 导入时关节名会保留）。

数据坐标系无需转换：我们的根轨迹是 URDF Z-up/米，Isaac Sim 世界同样是 Z-up/米。

脚本已按 **Isaac Lab 2.3.0 源码逐 API 校对**（`ArticulationInitStateCfg` 在 2.3+
是 `ArticulationCfg` 的嵌套类；根位姿写入要 (N,7) 张量、四元数 wxyz；内置 G1 配置名
为 `G1_CFG`；物理模式走 `set_joint_position_target`+`write_data_to_sim` 的 PD 目标），
同时保留旧版 Isaac Lab 的兼容分支。

### 3.5 bipeds.py / 回放崩溃: "GetPrimAtPath(Stage, NoneType)"

Isaac Sim 5.x 的内置资产 (地面 / 机器人 USD) 默认**按需从 NVIDIA 云端下载**
(Carb 设置 `/persistent/isaac/asset_root/cloud`)。该地址为空或网络不可达时,
地面 USD 拿到空引用 → 找不到 "Plane" 子 prim → `bind_physics_material(None)`
崩溃。**不是安装坏了**, 是资产没到本地。

排查 (本目录 `check_assets.py`):

```powershell
.\isaaclab.bat -p D:\BlenderPro\g1-rig-pipeline\isaac\check_assets.py
```

- 输出 `状态 2 = 云端可达` → 直接重试 bipeds (首次会下载, 慢是正常的;
  下载一次后进本地缓存, 以后离线也能用);
- 输出 `asset_root/cloud = None/空` → 按脚本打印的提示修复 (联网开一次 Sim UI
  或手动 set 回默认云端地址);
- 输出 `状态 0 = 不可达` (常见于部分网络环境) → ① 换网/代理后重试;
  ② 或完全绕开云端: 用 Sim 的 URDF Importer 把本地 URDF 转成 USD, 回放时
  `--usd` 指定它——**回放脚本已内置本地地面兜底** (云端失败自动换 Cuboid 地面),
  这条路完全离线可用。

### 3.7 回放排障速查

- **机器人陷进地面 / 悬在半空**：根轨迹是"绑定相对"语义（绑定时=(0,0,0)），
  回放端自动叠加初始摆放（2025-09-19 修复；旧回放脚本会把它当世界坐标直接写，
  导致 pelvis 被按到 z=0 陷地 0.79 m）。若仍异常，检查 npy 的 root_pos——站立帧
  的 z 应≈0，蹲下为负；
- **某些动作放不出来（如弯腰）**：内置 G1 只有 37 关节，**没有 waist_pitch /
  waist_roll / 手腕 / 手指**（腰只有 torso_joint=偏航）。K 在躯干上的弯腰会全落进
  `waist_pitch_joint`——内置机器人无此关节，直接被跳过。解法：URDF Importer 转
  DFQ 版 USD 后 `--usd` 回放（53 关节全匹配，见 3.6）；
- 查自己 npy 里哪些关节真的动了（Isaac 的 python 带 numpy）：

```powershell
& "C:\isaac-lab\_isaac_sim\python.bat" -c "import numpy as np; d = np.load(r'D:\BlenderPro\G1\g1_anim.npy'); [print(n, round(float(d[n].min()),3), round(float(d[n].max()),3)) for n in d.dtype.names if n not in ('time','root_pos','root_quat_wxyz') and np.abs(d[n]).max() > 0.05]"
```

### 4. 下一步学习路线

- 官方教程（就在本地仓库）：`C:\isaac-lab\scripts\tutorials\` 从 `00_sim` 往后按序看；
- RL 训练一个任务试试水：`isaaclab.bat -p scripts\reinforcement_learning\rsl_rl\train.py --task=Isaac-Ant-v0 --headless`；
- **pink-IK**：pink 是独立的 Python 库（基于 pinocchio，`isaaclab.bat -m pip install pink` 装），典型用法是离线或在控制循环里解 IK 生成关节目标——我们导出的根轨迹+关节角正是它的参考输入/初值来源；pink 解出的目标序列同样可以用回放脚本预览。

### 5. 完整链路（推荐工作流）

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
