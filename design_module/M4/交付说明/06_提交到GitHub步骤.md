# 06 · 把 M4 成果提交到 GitHub（按组长要求：先 fork → 提交到 develop）

> **组长原话**：「现在**已经有 github 仓库**，需要用这个来做，**先 fork**，然后做好后
> 自己 pull 到 **develop 分支**」。
>
> 这份是**照抄就能做**的分步操作。

---

## 0. ★ 当前进度（2026-10-10 实测）

| 步骤 | 状态 |
|---|---|
| 仓库地址 | `https://github.com/YYSY-FISH7/ai4.git`（组长原仓库 `cranky269/ai4`） |
| **Fork** | ✅ 已完成（`YYSY-FISH7/ai4`，Public） |
| **Clone** | ✅ 已完成（宿主机 `_github/ai4`） |
| **分支** | ✅ 已建 `develop`（从 `master`） |
| **放文件** | ✅ 已完成：**41 项**（`design_module/M4/交付说明/` 13 + 代码/脚本/测试 26 + `install_m4_deps.sh` + `.gitignore`/`requirements.txt` 两处改动） |
| **commit** | ✅ 已创建 `5e71a07`（41 files changed, 8340 insertions） |
| **push** | ⏳ **只剩这一步**（需要你自己的 GitHub 凭据） |

### ⚠️ 两个只在 Windows 上才会遇到的坑（**已处理，但你要知道**）

1. **仓库里 `lib/*.py` 是 git 软链**（mode `120000` → `../../../../code/xxx.py`）。
   Windows 建不出真软链，所以 `git status` 一度显示这两个文件"已删除"——
   **这时候敲 `git add -A` 会把软链从仓库里删掉！**
   已用 `git update-index --skip-worktree` 保护，现在 `git add -A` 是安全的。
   > 到 Linux（虚拟机）clone 下来后，git 会自动把这两个变回真软链。

2. **`core.autocrlf=true`**：git 会把 CRLF 存成 LF。
   所以仓库里组长的文件（LF）和你本地的（CRLF）**"看起来不同"，其实只是换行符**。
   **不要去"修"它们** —— 那会产生 63 个文件的纯换行符噪音 diff。

### 仓库里**不要碰**的东西（组长已重排过目录）

- 组长把文档移到了 **`skill/`**（`一些暂时设定.md`、`ssh连接.md`、`代码书写原则.md` 等）。
  工程根目录下同名的旧文件是**重复品，不要提交**。
- 组长**已有完善的 `.gitignore`**，我们是**追加**（`lib/m4_deps/`、`*.pt`），不是覆盖。

### 你现在只需做这一步

```bash
cd <你的仓库目录>          # 例如宿主机：...\uav_intercept\_github\ai4
git push -u origin develop
```

推的时候会问用户名/密码：**用户名填你的 GitHub 账号，密码处粘贴 Personal Access Token**
（网页：Settings → Developer settings → Personal access tokens → Generate new token，勾 `repo`）。

> ⚠️ **先别急着 push** —— 需要先跟组长确认一句「develop 分支到底在哪」
> （他的仓库目前**只有 `master` 一个分支**，见 §3 的说明）。

---

## 0. 开始前需要两样东西

| 需要 | 从哪来 |
|---|---|
| **仓库地址**（形如 `https://github.com/某人/uav_intercept.git`） | **群里问组长要**（我这边没有） |
| **GitHub 账号** | 你自己的 |

> **别用密码推送**。GitHub 现在要求用 **Personal Access Token**（设置 → Developer settings
> → Personal access tokens → 生成，勾 `repo` 权限），推送时**密码处粘贴这个 token**。

---

## 1. Fork（在网页上点一下）

1. 打开仓库地址
2. 右上角点 **Fork**
3. 目标账号选你自己 → **Create fork**

做完后你会有自己的副本：`https://github.com/<你的账号>/<仓库名>.git`

---

## 2. Clone 你自己的 fork

```bash
mkdir -p ~/github && cd ~/github
git clone https://github.com/<你的账号>/<仓库名>.git
cd <仓库名>
```

---

## 3. 建 develop 分支（组长的目标分支）

```bash
# 看有没有远程 develop
git branch -r

# 有：切过去（基于远程 develop）
git checkout -b develop origin/develop
# 没有：直接从主干建一个
git checkout -b develop
```

> 如果第 4 步提交时组长要求"提到我仓库的 develop"，
> 那通常是 **向原仓库发 Pull Request（PR）**：fork 里 push → 网页上点
> **Compare & pull request** → base 选原仓库的 `develop`。
> **不确定就直接问组长一句**：「是直接 push 你的 develop，还是我发 PR？」

---

## 4. 把 M4 的成果复制进去（**只复制 M4 新增的，别覆盖组长的文件**）

```bash
SRC=/home/demo/桌面/ai4          # 虚拟机里已跑通的副本
DST=~/github/<仓库名>            # 刚 clone 下来的
```

### 4.1 一次性复制（`-a` 保留软链，很重要）

```bash
cd "$DST"

# ① 交付说明（全是新增）
cp -a "$SRC/design_module/M4/交付说明" design_module/M4/

# ② 依赖安装脚本 + gitignore
cp -a "$SRC/install_m4_deps.sh" .
cp -a "$SRC/.gitignore" .

# ③ 模块 4/5 的纯逻辑（code/ 与 lib/ 之间有软链，必须 -a）
cp -a "$SRC/code/detector.py" "$SRC/code/visual_tracker.py" code/
cp -a "$SRC/catkin_ws/src/uav_tracking/lib/detector.py" \
      "$SRC/catkin_ws/src/uav_tracking/lib/visual_tracker.py" \
      catkin_ws/src/uav_tracking/lib/

# ④ 三个节点壳
cp -a "$SRC/catkin_ws/src/uav_tracking/scripts/camera_node.py" \
      "$SRC/catkin_ws/src/uav_tracking/scripts/detector_node.py" \
      "$SRC/catkin_ws/src/uav_tracking/scripts/visual_tracker_node.py" \
      catkin_ws/src/uav_tracking/scripts/

# ⑤ 测试（三个目录全是新增）
mkdir -p test
cp -a "$SRC/test/camera" "$SRC/test/detector" "$SRC/test/visual_tracker" test/
```

> **完整新增文件清单**见 `01_新增文件说明.md`（共 26 个代码/测试文件 + 文档）。
> 若上面漏了什么，对着那份清单补。

### 4.2 关于 `requirements.txt`

M4 在该文件**只追加了注释**（规则允许）。若仓库里已有别人的改动，**建议不合并这一处**，
或在 PR 里说明。详见 `02_修改说明.md`。

---

## 5. ⚠️ 提交前必查：确认 1.1 GB 的依赖没被带上

```bash
git status --short | head -50
```

**必须满足三点：**

| 检查 | 期望 |
|---|---|
| 出现 `catkin_ws/src/uav_tracking/lib/m4_deps/...` | ❌ **绝对不能有！** 说明 `.gitignore` 没生效 |
| 出现 `catkin_ws/build/` 或 `devel/` | ❌ 不该有，同上 |
| 出现 **组长原有文件被修改** | ⚠️ **停下来核对**（`02_修改说明.md` 记录：除 `requirements.txt` 外应无改动） |

补查一条：

```bash
# 已暂存文件里是否有大文件（不该有）
git ls-files | grep -c m4_deps          # 期望输出 0
```

若 `m4_deps` 已被跟踪：

```bash
git rm -r --cached catkin_ws/src/uav_tracking/lib/m4_deps
```

---

## 6. 提交并推送

```bash
git add -A
git status --short | wc -l          # 看一下要提交多少个文件（几十个属正常）

git commit -m "M4 视觉感知链：检测(detector) + 跟踪(visual_tracker) + 相机驱动 + 测试与交付说明

- 模块4 YOLO 检测（/detections）、模块5 BoT-SORT 跟踪（/visual/tracks）
- 视觉依赖按组长要求隔离在 lib/m4_deps（系统 numpy 1.17.4 未动），提供 install_m4_deps.sh
- 单元测试 234 项、三个 E2E 全 PASS；虚拟机内实跑通过（含 VISUAL_LOCK）
- 对组长原有文件改动：0（requirements.txt 仅追加注释）"

git push -u origin develop
```

---

## 7. 推完自检

```bash
git log --oneline -3
git ls-files | wc -l                # 总文件数
git ls-files | grep m4_deps         # 必须为空
```

到网页上看你的 fork → `develop` 分支，确认文件都在。

---

## 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| `Authentication failed` | 密码处要用 **Personal Access Token**，不是账号密码 |
| `Updates were rejected` | 远程有新提交：先 `git pull --rebase origin develop` 再 push |
| `git status` 里有几千个文件 | `.gitignore` 没生效（注意别把 `build/`、`m4_deps/` 加进去） |
| 提示 symlink / 文件变成普通文件 | 复制时忘了 `-a`，重来一次（`code/` 与 `lib/` 之间有软链） |
| 提交太大 / 很慢 | 八成把 `m4_deps` 或模型带上了，回到第 5 步 |

---

## 一句话

> **只加自己的文件、别覆盖组长的、确认 `m4_deps` 没被带上** —— 这三点做到了，
> 提交就是安全的。第 5 步的 `git status --short` **一定要看一遍**。
