# 小车开发目录与 GitHub 同步

整理日期：2026-09-25；设备：`smartcar-desktop`。

## 日常入口

主 Git 仓库：`/home/smartcar/projects/smart-car`。

```text
~/projects/smart-car/
├── workspaces/
│   ├── control  -> ../vehicle/newcar_ws/znxc/config/teleop
│   ├── cameras  -> ../vehicle/newcar_ws/znxc/config/cam_test_ws
│   └── lidar    -> ../vehicle/newcar_ws/znxc/config/leishen_ws
├── modules/
│   ├── lane_follow    -> ../vehicle/lane_follow_20260923_v2
│   └── lane_follow_v1 -> ../vehicle/lane_follow_20260923
├── vehicle/            # 保留原源码目录，以上入口指向同一份文件
├── scripts/car-git.sh  # Git 状态、拉取、推送入口
└── docs/

~/smartcar-data/
├── ros/                # 三个工作空间的 build、devel
├── runs/               # 两个版本循迹的试跑数据
├── archives/           # 原有源码压缩包
└── backups/            # 完整备份、嵌套 Git 历史和移动记录
```

修改 `workspaces/control/src/`、`workspaces/cameras/src/`、`workspaces/lidar/src/` 和 `modules/lane_follow/` 即是在修改 `vehicle/` 中受 Git 管理的文件。Git 状态显示真实的 `vehicle/...` 路径，提交时使用这些路径；实际源码只有一个工作副本。

`vehicle/newcar_ws/znxc/config origal/` 和其他重复工作空间是历史副本，保留供对照；日常开发使用上述明确入口。

## 原有入口

旧路径 `/home/smartcar/newcar_ws`、`/home/smartcar/lane_follow_20260923`、`/home/smartcar/lane_follow_20260923_v2` 已改为指向仓库对应目录的符号链接。

原 `.bashrc` 和 `setup_robot.sh` 不需改写。ROS 缓存中的绝对路径仍通过兼容链接访问原文件，构建目录和试跑目录也在原位置保留链接。没有删掉或清空历史数据。

构建数据、录像和 SSH 私钥不进入 Git。Git 跟踪的八个 Catkin `CMakeLists.txt` 链接和五个开发入口链接应保留链接类型，不能替换为普通文本后提交。Windows 未启用符号链接时，应直接编辑 `vehicle/` 中的真实文件。

## GitHub 连接

车上配置了专用于本仓库的 SSH deploy key，私钥在 `~/.ssh/smart_car_github`，没有把个人 GitHub Token 放到小车。该凭据可推送本仓库开发分支，`main` 仍受审核规则保护。

连接使用 GitHub SSH 的 443 端口，设置保存在本仓库的 `core.sshCommand`，不会改变其他 SSH 连接。主机公钥从 GitHub 官方 API 获取并启用严格校验。

如需撤销车上访问权限，在仓库 **Settings → Deploy keys** 删除 `smart-car-nano-20260925`。共享小车的 deploy key 标识设备；提交作者仍由 `git config user.name` 和 `git config user.email` 决定，队员提交前应设置自己的真实身份。个人电脑继续使用各自 GitHub 账户。

## 开发与同步

**基线 PR 合并前，源码在 `snapshot/car-20260925`，`main` 仍只有骨架。** 此时从现有源码分支创建功能分支，不要切到缺少源码的 `main` 后运行车辆。

基线合并后，按以下流程开发（车上 Git 2.17 使用 `checkout`）：

```bash
cd ~/projects/smart-car
./scripts/car-git.sh status
# 确保无未提交修改，并且没有程序正在使用待切换的代码
git fetch origin
git checkout main                     # 仅在基线 PR 已合并后
git pull --ff-only origin main
git checkout -b feat/your-module

# 修改代码，运行对应测试
git status --short
git add vehicle/实际修改的文件
git commit -m "feat(module): 说明本次修改"
./scripts/car-git.sh push
```

到 GitHub 发起目标为 `main` 的 PR，经负责人确认后合并。脚本不自动暂存、提交或合并，不直接推送 `main`；有未提交修改时停止，拉取只允许快进更新。

取得当前分支更新：`./scripts/car-git.sh pull`。新分支首次推送使用 `./scripts/car-git.sh push`，会建立 upstream；需要 upstream 后才能使用简化的 pull。

## 备份与验证

原三个工作目录完整归档在 `~/smartcar-data/backups/workspace-20260925-152948/original-workspaces.tar`，包含源码、构建结果、运行数据及嵌套 Git 历史。`operations.json` 记录移动和链接；`nested-git/` 单独保存原 Git 元数据。

恢复前先停止使用这些目录的程序，保留新增代码和数据，再根据操作记录反向恢复，或把完整备份解压到独立目录进行比对。不要覆盖正在使用的工作目录。

整理后核对原始文件校验值、旧路径、ROS 环境及包发现、循迹单元测试和 Git 同步；没有驾驶试跑，也未重新编译全部 ROS 包。
