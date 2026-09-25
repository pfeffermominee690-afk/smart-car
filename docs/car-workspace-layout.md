# 小车开发目录与 GitHub 同步

## 唯一开发入口

`/home/smartcar/projects/smart-car` 是车上的 Git 仓库和 Catkin 工作空间。

- `src/<包名>/`：所有实际模块源码，模块相关 launch、配置和测试也放在包中。
- `scripts/`：构建、环境加载、Git 同步和手动试验入口。
- `output/build/`：CMake 缓存、目标文件及测试结果。
- `output/devel/`：开发环境、编译后的可执行文件、共享库和消息生成代码。
- `output/install/`：执行 install 目标时生成的安装结果。

`output/` 在车上链接到 `~/smartcar-data/catkin/unified`，Git 明确忽略整个输出目录。在其他电脑上构建时，脚本直接创建本地 output 目录，无需建立链接。

录像、日志、模型和备份在 `~/smartcar-data/` 中，不能放进源码目录提交。Python 临时字节码即使产生在源码目录也会被 Git 忽略。

## 开发流程

车上 Git 2.17 使用 `checkout` 命令。没有程序使用待切换的代码、且当前修改已保存后再切换分支。

```bash
cd ~/projects/smart-car
./scripts/car-git.sh status
# 首次目录迁移 PR 合并前，从 refactor/unified-src-layout 开始开发。
# 合并后再使用以下 main 起点：
git fetch origin
git checkout main
git pull --ff-only origin main
git checkout -b feat/your-module

./scripts/build.sh
source scripts/setup_robot.sh
./scripts/build.sh run_tests_lane_follow
catkin_test_results output/build/test_results

git status --short
git add src/实际修改的包
git commit -m "feat(module): 说明修改"
./scripts/car-git.sh push
```

到 GitHub 创建目标为 main 的 PR，等待负责人确认。`car-git.sh` 不自动暂存、提交或合并，不推送 main；存在未提交修改时停止同步，拉取只允许快进。

同一开发分支的更新使用 `./scripts/car-git.sh pull`。切换分支后重新构建，再加载环境，不要继续使用上个分支的生成结果。

## 模块入口

- 底盘：`src/base_controller`；键盘入口 `scripts/keyboard_drive.sh`。
- 相机：`src/usb_cam`；采用原相机工作空间中实际使用的版本，保留各 launch 文件。
- 雷达：`src/ls01b_v2`。
- 循线：`src/lane_follow`，Python 算法在 `python/lane_follow/`，节点在 `scripts/`，配置在 `config/`，测试在 `tests/`。
- 现有历史视觉：`src/smartcar`，保留其 ROS 包名和消息类型，不作为完整比赛能力已经验证的依据。

巡线节点通过 ROS 查找配置和雷达节点，不再写死旧工作空间的 devel 路径。仍默认观察，显式 `--execute` 才允许驱动，原有 3 秒上限和停车条件保持不变。

```bash
source ~/projects/smart-car/scripts/setup_robot.sh
rosrun lane_follow lane_follow_node.py --help
# 实际观察会订阅传感器，并可能启动雷达；本次迁移验证不执行此命令：
roslaunch lane_follow observe.launch
```

## 认证与身份

小车使用本仓库专用 deploy key，经 GitHub SSH 443 连接。私钥位于 `~/.ssh/smart_car_github`，不进入仓库；撤销访问可在 Settings → Deploy keys 删除 `smart-car-nano-20260925`。

提交前设置自己的 `git config user.name` 和 `git config user.email`。deploy key 标识共享设备，个人电脑继续使用各自 GitHub 账户。

## 旧路径与回退

旧路径仅用于兼容已有命令，日常修改源码一律进入 `~/projects/smart-car/src`。构建验证成功后，旧环境入口改为加载统一 output/devel；旧新车工作空间通过仓库外的兼容目录指向新源码与构建输出。v1 循线保留为归档，不放进当前工作空间。

2026-09-25 初次完整备份位于 `~/smartcar-data/backups/workspace-20260925-152948/original-workspaces.tar`。本次迁移另保存原 vehicle 目录、原 .bashrc 和旧路径链接记录。回退前先保存新修改并停止使用相关代码的程序，不能直接覆盖正在使用的工作目录。详见 [迁移记录](unified-src-migration.md)。
