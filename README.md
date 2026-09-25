# 智能车比赛开发仓库

团队统一使用一个 ROS Melodic / Catkin 工作空间。**所有模块源码在 `src/`，全部编译产物在 `output/`，运行数据在仓库外。**

## 目录

```text
smart-car/
├── src/
│   ├── base_controller/          # STM32 底盘控制
│   ├── ackermann_drive_teleop/   # Ackermann 键盘/手柄遥控
│   ├── usb_cam/                  # 双摄像头驱动
│   ├── ls01b_v2/                 # 雷达驱动
│   ├── serial/                   # 串口通信库
│   ├── laser_test/               # 激光处理及消息
│   ├── smartcar/                 # 现有历史视觉程序及消息
│   ├── teleop_twist_keyboard/    # 通用 Twist 遥控工具
│   └── lane_follow/              # 当前循线：节点、算法、配置、测试
├── scripts/                      # 构建、环境与 Git 辅助脚本
├── docs/                         # 协作、部署和验证记录
└── output/                       # build/、devel/、install/，不提交 Git
```

各包内部的配置、launch 文件和测试与模块放在一起。仓库不再包含 `vehicle/`、`workspaces/`、`modules/` 等重复层级，源码目录是真实文件夹，在 Windows 和 Ubuntu 上都能直接浏览。

## 构建与验证

在已安装项目依赖的 Ubuntu 18.04 / ROS Melodic 环境中：

```bash
cd ~/projects/smart-car
./scripts/build.sh
source scripts/setup_robot.sh
./scripts/build.sh run_tests_lane_follow
catkin_test_results output/build/test_results
rosrun lane_follow lane_follow_node.py --help
```

构建不会启动相机、雷达或车辆控制。`src/lane_follow/launch/observe.launch` 是限时观察入口，不发送行驶命令；实际使用前需确认传感器配置。驱动和历史业务代码的可运行范围见各模块及 [车端指南](docs/car-workspace-layout.md)。

## 团队协作

负责人及指定审核人：[@pfeffermominee690-afk](https://github.com/pfeffermominee690-afk)。队友从 `main` 新建功能分支，在 `src/模块名/` 中开发，提交测试证据后发起 PR。

合并到 `main` 必须经过负责人 Code Owner 审核；新提交使旧批准失效，讨论须解决，禁止强制推送和删除主分支。GitHub 不允许 PR 作者批准自己的 PR，负责人发起的 PR 需要其明确确认后按管理员流程处理，并恢复审核保护。

```bash
./scripts/car-git.sh status
# 检查、测试并 git commit 后
./scripts/car-git.sh push
```

详细步骤见 [CONTRIBUTING.md](CONTRIBUTING.md)。

## 数据与历史

车上 `output` 指向 `~/smartcar-data/catkin/unified`，包含可重新生成的编译结果。录像、模型和备份分别保存在 `~/smartcar-data/runs`、`models`、`backups`，不上传 Git。

原始源码基线保存在 Git 提交 `e69cf0d` 和车上完整备份中；旧版循线和重复工作空间不再放进当前构建树。原始下载校验清单为历史记录，不表示重构后的路径或内容仍与旧快照相同。参见 [迁移记录](docs/unified-src-migration.md)。
