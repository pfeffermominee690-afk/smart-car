# 统一 src 布局迁移

原基线提交：`e69cf0d2d4f130728993f2ded4b8ed4b117569e0`。目标为一个实际的 src 源码树与一个独立 output 构建输出目录。

## 模块来源

| 新位置 | 旧来源（相对原 vehicle） |
| --- | --- |
| src/base_controller | newcar_ws/znxc/config/teleop/src/base_controller |
| src/ackermann_drive_teleop | newcar_ws/znxc/config/teleop/src/ackermann-drive-teleop-master |
| src/serial、laser_test、smartcar、teleop_twist_keyboard | newcar_ws/znxc/config/teleop/src/ 下同名包 |
| src/usb_cam | newcar_ws/znxc/config/cam_test_ws/src/usb_cam |
| src/ls01b_v2 | newcar_ws/znxc/config/leishen_ws/src/ls01b_v2 |
| src/lane_follow | lane_follow_20260923_v2 |

usb_cam 采用原环境中 rospack 实际找到的相机工作空间版本；teleop 中不同的 usb_cam_node.cpp 留在旧提交中，额外的 usb_cam-test.launch 并入唯一 usb_cam 包。历史 config origal、其他重复工作空间、循线 v1 均可从旧提交或车上备份恢复，不参与当前构建。

## 必要的适配

- 补齐 base_controller 对 serial / ackermann_msgs / geometry_msgs、smartcar 对 laser_test、雷达对 sensor_msgs 的依赖声明，保证统一工作空间构建顺序。
- 修正 laser_test 的 Catkin COMPONENTS 声明，补齐实际使用的依赖。
- 巡线封装为 ROS 包，调整 Python 模块导入、配置定位及雷达可执行文件查找；算法、标定值、停车阈值和 3 秒限制保持原值。
- 构建和环境脚本只使用 src 与 output，不依赖旧工作空间的编译缓存。
- 原键盘驾驶和直行试验脚本移入 src/base_controller/scripts；根目录 scripts 只保留构建、环境及 Git 工具。旧路径由仓库外的兼容入口保留。

## 验证记录

验证环境为车上 Ubuntu 18.04 / ROS Melodic，GNU 7.3，Python 2.7。

- 从新的 output 目录完整构建 9 个包成功，配置日志确认唯一 underlay 为 `/opt/ros/melodic`，未使用旧工作空间的编译缓存。
- `run_tests_lane_follow`：6 项测试通过，0 errors / failures / skipped。
- 9 个包的 `rospack find` 全部指向仓库 `src/`，巡线节点 `rosrun ... --help` 正常，`roslaunch --nodes lane_follow observe.launch` 解析成功。
- 17 个 package.xml 和 launch 文件通过 XML 解析；构建、环境、同步和底盘脚本通过 Bash 语法检查。
- 视觉算法、转向控制算法和标定 JSON 与原始 v2 文件逐字节一致。源码目录未发现 `.o`、`.so`、`.pyc` 或 CMakeCache.txt，Git 工作区干净。
- 从旧 v2 路径执行的 6 项测试和帮助入口也通过。兼容转发器避免旧 `lane_follow.py` 文件名遮蔽新的 `lane_follow` Python 包。

日志在车上 `~/smartcar-data/catkin/unified/` 下的 `build-first.log`、`build-final.log` 和 `test-lane.log`。构建中存在旧 laser_test/PCL 的 C++ 标准兼容警告，但编译成功；未据此修改算法。

本次没有执行设备启动/停止或车辆运动命令。最后的 ROS 节点检查中已无原 usb_cam_2 采集节点，未确认其退出原因，因此本次结果仅代表构建、包发现、入口解析及离线测试通过，不代表相机流或实车行驶验收。

## 车上部署与回退资料

当前开发目录为 `~/projects/smart-car`，部署分支为 `refactor/unified-src-layout`；在本 PR 合并前不要切换到仍采用旧布局的 main 后启动程序。

- 输出：`~/smartcar-data/catkin/unified`，仓库 output 是其入口。
- 本次备份：`~/smartcar-data/backups/unified-src-20260925-161205`，包含原 vehicle、原 .bashrc、旧链接目标和原提交号。
- 迁移状态：`~/smartcar-data/unified-src-migration.json`。
- 兼容入口：`~/smartcar-data/compat/unified-src`，旧 newcar_ws 与 v2 路径转到这里；v1 保留在备份中。
- 新终端的 .bashrc 加载仓库 `scripts/setup_robot.sh`，不再叠加旧的三个工作空间。

回退时先保留新 Git 改动并停止使用相关代码的程序，再依照备份中的提交和 home-links.json 恢复工作树及旧入口，恢复保存的 .bashrc。原始完整 tar 备份仍保留，本次没有删除旧构建数据、录像或模型。
