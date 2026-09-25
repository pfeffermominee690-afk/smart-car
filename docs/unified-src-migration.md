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

迁移验证使用车上 Ubuntu 18.04 / ROS Melodic 原环境；结果在构建与检查完成后记录。相机预览保持运行，不启动新的驾驶程序。
