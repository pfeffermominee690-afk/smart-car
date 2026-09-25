# 车上源码快照

来源：`smartcar-desktop` 的 `/home/smartcar`，读取日期 2026-09-25。

| 本仓库目录 | 车上目录 | 内容 |
| --- | --- | --- |
| `newcar_ws/` | `/home/smartcar/newcar_ws/` | ROS 工作空间源码、启动脚本和配置 |
| `lane_follow_20260923/` | `/home/smartcar/lane_follow_20260923/` | 循迹原型 v1、配置和离线测试 |
| `lane_follow_20260923_v2/` | `/home/smartcar/lane_follow_20260923_v2/` | 循迹原型 v2、转向控制、配置和离线测试 |

## 从哪里开始看

- `newcar_ws/setup_robot.sh`：加载车上 ROS 环境及三个工作空间。
- `newcar_ws/znxc/config/teleop/src/`：底盘、遥控、激光处理及历史视觉程序。
- `newcar_ws/znxc/config/cam_test_ws/src/usb_cam/`：相机驱动。
- `newcar_ws/znxc/config/leishen_ws/src/ls01b_v2/`：雷达驱动。
- `lane_follow_20260923_v2/`：最近部署的循迹原型版本；测试通过不代表完整比赛功能已验证。

`znxc/config origal/` 和其他重复工作空间是车上已有的历史副本，本次保留以便追溯。`setup_robot.sh` 实际加载的是 `znxc/config/` 下的工作空间。不要把多份同名 ROS 包放进同一个构建空间。

## 使用边界

这是源码基线，不是系统镜像或可直接运行的安装包。`build/`、`devel/` 和运行数据未上传。ROS Melodic、Python 2、OpenCV、串口和相机等依赖仍由车上环境提供。

原脚本保留了 `/home/smartcar`、`$HOME/newcar_ws` 以及部分旧 `/home/nano` 路径，克隆仓库后不能假定直接运行就能工作。部署前需确认路径、构建对应工作空间并检查配置；本次没有改写或重新部署车上程序。

八个 `CMakeLists.txt` 是指向 `/opt/ros/melodic/share/catkin/cmake/toplevel.cmake` 的 Linux 符号链接。Git 中保留了链接类型；在 Windows 未启用符号链接时，它们显示为保存目标路径的文本文件，构建应在兼容 Linux 环境进行。

完整校验与测试记录见 [同步说明](../docs/car-source-20260925.md)。第三方包原有的许可证和版权声明均保留，本次没有给这些代码统一改换许可证。
