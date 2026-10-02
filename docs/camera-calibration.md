# 前后摄像头标定操作

本工具分别求前、后相机的内参及畸变（OpenCV 针孔 / ROS `plumb_bob` 五参数模型），不求两个相机之间或相机到底盘的外参。不是双目立体标定。两路样本不可混用；鱼眼镜头若边缘持续无法校正，需要单独评估鱼眼模型。

已与操作者确认：棋盘为 **9×7 格、8×6 内角点、方格边长 25 mm**，100 mm 打印校验线实测正确，纸已固定在平整硬板上。前相机 `/usb_cam_2/image`，后相机 `/usb_cam_1/image`。建议每路采集 25–35 张，最低 20 张。

源码：`src/smartcar_calibration/`；编译结果：`output/`；采集原图和候选参数：`~/smartcar-data/calibration/`。运行时只订阅 ROS 图像，不发布底盘命令，不自动修改驱动参数。

## 1. 在 VS Code 的 Ubuntu 远程终端准备

```bash
cd ~/projects/smart-car
source scripts/setup_robot.sh
rostopic list | grep -E '^/usb_cam_[12]/(image|camera_info)$'
rosnode list
```

若模块尚未构建，执行 `./scripts/build.sh`，然后再次 `source scripts/setup_robot.sh`。当前环境是 Ubuntu 18.04 / ROS Melodic，使用系统 **Python 2** 的 rospy、cv_bridge 和 OpenCV 3.2；不要改用普通 Python 3 虚拟环境启动。

已运行的相机继续使用。只有确认对应驱动没有运行、设备没有被其他进程占用时，才启动缺少的一路。不要对已运行的 `/usb_cam_2` 重复 roslaunch，否则 ROS 会关闭同名节点。

```bash
# 仅后相机缺失时，在另一终端运行并保持该终端打开：
source ~/projects/smart-car/scripts/setup_robot.sh
roslaunch smartcar_calibration cameras.launch rear:=true
# 两路都未运行时，改用 front:=true rear:=true；仅前路缺失则 front:=true。
```

该 launch 不启动底盘、雷达或 image_view。默认 640×480 / YUYV，以 USB 物理路径区分相机。不要依赖可能变化的 `/dev/video0` 序号。前 USB 路径为 `...usb-0:2.3:1.0-video-index0`，后为 `...usb-0:2.4:1.0-video-index0`。USB 插口变动可通过 `front_device:=...`、`rear_device:=...` 指定。

标定和比赛使用相同分辨率、裁剪、焦距及镜头固定方式。驱动 launch 设置 `autofocus=false`，但硬件是否支持、系统是否有 v4l2-ctl 均需现场核实；固定焦镜头无需设置。如果为自动对焦镜头，先固定对焦，再从空会话采集，避免一组样本内焦距变化。改变镜头/焦距或裁剪后重新标定。

## 2. 启动控制页面

```bash
cd ~/projects/smart-car
source scripts/setup_robot.sh
rosrun smartcar_calibration calibration_web.py
```

终端会输出本次 **Session directory**，请记下。默认自动创建不同目录，避免覆盖已有采集。窗口要保持运行。

在 VS Code 底部 **PORTS / 端口**中转发远程端口 **8766**，然后在电脑的外部浏览器打开 `http://127.0.0.1:8766`。页面限制 iframe 嵌入，请使用端口列表中的“在浏览器中打开”，不使用 Simple Browser。现有预览页面 8765 可继续使用。若本机端口被占用，使用 VS Code 给出的本地转发端口。

## 3. 逐路采集

1. 选择“前摄像头 cam_2”。遮挡一下对应镜头核对画面，再拿开遮挡。
2. 把整块棋盘放入画面，等角点显示、状态提示“棋盘已识别”。避免反光、运动模糊和画面边缘截断。将棋盘放平整，采图时短暂停稳。
3. 点击 **采集一张**，或焦点不在下拉框/按钮时按 **空格**。连续点击相同姿态会被拒绝。工具不自动采图，操作者自己决定时机。
4. 让棋盘中心分别出现在左、右、上、下、四角；再改变距离，使棋盘大小不同，并向上下、左右倾斜约 15–35°。不要一直正对相机。建议 25–35 张清晰图，尽量让角点覆盖边缘。
5. 参考页面覆盖提示：中心水平/垂直移动跨度均 ≥25%，所有角点覆盖跨度均 ≥60%，远近尺寸比 ≥1.30。这是基础检查，不代表标定精度已经合格。
6. 最新一张不满意时点击 **撤销上一张**，它从有效样本清单移除，原 PNG 留作审计。想全面重来可退出服务，用新的默认会话启动。
7. 点击 **计算并保存候选参数**。数据不够或覆盖不足会拒绝计算；高误差等情况会保存可检查的候选结果并明确标为需补采。
8. 切换“后摄像头 cam_1”，重复以上操作。两路的样本、参数和报告分开存放。

切换相机不会丢失另一相机数据。图像超过 2 秒未更新会禁用采图；会话内分辨率变化会拒绝采样。计算过程中暂时禁止更改样本。

## 4. 检查结果

每次计算生成一个新的目录：

```text
~/smartcar-data/calibration/session-日期-编号/
  front/                       # 后相机对应 rear/
    image-*.png                # 未绘制角点的原始图片
    dataset.json               # 当前有效样本、角点、采集时间、棋盘参数
    results/日期-编号/
      camera.yaml              # ROS camera_info_manager 兼容内参文件
      report.json              # 总 RMS、各图误差、覆盖范围、检查结果、YAML 校验和
      dataset.json             # 本次计算使用的样本快照
```

自动检查阈值：总 RMS ≤1.0 px、各图 RMS ≤2.0 px、估计的棋盘法向跨度 ≥10°，且主点/焦距通过基本范围检查。**这些是训练样本的重投影误差，不是独立精度证明。** 数值越低通常越好，但必须结合实物检查。不要为压低误差只留下类似姿态。

点击 **切换原图 / 去畸变**，拿新的棋盘姿态、门框或直边物体验证边缘是否变直、中心和边缘是否一致。去畸变有黑边属于正常现象。如果明显弯曲、拉伸或误差大，先检查板面、运动模糊、内角点数和焦距，再补充不同位置/倾斜样本重新计算。每次采图或撤销会使当前页面的旧结果失效，磁盘上的历史结果保留。

本工具不自动删除高误差图。如果报告提示某个旧样本异常，可让 Codex 基于原图审查后建立干净的新会话；不要未经记录手改旧结果文件。

## 交付时的验证记录（2026-09-25）

- Ubuntu 小车上完成全工作空间构建，新增模块 5 项测试通过，0 失败；包含合成内参恢复、棋盘识别、数据恢复/隔离及无效样本拒绝。
- `rosrun ... --help` 与相机 launch 的 `--nodes` 解析通过；页面 JavaScript 语法检查通过。
- 在临时端口 18766 实测 HTTP 页面与现有前相机 JPEG，验证相机切换、错误 token/Host 拦截及前后选择不一致时拒绝采集。测试服务已退出，测试没有采集样本。
- 后相机当时没有发布图像，页面正确报告无图像；尚未进行后相机实拍、真实棋盘多姿态采集或参数加载。合成测试不代表真实标定已经完成。

## 5. 恢复与退出

Ctrl+C 只停止标定页面，独立启动的相机和预览服务仍运行。采集样本逐张保存，可以恢复：

```bash
rosrun smartcar_calibration calibration_web.py \
  --session-dir /home/smartcar/smartcar-data/calibration/实际会话目录
```

恢复后重新计算即可重建结果及去畸变预览。棋盘尺寸或话题若与已保存会话不一致，程序拒绝复用。尺寸有变化时新建会话，可传入 `--square-mm 实测值`、`--cols 8 --rows 6`。

## 6. 加载验证过的参数

候选 YAML 不会自动应用。先确认两路 `report.json` 中 `automatic_checks_passed=true`，完成新画面验证，记录最后选定的绝对路径。操作者允许重启相机后，在原驱动终端 Ctrl+C 关闭相应相机（若旧 launch 会自动重启节点，必须停原 launch），确认同名节点及设备已释放，再在新的终端加载：

```bash
source ~/projects/smart-car/scripts/setup_robot.sh
roslaunch smartcar_calibration cameras.launch front:=true rear:=true \
  front_url:=file:///home/smartcar/smartcar-data/calibration/实际会话/front/results/实际结果/camera.yaml \
  rear_url:=file:///home/smartcar/smartcar-data/calibration/实际会话/rear/results/实际结果/camera.yaml
```

随后分别检查：

```bash
rostopic echo -n 1 /usb_cam_2/camera_info
rostopic echo -n 1 /usb_cam_1/camera_info
```

确认 width/height、D/K/R/P 与对应 YAML 一致且 K[0] 非零；前后 camera_name 分别是 `smartcar_front` / `smartcar_rear`，光学 frame 分别为 `front_camera_optical` / `rear_camera_optical`。原有 launch 未传入这些 URL 时不会自动读取本会话参数；以后使用的正式入口也应显式加载选定 YAML。只看到参数发布不等于图像已去畸变，后续算法必须使用对应参数做校正。此模块没有建立相机与底盘之间的 TF 外参。

参考：[OpenCV 相机标定接口](https://docs.opencv.org/3.4.7/d9/d0c/group__calib3d.html)。
