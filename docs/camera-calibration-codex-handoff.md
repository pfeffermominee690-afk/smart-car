# 给 Ubuntu / VS Code Codex 的交接提示词

复制以下内容到通过 SSH 连接小车的 VS Code Codex：

```text
请协助我完成智能车前、后两个摄像头的独立内参及畸变标定。

环境：小车 192.168.43.30，Ubuntu 18.04 / ROS Melodic / 系统 Python 2 / OpenCV 3.2。
仓库 /home/smartcar/projects/smart-car；本次代码分支 feat/camera-calibration。
先读取 docs/camera-calibration.md、src/smartcar_calibration/ 和 git status，保留其他人未提交的改动，不重置或覆盖工作区。
所有源码在 src/，编译结果在 output/，原始样本和结果在 /home/smartcar/smartcar-data/calibration/。

已确认：9×7 格棋盘，8×6 内角点，25 mm 方格，100 mm 校验线实测正确，纸已固定平整。
前摄像头 /usb_cam_2/image，后摄像头 /usb_cam_1/image，通常 640×480。
前 USB 固定路径包含 0:2.3，后包含 0:2.4。现场还请用遮挡镜头的办法核对。

请按文档检查 ROS 话题、设备占用及模块是否构建，只启动缺少的相机驱动。
不要重复启动已有同名相机节点；现有 8765 预览服务可继续运行。
启动 rosrun smartcar_calibration calibration_web.py，告诉我会话目录，并协助在 VS Code 转发 8766 端口打开控制页。
只做相机标定，不启动循线、底盘运动或雷达，不发布运动指令。

请一步步指导我移动棋盘并自己点击采集：先前相机再后相机，各收集 25–35 张清晰、多位置、多距离、有倾角的图。
每一阶段检查实际页面状态和样本报告；不能用现成棋盘照片或合成测试结果冒充实测标定。
如果图像未更新、识别失败或重复姿态被拒绝，请根据日志诊断。固定分辨率和焦距，不混用两路样本。
计算后读取两路 report.json，检查总误差、逐图误差、覆盖和姿态；指导我用新画面查看去畸变效果。
保存并告诉我选定的两份 camera.yaml 绝对路径和质量结论。
自动检查不通过时应补采、重新采集或定位模型问题，不把候选参数当作已合格结果。

完成实物验证后先告诉我准备重启哪两个相机，再在我确认时按文档加载选定 YAML，核对两路 camera_info 的尺寸和 D/K/R/P。
结束时给出前/后相机各自的样本数、RMS、参数位置、是否已加载、验证结论和未完成事项。
原始图片留在仓库外，不提交到公开 GitHub。代码若需要修改，提交功能分支并发 PR；合并 main 仍需我明确确认。
```
