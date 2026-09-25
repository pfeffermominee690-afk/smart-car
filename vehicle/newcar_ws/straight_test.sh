#!/usr/bin/env bash

set -u

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
serial_device="/dev/serial/by-id/usb-STMicroelectronics_ZDRB_USBCOM_368032723034-if00"
launch_log="/tmp/smartcar_straight_test.log"
launch_pid=""
test_speed="${SMARTCAR_STRAIGHT_TEST_SPEED:--40}"

if [[ "$test_speed" != "-40" && "$test_speed" != "0" ]]; then
  echo "错误：测试速度只允许 -40 或 0。" >&2
  exit 1
fi

source "$workspace_dir/setup_robot.sh"

stop_car() {
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    rostopic pub -1 /ackermann_cmd ackermann_msgs/AckermannDriveStamped \
      '{drive: {speed: 0.0, steering_angle: 0.0}}' >/dev/null 2>&1 || true
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}

trap stop_car EXIT INT TERM

if [[ ! -e "$serial_device" ]]; then
  echo "错误：没有检测到底盘 STM32 串口，请检查 USB 线和供电。" >&2
  exit 1
fi

if [[ ! -r "$serial_device" || ! -w "$serial_device" ]]; then
  if [[ "${SMARTCAR_DIALOUT_REFRESHED:-0}" != "1" ]] && \
     id -nG "$USER" | tr ' ' '\n' | grep -qx dialout; then
    echo "当前终端的 dialout 权限尚未刷新，正在自动切换用户组……"
    exec sg dialout -c \
      "SMARTCAR_DIALOUT_REFRESHED=1 '$workspace_dir/straight_test.sh'"
  fi
  echo "错误：当前终端没有串口权限，请先执行 newgrp dialout。" >&2
  exit 1
fi

if rosnode list 2>/dev/null | grep -Fxq /base_controller; then
  echo "错误：已有底盘控制器正在运行，请先退出原来的键盘控制。" >&2
  exit 1
fi

: >"$launch_log"
roslaunch base_controller keyboard_drive.launch >"$launch_log" 2>&1 &
launch_pid=$!

controller_ready=0
for _ in {1..150}; do
  if [[ "$(rosparam get /base_controller/serial_ready 2>/dev/null)" == "true" ]]; then
    controller_ready=1
    break
  fi
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    echo "底盘控制器启动失败：" >&2
    tail -n 30 "$launch_log" >&2
    exit 1
  fi
  sleep 0.1
done

if [[ "$controller_ready" -ne 1 ]]; then
  echo "底盘控制器未就绪：" >&2
  tail -n 30 "$launch_log" >&2
  exit 1
fi

echo "底盘已就绪，2 秒后以速度 $test_speed 直行 1.5 秒。"
echo "请确保车辆前后都没有障碍物。"
sleep 2

python - "$test_speed" <<'PY'
import sys
import time
import rospy
from ackermann_msgs.msg import AckermannDriveStamped

rospy.init_node('straight_test_sender', anonymous=True, disable_signals=True)
publisher = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped, queue_size=1)
stop = AckermannDriveStamped()
forward = AckermannDriveStamped()
forward.drive.speed = float(sys.argv[1])

try:
    deadline = time.time() + 5.0
    while publisher.get_num_connections() < 1 and time.time() < deadline:
        time.sleep(0.05)
    if publisher.get_num_connections() < 1:
        sys.stderr.write('Controller did not subscribe to /ackermann_cmd.\n')
        sys.exit(1)

    started = time.time()
    sent = 0
    while time.time() - started < 1.5:
        publisher.publish(forward)
        sent += 1
        time.sleep(0.05)
    print('Published %d commands at speed %.1f.' % (sent, forward.drive.speed))
finally:
    for _ in range(5):
        publisher.publish(stop)
        time.sleep(0.05)
PY
send_status=$?

if [[ "$send_status" -ne 0 ]]; then
  echo "直行指令发送失败；未进行运动测试。" >&2
  exit "$send_status"
fi

# roslaunch buffers redirected node output; close it before inspecting the log.
stop_car

if grep -q "TX command speed=${test_speed}.0 .*bytes_written=11" "$launch_log"; then
  echo "底盘日志确认：速度 $test_speed 的 11 字节命令已经写入 STM32 串口。"
else
  echo "未能在底盘日志中确认速度 $test_speed 指令；请查看 $launch_log" >&2
  grep 'TX command' "$launch_log" | tail -n 8 >&2 || true
  exit 1
fi
echo "测试结束，已发送停车指令。"
