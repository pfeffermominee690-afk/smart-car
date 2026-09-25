#!/usr/bin/env bash

set -u

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
launch_pid=""
launch_log="/tmp/smartcar_keyboard_drive.log"

cleanup() {
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

if [[ ! -t 0 ]]; then
  echo "错误：键盘控制必须在交互式终端中运行。" >&2
  exit 1
fi

source "$workspace_dir/setup_robot.sh"

serial_device="/dev/serial/by-id/usb-STMicroelectronics_ZDRB_USBCOM_368032723034-if00"
if [[ ! -e "$serial_device" ]]; then
  echo "错误：没有检测到底盘 STM32 串口：$serial_device" >&2
  echo "请检查 STM32 的 USB 线和供电。" >&2
  exit 1
fi

if [[ ! -r "$serial_device" || ! -w "$serial_device" ]]; then
  echo "错误：当前终端没有 STM32 串口访问权限：$serial_device" >&2
  echo "当前用户组：$(id -nG)" >&2
  echo "请先执行 newgrp dialout，再重新运行本脚本。" >&2
  exit 1
fi

if ! rosnode list 2>/dev/null | grep -Fxq /base_controller; then
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
      echo "底盘控制器启动失败，日志如下：" >&2
      cat "$launch_log" >&2
      exit 1
    fi
    sleep 0.1
  done

  if [[ "$controller_ready" -ne 1 ]]; then
    echo "底盘串口在 15 秒内未就绪，日志如下：" >&2
    tail -n 30 "$launch_log" >&2
    exit 1
  fi
fi

sleep 0.5
if ! rosnode list 2>/dev/null | grep -Fxq /base_controller || \
   { [[ -n "$launch_pid" ]] && ! kill -0 "$launch_pid" 2>/dev/null; }; then
  echo "底盘控制器未就绪，请查看 $launch_log" >&2
  tail -n 30 "$launch_log" >&2
  exit 1
fi

echo "底盘控制器已就绪。请使用英文小写按键：W/S 前进后退，A/D 转向，空格停车，Tab 回正，Q 退出。"
echo "每按一次 W/S 增减 4；车辆架空或周围留出空间后再操作。"
rosrun ackermann_drive_teleop keyop.py "${1:-20}" "${2:-22}"
