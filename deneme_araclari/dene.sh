#!/bin/bash
# Kullanim: ~/dene.sh <betik.py> [argumanlar]   ornek: ~/dene.sh ~/teslim.py yesil kirmizi mavi
# Windows/WSL'de her denemeyi temiz baslatmak icin PowerShell'den:
#   wsl --shutdown; wsl -d Ubuntu-22.04 -- bash -ic "~/dene.sh ~/teslim.py yesil kirmizi mavi"
# NOT: test icin nav2_safety_supervisor kapatilir ve /nav2_cmd_vel dogrudan /cmd_vel'e aktarilir.
WS=~/robotics_competition_ws
MAP=$WS/maps/competition_mapping/20261125T103259Z_572d1928/nav2_portable_example.yaml
cd $WS
source /opt/ros/humble/setup.bash
source install/setup.bash
echo ">>> Eski surecler kapatiliyor..."
pkill -9 -f "ros2 launch"; pkill -9 -f "ign gazebo"; pkill -f rviz2; pkill -f aktarici
pkill -f "python3 .*gecit_testi"; pkill -f "python3 .*tam_saha"; pkill -f "python3 .*teslim"
pkill -f competition_bringup; pkill -f parameter_bridge; pkill -f robot_state_publisher
pkill -f nav2_; pkill -f amcl; pkill -f map_server; pkill -f lifecycle_manager; pkill -f bt_navigator
for k in $(seq 1 20); do pgrep -f "ign gazebo|planner_server|map_server" >/dev/null || break; pkill -9 -f "ign gazebo|planner_server|controller_server|map_server|amcl|bt_navigator|lifecycle_manager|behavior_server|parameter_bridge|rviz2"; sleep 1; done
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
ros2 daemon stop >/dev/null 2>&1
echo ">>> Nav2 + Gazebo aciliyor..."
ros2 launch competition_bringup competition_nav2.launch.py gui:=true rviz:=true enable_motion:=true map:=$MAP > /tmp/nav2.log 2>&1 &
for i in $(seq 1 60); do
  sleep 3
  if grep -q "Managed nodes are active" /tmp/nav2.log && grep -q "AMCL initialized" /tmp/nav2.log; then
    echo ">>> Nav2 hazir ($((i*3)) sn)"; break
  fi
  echo "    bekleniyor... $((i*3)) sn"
  if [ $i -eq 20 ]; then echo ">>> Nav2 60 sn icinde acilmadi. Log:"; grep -iE "error|already running|critical" /tmp/nav2.log | tail -5; exit 1; fi
done
sleep 5
echo ">>> Gozetmen kapatiliyor, Nav2 komutu dogrudan robota aktariliyor (sadece test)"
pkill -f nav2_safety_supervisor
python3 - > /tmp/aktarici.log 2>&1 << 'PY' &
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
rclpy.init()
n = Node('aktarici')
pub = n.create_publisher(Twist, '/cmd_vel', 10)
n.create_subscription(Twist, '/nav2_cmd_vel', pub.publish, 10)
rclpy.spin(n)
PY
sleep 2
echo ">>> Test basliyor..."
python3 "$@"
echo ">>> Bitti."
