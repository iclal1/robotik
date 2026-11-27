#!/bin/bash
# Resmi gorevi (competition_task.launch.py) calistirir, bitince Gazebo'dan sonucu olcer.
# PowerShell:  wsl --shutdown; wsl -d Ubuntu-22.04 -- bash -ic "~/robotics_competition_ws/deneme_araclari/gorev_dene.sh"
WS=~/robotics_competition_ws
cd $WS
source /opt/ros/humble/setup.bash
source install/setup.bash
echo ">>> Eski surecler kapatiliyor..."
pkill -9 -f "ros2 launch"; pkill -9 -f "ign gazebo"; pkill -f rviz2; pkill -f aktarici
pkill -f "python3 .*teslim"; pkill -f competition_bringup; pkill -f parameter_bridge
pkill -f robot_state_publisher; pkill -f nav2_; pkill -f amcl; pkill -f map_server
pkill -f lifecycle_manager; pkill -f bt_navigator
for k in $(seq 1 20); do pgrep -f "ign gazebo|planner_server|map_server" >/dev/null || break; pkill -9 -f "ign gazebo|planner_server|controller_server|map_server|amcl|bt_navigator|lifecycle_manager|behavior_server|parameter_bridge|rviz2|cube_pusher"; sleep 1; done
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
ros2 daemon stop >/dev/null 2>&1
echo ">>> Gorev baslatiliyor (tek launch: Gazebo + Nav2 + cube_pusher)..."
ros2 launch competition_bringup competition_task.launch.py gui:=true rviz:=true "$@" > /tmp/gorev.log 2>&1 &
L=$!
sleep 1
tail -n +1 -f /tmp/gorev.log | grep --line-buffered "cube_pusher" | sed -u 's/^.*\]: /  /' &
T=$!
while kill -0 $L 2>/dev/null; do
  grep -qE "GOREV BITTI|cube_pusher.py-[0-9]+\]: process has died|ACIL DURDURMA" /tmp/gorev.log && break
  sleep 2
done
sleep 1; pkill -f "tail -n \+1 -f /tmp/gorev.log"
if ! grep -q "GOREV BITTI" /tmp/gorev.log; then
  echo ">>> Gorev bitmeden durdu. Son hatalar:"; grep -iE "error|traceback|died|exception" /tmp/gorev.log | tail -8
fi
echo ">>> Gercek konumlar (yalnizca olcum):"
python3 $WS/deneme_araclari/sonuc_olc.py
echo ">>> Simulasyon kapatiliyor."
kill $L 2>/dev/null; sleep 3; pkill -9 -f "ign gazebo"
echo ">>> Bitti."
