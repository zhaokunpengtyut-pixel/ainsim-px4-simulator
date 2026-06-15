#!/bin/bash
export PX4_SYS_AUTOSTART=10016
export PX4_SIM_HOST_ADDR=172.28.208.1
cd /home/hw/PX4-Autopilot
rm -f /home/hw/px4_output.log
nohup ./build/px4_sitl_default/bin/px4 -i 0 > /home/hw/px4_output.log 2>&1 &
echo "PX4 PID: "
