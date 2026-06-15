#!/bin/bash
cd /home/hw/PX4-Autopilot
PX4_SYS_AUTOSTART=10016 exec ./build/px4_sitl_default/bin/px4 -i 0
