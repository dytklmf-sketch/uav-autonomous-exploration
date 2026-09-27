#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash
source /mnt/nvme/ws/fastlio_ws/devel/setup.bash
cd /home/nvidia
exec python3 odom_to_mavros.py
