#!/bin/bash
set -e

# setup ros2 environment
echo "source \"/opt/ros/$ROS_DISTRO/setup.bash\"" >> /root/.bashrc
echo "source \"/opt/ros/$ROS_DISTRO/setup.bash\"" >> /root/.profile

source /root/.bashrc
source /root/.profile

ros2 launch rosbridge_server rosbridge_websocket_launch.xml & poetry run python run.py  simulation.json

exec "$@"
