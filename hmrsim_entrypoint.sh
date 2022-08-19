#!/bin/bash
set -e

# setup poetry shell
# echo "poetry shell" >> /root/.bashrc
# echo "poetry shell" >> /root/.profile

# setup ros2 environment
echo "source \"/opt/ros/$ROS_DISTRO/setup.bash\"" >> /root/.bashrc
echo "source \"/opt/ros/$ROS_DISTRO/setup.bash\"" >> /root/.profile

source /root/.bashrc
source /root/.profile

# run rosbridge server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

exec "$@"
