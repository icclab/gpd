#!/usr/bin/env bash
set +e

source "/catkin_ws/devel/setup.bash"

Xvfb :1 -screen 0 1600x1200x16  &
export DISPLAY=:1.0

python /catkin_ws/src/gpd/scripts/gripper_hack.py &
sleep 2

roslaunch /catkin_ws/src/gpd/launch/tiago_playground_no_gpd_headless.launch &
sleep 20

python /catkin_ws/src/gpd/scripts/downsample_cloud.py

exec "$@"
