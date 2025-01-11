#!/bin/bash

container=ros2_px4_inz_devcontainer-workspace-1
docker cp modified/px4.launch $container:/home/developer/PX4-Autopilot/launch/px4.launch
docker cp modified/mono_cam $container:/home/developer/PX4-Autopilot/Tools/simulation/gz/models
docker cp modified/x500_mono_cam $container:/home/developer/PX4-Autopilot/Tools/simulation/gz/models
docker cp modified/car $container:/home/developer/PX4-Autopilot/Tools/simulation/gz/models
docker cp modified/tracking_static.sdf $container:/home/developer/PX4-Autopilot/Tools/simulation/gz/worlds
docker cp modified/tracking_circle.sdf $container:/home/developer/PX4-Autopilot/Tools/simulation/gz/worlds
docker cp modified/tracking_eight.sdf $container:/home/developer/PX4-Autopilot/Tools/simulation/gz/worlds
docker cp modified/launch_static.sh $container:/home/developer/ros2_ws
docker cp modified/launch_circle.sh $container:/home/developer/ros2_ws
docker cp modified/launch_eight.sh $container:/home/developer/ros2_ws
