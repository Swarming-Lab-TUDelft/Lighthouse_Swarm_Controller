#!/bin/bash
source ./install/setup.bash
(trap 'kill 0' SIGINT; ros2 launch swarm_operation launch.py & ./../collision_avoidance/LinuxBuild/LinuxCA.x86_64)