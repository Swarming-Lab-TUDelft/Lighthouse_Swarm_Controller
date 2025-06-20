NUM_CFS=$1
START_IDX_CFS=$2
CFS_PER_RADIO=$3
numcfs_line="NUM_CFS = "$NUM_CFS
start_line="START_IDX_CFS = "$START_IDX_CFS
radio_line="CFS_PER_RADIO = "$CFS_PER_RADIO

docker container start swl_container

docker exec swl_container sed -i "/NUM_CFS = /c\\${numcfs_line}" "/home/lilpharaoh1/Lighthouse_Swarm_Controller/ros2_ws/src/swarm_operation/swarm_operation/config.py"
docker exec swl_container sed -i "/START_IDX_CFS = /c\\${start_line}" "/home/lilpharaoh1/Lighthouse_Swarm_Controller/ros2_ws/src/swarm_operation/swarm_operation/config.py"
docker exec swl_container sed -i "/CFS_PER_RADIO = /c\\${radio_line}" "/home/lilpharaoh1/Lighthouse_Swarm_Controller/ros2_ws/src/swarm_operation/swarm_operation/config.py" 

/mnt/c/Users/emran/Documents/CollisionAvoidance/WindowsBuild/WindowsCA.exe | sleep 7s | docker exec swl_container /bin/bash launch_swl.sh
