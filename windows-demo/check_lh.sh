NUM_CFS=$1
NUM_RADIOS=$2
CFS_PER_RADIO=$3
CHECK_POSE=$4
docker container start swl_container

echo "check_lh params..."
echo $NUM_CFS
echo $NUM_RADIOS
echo $CFS_PER_RADIO
echo $CHECK_POSE

if [[ "$CHECK_POSE" -gt 0 ]]; then
	radio=20
	for ((i=1;i<$NUM_RADIOS*$CFS_PER_RADIO;i=$i+$CFS_PER_RADIO))
	do
		echo "radio channel "$radio
		channel_line="CHANNEL = "$radio
		docker exec swl_container sed -i "/CHANNEL = /c\\${channel_line}" "/home/lilpharaoh1/Lighthouse_Swarm_Controller/check_pose.py"	
		echo "start stop = "$i" "$(($i+$CFS_PER_RADIO-1))
		uri_line="URI_IDX = "$i", "$(($i+$CFS_PER_RADIO-1))
		docker exec swl_container sed -i "/URI_IDX = /c\\${uri_line}" "/home/lilpharaoh1/Lighthouse_Swarm_Controller/check_pose.py"
		docker exec -w /home/lilpharaoh1/Lighthouse_Swarm_Controller swl_container python3 /home/lilpharaoh1/Lighthouse_Swarm_Controller/check_pose.py > lh_tmp.txt
	       	sleep 10
		./parse_lh.sh	
		echo "---------------------------------------"

		radio=$(($radio + 20))
	done
fi
