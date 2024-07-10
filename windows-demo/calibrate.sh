NUM_CFS=$1
NUM_RADIOS=$2
CFS_PER_RADIO=$3
CFCLIENT=$4
RECALIB=$5
docker container start swl_container

echo "calibrate params..."
echo $NUM_CFS
echo $NUM_RADIOS
echo $CFS_PER_RADIO
echo $CFCLIENT
echo $RECALIB

if [[ "$CFCLIENT" -gt 0 ]]; then
	docker exec -i swl_container python3 -m cfclient.gui
fi

docker exec -i swl_container sh -c "cat /home/lilpharaoh1/demo-config.yaml > /home/lilpharaoh1/Lighthouse_Swarm_Controller/configuration.yaml"

if [[ "$RECALIB" -gt 0 ]]; then
	radio=20
	for ((i=1;i<$NUM_RADIOS*$CFS_PER_RADIO;i=$i+$CFS_PER_RADIO))
	do
		echo "radio channel "$radio
		channel_line="CHANNEL = "$radio
		docker exec swl_container sed -i "/CHANNEL = /c\\${channel_line}" "/home/lilpharaoh1/Lighthouse_Swarm_Controller/upload_lh_config.py"	
		echo "start stop = "$i" "$(($i+$CFS_PER_RADIO-1))
		uri_line="URI_IDX = "$i", "$(($i+$CFS_PER_RADIO-1))
		docker exec swl_container sed -i "/URI_IDX = /c\\${uri_line}" "/home/lilpharaoh1/Lighthouse_Swarm_Controller/upload_lh_config.py"
		docker exec -w /home/lilpharaoh1/Lighthouse_Swarm_Controller swl_container python3 /home/lilpharaoh1/Lighthouse_Swarm_Controller/upload_lh_config.py
		echo "---------------------------------------"

		radio=$(($radio + 20))
	done
fi
