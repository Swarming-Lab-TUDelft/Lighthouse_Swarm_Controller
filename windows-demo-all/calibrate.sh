CFCLIENT=$1
RECALIB=$2
docker container start swl_container

echo "Launch cfclinet : "$CFCLIENT
echo "Recalibrate : "$RECALIB

if [[ "$CFCLIENT" -gt 0 ]]; then
	docker exec -i swl_container python3 -m cfclient.gui
fi

if [[ "$RECALIB" -gt 0 ]]; then
	docker exec -i swl_container sh -c "cat /home/lilpharaoh1/demo-config.yaml > /home/lilpharaoh1/Lighthouse_Swarm_Controller/configuration.yaml"
	docker exec -w /home/lilpharaoh1/Lighthouse_Swarm_Controller swl_container python3 /home/lilpharaoh1/Lighthouse_Swarm_Controller/upload_lh_config_all.py
fi
