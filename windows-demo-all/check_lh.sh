CHECK_POSE=$1
docker container start swl_container

echo "Check pose : "$CHECK_POSE

if [[ "$CHECK_POSE" -gt 0 ]]; then	
	docker exec -w /home/lilpharaoh1/Lighthouse_Swarm_Controller swl_container python3 /home/lilpharaoh1/Lighthouse_Swarm_Controller/check_pose_all.py > lh_tmp.txt
       	cat lh_tmp.txt 
	sleep 20
	./parse_lh.sh	
fi
