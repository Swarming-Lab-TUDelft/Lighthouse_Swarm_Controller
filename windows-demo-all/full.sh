NUM_CFS=$1
START_IDX_CFS=$2
NUM_RADIOS=$3
CFS_PER_RADIO=$4
CFCLIENT=$5
RECALIB=$6
CHECK_POSE=$7

echo $NUM_CFS $START_IDX_CFS $NUM_RADIOS $CFS_PER_RADIO $CFCLIENT $RECALIB $CHECK_POSE

scripts_path="/home/lilpharaoh1"

cd $scripts_path
./check_docker.sh && \	
	./check_radios.sh $NUM_RADIOS && \
	./write_to_config.sh $NUM_CFS $START_IDX_CFS $CFS_PER_RADIO && \
	./calibrate.sh $CFCLIENT $RECALIB && \
	./check_lh.sh $CHECK_POSE && \
	./launch_swl.sh 
