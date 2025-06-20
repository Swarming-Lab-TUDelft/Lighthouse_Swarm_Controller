safe=0

while read -r line; do
	if [[ "$line" == *"Pose unstable"* ]]; then
		echo $line
        	((safe++))
    	fi
done < lh_tmp.txt

if [[ "$safe" -gt 0 ]]; then
	exit 1        
fi
