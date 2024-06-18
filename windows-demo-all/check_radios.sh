EXPECTED_RADIOS=$1

docker container start swl_container
wsl_radios=$(sh -c "lsusb | grep -c \"Crazyradio\"")
docker_radios=$(docker exec -i swl_container sh -c "lsusb | grep -c \"Crazyradio\"")

if [[ "$wsl_radios" < "$EXPECTED_RADIOS" ]]; then
	echo "Assertion failed: Radio(s) not detected by WSL!"
        exit 1
fi

if [[ "$wsl_radios" != "$docker_radios" ]]; then
	echo "Assertion failed: Radio forwaring failed!"
	exit 1
fi
