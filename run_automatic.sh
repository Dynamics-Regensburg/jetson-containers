#!/bin/bash

GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

LOG_DIRECTORY="/nvme/code/jetson-containers/logs/"
LOG_NAME_DOCKER="docker.log"

echo "" > $LOG_DIRECTORY$LOG_NAME_DOCKER

CONTAINER_NAME="ros"

GPS_ERROR_STRING="Error response from daemon: error gathering device information while adding custom device"

mkdir -p $LOG_DIRECTORY

# Get lidar ip address
LIDAR_IP=`arp -i eth5 | awk 'END{print $1}'`


# Exit handler to stop docker container on Ctrl-C
keepgoing=1
trap '{ echo; echo -e ${RED}STOPPED!${NC}; sudo docker kill $CONTAINER_NAME > /dev/null; exit 0; }' SIGINT


# If the docker container is running, stop it
if sudo docker container list | awk 'NF>0{print $NF}' | grep -q -sw "$CONTAINER_NAME"
then
	echo "Container with the same name is already running."
	echo "Stopping existing docker container"
	sudo docker stop ros > /dev/null
	echo -e "${GREEN}Successfully stopped existing container${NC}"
fi


{
	# Try starting docker container with gps	
	sudo docker run -it -d --rm --name ros --runtime nvidia -e LIDAR_IP=$LIDAR_IP --device=/dev/ttyUSB0 --network host -v /nvme/code/rp-driverless:/workspace dynamicsev/ros_env:latest >> "$LOG_DIRECTORY$LOG_NAME_DOCKER" 2>&1
	
} || 
{
	# If the starting fails
	echo -e "${RED}Failed starting docker container with gps!${NC}"
	echo "Logs are in $LOG_DIRECTORY$LOG_NAME_DOCKER"
	echo -e "${RED}Trying again without gps!${NC}"

	{
		# Try again without gps
		sudo docker run -it -d --rm --name ros --runtime nvidia -e LIDAR_IP=$LIDAR_IP --network host -v /nvme/code/rp-driverless:/workspace dynamicsev/ros_env:latest >> "$LOG_DIRECTORY$LOG_NAME_DOCKER" 2>&1
		echo -e "${GREEN}Successfully started docker container WITHOUT gps${NC}"
	} ||
	{
		# If it also fails without gps
		echo -e "${RED}Failed starting docker container WITHOUT gps!${NC}"
		echo -e "${RED}EXITING!${NC}"
		exit 1
		
	}
}

source /opt/ros/melodic/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch &

# If the program has not exited here, a docker container is running
{
	sudo docker exec -i ros /workspace/launch/start.sh  #>> "$LOG_DIRECTORY$LOG_NAME_DOCKER" 2>&1
} ||
{
	echo -e "${RED}FAILED${NC}"
}


while (( keepgoing ))
do
        sleep 1
done
