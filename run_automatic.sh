#!/bin/bash
# Stopping docker container if running
sudo docker stop ros
# Starting new docker container with name "ros" 
sudo docker run -it -d --rm --name ros --runtime nvidia --device=/dev/ttyUSB0 --network host -v /nvme/code/rp-driverless:/workspace dynamicsev/ros_env:latest
echo Docker started

# Executing ros node startup in docker
sudo docker exec -i ros /workspace/launch/start.sh
# Starting web interface
./start-web.sh > web.log &
echo Wow, success!!!
