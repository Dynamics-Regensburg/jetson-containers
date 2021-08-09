#!/bin/bash
docker run -it --rm --name ros --runtime nvidia --network host -v /nvme/code/rp-driverless:/workspace dynamicsev/ros_env:latest
