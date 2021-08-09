#!/bin/bash
docker run -it --rm --name ros --runtime nvidia --device=/dev/ttyUSB0 --network host -v /nvme/code/rp-driverless:/workspace dockertest:latest
