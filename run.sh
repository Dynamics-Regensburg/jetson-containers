#!/bin/bash
docker run -it --rm --runtime nvidia --device=/dev/ttyUSB0 --network host -v /home/driverless/code/rp-driverless:/workspace dockertest:latest
