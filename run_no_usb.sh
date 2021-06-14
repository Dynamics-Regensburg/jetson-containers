#!/bin/bash
docker run -it --rm --runtime nvidia --network host -v /home/driverless/code/rp-driverless:/workspace dockertest:latest
