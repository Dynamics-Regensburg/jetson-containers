#!/bin/bash
docker run -it --rm --runtime nvidia --device=/dev/ttyUSB0 --network host dockertest:latest
