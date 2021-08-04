#!/bin/bash

cd /usr/src/app/data/
rm -rf images
wget $zipfile -O data.zip
unzip data.zip

rm data.zip

cd ..

wget $yaml -O data.yaml

python3 train.py --img $imgsize --batch $batchsize --epochs $epochs --data data.yaml --weights yolov5s6.pt

zip -r results.zip runs

curl --upload-file ./results.zip https://transfer.sh/results.zip
