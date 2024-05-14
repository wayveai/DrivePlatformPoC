#!/bin/bash

export WORKSPACE=/home/wayve/nvidia-workspace

xhost +local:docker
docker run -dt --privileged --net=host --gpus=all \
	--sysctl fs.mqueue.msg_max=4096 \
	--sysctl fs.mqueue.queues_max=512 --ulimit msgqueue=2097152 \
	-e DISPLAY \
	-e NVIDIA_DRIVER_CAPABILITIES=all \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v /dev/bus/usb:/dev/bus/usb \
       	-v ${WORKSPACE}/drive_flashing:/drive_flashing \
       	-v ${WORKSPACE}:/home/nvidia/ \
       	-v /srv/nfs/driveworks-aarch64-linux-gnu:/srv/nfs/driveworks-aarch64-linux-gnu \
	nvcr.io/drive/driveos-sdk/drive-agx-orin-linux-aarch64-sdk-build-x86:latest
	
#	-v /usr/local/driveworks:/usr/local/driveworks \
#	-p 2222:22 \
#      	drive-dev:latest

# xhost -local:docker
