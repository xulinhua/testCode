#!/bin/bash
xhost + 

docker run \
    --gpus all \
    -it \
    --network host \
    --runtime nvidia \
    -e DISPLAY=$DISPLAY \
    --device=/dev/video0 \
    --device=/dev/video1 \
    -v /lib/modules:/lib/modules \
    --cap-add SYS_PTRACE \
    -v /tmp:/tmp \
    l4t-ros2-csi-node:latest /bin/bash 