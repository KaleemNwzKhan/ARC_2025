#!/bin/bash
xhost +local:root
IMG=kk5271/rapid_v2v:1.0
CODEBASE_DIRECTORY=/home/kk5271/RIT_Research/ARC_Repo/Code_base/
DATASET_DIRECTORY=/home/kk5271/RIT_Research/ARC_Repo/Dataset/
# Check if directories that we want to mount exist
if [ ! -d "$DATASET_DIRECTORY" ]; then
        echo "The dataset directory at $DATASET_DIRECTORY does not exist"
        return 1
fi

if [ ! -d "$CODEBASE_DIRECTORY" ]; then
        echo "The code base directory at $CODEBASE_DIRECTORY does not exist"
        return 1
fi
# If NVIDIA is present, use Nvidia-docker
if test -c /dev/nvidia0
then
    docker run --rm -it \
      --runtime=nvidia \
      --privileged \
      --device /dev/dri:/dev/dri \
      --env="DISPLAY" \
      --env="QT_X11_NO_MITSHM=1" \
      -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
      -v "$DATASET_DIRECTORY:/dataset:rw" \
      -v "$CODEBASE_DIRECTORY:/workspace:rw" \
      --name rapid_v2v \
      --network host \
      $IMG \
      bash
else
    docker run --rm -it \
      -e DISPLAY \
      --device=/dev/dri:/dev/dri \
      -v "/tmp/.X11-unix:/tmp/.X11-unix" \
      -v "$DATASET_DIRECTORY:/dataset:rw" \
      -v "$CODEBASE_DIRECTORY:/workspace:rw" \
      --name rapid_v2v \
      $IMG \
      bash
fi
