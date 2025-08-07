#!/bin/bash

echo "🚀 Creating and entering ROS2 Self-Driving Car container..."

# Set up X11 forwarding for GUI applications
xhost +local:root
XAUTH=/tmp/.docker.xauth

# Remove any existing container with the same name
echo "🧹 Cleaning up any existing container..."
docker stop ros2_sdc_container 2>/dev/null || true
docker rm ros2_sdc_container 2>/dev/null || true

echo "🐳 Starting new container..."
docker run -it \
    --name=ros2_sdc_container \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    --runtime=nvidia \
    khairiesprit/ros2-self-driving-car:latest \
    bash

echo "👋 Exited from container."