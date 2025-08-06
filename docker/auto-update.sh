#!/bin/bash

# Simple auto-update script for ROS2 Self Driving Car
# Pulls latest image and restarts container

IMAGE_NAME="khairiesprit/ros2-self-driving-car:latest"
CONTAINER_NAME="yourapp"

echo "$(date): Starting update..."

# Pull latest image
docker pull $IMAGE_NAME

# Stop and remove old container
docker stop $CONTAINER_NAME 2>/dev/null
docker rm $CONTAINER_NAME 2>/dev/null

# Start new container
docker run -d --name $CONTAINER_NAME --restart=always -p 80:80 $IMAGE_NAME

# Clean up old images
docker image prune -f

echo "$(date): Update completed"
