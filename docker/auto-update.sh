#!/bin/bash

# Simple auto-update script for ROS2 Self Driving Car
# Pulls latest code and restarts container

IMAGE_NAME="khairiii/ros2-self-driving-car:latest"
CONTAINER_NAME="ros2-sdc-interactive"

echo "$(date): Starting update for $CONTAINER_NAME..."

# Navigate to project directory and pull latest code
echo "$(date): Pulling latest code..."
cd ~/ROS2-Self-Driving-Car
git pull origin main

# Pull latest Docker image
echo "$(date): Pulling latest Docker image..."
docker pull $IMAGE_NAME

# Stop and remove old container
echo "$(date): Stopping old container..."
docker stop $CONTAINER_NAME 2>/dev/null || echo "Container not running"
docker rm $CONTAINER_NAME 2>/dev/null || echo "Container not found"

# Start new container with volume mounting
echo "$(date): Starting new container..."
docker run -it -d --name $CONTAINER_NAME \
  -v $(pwd):/workspace \
  -e DISPLAY=$DISPLAY \
  --net=host \
  $IMAGE_NAME bash

# Build the packages inside the container
echo "$(date): Building ROS2 packages..."
docker exec $CONTAINER_NAME bash -c "
  cd /workspace
  source /opt/ros/humble/setup.bash
  colcon build --packages-select self_driving_car_pkg self_driving_car_pkg_models
  source install/setup.bash
"

# Clean up old images
echo "$(date): Cleaning up old images..."
docker image prune -f

echo "$(date): Update completed successfully!"

# Show container status
docker ps | grep $CONTAINER_NAME
