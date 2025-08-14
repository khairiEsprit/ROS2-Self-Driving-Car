#!/bin/bash

# Simple auto-update script for ROS2 Self Driving Car
# Pulls latest code and restarts container

IMAGE_NAME="khairiii/ros2-self-driving-car:latest"
CONTAINER_NAME="ros2-sdc-interactive"

echo "$(date): Starting update for $CONTAINER_NAME..."

# Check disk space before starting
echo "$(date): Checking disk space..."
df -h /
AVAILABLE_SPACE=$(df / | awk 'NR==2 {print $4}' | sed 's/G//')
if [ "$AVAILABLE_SPACE" -lt 2 ]; then
    echo "Warning: Less than 2GB available space. Cleaning up Docker resources first..."
    docker system prune -a -f
    docker volume prune -f
fi

# Navigate to project directory and pull latest code
echo "$(date): Pulling latest code..."
cd ~/ROS2-Self-Driving-Car

# Reset any local changes and pull latest
echo "$(date): Resetting local changes and pulling latest code..."
git reset --hard HEAD
git clean -fd
git pull origin main

# Stop and remove old containers first to free space
echo "$(date): Stopping old containers..."
docker stop $CONTAINER_NAME 2>/dev/null || echo "Container not running"
docker rm $CONTAINER_NAME 2>/dev/null || echo "Container not found"

# Also clean up the problematic ros2-sdc container
docker stop ros2-sdc 2>/dev/null || echo "ros2-sdc container not running"
docker rm ros2-sdc 2>/dev/null || echo "ros2-sdc container not found"

# Clean up old images before pulling new one
echo "$(date): Cleaning up old Docker resources..."
docker image prune -f
docker container prune -f

# Pull latest Docker image
echo "$(date): Pulling latest Docker image..."
if ! docker pull $IMAGE_NAME; then
    echo "Warning: Failed to pull latest image, using existing image"
    # Check if image exists locally
    if ! docker image inspect $IMAGE_NAME >/dev/null 2>&1; then
        echo "Error: No Docker image available locally"
        exit 1
    fi
fi

# Start new container with volume mounting
echo "$(date): Starting new container..."
CONTAINER_ID=$(docker run -it -d --name $CONTAINER_NAME \
  -v $(pwd):/workspace \
  -e DISPLAY=$DISPLAY \
  --net=host \
  $IMAGE_NAME bash)

if [ $? -eq 0 ]; then
    echo "Container started successfully: $CONTAINER_ID"
else
    echo "Error: Failed to start container"
    exit 1
fi

# Wait a moment for container to be ready
sleep 2

# Build the packages inside the container
echo "$(date): Building ROS2 packages..."
if docker exec $CONTAINER_NAME bash -c "
  cd /workspace
  source /opt/ros/humble/setup.bash
  colcon build --packages-select self_driving_car_pkg self_driving_car_pkg_models
  source install/setup.bash
"; then
    echo "ROS2 packages built successfully"
else
    echo "Warning: Package build failed, but container is running"
fi

# Final cleanup
echo "$(date): Final cleanup..."
docker image prune -f

echo "$(date): Update completed successfully!"

# Show container status and disk usage
echo "Container status:"
docker ps | grep $CONTAINER_NAME
echo "Disk usage:"
df -h /
echo "Docker disk usage:"
docker system df