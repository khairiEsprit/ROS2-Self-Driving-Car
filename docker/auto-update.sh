#!/bin/bash

# Simple auto-update script for ROS2 Self Driving Car
# Pulls latest image and restarts container

# Get your Docker Hub username - replace 'YOUR_DOCKER_USERNAME' with your actual username
IMAGE_NAME="YOUR_DOCKER_USERNAME/ros2-self-driving-car:latest"
CONTAINER_NAME="ros2-sdc"

echo "$(date): Starting update for $IMAGE_NAME..."

# Pull latest image
echo "$(date): Pulling latest image..."
docker pull $IMAGE_NAME

# Stop and remove old container
echo "$(date): Stopping old container..."
docker stop $CONTAINER_NAME 2>/dev/null || echo "Container not running"
docker rm $CONTAINER_NAME 2>/dev/null || echo "Container not found"

# Start new container
echo "$(date): Starting new container..."
docker run -d --name $CONTAINER_NAME --restart=always -p 80:80 $IMAGE_NAME

# Clean up old images
echo "$(date): Cleaning up old images..."
docker image prune -f

echo "$(date): Update completed successfully!"

# Show container status
docker ps | grep $CONTAINER_NAME
