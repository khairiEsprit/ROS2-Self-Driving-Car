#!/bin/bash

echo "🚀 Creating and entering ROS2 Self-Driving Car container..."
echo "📦 Using original working image: noshluk2/ros2-self-driving-car-ai-using-opencv"

# Get the project directory
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo "📁 Project directory: $PROJECT_DIR"

# Set up X11 forwarding for GUI applications
echo "🖥️  Setting up display forwarding..."
if [ -n "$DISPLAY" ]; then
    echo "Display detected: $DISPLAY"
    xhost +local:root 2>/dev/null || echo "Warning: Could not set xhost permissions"
else
    echo "No display detected, setting default"
    export DISPLAY=:0
fi

# Container configuration
CONTAINER_NAME="ros2_sdc_interactive"
IMAGE_NAME="noshluk2/ros2-self-driving-car-ai-using-opencv"

# Remove any existing container with the same name
echo "🧹 Cleaning up any existing containers..."
docker stop $CONTAINER_NAME 2>/dev/null || true
docker rm $CONTAINER_NAME 2>/dev/null || true
docker stop ros2_sdc_container 2>/dev/null || true
docker rm ros2_sdc_container 2>/dev/null || true

# Pull latest image
echo "📥 Pulling latest original image..."
docker pull $IMAGE_NAME || echo "Warning: Could not pull latest image, using local version"

echo "🐳 Starting new interactive container..."
docker run -it \
    --name=$CONTAINER_NAME \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="LIBGL_ALWAYS_SOFTWARE=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$PROJECT_DIR:/workspace" \
    --net=host \
    --privileged \
    $IMAGE_NAME \
    bash -c "
        echo '🔧 Setting up ROS environment...'
        source /opt/ros/foxy/setup.bash
        cd /workspace

        echo '📦 Installing dependencies...'
        rosdep update 2>/dev/null || true
        rosdep install --from-paths . --ignore-src -r -y 2>/dev/null || true

        echo '🔨 Building packages...'
        colcon build --packages-select self_driving_car_pkg_models self_driving_car_pkg --symlink-install

        if [ -f install/setup.bash ]; then
            source install/setup.bash
            echo '✅ Environment ready!'
        fi

        echo '🎯 Welcome to ROS2 Self-Driving Car Development Environment!'
        echo '📁 Your code is mounted at /workspace'
        echo '🚀 ROS2 environment is sourced and ready'
        echo ''
        exec bash
    "

echo "👋 Exited from container."