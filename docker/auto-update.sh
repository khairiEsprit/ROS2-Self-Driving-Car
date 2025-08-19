#!/bin/bash

# Simplified auto-update script for ROS2 Self Driving Car
# Uses original working image: noshluk2/ros2-self-driving-car-ai-using-opencv
# Automatically syncs code changes and restarts container

IMAGE_NAME="noshluk2/ros2-self-driving-car-ai-using-opencv"
CONTAINER_NAME="ros2-sdc-auto"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "$(date): Starting automated update for ROS2 Self Driving Car..."
echo "$(date): Using original working image: $IMAGE_NAME"
echo "$(date): Project directory: $PROJECT_DIR"

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
echo "$(date): Syncing latest code changes..."
cd "$PROJECT_DIR"

# Check if we're in a git repository
if [ -d ".git" ]; then
    echo "$(date): Git repository detected, pulling latest changes from khairiEsprit/ROS2-Self-Driving-Car..."
    # Reset any local changes and pull latest
    git reset --hard HEAD
    git clean -fd

    # Configure git pull strategy to avoid divergent branches error
    git config pull.rebase false

    # Set the correct remote URL (in case it's different)
    git remote set-url origin https://github.com/khairiEsprit/ROS2-Self-Driving-Car.git

    # Force pull latest changes
    git fetch origin main
    git reset --hard origin/main
    echo "$(date): Successfully pulled latest changes from your repository"
else
    echo "$(date): No git repository found, using local files as-is..."
fi

# Stop and remove existing container
echo "$(date): Stopping existing container..."
docker stop $CONTAINER_NAME 2>/dev/null || echo "Container not running"
docker rm $CONTAINER_NAME 2>/dev/null || echo "Container not found"

# Clean up any other related containers
docker stop ros2-sdc 2>/dev/null || echo "ros2-sdc container not running"
docker rm ros2-sdc 2>/dev/null || echo "ros2-sdc container not found"
docker stop ros2_sdc_container 2>/dev/null || echo "ros2_sdc_container not running"
docker rm ros2_sdc_container 2>/dev/null || echo "ros2_sdc_container not found"

# Pull latest original Docker image
echo "$(date): Pulling latest original Docker image: $IMAGE_NAME..."
if ! docker pull $IMAGE_NAME; then
    echo "Warning: Failed to pull latest image, checking if image exists locally..."
    # Check if image exists locally
    if ! docker image inspect $IMAGE_NAME >/dev/null 2>&1; then
        echo "Error: Original Docker image not available. Please check your internet connection."
        echo "Trying to pull image manually: docker pull $IMAGE_NAME"
        exit 1
    fi
    echo "Using existing local image..."
fi

# Setup X11 forwarding for GUI applications (if available)
echo "$(date): Setting up display forwarding..."
if [ -n "$DISPLAY" ]; then
    echo "Display detected: $DISPLAY"
    xhost +local:docker 2>/dev/null || echo "Warning: Could not set xhost permissions (normal on headless systems)"
else
    echo "No display detected, running in headless mode"
    export DISPLAY=:0
fi

# Start new container with the original working image
echo "$(date): Starting container with original working image..."
echo "$(date): Mounting project directory: $PROJECT_DIR -> /workspace"

CONTAINER_ID=$(docker run -it -d --name $CONTAINER_NAME \
  -v "$PROJECT_DIR":/workspace \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  --net=host \
  --privileged \
  $IMAGE_NAME bash)

if [ $? -eq 0 ]; then
    echo "$(date): Container started successfully!"
    echo "Container ID: $CONTAINER_ID"
else
    echo "$(date): Error: Failed to start container"
    exit 1
fi

# Wait for container to be ready
echo "$(date): Waiting for container to initialize..."
sleep 3

# Setup ROS environment and build packages
echo "$(date): Setting up ROS environment and building packages..."
docker exec $CONTAINER_NAME bash -c "
  echo 'Setting up ROS environment...'
  source /opt/ros/foxy/setup.bash
  cd /workspace

  # Install any missing dependencies
  rosdep update 2>/dev/null || true
  rosdep install --from-paths . --ignore-src -r -y 2>/dev/null || true

  # Build the packages
  echo 'Building ROS2 packages...'
  colcon build --packages-select self_driving_car_pkg_models self_driving_car_pkg --symlink-install

  # Source the built packages
  if [ -f install/setup.bash ]; then
    source install/setup.bash
    echo 'ROS2 packages built and sourced successfully!'
  else
    echo 'Warning: Build may have failed, but container is running'
  fi
"

# Clean up old Docker resources
echo "$(date): Cleaning up old Docker resources..."
docker image prune -f

echo "$(date): ✅ Automated update completed successfully!"
echo ""
echo "📊 System Status:"
echo "Container status:"
docker ps --filter name=$CONTAINER_NAME --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
echo ""
echo "💾 Disk usage:"
df -h / | head -2
echo ""
echo "🐳 Docker usage:"
docker system df

echo ""
echo "🚀 To connect to the container, run:"
echo "   docker exec -it $CONTAINER_NAME bash"
echo ""
echo "🔄 Auto-update will run again in 30 minutes"