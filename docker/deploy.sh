#!/bin/bash

# 🚀 Simplified Deployment Script for ROS2 Self-Driving Car
# Uses original working image: noshluk2/ros2-self-driving-car-ai-using-opencv
# Perfect for EC2 instances and automated deployments

set -e  # Exit on any error

# Configuration
IMAGE_NAME="noshluk2/ros2-self-driving-car-ai-using-opencv"
CONTAINER_NAME="ros2-sdc-production"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "🚀 ROS2 Self-Driving Car - Simplified Deployment"
echo "================================================"
echo "📦 Image: $IMAGE_NAME"
echo "📁 Project: $PROJECT_DIR"
echo "🐳 Container: $CONTAINER_NAME"
echo ""

# Function to check if Docker is running
check_docker() {
    if ! docker info >/dev/null 2>&1; then
        echo "❌ Error: Docker is not running or not accessible"
        echo "Please start Docker and try again"
        exit 1
    fi
    echo "✅ Docker is running"
}

# Function to clean up existing containers
cleanup_containers() {
    echo "🧹 Cleaning up existing containers..."
    
    # Stop and remove any existing containers
    for container in $CONTAINER_NAME ros2-sdc ros2_sdc_container ros2-sdc-interactive ros2-sdc-auto; do
        if docker ps -a --format '{{.Names}}' | grep -q "^${container}$"; then
            echo "  Stopping and removing: $container"
            docker stop $container 2>/dev/null || true
            docker rm $container 2>/dev/null || true
        fi
    done
    
    echo "✅ Cleanup completed"
}

# Function to pull latest image
pull_image() {
    echo "📥 Pulling latest original image..."
    if docker pull $IMAGE_NAME; then
        echo "✅ Image pulled successfully"
    else
        echo "⚠️  Warning: Could not pull latest image, checking local availability..."
        if docker image inspect $IMAGE_NAME >/dev/null 2>&1; then
            echo "✅ Using existing local image"
        else
            echo "❌ Error: Image not available locally and cannot pull from registry"
            echo "Please check your internet connection and try again"
            exit 1
        fi
    fi
}

# Function to sync code changes (if git repo)
sync_code() {
    echo "🔄 Syncing code changes from khairiEsprit/ROS2-Self-Driving-Car..."
    cd "$PROJECT_DIR"

    if [ -d ".git" ]; then
        echo "  Git repository detected, pulling latest changes..."

        # Set the correct remote URL to your repository
        git remote set-url origin https://github.com/khairiEsprit/ROS2-Self-Driving-Car.git

        # Reset any local changes
        git reset --hard HEAD
        git clean -fd

        # Configure git pull strategy
        git config pull.rebase false

        # Try to pull latest changes from your repo
        if git fetch origin main && git reset --hard origin/main; then
            echo "✅ Code synchronized with your repository (khairiEsprit/ROS2-Self-Driving-Car)"
        elif git fetch origin master && git reset --hard origin/master; then
            echo "✅ Code synchronized with your repository (master branch)"
        else
            echo "⚠️  Warning: Could not sync with remote, using local code"
        fi
    else
        echo "  No git repository found, using local files"
    fi
}

# Function to start container
start_container() {
    echo "🐳 Starting production container..."
    
    # Setup display for GUI applications (if available)
    if [ -n "$DISPLAY" ]; then
        echo "  Display forwarding enabled: $DISPLAY"
        xhost +local:docker 2>/dev/null || echo "  Warning: Could not set xhost permissions"
    else
        echo "  Running in headless mode"
        export DISPLAY=:0
    fi
    
    # Start the container
    CONTAINER_ID=$(docker run -d --name $CONTAINER_NAME \
        -v "$PROJECT_DIR":/workspace \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -e DISPLAY="$DISPLAY" \
        -e QT_X11_NO_MITSHM=1 \
        -e LIBGL_ALWAYS_SOFTWARE=1 \
        --net=host \
        --privileged \
        --restart=unless-stopped \
        $IMAGE_NAME \
        bash -c "
            source /opt/ros/foxy/setup.bash
            cd /workspace
            
            # Keep container running
            tail -f /dev/null
        ")
    
    if [ $? -eq 0 ]; then
        echo "✅ Container started successfully!"
        echo "   Container ID: $CONTAINER_ID"
    else
        echo "❌ Error: Failed to start container"
        exit 1
    fi
}

# Function to setup ROS environment
setup_ros_environment() {
    echo "🔧 Setting up ROS environment..."
    
    # Wait for container to be ready
    sleep 2
    
    # Setup and build packages
    docker exec $CONTAINER_NAME bash -c "
        source /opt/ros/foxy/setup.bash
        cd /workspace
        
        echo 'Installing dependencies...'
        rosdep update 2>/dev/null || true
        rosdep install --from-paths . --ignore-src -r -y 2>/dev/null || true
        
        echo 'Building ROS2 packages...'
        colcon build --packages-select self_driving_car_pkg_models self_driving_car_pkg --symlink-install
        
        if [ -f install/setup.bash ]; then
            source install/setup.bash
            echo 'ROS2 environment setup completed!'
        else
            echo 'Warning: Package build may have failed'
        fi
    "
    
    echo "✅ ROS environment ready"
}

# Function to show status
show_status() {
    echo ""
    echo "📊 Deployment Status"
    echo "===================="
    echo "Container status:"
    docker ps --filter name=$CONTAINER_NAME --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
    echo ""
    echo "💾 Disk usage:"
    df -h / | head -2
    echo ""
    echo "🐳 Docker usage:"
    docker system df
    echo ""
    echo "🎯 Quick Commands:"
    echo "   Connect to container:    docker exec -it $CONTAINER_NAME bash"
    echo "   View container logs:     docker logs $CONTAINER_NAME"
    echo "   Stop container:          docker stop $CONTAINER_NAME"
    echo "   Restart deployment:      $0"
    echo ""
}

# Main deployment flow
main() {
    echo "Starting deployment process..."
    
    check_docker
    cleanup_containers
    pull_image
    sync_code
    start_container
    setup_ros_environment
    
    echo ""
    echo "🎉 Deployment completed successfully!"
    show_status
}

# Handle script arguments
case "${1:-}" in
    "status")
        show_status
        ;;
    "stop")
        echo "🛑 Stopping deployment..."
        docker stop $CONTAINER_NAME 2>/dev/null || echo "Container not running"
        docker rm $CONTAINER_NAME 2>/dev/null || echo "Container not found"
        echo "✅ Deployment stopped"
        ;;
    "restart")
        echo "🔄 Restarting deployment..."
        docker stop $CONTAINER_NAME 2>/dev/null || true
        docker rm $CONTAINER_NAME 2>/dev/null || true
        main
        ;;
    "logs")
        echo "📋 Container logs:"
        docker logs -f $CONTAINER_NAME
        ;;
    "shell")
        echo "🐚 Connecting to container..."
        docker exec -it $CONTAINER_NAME bash
        ;;
    *)
        main
        ;;
esac
