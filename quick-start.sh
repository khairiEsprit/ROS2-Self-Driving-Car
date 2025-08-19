#!/bin/bash

# 🚀 Quick Start Script for ROS2 Self-Driving Car
# One-command setup for Ubuntu/EC2 instances
# Uses original working image: noshluk2/ros2-self-driving-car-ai-using-opencv

set -e  # Exit on any error

echo "🚀 ROS2 Self-Driving Car - Quick Start"
echo "======================================"
echo "This script will set up everything you need!"
echo ""

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    echo "❌ Please don't run this script as root (without sudo)"
    echo "   Run as: ./quick-start.sh"
    exit 1
fi

# Function to check and install Docker
install_docker() {
    if command -v docker >/dev/null 2>&1; then
        echo "✅ Docker is already installed"
        
        # Check if user is in docker group
        if groups $USER | grep -q docker; then
            echo "✅ User is in docker group"
        else
            echo "⚠️  Adding user to docker group..."
            sudo usermod -aG docker $USER
            echo "⚠️  Please log out and log back in, then run this script again"
            exit 1
        fi
    else
        echo "📦 Installing Docker..."
        
        # Update package index
        sudo apt-get update
        
        # Install required packages
        sudo apt-get install -y \
            apt-transport-https \
            ca-certificates \
            curl \
            gnupg \
            lsb-release
        
        # Add Docker's official GPG key
        curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
        
        # Set up stable repository
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
        
        # Install Docker Engine
        sudo apt-get update
        sudo apt-get install -y docker-ce docker-ce-cli containerd.io
        
        # Add user to docker group
        sudo usermod -aG docker $USER
        
        # Start and enable Docker
        sudo systemctl start docker
        sudo systemctl enable docker
        
        echo "✅ Docker installed successfully!"
        echo "⚠️  Please log out and log back in, then run this script again"
        exit 1
    fi
}

# Function to install Git if not present
install_git() {
    if command -v git >/dev/null 2>&1; then
        echo "✅ Git is already installed"
    else
        echo "📦 Installing Git..."
        sudo apt-get update
        sudo apt-get install -y git
        echo "✅ Git installed successfully!"
    fi
}

# Function to clone or update repository
setup_repository() {
    REPO_URL="https://github.com/khairiEsprit/ROS2-Self-Driving-Car.git"
    PROJECT_NAME="ROS2-Self-Driving-Car"

    if [ -d "$PROJECT_NAME" ]; then
        echo "📁 Project directory exists, updating from your repository..."
        cd "$PROJECT_NAME"
        git remote set-url origin "$REPO_URL"
        git fetch origin
        git reset --hard origin/main
        cd ..
    else
        echo "📥 Cloning your repository: khairiEsprit/ROS2-Self-Driving-Car..."
        git clone "$REPO_URL"
    fi

    echo "✅ Repository ready: $PROJECT_NAME"
}

# Function to setup the environment
setup_environment() {
    PROJECT_NAME="ROS2-Self-Driving-Car"

    echo "🔧 Setting up environment..."
    cd "$PROJECT_NAME"

    # Make scripts executable
    chmod +x docker/*.sh
    chmod +x quick-start.sh 2>/dev/null || true

    # Run setup script
    ./docker/setup-auto-update.sh

    echo "✅ Environment setup completed!"
}

# Function to start the application
start_application() {
    PROJECT_NAME="ROS2-Self-Driving-Car"

    echo "🚀 Starting ROS2 Self-Driving Car..."
    cd "$PROJECT_NAME"

    # Start the deployment
    ./docker/deploy.sh

    echo "✅ Application started successfully!"
}

# Function to show final instructions
show_instructions() {
    PROJECT_NAME="ROS2-Self-Driving-Car"

    echo ""
    echo "🎉 Setup completed successfully!"
    echo "================================"
    echo ""
    echo "📁 Project location: $(pwd)/$PROJECT_NAME"
    echo "🔗 Repository: https://github.com/khairiEsprit/ROS2-Self-Driving-Car"
    echo ""
    echo "🎯 Available commands:"
    echo "   cd $PROJECT_NAME"
    echo "   ./docker/deploy.sh          # Start/restart the application"
    echo "   ./docker/deploy.sh status   # Check status"
    echo "   ./docker/deploy.sh stop     # Stop the application"
    echo "   ./docker/deploy.sh shell    # Connect to container"
    echo "   ./docker/deploy.sh logs     # View logs"
    echo ""
    echo "🔄 Auto-update is configured to run every 30 minutes"
    echo "📋 Check auto-update logs: tail -f /var/log/ros2-sdc/auto-update.log"
    echo ""
    echo "💡 When you push changes to your GitHub repo, they will be automatically"
    echo "   pulled and deployed within 30 minutes, or run: ./docker/auto-update.sh"
    echo ""
    echo "🌐 If you're on EC2, make sure to:"
    echo "   • Open necessary ports in security groups"
    echo "   • Configure any required environment variables"
    echo ""
}

# Main execution
main() {
    echo "Starting quick setup..."
    
    # Update system packages
    echo "📦 Updating system packages..."
    sudo apt-get update
    
    # Install required tools
    install_git
    install_docker
    
    # Setup project
    setup_repository
    setup_environment
    start_application
    
    # Show final instructions
    show_instructions
}

# Handle script arguments
case "${1:-}" in
    "docker-only")
        install_docker
        ;;
    "git-only")
        install_git
        ;;
    "repo-only")
        setup_repository
        ;;
    *)
        main
        ;;
esac
