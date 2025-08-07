#!/bin/bash

# Complete EC2 Setup Script for ROS2 Self Driving Car Auto-Deployment
# Run this script on your Ubuntu EC2 instance

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚀 Setting up ROS2 Self Driving Car Auto-Deployment on EC2...${NC}"

# Get user's Docker Hub username
echo -e "${YELLOW}📝 Please enter your Docker Hub username:${NC}"
read -p "Docker Hub Username: " DOCKER_USERNAME

if [ -z "$DOCKER_USERNAME" ]; then
    echo -e "${RED}❌ Error: Docker Hub username cannot be empty${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Using Docker Hub username: $DOCKER_USERNAME${NC}"

# Update system
echo -e "${BLUE}📦 Updating system packages...${NC}"
sudo apt update

# Install Docker
echo -e "${BLUE}🐳 Installing Docker...${NC}"
sudo apt install -y docker.io
sudo systemctl start docker
sudo systemctl enable docker

# Add user to docker group
echo -e "${BLUE}👤 Adding user to docker group...${NC}"
sudo usermod -aG docker ubuntu

# Create auto-update script with correct username
echo -e "${BLUE}📝 Creating auto-update script...${NC}"
cat > auto-update.sh << EOF
#!/bin/bash

# Simple auto-update script for ROS2 Self Driving Car
# Pulls latest image and restarts container

IMAGE_NAME="$DOCKER_USERNAME/ros2-self-driving-car:latest"
CONTAINER_NAME="ros2-sdc"

echo "\$(date): Starting update for \$IMAGE_NAME..."

# Pull latest image
echo "\$(date): Pulling latest image..."
docker pull \$IMAGE_NAME

# Stop and remove old container
echo "\$(date): Stopping old container..."
docker stop \$CONTAINER_NAME 2>/dev/null || echo "Container not running"
docker rm \$CONTAINER_NAME 2>/dev/null || echo "Container not found"

# Start new container
echo "\$(date): Starting new container..."
docker run -d --name \$CONTAINER_NAME --restart=always -p 80:80 \$IMAGE_NAME

# Clean up old images
echo "\$(date): Cleaning up old images..."
docker image prune -f

echo "\$(date): Update completed successfully!"

# Show container status
docker ps | grep \$CONTAINER_NAME
EOF

# Make script executable
chmod +x auto-update.sh

# Create log file
sudo touch /var/log/auto-update.log
sudo chmod 666 /var/log/auto-update.log

# Set up cron job for auto-update (every 30 minutes)
echo -e "${BLUE}⏰ Setting up cron job for auto-updates...${NC}"
(crontab -l 2>/dev/null; echo "*/30 * * * * $(pwd)/auto-update.sh >> /var/log/auto-update.log 2>&1") | crontab -

# Pull and run the initial container
echo -e "${BLUE}🚀 Pulling and running initial container...${NC}"
docker pull $DOCKER_USERNAME/ros2-self-driving-car:latest
docker run -d --name ros2-sdc --restart=always -p 80:80 $DOCKER_USERNAME/ros2-self-driving-car:latest

echo -e "${GREEN}🎉 Setup completed successfully!${NC}"
echo
echo -e "${BLUE}📋 Summary:${NC}"
echo -e "${GREEN}✅ Docker installed and configured${NC}"
echo -e "${GREEN}✅ Auto-update script created: $(pwd)/auto-update.sh${NC}"
echo -e "${GREEN}✅ Cron job set up (runs every 30 minutes)${NC}"
echo -e "${GREEN}✅ Container started: ros2-sdc${NC}"
echo -e "${GREEN}✅ Port mapping: 80:80${NC}"
echo
echo -e "${BLUE}🔧 Useful Commands:${NC}"
echo -e "${YELLOW}Check container status:${NC} docker ps"
echo -e "${YELLOW}Check container logs:${NC} docker logs ros2-sdc"
echo -e "${YELLOW}Manual update:${NC} ./auto-update.sh"
echo -e "${YELLOW}Check auto-update logs:${NC} tail -f /var/log/auto-update.log"
echo -e "${YELLOW}Check cron job:${NC} crontab -l"
echo
echo -e "${GREEN}🚀 Your EC2 instance will now automatically update every 30 minutes!${NC}"
echo -e "${BLUE}💡 Every time you push changes to GitHub, they'll be deployed to EC2 within 30 minutes.${NC}"
