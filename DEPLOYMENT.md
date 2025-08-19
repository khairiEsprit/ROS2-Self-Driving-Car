# 🚀 ROS2 Self-Driving Car - Simplified Deployment Guide

This guide shows you how to deploy the ROS2 Self-Driving Car project using the **original working Docker image** (`noshluk2/ros2-self-driving-car-ai-using-opencv`) with automated updates.

## 📋 Prerequisites

- Ubuntu 18.04+ or EC2 instance
- Internet connection
- Sudo access

## 🎯 One-Command Setup

### Option 1: Complete Auto-Setup (Recommended)

```bash
# Download and run the quick-start script
curl -fsSL https://raw.githubusercontent.com/khairiEsprit/ROS2-Self-Driving-Car/main/quick-start.sh | bash
```

### Option 2: Manual Setup

```bash
# 1. Clone your repository
git clone https://github.com/khairiEsprit/ROS2-Self-Driving-Car.git
cd ROS2-Self-Driving-Car

# 2. Make scripts executable
chmod +x quick-start.sh
chmod +x docker/*.sh

# 3. Run quick setup
./quick-start.sh
```

## 🔧 Manual Installation Steps

If you prefer to install components manually:

### 1. Install Docker

```bash
# Update system
sudo apt-get update

# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add user to docker group
sudo usermod -aG docker $USER

# Log out and log back in, then test
docker --version
```

### 2. Install Git

```bash
sudo apt-get install -y git
```

### 3. Clone and Setup Project

```bash
# Clone your repository
git clone https://github.com/khairiEsprit/ROS2-Self-Driving-Car.git
cd ROS2-Self-Driving-Car

# Setup automated deployment
./docker/setup-auto-update.sh

# Start the application
./docker/deploy.sh
```

## 🎮 Usage Commands

Once setup is complete, use these commands:

```bash
# Navigate to project directory
cd ROS2-Self-Driving-Car

# Start/restart the application
./docker/deploy.sh

# Check application status
./docker/deploy.sh status

# Stop the application
./docker/deploy.sh stop

# View application logs
./docker/deploy.sh logs

# Connect to container shell
./docker/deploy.sh shell

# Interactive development container
./docker/create_container.bash

# Manual update (auto-update runs every 30 minutes)
./docker/auto-update.sh
```

## 🔄 Automated Updates

The system is configured to automatically:

- Pull latest code changes every 30 minutes
- Restart containers with updated code
- Use the original working Docker image
- Log all activities

### Check Auto-Update Logs

```bash
# View auto-update logs
tail -f /var/log/ros2-sdc/auto-update.log

# View recent auto-update activity
sudo journalctl -u cron | grep auto-update
```

## 🌐 EC2 Deployment

### Security Group Settings

Make sure your EC2 security group allows:

- SSH (port 22) for access
- Any additional ports your application needs

### EC2 Setup Commands

```bash
# Connect to your EC2 instance
ssh -i your-key.pem ubuntu@your-ec2-ip

# Run the one-command setup
curl -fsSL https://raw.githubusercontent.com/khairiEsprit/ROS2-Self-Driving-Car/main/quick-start.sh | bash

# Or manual setup
git clone https://github.com/khairiEsprit/ROS2-Self-Driving-Car.git
cd ROS2-Self-Driving-Car
./quick-start.sh
```

## 🔍 Troubleshooting

### Docker Issues

```bash
# Check Docker status
sudo systemctl status docker

# Restart Docker
sudo systemctl restart docker

# Check Docker permissions
groups $USER  # Should include 'docker'
```

### Container Issues

```bash
# List all containers
docker ps -a

# Remove stuck containers
docker stop $(docker ps -aq)
docker rm $(docker ps -aq)

# Clean up Docker resources
docker system prune -a
```

### Auto-Update Issues

```bash
# Check cron jobs
crontab -l

# Check auto-update logs
tail -f /var/log/ros2-sdc/auto-update.log

# Manual update
./docker/auto-update.sh
```

## 📁 Project Structure

```
ROS2-Self-Driving-Car/
├── quick-start.sh              # One-command setup script
├── DEPLOYMENT.md              # This file
├── docker/
│   ├── deploy.sh              # Main deployment script
│   ├── auto-update.sh         # Automated update script
│   ├── create_container.bash  # Interactive container
│   └── setup-auto-update.sh   # Setup automation
├── self_driving_car_pkg/      # ROS2 packages
└── ...
```

## 🎯 Key Features

- ✅ Uses original working Docker image
- ✅ Automated code synchronization
- ✅ Zero-downtime updates
- ✅ Easy EC2 deployment
- ✅ Comprehensive logging
- ✅ Simple command interface
- ✅ No custom Docker builds required

## 📞 Support

If you encounter issues:

1. Check the logs: `tail -f /var/log/ros2-sdc/auto-update.log`
2. Verify Docker is running: `docker ps`
3. Try manual deployment: `./docker/deploy.sh`
4. Clean and restart: `./docker/deploy.sh stop && ./docker/deploy.sh`

---

**Happy Coding! 🚗💨**
