#!/bin/bash

# Setup script for automated ROS2 Self-Driving Car deployment
# Uses original working image: noshluk2/ros2-self-driving-car-ai-using-opencv

echo "🚀 Setting up automated deployment for ROS2 Self-Driving Car..."
echo "================================================================"

# Get the script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "📁 Project directory: $PROJECT_DIR"
echo "📁 Script directory: $SCRIPT_DIR"

# Make scripts executable
echo "🔧 Making scripts executable..."
chmod +x "$SCRIPT_DIR/auto-update.sh"
chmod +x "$SCRIPT_DIR/deploy.sh"
chmod +x "$SCRIPT_DIR/create_container.bash"

# Create log directory
sudo mkdir -p /var/log/ros2-sdc
sudo chown $USER:$USER /var/log/ros2-sdc

echo "⏰ Setting up automated updates..."

# Remove any existing cron jobs for this project
crontab -l 2>/dev/null | grep -v "auto-update.sh" | grep -v "ros2-sdc" | crontab -

# Add new cron job to run every 30 minutes
(crontab -l 2>/dev/null; echo "*/30 * * * * $SCRIPT_DIR/auto-update.sh >> /var/log/ros2-sdc/auto-update.log 2>&1") | crontab -

echo "✅ Setup completed successfully!"
echo ""
echo "📋 What was configured:"
echo "   • Scripts made executable"
echo "   • Auto-update scheduled every 30 minutes"
echo "   • Logs will be saved to /var/log/ros2-sdc/"
echo ""
echo "🎯 Quick commands:"
echo "   • Start deployment:      $SCRIPT_DIR/deploy.sh"
echo "   • Manual update:         $SCRIPT_DIR/auto-update.sh"
echo "   • Interactive container: $SCRIPT_DIR/create_container.bash"
echo "   • Check auto-update logs: tail -f /var/log/ros2-sdc/auto-update.log"
echo ""
echo "🔄 Auto-update will start running in 30 minutes"
echo "   To run it now: $SCRIPT_DIR/auto-update.sh"
