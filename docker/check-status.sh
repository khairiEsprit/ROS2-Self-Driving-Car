#!/bin/bash

# Monitoring script for ROS2 Self Driving Car Docker deployment
# Use this to check the status of your auto-update system

# Configuration
IMAGE_NAME="khairiesprit/ros2-self-driving-car:latest"
SERVICE_NAME="yourapp"
LOG_FILE="/var/log/ros2-sdc-auto-update.log"

echo "=== ROS2 Self Driving Car System Status ==="
echo ""

# Check Docker service
echo "🐳 Docker Service Status:"
if systemctl is-active --quiet docker; then
    echo "✅ Docker is running"
else
    echo "❌ Docker is not running"
fi
echo ""

# Check if our service exists and its status
echo "🚗 Application Service Status:"
if systemctl list-unit-files | grep -q "$SERVICE_NAME.service"; then
    if systemctl is-active --quiet "$SERVICE_NAME"; then
        echo "✅ Service $SERVICE_NAME is running"
    else
        echo "❌ Service $SERVICE_NAME is not running"
        echo "   Status: $(systemctl is-active $SERVICE_NAME)"
    fi
else
    echo "⚠️  Service $SERVICE_NAME is not installed"
fi
echo ""

# Check Docker containers
echo "📦 Docker Container Status:"
CONTAINER_STATUS=$(docker ps -a --filter "name=$SERVICE_NAME" --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}")
if [ -n "$CONTAINER_STATUS" ]; then
    echo "$CONTAINER_STATUS"
else
    echo "❌ No container found with name: $SERVICE_NAME"
fi
echo ""

# Check Docker image
echo "🖼️  Docker Image Information:"
IMAGE_INFO=$(docker images "$IMAGE_NAME" --format "table {{.Repository}}\t{{.Tag}}\t{{.CreatedAt}}\t{{.Size}}")
if [ -n "$IMAGE_INFO" ]; then
    echo "$IMAGE_INFO"
else
    echo "❌ Image not found: $IMAGE_NAME"
fi
echo ""

# Check cron job
echo "⏰ Cron Job Status:"
CRON_JOB=$(sudo crontab -l 2>/dev/null | grep "auto-update.sh")
if [ -n "$CRON_JOB" ]; then
    echo "✅ Cron job is configured:"
    echo "   $CRON_JOB"
else
    echo "❌ No cron job found for auto-update"
fi
echo ""

# Show recent logs
echo "📝 Recent Update Logs (last 10 lines):"
if [ -f "$LOG_FILE" ]; then
    tail -10 "$LOG_FILE"
else
    echo "❌ Log file not found: $LOG_FILE"
fi
echo ""

# Network status
echo "🌐 Network Connectivity:"
if curl -s --max-time 5 https://hub.docker.com > /dev/null; then
    echo "✅ Internet connection is working"
else
    echo "❌ No internet connection or Docker Hub is unreachable"
fi
echo ""

# Disk space check
echo "💾 Disk Space Usage:"
df -h / | tail -1
echo ""

# Show system resources
echo "🖥️  System Resources:"
echo "CPU Usage: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | sed 's/%us,//')"
echo "Memory Usage: $(free -h | awk '/^Mem:/ {print $3 "/" $2}')"
echo ""

echo "=== End of Status Report ==="
