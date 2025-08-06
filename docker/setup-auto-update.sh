#!/bin/bash

# Simple setup for auto-update cron job

echo "Setting up auto-update..."

# Make script executable
chmod +x auto-update.sh

# Add cron job to run every 30 minutes
(crontab -l 2>/dev/null; echo "*/30 * * * * $(pwd)/auto-update.sh >> /var/log/auto-update.log 2>&1") | crontab -

echo "Setup complete!"
echo "Auto-update will run every 30 minutes"
echo "Check logs with: tail -f /var/log/auto-update.log"
