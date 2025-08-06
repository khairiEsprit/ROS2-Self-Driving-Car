# Simple Auto-Update Setup

## Quick Setup on EC2

1. **Upload files to EC2:**
   ```bash
   scp -i your-key.pem docker/* ubuntu@your-ec2-ip:/home/ubuntu/
   ```

2. **Run setup:**
   ```bash
   ssh -i your-key.pem ubuntu@your-ec2-ip
   cd /home/ubuntu
   ./setup-auto-update.sh
   ```

## What it does

- Pulls latest Docker image every 30 minutes
- Restarts your container automatically
- Cleans up old images

## Commands

- **Check if running:** `docker ps`
- **View logs:** `tail -f /var/log/auto-update.log`
- **Manual update:** `./auto-update.sh`
- **Check cron job:** `crontab -l`

That's it! Your container will auto-update every 30 minutes.
