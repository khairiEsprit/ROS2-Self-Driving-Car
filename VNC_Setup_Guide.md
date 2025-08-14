# Complete VNC Setup Guide for EC2 Ubuntu Instance

## Prerequisites

- EC2 Ubuntu instance running
- SSH key pair (`.pem` file) for EC2 access
- Security group configured to allow SSH (port 22)
- VNC client on Windows (like RealVNC Viewer, TightVNC, or UltraVNC)

## Part 1: EC2 Security Group Configuration

### 1.1 Configure Security Group

1. Go to AWS Console → EC2 → Security Groups
2. Select your instance's security group
3. Add the following inbound rules:
   - **SSH**: Port 22, Source: Your IP or 0.0.0.0/0
   - **VNC**: Port 5901, Source: Your IP (for direct VNC access - optional)

**Note**: We'll use SSH tunneling instead of direct VNC access for better security.

## Part 2: Server Setup (On EC2 Ubuntu Instance)

### 2.1 Connect to EC2 Instance

```bash
# From Windows PowerShell/Command Prompt
ssh -i your-key.pem ubuntu@your-ec2-public-ip
```

### 2.2 Update System

```bash
sudo apt update
sudo apt upgrade -y
```

### 2.3 Install Desktop Environment

```bash
# Install lightweight desktop (XFCE)
sudo apt install xfce4 xfce4-goodies -y

# Alternative: Install Ubuntu Desktop (heavier)
# sudo apt install ubuntu-desktop-minimal -y
```

### 2.4 Install VNC Server

```bash
# Install TigerVNC Server
sudo apt install tigervnc-standalone-server tigervnc-xorg-extension -y

# Alternative: Install TightVNC
# sudo apt install tightvncserver -y
```

### 2.5 Configure VNC Server

#### 2.5.1 Set VNC Password

```bash
vncpasswd
# Enter password (8 characters max)
# Enter verify password
# Choose 'n' for view-only password (or 'y' if you want read-only access)
```

#### 2.5.2 Create VNC Startup Script

```bash
# Create xstartup file
nano ~/.vnc/xstartup
```

Add the following content:

```bash
#!/bin/bash
xrdb $HOME/.Xresources
startxfce4 &
```

Make it executable:

```bash
chmod +x ~/.vnc/xstartup
```

### 2.6 Start VNC Server

```bash
# Start VNC server on display :1 with specific resolution
vncserver :1 -geometry 1920x1080 -depth 24

# Check if VNC is running
ps aux | grep vnc
```

### 2.7 Configure VNC as System Service (Optional)

Create systemd service file:

```bash
sudo nano /etc/systemd/system/vncserver@.service
```

Add the following content:

```ini
[Unit]
Description=Start TigerVNC server at startup
After=syslog.target network.target

[Service]
Type=forking
User=ubuntu
Group=ubuntu
WorkingDirectory=/home/ubuntu

PIDFile=/home/ubuntu/.vnc/%H:%i.pid
ExecStartPre=-/usr/bin/vncserver -kill :%i > /dev/null 2>&1
ExecStart=/usr/bin/vncserver -depth 24 -geometry 1920x1080 :%i
ExecStop=/usr/bin/vncserver -kill :%i

[Install]
WantedBy=multi-user.target
```

Enable and start the service:

```bash
sudo systemctl daemon-reload
sudo systemctl enable vncserver@1.service
sudo systemctl start vncserver@1.service
```

## Part 3: Client Setup (On Windows)

### 3.1 Install VNC Client

Download and install one of these VNC clients:

- **RealVNC Viewer**: https://www.realvnc.com/en/connect/download/viewer/
- **TightVNC Viewer**: https://www.tightvnc.com/download.php
- **UltraVNC**: https://uvnc.com/downloads/ultravnc.html

### 3.2 Connect Using SSH Tunnel (Recommended - Secure Method)

#### 3.2.1 Create SSH Tunnel

```powershell
# From Windows PowerShell
ssh -i RosKey.pem -L 5901:localhost:5901 ubuntu@your-ec2-public-ip
```

This command:

- `-i RosKey.pem`: Uses your SSH key
- `-L 5901:localhost:5901`: Forwards local port 5901 to remote port 5901
- Keeps the connection open

#### 3.2.2 Connect VNC Client

1. Open your VNC client
2. Connect to: `localhost:5901` or `127.0.0.1:5901`
3. Enter the VNC password you set earlier

### 3.3 Connect Directly (Less Secure - Not Recommended)

If you added port 5901 to security group:

1. Connect to: `your-ec2-public-ip:5901`
2. Enter VNC password

## Part 4: Troubleshooting

### 4.1 Common Issues and Solutions

#### VNC Server Not Starting

```bash
# Check VNC log files
ls ~/.vnc/
cat ~/.vnc/your-hostname:1.log

# Kill existing VNC processes
vncserver -kill :1

# Restart VNC server
vncserver :1 -geometry 1920x1080 -depth 24
```

#### Connection Refused

```bash
# Check if VNC is running
ps aux | grep vnc

# Check listening ports
netstat -tlnp | grep 5901

# Restart VNC server
vncserver -kill :1
vncserver :1 -geometry 1920x1080 -depth 24
```

#### Black Screen in VNC

```bash
# Check xstartup file
cat ~/.vnc/xstartup

# Make sure it's executable
chmod +x ~/.vnc/xstartup

# Restart VNC
vncserver -kill :1
vncserver :1 -geometry 1920x1080 -depth 24
```

#### SSH Tunnel Issues

```powershell
# Make sure SSH tunnel is active
# Check if port 5901 is forwarded
netstat -an | findstr 5901

# Reconnect SSH tunnel
ssh -i RosKey.pem -L 5901:localhost:5901 ubuntu@your-ec2-public-ip
```

### 4.2 Security Considerations

1. **Use SSH Tunneling**: Always use SSH tunnel instead of direct VNC connection
2. **Strong VNC Password**: Use a strong 8-character password
3. **Firewall Rules**: Restrict security group rules to your IP only
4. **Regular Updates**: Keep your system updated
5. **VNC over SSL**: Consider using VNC over SSL for additional security

### 4.3 Performance Optimization

```bash
# Start VNC with lower color depth for better performance
vncserver :1 -geometry 1280x720 -depth 16

# Use compression in VNC client settings
# Enable "Adaptive" or "Fast" quality settings in VNC viewer
```

## Part 5: Useful Commands

### VNC Server Management

```bash
# List VNC sessions
vncserver -list

# Start VNC server
vncserver :1 -geometry 1920x1080 -depth 24

# Stop VNC server
vncserver -kill :1

# Stop all VNC servers
vncserver -kill :*

# Check VNC processes
ps aux | grep vnc

# View VNC log
tail -f ~/.vnc/*.log
```

### System Information

```bash
# Check system resources
htop
free -h
df -h

# Check network connections
netstat -tlnp | grep 5901
ss -tlnp | grep 5901
```

## Part 6: Alternative Desktop Environments

### GNOME (Heavy but full-featured)

```bash
sudo apt install ubuntu-desktop-minimal -y
# Update xstartup to: gnome-session &
```

### LXDE (Very lightweight)

```bash
sudo apt install lxde -y
# Update xstartup to: startlxde &
```

### KDE Plasma (Feature-rich)

```bash
sudo apt install kde-plasma-desktop -y
# Update xstartup to: startkde &
```

## Quick Setup Script

Here's a quick setup script for Ubuntu:

```bash
#!/bin/bash
# Quick VNC Setup Script for Ubuntu EC2

# Update system
sudo apt update && sudo apt upgrade -y

# Install desktop environment and VNC server
sudo apt install xfce4 xfce4-goodies tigervnc-standalone-server tigervnc-xorg-extension -y

# Create VNC directory
mkdir -p ~/.vnc

# Create xstartup file
cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
xrdb $HOME/.Xresources
startxfce4 &
EOF

# Make xstartup executable
chmod +x ~/.vnc/xstartup

echo "Setup complete! Now run 'vncpasswd' to set password and 'vncserver :1 -geometry 1920x1080 -depth 24' to start VNC server"
```

Save this as `setup_vnc.sh`, make it executable with `chmod +x setup_vnc.sh`, and run with `./setup_vnc.sh`.
