# Gazebo Docker Display and Audio Issues - Solutions

## Problem Analysis

The errors you're seeing are common when running Gazebo in Docker containers, especially with GUI applications. Here are the main issues and solutions:

### Issue 1: X11 Display Authorization

**Error**: `Authorization required, but no authorization protocol specified`

**Cause**: Docker container can't access the host's X11 display server.

**Solutions**:

#### Solution A: Enable X11 Forwarding (Recommended)

```bash
# On your EC2 instance, allow X11 forwarding
xhost +local:docker

# Or more securely, allow specific user
xhost +local:$(whoami)

# Run container with proper X11 setup
docker run -it --rm \
  --name ros2-sdc-interactive \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd):/workspace \
  --net=host \
  khairiii/ros2-self-driving-car:latest
```

#### Solution B: Use VNC Display (Current Setup)

Since you're using VNC, set the display to VNC:

```bash
# Inside container
export DISPLAY=:1

# Or when running container
docker run -it --rm \
  --name ros2-sdc-interactive \
  -e DISPLAY=:1 \
  -v $(pwd):/workspace \
  --net=host \
  khairiii/ros2-self-driving-car:latest
```

#### Solution C: Run Gazebo Headless (No GUI)

```bash
# Use gzserver only (no GUI)
ros2 launch self_driving_car_pkg world_gazebo.launch.py headless:=true

# Or set environment variable
export GAZEBO_HEADLESS=1
ros2 launch self_driving_car_pkg world_gazebo.launch.py
```

### Issue 2: Audio Device Access

**Error**: ALSA audio errors

**Cause**: Container can't access host audio devices.

**Solutions**:

#### Solution A: Mount Audio Devices

```bash
docker run -it --rm \
  --name ros2-sdc-interactive \
  --device /dev/snd \
  -e PULSE_RUNTIME_PATH=/var/run/pulse \
  -v $(pwd):/workspace \
  khairiii/ros2-self-driving-car:latest
```

#### Solution B: Disable Audio (Recommended for Headless)

```bash
# Set environment variables to disable audio
export ALSA_CARD=-1
export PULSE_RUNTIME_PATH=""

# Or add to Dockerfile
ENV ALSA_CARD=-1
ENV PULSE_RUNTIME_PATH=""
```

### Issue 3: Gazebo Client Crashes

**Cause**: GUI client can't render without proper display setup.

**Solutions**:

#### Solution A: Use Headless Mode

```bash
# Run without GUI
gzserver world_file.world

# Or modify launch file to disable GUI
```

#### Solution B: Use Software Rendering

```bash
# Use software rendering instead of hardware
export LIBGL_ALWAYS_SOFTWARE=1
export GAZEBO_RESOURCE_PATH=""
```

### Issue 4: Entity Spawn Timeouts

**Cause**: Gazebo simulation not fully initialized when spawn commands run.

**Solutions**:

#### Solution A: Add Delays in Launch File

```python
# In your launch file, add delays between spawns
import time
from launch.actions import TimerAction

# Add timer before spawning
TimerAction(period=5.0, actions=[spawn_entity_action])
```

#### Solution B: Increase Timeout

```python
# In spawn service call, increase timeout
timeout_sec=30.0
```

## Complete Fix for Your Setup

### Step 1: Update Docker Run Command

Create an updated container startup script:

```bash
#!/bin/bash
# run_gazebo_container.sh

# Enable X11 forwarding for VNC
export DISPLAY=:1

# Allow X11 connections
xhost +local:docker

# Run container with proper environment
docker run -it --rm \
  --name ros2-sdc-interactive \
  -e DISPLAY=:1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e ALSA_CARD=-1 \
  -e PULSE_RUNTIME_PATH="" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd):/workspace \
  --net=host \
  --privileged \
  khairiii/ros2-self-driving-car:latest bash
```

### Step 2: Create Headless Launch Option

Create a headless version of your launch file:

```python
# world_gazebo_headless.launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Use gzserver only (no GUI)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gzserver.launch.py'
        ]),
        launch_arguments={'world': world_file_path}.items()
    )

    return LaunchDescription([
        gazebo,
        # Add your other nodes here
    ])
```

### Step 3: Test Solutions

#### Test 1: Headless Mode

```bash
# Inside container
ros2 launch self_driving_car_pkg world_gazebo.launch.py headless:=true
```

#### Test 2: With VNC Display

```bash
# Set display and run
export DISPLAY=:1
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch self_driving_car_pkg world_gazebo.launch.py
```

#### Test 3: Check X11 Connection

```bash
# Test if X11 is working
xeyes  # Should open a GUI window in VNC
glxinfo | grep "direct rendering"  # Check OpenGL
```

## Quick Fix Commands

Run these commands on your EC2 instance:

```bash
# 1. Allow X11 forwarding
xhost +local:docker

# 2. Set environment for software rendering
export DISPLAY=:1
export LIBGL_ALWAYS_SOFTWARE=1
export ALSA_CARD=-1

# 3. Run container with proper setup
docker run -it --rm \
  --name ros2-sdc-interactive \
  -e DISPLAY=:1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e ALSA_CARD=-1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd):/workspace \
  --net=host \
  $(pwd):/workspace \
  khairiii/ros2-self-driving-car:latest bash

# 4. Inside container, test Gazebo
gzserver --verbose  # Test server only first
```

## Prevention

### Add to Dockerfile

```dockerfile
# Add these environment variables to prevent audio/display issues
ENV DISPLAY=:1
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV ALSA_CARD=-1
ENV PULSE_RUNTIME_PATH=""
ENV QT_X11_NO_MITSHM=1

# Install minimal X11 dependencies
RUN apt-get update && apt-get install -y \
    x11-utils \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*
```

### Update Launch Files

Add headless options to your launch files:

```python
headless = LaunchConfiguration('headless', default='false')

# Conditional GUI launching
gui_condition = PythonExpression([
    "not ", headless
])
```

This should resolve the authorization, audio, and GUI issues you're experiencing with Gazebo in Docker.
