# Use official ROS2 Humble base image (Ubuntu 22.04)
FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    python3-sklearn \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-setuptools \
    python3-vcstool \
    curl \
    git \
    wget \
    vim \
    nano \
    htop \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-calibration-parsers \
    ros-humble-camera-info-manager \
    ros-humble-launch-testing-ament-cmake \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Set working directory
WORKDIR /workspace

# Copy the entire project
COPY . /workspace/

# Install Python dependencies if requirements file exists
RUN if [ -f "self_driving_car_pkg/self_driving_car_pkg/data/Requirements.txt" ]; then \
        pip3 install -r self_driving_car_pkg/self_driving_car_pkg/data/Requirements.txt; \
    fi

# Install any missing dependencies using rosdep
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths . --ignore-src -r -y || true"

# Source ROS2 setup and build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select self_driving_car_pkg self_driving_car_pkg_models --symlink-install"

# Set up entrypoint
RUN echo '#!/bin/bash\nsource /opt/ros/humble/setup.bash\nsource /workspace/install/setup.bash\nexec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# Expose common ROS2 ports
EXPOSE 11311 11345
