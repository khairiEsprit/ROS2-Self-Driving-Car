# Use official ROS2 Humble base image (Ubuntu 22.04)
FROM osrf/ros:humble-desktop-full

# Maintainer info
LABEL maintainer="khairiEsprit"
LABEL description="ROS2 Self Driving Car with AI and Computer Vision"

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
RUN pip3 install --upgrade pip setuptools wheel && \
    pip3 install --upgrade "packaging>=21.0" && \
    pip3 install \
        tensorflow \
        keras \
        visualkeras \
        opencv-python \
        scikit-learn \
        pandas \
        pillow && \
    pip3 install git+https://github.com/keplr-io/quiver.git || true

# Install any missing dependencies using rosdep
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths . --ignore-src -r -y || true"

# Source ROS2 setup and try to build the workspace (optional)
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    echo 'Attempting to build ROS2 packages...' && \
    (colcon build --packages-select self_driving_car_pkg_models || echo 'self_driving_car_pkg_models build failed, continuing...') && \
    echo 'Build completed successfully or with acceptable errors.'"

# Set up entrypoint
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
if [ -d "/workspace/install" ]; then\n\
    source /workspace/install/setup.bash\n\
else\n\
    echo "No install directory found. Building packages..."\n\
    cd /workspace\n\
    colcon build --packages-select self_driving_car_pkg_models self_driving_car_pkg --symlink-install --continue-on-error || true\n\
    if [ -d "/workspace/install" ]; then\n\
        source /workspace/install/setup.bash\n\
    fi\n\
fi\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# Expose common ROS2 ports
EXPOSE 11311 11345
