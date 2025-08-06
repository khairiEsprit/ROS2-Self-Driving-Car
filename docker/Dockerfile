# Use official ROS2 Humble base image
FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

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
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-calibration-parsers \
    ros-humble-camera-info-manager \
    ros-humble-launch-testing-ament-cmake \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /workspace

# Copy the entire project
COPY . /workspace/

# Install Python dependencies if requirements file exists
RUN pip3 install --upgrade pip && \
    pip3 install \
        tensorflow \
        keras \
        visualkeras \
        opencv-python \
        scikit-learn \
        pandas \
        pillow && \
    pip3 install git+https://github.com/keplr-io/quiver.git || true

# Source ROS2 setup and build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select self_driving_car_pkg self_driving_car_pkg_models --continue-on-error"

# Set up entrypoint
RUN echo '#!/bin/bash\nsource /opt/ros/humble/setup.bash\nsource /workspace/install/setup.bash\nexec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# Expose common ROS2 ports
EXPOSE 11311 11345
