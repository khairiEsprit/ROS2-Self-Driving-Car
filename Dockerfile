# Production-ready Dockerfile based on working noshluk2 image
# Optimized for Docker Hub automated builds
FROM osrf/ros:foxy-desktop-full

LABEL maintainer="khairiEsprit"
LABEL description="ROS2 Self Driving Car - Production Ready"

# Environment setup
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=foxy
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

SHELL ["/bin/bash", "-c"]

# Install system dependencies in optimized layers
RUN apt-get update && apt-get install -y \
    curl wget git vim nano htop \
    build-essential cmake pkg-config \
    python3-pip python3-dev python3-setuptools python3-wheel \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 packages
RUN source /opt/ros/foxy/setup.bash && \
    apt-get update && apt-get install -y \
    python3-colcon-common-extensions python3-rosdep python3-vcstool \
    ros-foxy-cv-bridge ros-foxy-image-transport \
    ros-foxy-camera-calibration-parsers ros-foxy-camera-info-manager \
    ros-foxy-robot-state-publisher ros-foxy-joint-state-publisher \
    ros-foxy-xacro ros-foxy-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# Install OpenCV and ML dependencies
RUN apt-get update && apt-get install -y \
    libopencv-dev python3-opencv libopencv-contrib-dev \
    libgtk-3-dev libgtk2.0-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev libxvidcore-dev libx264-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    libatlas-base-dev gfortran libeigen3-dev \
    liblapack-dev libblas-dev libhdf5-dev \
    python3-numpy python3-scipy python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages with fixed versions
RUN pip3 install --no-cache-dir \
    numpy==1.21.6 \
    scipy==1.7.3 \
    matplotlib==3.5.3 \
    opencv-python==4.6.0.66 \
    scikit-learn==1.1.3 \
    pandas==1.4.4 \
    pillow==9.2.0 \
    tensorflow==2.9.3 \
    keras==2.9.0

WORKDIR /workspace
COPY . /workspace/

# Create necessary directories
RUN mkdir -p /workspace/data/Output

# Install ROS dependencies
RUN rosdep update && \
    rosdep install --from-paths . --ignore-src -r -y || true

# Build workspace
RUN source /opt/ros/foxy/setup.bash && \
    colcon build --packages-select self_driving_car_pkg_models self_driving_car_pkg --symlink-install || true

# Final setup
RUN echo "All Done"

# Entrypoint
RUN echo '#!/bin/bash\n\
source /opt/ros/foxy/setup.bash\n\
if [ -d "/workspace/install" ]; then\n\
    source /workspace/install/setup.bash\n\
fi\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
EXPOSE 11311 11345
