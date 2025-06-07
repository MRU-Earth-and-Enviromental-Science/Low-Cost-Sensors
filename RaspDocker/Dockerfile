FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# OSDK Dependencies
RUN apt update && apt install -y --no-install-recommends \
    git \
    cmake \
    build-essential \
    libusb-1.0-0-dev \
    libeigen3-dev \
    libtinyxml2-dev \
    libserial-dev \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    libssl-dev \
    nano \
    sudo \
    wget \
    && rm -rf /var/lib/apt/lists/*

# ROS Noetic Dependencies
# ROS Noetic Dependencies
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 1 Repo
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install ROS Noetic and tools
RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    build-essential

# Initialize rosdep (safely for Docker)
RUN mkdir -p /etc/ros/rosdep/sources.list.d && \
    rosdep init || true && \
    rosdep update || true

RUN useradd -m -s /bin/bash dji && echo "dji ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER dji
WORKDIR /home/dji

# Copy your edited Onboard-SDK source into the container
COPY Onboard-SDK /home/dji/Onboard-SDK

RUN sudo chown -R dji:dji /home/dji/Onboard-SDK

# Build it
WORKDIR /home/dji/Onboard-SDK
RUN mkdir build && cd build && cmake .. -DADVANCED_SENSING=OFF && make -j$(nproc)

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

COPY catkin_ws /root/catkin_ws
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
