FROM arm64v8/ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# OSDK Dependencies
RUN apt update && apt install -y --no-install-recommends \
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
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    ca-certificates && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# ROS1 Repo
RUN mkdir -p /etc/apt/keyrings && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" \
    > /etc/apt/sources.list.d/ros1.list

# ROS Noetic and tools
RUN apt-get update && apt-get install -y \
    ros-noetic-ros-base \
    ros-noetic-roslaunch \
    ros-noetic-roscpp \
    ros-noetic-std-msgs \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    build-essential

# Initialize rosdep (safe fallback for Docker)
RUN sudo rosdep init || true && rosdep update || true

RUN useradd -m -s /bin/bash dji && echo "dji ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN usermod -aG dialout dji

# Copy your edited Onboard-SDK source into the container
COPY Onboard-SDK /home/dji/Onboard-SDK

RUN chown -R dji:dji /home/dji/Onboard-SDK

#catkin setup
COPY catkin_ws /home/dji/catkin_ws
RUN chown -R dji:dji /home/dji/catkin_ws

WORKDIR /home/dji

# Source ROS in bash
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/dji/.bashrc

WORKDIR /home/dji/Onboard-SDK
RUN mkdir build && cd build && cmake .. -DADVANCED_SENSING=OFF && make -j1

WORKDIR /home/dji/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"
RUN echo "source /home/dji/catkin_ws/devel/setup.bash" >> /home/dji/.bashrc

COPY .env /home/dji/.env

COPY run.sh /home/dji/run.sh
RUN chmod +x /home/dji/run.sh && chown dji:dji /home/dji/run.sh
USER dji
ENTRYPOINT ["/home/dji/run.sh"]
