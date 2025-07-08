FROM arm64v8/ubuntu:16.04

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

RUN apt-get update && apt-get install -y libsdl2-dev

# ROS Kinetic Dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    ca-certificates && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# ROS1 Repo
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# ROS Kinetic and tools
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-base \
    ros-kinetic-roslaunch \
    ros-kinetic-roscpp \
    ros-kinetic-std-msgs \
    ros-kinetic-nmea-msgs \
    ros-kinetic-sensor-msgs \
    ros-kinetic-geometry-msgs \
    ros-kinetic-nav-msgs \
    ros-kinetic-tf \
    ros-kinetic-message-generation \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    python-catkin-tools \
    build-essential

RUN rosdep init || true && rosdep update || true

RUN useradd -m -s /bin/bash dji && echo "dji ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
RUN usermod -aG dialout dji

COPY catkin_ws /home/dji/catkin_ws
RUN chown -R dji:dji /home/dji/catkin_ws

RUN git clone --recurse-submodules https://github.com/dji-sdk/Onboard-SDK-ROS.git /home/dji/catkin_ws/src/dji_osdk_ros
RUN git clone https://github.com/shivam-2507/Onboard-SDK.git /home/dji/Onboard-SDK

WORKDIR /home/dji/Onboard-SDK
RUN mkdir build && cd build && \
    cmake .. && make -j4 && make install

WORKDIR /home/dji/catkin_ws
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make"

WORKDIR /home/dji
RUN echo "source /opt/ros/kinetic/setup.bash" >> /home/dji/.bashrc
RUN echo "source /home/dji/catkin_ws/devel/setup.bash" >> /home/dji/.bashrc

COPY .env /home/dji/.env
COPY run.sh /home/dji/run.sh
RUN chmod +x /home/dji/run.sh && chown dji:dji /home/dji/run.sh

USER dji
ENTRYPOINT ["/home/dji/run.sh"]