FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Install curl (needed to fix ROS GPG key)
RUN apt-get update && apt-get install -y curl

# Fix expired ROS GPG key
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add -

# Install the rest of your dependencies
RUN apt-get update && apt-get install -y \
    git build-essential cmake \
    libusb-1.0-0-dev libtinyxml2-dev libboost-all-dev \
    python3-catkin-tools \
    ros-noetic-sensor-msgs \
    ros-noetic-roscpp

# Optional: install serial tools for debugging
RUN apt-get install -y minicom

# Create catkin workspace
WORKDIR /root/catkin_ws/src

# Clone DJI Onboard SDK (C++ base library, system-wide install)
RUN git clone https://github.com/dji-sdk/Onboard-SDK.git && \
    cd Onboard-SDK && mkdir build && cd build && \
    cmake .. && make -j4 && make install

# Clone DJI ROS wrapper
RUN git clone https://github.com/dji-sdk/Onboard-SDK-ROS.git dji_osdk_ros

# Copy your UART forwarder into workspace
COPY UART_Forwarder/ ./UART_Forwarder/

# Build everything
WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Set entrypoint
COPY launch_osdk.sh /root/
RUN chmod +x /root/launch_osdk.sh

CMD ["/root/launch_osdk.sh"]