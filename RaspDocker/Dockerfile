FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# Install all required build dependencies
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

# Create a non-root user
RUN useradd -m -s /bin/bash dji && echo "dji ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER dji
WORKDIR /home/dji

# Copy your edited Onboard-SDK source into the container
COPY Onboard-SDK /home/dji/Onboard-SDK

RUN sudo chown -R dji:dji /home/dji/Onboard-SDK

# Build it
WORKDIR /home/dji/Onboard-SDK
RUN mkdir build && cd build && cmake .. -DADVANCED_SENSING=OFF && make -j$(nproc)

CMD ["/bin/bash"]
