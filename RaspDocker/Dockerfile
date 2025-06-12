FROM arm64v8/ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    libusb-1.0-0-dev \
    libeigen3-dev \
    libtinyxml2-dev \
    libserial-dev \
    libssl-dev \
    git \
    nano \
    sudo \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Create user
RUN useradd -m -s /bin/bash dji && echo "dji ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
RUN usermod -aG dialout dji

USER dji
WORKDIR /home/dji

# Clone Onboard SDK with samples
RUN git clone https://github.com/dji-sdk/Onboard-SDK.git

# Build SDK and Linux samples
WORKDIR /home/dji/Onboard-SDK
RUN mkdir build && cd build && \
    cmake .. && \
    make -j4 && \
    sudo make install

# Provide default UserConfig.txt
COPY --chown=dji:dji UserConfig.txt /home/dji/Onboard-SDK/sample/platform/linux/common/UserConfig.txt

# Provide entry script
COPY --chown=dji:dji run_sample.sh /home/dji/run_sample.sh
RUN chmod +x /home/dji/run_sample.sh

ENTRYPOINT ["/home/dji/run_sample.sh"]