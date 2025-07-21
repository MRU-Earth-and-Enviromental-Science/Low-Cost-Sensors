#!/bin/bash

# =============================================================================
# Low-Cost Sensors System Installation Script
# =============================================================================
# This script installs all dependencies required for the Low-Cost Sensors system
# including ROS Noetic, DJI Onboard SDK, and other necessary packages.
#
# Usage: sudo ./install.sh
# =============================================================================

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging function
log() {
    echo -e "${BLUE}[$(date +'%Y-%m-%d %H:%M:%S')]${NC} $1"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1"
    exit 1
}

warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

# Check if running as root
if [[ $EUID -eq 0 ]]; then
   error "This script should not be run as root. Please run as a regular user with sudo privileges."
fi

# Check Ubuntu version
UBUNTU_VERSION=$(lsb_release -rs)
if [[ "$UBUNTU_VERSION" != "20.04" ]]; then
    warning "This script is designed for Ubuntu 20.04. You are running $UBUNTU_VERSION. Proceeding anyway..."
fi

log "Starting Low-Cost Sensors System installation..."

# =============================================================================
# System Update
# =============================================================================
log "Updating system packages..."
sudo apt update && sudo apt upgrade -y

# =============================================================================
# Install Basic Dependencies
# =============================================================================
log "Installing basic dependencies..."
sudo apt install -y \
    curl \
    wget \
    git \
    build-essential \
    cmake \
    python3-pip \
    python3-dev \
    pkg-config \
    software-properties-common \
    apt-transport-https \
    ca-certificates \
    gnupg \
    lsb-release \
    vim \
    htop \
    screen \
    tmux

# =============================================================================
# Install ROS Noetic
# =============================================================================
log "Installing ROS Noetic..."

# Setup ROS repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Update package list
sudo apt update

# Install ROS Noetic Desktop Full
sudo apt install -y ros-noetic-desktop-full

# Initialize rosdep
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    sudo rosdep init
fi
rosdep update

# Setup ROS environment
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Install additional ROS packages
log "Installing additional ROS packages..."
sudo apt install -y \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    ros-noetic-rosserial-arduino \
    ros-noetic-rosserial \
    ros-noetic-rosserial-python \
    ros-noetic-diagnostic-aggregator \
    ros-noetic-tf2-tools \
    ros-noetic-rqt-graph \
    ros-noetic-rviz

# =============================================================================
# Install DJI Onboard SDK
# =============================================================================
log "Installing DJI Onboard SDK dependencies..."

# Install DJI SDK dependencies
sudo apt install -y \
    libudev-dev \
    libusb-1.0-0-dev \
    libssl-dev

# Clone and build DJI Onboard SDK
log "Cloning DJI Onboard SDK..."
cd /tmp
if [ -d "Onboard-SDK" ]; then
    rm -rf Onboard-SDK
fi
git clone https://github.com/dji-sdk/Onboard-SDK.git
cd Onboard-SDK

# Build DJI SDK
log "Building DJI Onboard SDK..."
mkdir -p build
cd build
cmake ..
make -j$(nproc)
sudo make install

# Install DJI ROS packages
log "Installing DJI ROS packages..."
cd ~/
source /opt/ros/noetic/setup.bash

# Create catkin workspace for DJI if it doesn't exist
if [ ! -d "dji_ws" ]; then
    mkdir -p dji_ws/src
    cd dji_ws
    catkin_make
    echo "source ~/dji_ws/devel/setup.bash" >> ~/.bashrc
fi

cd ~/dji_ws/src
if [ ! -d "Onboard-SDK-ROS" ]; then
    git clone https://github.com/dji-sdk/Onboard-SDK-ROS.git
fi

cd ~/dji_ws
catkin_make

# =============================================================================
# Install Arduino and PlatformIO (for ESP32 development)
# =============================================================================
log "Installing Arduino IDE and PlatformIO..."

# Install Arduino IDE
wget -O arduino.tar.xz https://downloads.arduino.cc/arduino-1.8.19-linux64.tar.xz
tar -xf arduino.tar.xz
sudo mv arduino-1.8.19 /opt/arduino
sudo /opt/arduino/install.sh
rm arduino.tar.xz

# Install PlatformIO
pip3 install --user platformio
echo 'export PATH=$PATH:~/.local/bin' >> ~/.bashrc

# =============================================================================
# Install Python Dependencies
# =============================================================================
log "Installing Python dependencies..."
pip3 install --user \
    pyserial \
    numpy \
    matplotlib \
    pandas \
    scipy \
    jupyter

# =============================================================================
# Setup Serial Port Permissions
# =============================================================================
log "Setting up serial port permissions..."
sudo usermod -a -G dialout $USER
sudo usermod -a -G tty $USER

# =============================================================================
# Install System Service Dependencies
# =============================================================================
log "Installing system service dependencies..."
sudo apt install -y \
    systemd \
    rsyslog

# =============================================================================
# Create Workspace and Setup
# =============================================================================
log "Setting up workspace..."
WORKSPACE_DIR="$HOME/low_cost_sensors_ws"

if [ ! -d "$WORKSPACE_DIR" ]; then
    mkdir -p $WORKSPACE_DIR/src
    cd $WORKSPACE_DIR
    source /opt/ros/noetic/setup.bash
    catkin_make
    echo "source $WORKSPACE_DIR/devel/setup.bash" >> ~/.bashrc
fi

# =============================================================================
# Final Setup
# =============================================================================
log "Performing final setup..."

# Create log directory
sudo mkdir -p /var/log/low-cost-sensors
sudo chown $USER:$USER /var/log/low-cost-sensors

mkdir -p $HOME/.low-cost-sensors

# Setup systemd service (optional)
if [ -f "$HOME/Low-Cost-Sensors/low-cost-sensors.service" ]; then
    log "Setting up systemd service..."
    
    # Update service file with correct user and path
    sed -i "s|User=ubuntu|User=$USER|g" "$HOME/Low-Cost-Sensors/low-cost-sensors.service"
    sed -i "s|Group=ubuntu|Group=$USER|g" "$HOME/Low-Cost-Sensors/low-cost-sensors.service"
    sed -i "s|/home/ubuntu/Low-Cost-Sensors|$HOME/Low-Cost-Sensors|g" "$HOME/Low-Cost-Sensors/low-cost-sensors.service"
    
    # Install service
    sudo cp "$HOME/Low-Cost-Sensors/low-cost-sensors.service" /etc/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl enable low-cost-sensors.service
    
    success "Systemd service installed and enabled"
else
    warning "Systemd service file not found. Auto-start on boot not configured."
fi

# Source ROS setup
source /opt/ros/noetic/setup.bash

# =============================================================================
# Installation Complete
# =============================================================================
success "Installation completed successfully!"
echo ""
echo "==============================================================================="
echo "Installation Summary:"
echo "- ROS Noetic: Installed"
echo "- DJI Onboard SDK: Installed"
echo "- Arduino IDE: Installed"
echo "- PlatformIO: Installed"
echo "- Python dependencies: Installed"
echo "- Serial permissions: Configured"
echo "==============================================================================="
echo ""
warning "IMPORTANT: Please reboot your system or logout/login to apply group permissions!"
echo ""
echo "After reboot, you can:"
echo "1. Clone your Low-Cost-Sensors repository"
echo "2. Build your workspace with 'catkin_make'"
echo "3. Run the system with './runOnBoot.sh'"
echo ""
log "Installation script completed. Please reboot your system."
