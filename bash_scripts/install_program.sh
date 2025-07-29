#!/bin/bash

# Low-Cost-Sensors Repository Installation Script
# This script installs all necessary components for the sensor system

set -e  # Exit on any error

# Colors for output formatting
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging function
log() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

header() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}"
}

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

log "Script directory: $SCRIPT_DIR"
log "Repository directory: $REPO_DIR"

# Check if running as root (not recommended for most operations)
if [[ $EUID -eq 0 ]]; then
   warn "This script should not be run as root for most operations."
   warn "Some operations may require sudo and will prompt accordingly."
fi

header "1. Updating System Packages"
log "Updating package lists..."
sudo apt update

header "2. Installing ROS Dependencies"
log "Installing ROS Noetic and required packages..."

# Check if ROS is already installed
if ! command -v roscore &> /dev/null; then
    log "Installing ROS Noetic..."
    
    # Add ROS repository
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    
    # Add ROS key
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    
    # Update package lists
    sudo apt update
    
    # Install ROS Noetic
    sudo apt install -y ros-noetic-desktop-full
    
    # Initialize rosdep
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi
    rosdep update
    
    # Setup environment
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source /opt/ros/noetic/setup.bash
else
    log "ROS Noetic is already installed."
fi

# Install additional ROS packages
log "Installing additional ROS packages..."
sudo apt install -y \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-std-msgs

header "3. Installing Development Tools and Dependencies"
log "Installing development tools and dependencies..."
sudo apt install -y \
    git \
    cmake \
    build-essential \
    python3-pip \
    curl \
    wget \
    nano \
    vim

log "Installing FFmpeg and multimedia libraries..."
sudo apt install -y \
    ffmpeg \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    libavresample-dev

log "Installing LibUSB and USB development libraries..."
sudo apt install -y \
    libusb-1.0-0-dev \
    libusb-dev \
    libudev-dev

log "Installing NMEA communication libraries and GPS tools..."
sudo apt install -y \
    gpsd \
    gpsd-clients \
    libgps-dev \
    python3-gps \
    minicom \
    setserial

# Install NMEA parsing libraries for Python
log "Installing Python NMEA libraries..."
pip3 install \
    pynmea2 \
    pyserial \
    gpsd-py3

header "4. Cloning Git Submodules"
log "Initializing and updating git submodules..."
cd "$REPO_DIR"

# Initialize and update all submodules
git submodule init
git submodule update --recursive --remote

# Specifically handle the Onboard-SDK-ROS submodule
if [ -d "ros_workspace/src/Onboard-SDK-ROS" ]; then
    log "Onboard-SDK-ROS submodule found."
    cd ros_workspace/src/Onboard-SDK-ROS
    git checkout mainTesting || git checkout master || log "Using default branch"
    cd "$REPO_DIR"
else
    error "Onboard-SDK-ROS submodule not found!"
    exit 1
fi

header "5. Setting up ROS Workspace"
log "Setting up ROS workspace and building packages..."

# Source ROS environment
source /opt/ros/noetic/setup.bash

# Navigate to ROS workspace
cd "$REPO_DIR/ros_workspace"

# Install dependencies using rosdep
log "Installing ROS package dependencies..."
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace using catkin_make
log "Building ROS workspace with catkin_make..."
catkin_make

# Source the workspace
if [ -f "devel/setup.bash" ]; then
    source devel/setup.bash
    log "ROS workspace built successfully!"
else
    error "Failed to build ROS workspace!"
    exit 1
fi

header "6. Installing Python Dependencies"
log "Installing Python dependencies for the sensor dashboard..."

# Install Python dependencies for the dashboard if requirements.txt exists
if [ -f "$REPO_DIR/ground_station/sensor-dashboard/requirements.txt" ]; then
    log "Installing Python requirements..."
    pip3 install -r "$REPO_DIR/ground_station/sensor-dashboard/requirements.txt"
fi

header "7. Installing System Service"
log "Setting up system service for automatic startup..."

# Update the service file with the correct paths
SERVICE_FILE="$SCRIPT_DIR/sensor_system.service"
TEMP_SERVICE="/tmp/sensor_system.service"

# Get the current user
CURRENT_USER=$(whoami)
HOME_DIR=$(eval echo ~$CURRENT_USER)

# Create a temporary service file with updated paths
sed "s|User=shivamwalia|User=$CURRENT_USER|g" "$SERVICE_FILE" > "$TEMP_SERVICE"
sed -i "s|/home/shivamwalia/Low-Cost-Sensors|$REPO_DIR|g" "$TEMP_SERVICE"
sed -i "s|Environment=ROS_LOG_DIR=/home/shivamwalia/.ros/log|Environment=ROS_LOG_DIR=$HOME_DIR/.ros/log|g" "$TEMP_SERVICE"

# Make the start script executable
chmod +x "$SCRIPT_DIR/start_sensor_system.sh"

# Update the start script with correct paths
START_SCRIPT="$SCRIPT_DIR/start_sensor_system.sh"
TEMP_START_SCRIPT="/tmp/start_sensor_system.sh"

sed "s|/home/shivamwalia/Low-Cost-Sensors|$REPO_DIR|g" "$START_SCRIPT" > "$TEMP_START_SCRIPT"

# Copy the updated start script back
cp "$TEMP_START_SCRIPT" "$START_SCRIPT"
chmod +x "$START_SCRIPT"

# Install the service
sudo cp "$TEMP_SERVICE" /etc/systemd/system/sensor_system.service

# Reload systemd and enable the service
sudo systemctl daemon-reload
sudo systemctl enable sensor_system.service

log "System service installed and enabled for automatic startup."

header "8. Setting up Environment"
log "Setting up environment variables and aliases..."

# Add ROS workspace to bashrc if not already present
if ! grep -q "source $REPO_DIR/ros_workspace/devel/setup.bash" ~/.bashrc; then
    echo "# Low-Cost-Sensors ROS Workspace" >> ~/.bashrc
    echo "source $REPO_DIR/ros_workspace/devel/setup.bash" >> ~/.bashrc
    log "Added ROS workspace to ~/.bashrc"
fi

# Create .ros directory if it doesn't exist
mkdir -p "$HOME_DIR/.ros/log"

header "9. Installation Complete!"
log "All components have been installed successfully!"
echo ""
log "Next steps:"
log "1. Reboot your system to start the service automatically, or run:"
log "   sudo systemctl start sensor_system.service"
log ""
log "2. To manually run the sensor system:"
log "   cd $REPO_DIR/bash_scripts"
log "   ./start_sensor_system.sh"
log ""
log "3. To check service status:"
log "   sudo systemctl status sensor_system.service"
log ""
log "4. To view service logs:"
log "   sudo journalctl -u sensor_system.service -f"
log ""
warn "Please source your bashrc or restart your terminal:"
warn "source ~/.bashrc"

# Clean up temporary files
rm -f "$TEMP_SERVICE" "$TEMP_START_SCRIPT"

log "Installation script completed successfully!"