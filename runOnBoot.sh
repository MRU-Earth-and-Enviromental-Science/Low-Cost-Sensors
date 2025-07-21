#!/bin/bash

# =============================================================================
# Low-Cost Sensors System Boot Script
# =============================================================================
# This script automatically starts the Low-Cost Sensors data collection system
# on boot. It manages all ROS nodes and handles system initialization.
#
# Usage: ./runOnBoot.sh [start|stop|restart|status]
# =============================================================================

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR/rosWorkspace"
LOG_DIR="/var/log/low-cost-sensors"
PID_DIR="$HOME/.low-cost-sensors"
ROS_LOG_DIR="$LOG_DIR/ros"

# Ensure directories exist
mkdir -p "$LOG_DIR" "$PID_DIR" "$ROS_LOG_DIR"

# PID files
ROSCORE_PID="$PID_DIR/roscore.pid"
SENSOR_SYSTEM_PID="$PID_DIR/sensor_system.pid"

# Log files
ROSCORE_LOG="$LOG_DIR/roscore.log"
SENSOR_SYSTEM_LOG="$LOG_DIR/sensor_system.log"
SYSTEM_LOG="$LOG_DIR/system.log"

# ESP32 Configuration (auto-detect or set manually)
ESP32_PORT=""
ESP32_BAUD="115200"

# Logging function
log() {
    echo -e "${BLUE}[$(date +'%Y-%m-%d %H:%M:%S')]${NC} $1"
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] $1" >> "$SYSTEM_LOG"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1"
    echo "[ERROR $(date +'%Y-%m-%d %H:%M:%S')] $1" >> "$SYSTEM_LOG"
}

warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
    echo "[WARNING $(date +'%Y-%m-%d %H:%M:%S')] $1" >> "$SYSTEM_LOG"
}

success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
    echo "[SUCCESS $(date +'%Y-%m-%d %H:%M:%S')] $1" >> "$SYSTEM_LOG"
}

# =============================================================================
# Utility Functions
# =============================================================================

# Check if process is running
is_running() {
    local pid_file="$1"
    if [ -f "$pid_file" ]; then
        local pid=$(cat "$pid_file")
        if ps -p "$pid" > /dev/null 2>&1; then
            return 0
        else
            rm -f "$pid_file"
            return 1
        fi
    fi
    return 1
}

# Wait for ROS master to be ready
wait_for_rosmaster() {
    log "Waiting for ROS master to be ready..."
    local timeout=30
    local count=0
    
    while [ $count -lt $timeout ]; do
        if rostopic list > /dev/null 2>&1; then
            success "ROS master is ready"
            return 0
        fi
        sleep 1
        count=$((count + 1))
        echo -n "."
    done
    
    error "ROS master failed to start within $timeout seconds"
    return 1
}

# Auto-detect ESP32 port
detect_esp32_port() {
    log "Auto-detecting ESP32 port..."
    
    # Common ESP32 ports
    local ports=("/dev/ttyUSB0" "/dev/ttyUSB1" "/dev/ttyACM0" "/dev/ttyACM1")
    
    for port in "${ports[@]}"; do
        if [ -e "$port" ]; then
            log "Found potential ESP32 port: $port"
            ESP32_PORT="$port"
            return 0
        fi
    done
    
    warning "No ESP32 port detected automatically. Please set ESP32_PORT manually in script."
    return 1
}

# Setup ROS environment
setup_ros_environment() {
    log "Setting up ROS environment..."
    
    # Source ROS
    source /opt/ros/noetic/setup.bash
    
    # Source workspace if it exists
    if [ -f "$WORKSPACE_DIR/devel/setup.bash" ]; then
        source "$WORKSPACE_DIR/devel/setup.bash"
    else
        warning "Workspace not built. Please run 'catkin_make' in $WORKSPACE_DIR"
    fi
    
    # Set ROS environment variables
    export ROS_MASTER_URI="http://localhost:11311"
    export ROS_HOSTNAME="localhost"
    export ROS_LOG_DIR="$ROS_LOG_DIR"
}

# =============================================================================
# Start Functions
# =============================================================================

start_roscore() {
    if is_running "$ROSCORE_PID"; then
        log "ROS core is already running"
        return 0
    fi
    
    log "Starting ROS core..."
    setup_ros_environment
    
    nohup roscore > "$ROSCORE_LOG" 2>&1 &
    echo $! > "$ROSCORE_PID"
    
    if wait_for_rosmaster; then
        success "ROS core started successfully (PID: $(cat $ROSCORE_PID))"
        return 0
    else
        error "Failed to start ROS core"
        return 1
    fi
}

start_sensor_system() {
    if is_running "$SENSOR_SYSTEM_PID"; then
        log "Sensor system is already running"
        return 0
    fi
    
    log "Starting sensor system..."
    setup_ros_environment
    
    # Auto-detect ESP32 port if not set
    if [ -z "$ESP32_PORT" ]; then
        detect_esp32_port
    fi
    
    # Check if ESP32 port is available
    if [ -n "$ESP32_PORT" ] && [ -e "$ESP32_PORT" ]; then
        log "Using ESP32 port: $ESP32_PORT"
        ESP32_ARGS="esp32Port:=$ESP32_PORT esp32Baud:=$ESP32_BAUD"
    else
        warning "ESP32 port not available. System will run without ESP32 connection."
        ESP32_ARGS=""
    fi
    
    # Start the sensor system
    cd "$WORKSPACE_DIR"
    nohup roslaunch ros_to_raw masterSystem.launch $ESP32_ARGS > "$SENSOR_SYSTEM_LOG" 2>&1 &
    echo $! > "$SENSOR_SYSTEM_PID"
    
    sleep 5  # Give system time to start
    
    if is_running "$SENSOR_SYSTEM_PID"; then
        success "Sensor system started successfully (PID: $(cat $SENSOR_SYSTEM_PID))"
        return 0
    else
        error "Failed to start sensor system"
        return 1
    fi
}

# =============================================================================
# Stop Functions
# =============================================================================

stop_sensor_system() {
    if is_running "$SENSOR_SYSTEM_PID"; then
        local pid=$(cat "$SENSOR_SYSTEM_PID")
        log "Stopping sensor system (PID: $pid)..."
        
        # Graceful shutdown
        kill -TERM "$pid" 2>/dev/null || true
        sleep 5
        
        # Force kill if still running
        if ps -p "$pid" > /dev/null 2>&1; then
            warning "Sensor system didn't stop gracefully, force killing..."
            kill -KILL "$pid" 2>/dev/null || true
        fi
        
        rm -f "$SENSOR_SYSTEM_PID"
        success "Sensor system stopped"
    else
        log "Sensor system is not running"
    fi
}

stop_roscore() {
    if is_running "$ROSCORE_PID"; then
        local pid=$(cat "$ROSCORE_PID")
        log "Stopping ROS core (PID: $pid)..."
        
        # Graceful shutdown
        kill -TERM "$pid" 2>/dev/null || true
        sleep 3
        
        # Force kill if still running
        if ps -p "$pid" > /dev/null 2>&1; then
            warning "ROS core didn't stop gracefully, force killing..."
            kill -KILL "$pid" 2>/dev/null || true
        fi
        
        rm -f "$ROSCORE_PID"
        success "ROS core stopped"
    else
        log "ROS core is not running"
    fi
}

# =============================================================================
# Status Function
# =============================================================================

show_status() {
    echo "==============================================================================="
    echo "Low-Cost Sensors System Status"
    echo "==============================================================================="
    
    # ROS Core Status
    if is_running "$ROSCORE_PID"; then
        echo -e "ROS Core: ${GREEN}RUNNING${NC} (PID: $(cat $ROSCORE_PID))"
    else
        echo -e "ROS Core: ${RED}STOPPED${NC}"
    fi
    
    # Sensor System Status
    if is_running "$SENSOR_SYSTEM_PID"; then
        echo -e "Sensor System: ${GREEN}RUNNING${NC} (PID: $(cat $SENSOR_SYSTEM_PID))"
    else
        echo -e "Sensor System: ${RED}STOPPED${NC}"
    fi
    
    # ESP32 Status
    if [ -n "$ESP32_PORT" ] && [ -e "$ESP32_PORT" ]; then
        echo -e "ESP32 Port: ${GREEN}AVAILABLE${NC} ($ESP32_PORT)"
    else
        echo -e "ESP32 Port: ${RED}NOT AVAILABLE${NC}"
    fi
    
    echo "==============================================================================="
    echo "Log files:"
    echo "  System: $SYSTEM_LOG"
    echo "  ROS Core: $ROSCORE_LOG"
    echo "  Sensor System: $SENSOR_SYSTEM_LOG"
    echo "==============================================================================="
}

# =============================================================================
# Main Functions
# =============================================================================

start_system() {
    log "Starting Low-Cost Sensors System..."
    
    # Check if workspace exists
    if [ ! -d "$WORKSPACE_DIR" ]; then
        error "Workspace directory not found: $WORKSPACE_DIR"
        exit 1
    fi
    
    start_roscore
    sleep 2
    start_sensor_system
    
    success "Low-Cost Sensors System started successfully!"
    show_status
}

stop_system() {
    log "Stopping Low-Cost Sensors System..."
    
    stop_sensor_system
    stop_roscore
    
    success "Low-Cost Sensors System stopped successfully!"
}

restart_system() {
    log "Restarting Low-Cost Sensors System..."
    stop_system
    sleep 3
    start_system
}

# =============================================================================
# Main Script Logic
# =============================================================================

case "${1:-start}" in
    start)
        start_system
        ;;
    stop)
        stop_system
        ;;
    restart)
        restart_system
        ;;
    status)
        show_status
        ;;
    *)
        echo "Usage: $0 {start|stop|restart|status}"
        echo ""
        echo "Commands:"
        echo "  start   - Start the sensor system"
        echo "  stop    - Stop the sensor system"
        echo "  restart - Restart the sensor system"
        echo "  status  - Show system status"
        exit 1
        ;;
esac
