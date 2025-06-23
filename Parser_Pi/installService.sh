#!/bin/bash
set -e

echo "[INFO] Building project..."
mkdir -p build && cd build
cmake ..
make
cd ..

echo "[INFO] Installing executable..."
sudo cp build/main /usr/local/bin/
sudo chmod +x /usr/local/bin/main

echo "[INFO] Copying systemd service file..."
sudo cp boot.service /etc/systemd/system/

echo "[INFO] Enabling service..."
sudo systemctl daemon-reexec
sudo systemctl enable boot.service
sudo systemctl start boot.service

echo "[DONE] Use 'sudo reboot' to restart the device"
