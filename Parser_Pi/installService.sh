#!/bin/bash
set -e

echo "[INFO] Building project..."
mkdir -p build && cd build
cmake ..
make
cd ..

echo "[INFO] Installing executable..."
sudo cp build/main /usr/local/bin/
sudo chmod +x /usr/local/bin/my_program

echo "[INFO] Copying systemd service file..."
sudo cp my_program.service /etc/systemd/system/

echo "[INFO] Enabling service..."
sudo systemctl daemon-reexec
sudo systemctl enable my_program.service
sudo systemctl start my_program.service

echo "[DONE] my_program is now running at boot."