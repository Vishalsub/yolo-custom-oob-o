#!/bin/bash

# System packages
sudo apt update
sudo apt install -y python3-pip python3-venv libgl1-mesa-glx libgtk-3-dev libusb-1.0-0-dev

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install Python packages
pip install --upgrade pip
pip install -r requirements.txt

# Install RealSense SDK
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
mkdir build && cd build
cmake ../ -DBUILD_EXAMPLES=true
make -j$(nproc)
sudo make install
