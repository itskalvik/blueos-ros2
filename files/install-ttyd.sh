#!/usr/bin/env bash

# Immediately exit on errors
set -e

echo "Building ttyd from source for better compatibility..."

# Install build dependencies
apt-get update
apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    libssl-dev \
    libjson-c-dev \
    libwebsockets-dev \
    pkg-config
apt-get autoremove -y
apt-get clean -y
rm -rf /var/lib/apt/lists/*

# Clone ttyd repository
cd /tmp
git clone https://github.com/tsl0922/ttyd.git
cd ttyd

# Build ttyd
mkdir build
cd build
cmake ..
make -j$(nproc)

# Install ttyd
cp ttyd /usr/bin/ttyd
chmod +x /usr/bin/ttyd

# Clean up
cd /
rm -rf /tmp/ttyd

echo "ttyd built and installed successfully!"
ttyd --version