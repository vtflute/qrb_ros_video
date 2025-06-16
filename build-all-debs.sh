#!/bin/bash

# Master build script for both qrb_video_v4l2_lib and qrb_ros_video packages

set -e  # Exit on any error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build-packages"

echo "=== Building QRB ROS Video Packages ==="
echo "Script directory: ${SCRIPT_DIR}"
echo "Build directory: ${BUILD_DIR}"

# Create build directory
mkdir -p "${BUILD_DIR}"

# Clean any previous packages
echo "Cleaning previous packages..."
rm -f "${BUILD_DIR}"/*.deb

echo ""
echo "=== Step 1: Building qrb_video_v4l2_lib ==="
echo ""

cd "${SCRIPT_DIR}/qrb_video_v4l2_lib"
./build-deb.sh

# Move generated packages to build directory
echo "Moving qrb_video_v4l2_lib packages..."
mv ../libqrb-video-v4l2-*.deb "${BUILD_DIR}/" 2>/dev/null || echo "No qrb_video_v4l2_lib packages found"

echo ""
echo "=== Step 2: Installing qrb_video_v4l2_lib for dependency ==="
echo ""

# Install the library packages so qrb_ros_video can find them
sudo dpkg -i "${BUILD_DIR}"/libqrb-video-v4l2-*.deb || {
    echo "Failed to install qrb_video_v4l2_lib packages"
    echo "Attempting to fix dependencies..."
    sudo apt-get install -f -y
    sudo dpkg -i "${BUILD_DIR}"/libqrb-video-v4l2-*.deb
}

echo ""
echo "=== Step 3: Building qrb_ros_video ==="
echo ""

cd "${SCRIPT_DIR}/qrb_ros_video"
./build-deb.sh

# Move generated packages to build directory
echo "Moving qrb_ros_video packages..."
mv ../ros-jazzy-qrb-ros-video*.deb "${BUILD_DIR}/" 2>/dev/null || echo "No qrb_ros_video packages found"

echo ""
echo "=== Build Summary ==="
echo "All packages built successfully!"
echo "Generated packages in ${BUILD_DIR}:"
ls -la "${BUILD_DIR}"/*.deb

echo ""
echo "=== Installation Instructions ==="
echo "To install all packages:"
echo "  sudo dpkg -i ${BUILD_DIR}/*.deb"
echo "  sudo apt-get install -f  # Fix any missing dependencies"