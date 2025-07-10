# Debian Packaging for QRB ROS Video

This document explains the separate debian packaging structure for the QRB ROS Video project.

## Package Structure

This project is split into two separate debian packages:

### 1. qrb_video_v4l2_lib
**Location**: `qrb_video_v4l2_lib/debian/`

**Generated Packages**:
- `libqrb-video-v4l2-1` - Runtime library package
- `libqrb-video-v4l2-dev` - Development package (headers, CMake config)

**Dependencies**: 
- libgoogle-glog-dev
- Standard C++ libraries

### 2. qrb_ros_video  
**Location**: `qrb_ros_video/debian/`

**Generated Packages**:
- `ros-jazzy-qrb-ros-video` - ROS 2 package with video nodes

**Dependencies**:
- libqrb-video-v4l2-dev (build-time)
- libqrb-video-v4l2-1 (runtime)
- ROS 2 Jazzy packages

## Build Order

Due to the dependency relationship, packages must be built in this order:

1. **qrb_video_v4l2_lib** (provides the library)
2. **qrb_ros_video** (depends on the library)

## Building Packages

### Option 1: Build All Packages (Recommended)

Use the master build script that handles the dependency order automatically:

```bash
./build-all-debs.sh
```

This script will:
1. Build qrb_video_v4l2_lib packages
2. Install them temporarily for dependency resolution
3. Build qrb_ros_video package
4. Collect all packages in `build-packages/` directory

### Option 2: Build Individual Packages

#### Build qrb_video_v4l2_lib:
```bash
cd qrb_video_v4l2_lib
./build-deb.sh
```

#### Build qrb_ros_video:
```bash
# First install qrb_video_v4l2_lib packages
sudo dpkg -i ../libqrb-video-v4l2-*.deb

# Then build qrb_ros_video
cd qrb_ros_video  
./build-deb.sh
```

## Installation

Install all packages:
```bash
sudo dpkg -i build-packages/*.deb
sudo apt-get install -f  # Fix any missing dependencies
```

Or install individually:
```bash
# Install library first
sudo dpkg -i libqrb-video-v4l2-1_*.deb libqrb-video-v4l2-dev_*.deb

# Then install ROS package
sudo dpkg -i ros-jazzy-qrb-ros-video_*.deb
```

## Package Contents

### libqrb-video-v4l2-1
- `/usr/lib/libv4l2codecs.so.1*` - Shared library

### libqrb-video-v4l2-dev  
- `/usr/lib/libv4l2codecs.so` - Development symlink
- `/usr/include/qrb_video/` - Header files
- `/usr/share/qrb_video_v4l2_lib/` - CMake configuration files

### ros-jazzy-qrb-ros-video
- `/opt/ros/jazzy/lib/libros2video_component.so` - ROS component library
- `/opt/ros/jazzy/share/qrb_ros_video/` - ROS package files
- `/opt/ros/jazzy/include/qrb_ros_video/` - ROS package headers

## Advantages of Separate Packaging

1. **Clean Dependencies**: Each package has only the dependencies it needs
2. **Reusable Library**: qrb_video_v4l2_lib can be used by other projects
3. **Standard Debian Practices**: Follows library packaging conventions
4. **Easier Maintenance**: Changes to one component don't affect the other
5. **Better Distribution**: Library and ROS packages can be distributed separately