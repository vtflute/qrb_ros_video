# Debian Packaging for QRB Video V4L2 Library

This document explains the separate debian packaging structure for the QRB Video V4L2 Library project.

## Package Structure

This project is split into two separate debian packages:

### qrb_video_v4l2_lib
**Location**: `qrb_video_v4l2_lib/debian/`

**Generated Packages**:
- `libqrb-video-v4l2-1` - Runtime library package
- `libqrb-video-v4l2-dev` - Development package (headers, CMake config)

**Dependencies**: 
- libgoogle-glog-dev
- Standard C++ libraries

## Build Order

Due to the dependency relationship, packages must be built in this order:

1. **qrb_video_v4l2_lib** (provides the library)
2. **qrb_ros_video** (depends on the library)

## Building Packages

### Build Individual Packages

#### Build qrb_video_v4l2_lib:
```bash
cd qrb_video_v4l2_lib
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
```

## Package Contents

### libqrb-video-v4l2-1
- `/usr/lib/libv4l2codecs.so.1*` - Shared library

### libqrb-video-v4l2-dev  
- `/usr/lib/libv4l2codecs.so` - Development symlink
- `/usr/include/qrb_video/` - Header files
- `/usr/share/qrb_video_v4l2_lib/` - CMake configuration files

## Advantages of Separate Packaging

1. **Clean Dependencies**: Each package has only the dependencies it needs
2. **Reusable Library**: qrb_video_v4l2_lib can be used by other projects
3. **Standard Debian Practices**: Follows library packaging conventions
4. **Easier Maintenance**: Changes to one component don't affect the other
5. **Better Distribution**: Library and ROS packages can be distributed separately