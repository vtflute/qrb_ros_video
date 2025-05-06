# Test Nodes for qrb_ros_video

This directory contains test nodes for the qrb_ros_video package, which provide GStreamer-based source and sink nodes for testing the video hardware acceleration capabilities. The nodes support both CompressedImage and custom Image message types.

## Available Nodes

### Source Nodes

Test nodes that act as input sources for the Encoder/Decoder. They extract data from a GStreamer pipeline using GstAppSink and publish it to a ROS2 topic.

#### CompressedReader
- **Topic**: `output`
- **Message Type**: `sensor_msgs/msg/CompressedImage`
- **GStreamer Pipelines**:
  - H264 mode: `filesrc ! h264parse ! stream-format=byte-stream,alignment=au ! appsink`
  - H265 mode: `filesrc ! h265parse ! stream-format=byte-stream,alignment=au ! appsink`
  - MP4 mode: `filesrc ! qtdemux ! h264parse ! stream-format=byte-stream,alignment=au ! appsink`

#### ImageReader
- **Topic**: `output`
- **Message Type**: `qrb_ros::transport::type::Image`
- **GStreamer Pipelines**:
  - Raw mode: `filesrc ! rawvideoparse ! appsink`

### Sink Nodes

Test nodes that act as output sinks for the Encoder/Decoder. They receive data from a ROS2 topic and send it to a GStreamer pipeline using GstAppSrc.

#### CompressedWriter
- **Topic**: `input`
- **Message Type**: `sensor_msgs/msg/CompressedImage`
- **GStreamer Pipelines**:
  - MP4 mode: `appsrc ! h264parse ! qtmux ! filesink`
  - Raw mode: `appsrc ! filesink`

#### ImageWriter
- **Topic**: `input`
- **Message Type**: `qrb_ros::transport::type::Image`
- **GStreamer Pipelines**:
  - Raw mode: `appsrc ! filesink`

## Parameters

Common parameters for all nodes:

- `format`: Pipeline format ("raw" or "mp4")
- `width`: Frame width
- `height`: Frame height
- `framerate`: Frame rate (e.g., "30/1")
- `log-level`: Logging level ("debug", "info", "warn")

Source node specific parameters:

- `url`: Path to input file
- `duration`: Duration for source ("1d", "2h", "34m56s")

Sink node specific parameters:

- `url`: Path to output file

## Notes

- Make sure the input file exists and is accessible
- The output file will be created in the specified location
- Both nodes will output debug information to help with troubleshooting
- The raw mode assumes raw video input (NV12 format)
- The mp4 mode assumes H.264 encoded input
- The ImageReader/Writer nodes use DMA buffers for efficient memory handling 