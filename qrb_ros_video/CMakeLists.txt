cmake_minimum_required(VERSION 3.8)
project(qrb_ros_video VERSION 0.1.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
include(GNUInstallDirs)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(qrb_video_v4l2_lib REQUIRED)

include_directories(std_msgs_INCLUDE_DIRS)
include_directories(sensor_msgs_INCLUDE_DIRS)
include_directories(rclcpp_components_INCLUDE_DIRS)

ament_auto_find_build_dependencies()
  
ament_auto_add_library(ros2video_component SHARED src/video_decoder.cpp src/video_encoder.cpp)
target_include_directories(ros2video_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ros2video_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_directories(ros2video_component PRIVATE $<TARGET_LINKER_FILE_DIR:v4l2codecs>)
set_target_properties(ros2video_component PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION 1)
target_link_libraries(ros2video_component lib_mem_dmabuf qrb_ros_transport_image_type v4l2codecs)
ament_target_dependencies(
  ros2video_component
  "std_msgs"
  "sensor_msgs"
  "rclcpp"
  "rclcpp_components"
  "qrb_video_v4l2_lib"
)

rclcpp_components_register_nodes(
  ros2video_component
  "qrb_ros::video::Decoder"
  "qrb_ros::video::PerfDecoder"
  "qrb_ros::video::Encoder"
  "qrb_ros::video::PerfEncoder"
)

ament_auto_package(INSTALL_TO_SHARE launch)
