/*
**************************************************************************************************
* Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#ifndef QRB_ROS_VIDEO_TEST_BASE_NODE_HPP_
#define QRB_ROS_VIDEO_TEST_BASE_NODE_HPP_

#include <gst/gst.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace qrb_ros
{
namespace video
{

/**
 * @brief Base node template class for GStreamer-based ROS nodes
 */
class BaseNode : public rclcpp::Node
{
public:
  explicit BaseNode(const std::string & node_name, const rclcpp::NodeOptions & options)
    : Node(node_name, options)
    , format_(this->declare_parameter("format", "raw"))
    , pixel_format_(this->declare_parameter("pixel-format", "nv12"))
    , width_(this->declare_parameter("width", "640"))
    , height_(this->declare_parameter("height", "480"))
    , framerate_(this->declare_parameter("framerate", "30/1"))
    , log_level_(this->declare_parameter("log-level", "warning"))
    , url_(this->declare_parameter("url", ""))
  {
    // Initialize GStreamer
    if (!gst_is_initialized()) {
      gst_init(nullptr, nullptr);
    }

    auto pos = framerate_.find_first_of('/');
    std::string denominator = "1";
    if (pos != std::string::npos) {
      denominator = framerate_.substr(pos + 1);
    }
    std::string numerator = framerate_.substr(0, pos);
    fps_ = std::stof(numerator) / std::stof(denominator);

    // Set ROS Node log level
    auto log_level = rclcpp::Logger::Level::Warn;
    if (log_level_ == "info") {
      log_level = rclcpp::Logger::Level::Info;
    } else if (log_level_ == "debug") {
      log_level = rclcpp::Logger::Level::Debug;
    }

    get_logger().set_level(log_level);
  }

  virtual ~BaseNode()
  {
    // Clean up GStreamer pipeline
    if (pipeline_) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(pipeline_);
    }
  }

protected:
  /**
   * @brief Setup the GStreamer pipeline
   */
  virtual void setup_pipeline() = 0;

  // Common logging utility
  void log_pipeline_creation(const std::string & pipeline_str)
  {
    RCLCPP_INFO(this->get_logger(), "Creating pipeline: %s", pipeline_str.c_str());
  }

  // Common error handling for pipeline creation
  bool create_pipeline(const std::string & pipeline_str)
  {
    GError * error = nullptr;

    log_pipeline_creation(pipeline_str);

    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);

    if (error) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create pipeline: %s", error->message);
      g_error_free(error);
      return false;
    }

    return true;
  }

  std::string format_;
  std::string pixel_format_;
  std::string width_;
  std::string height_;
  std::string framerate_;
  std::string log_level_;
  std::string url_;
  float fps_ = 30.0f;
  GstElement * pipeline_ = nullptr;
};

}  // namespace video
}  // namespace qrb_ros

#endif  // QRB_ROS_VIDEO_TEST_BASE_NODE_HPP_