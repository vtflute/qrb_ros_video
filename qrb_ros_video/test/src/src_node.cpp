/*
**************************************************************************************************
* Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "base_node.hpp"
#include "qrb_ros_transport_image_type/image.hpp"
#include "qrb_ros_transport_image_type/image_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

namespace qrb_ros
{
namespace video
{

// Forward declarations of specialized classes for registration
class CompressedReader;
class ImageReader;

/**
 * @brief Base source node template class for different message types
 * @tparam MessageT The message type to publish
 */
template <typename MessageT>
class SourceNodeBase : public BaseNode
{
public:
  explicit SourceNodeBase(const std::string & node_name, const rclcpp::NodeOptions & options)
    : BaseNode(node_name, options)

  {
    // Create publisher for output data - specific type will be set by derived
    // class
    publisher_ = this->create_publisher<MessageT>("output", 10);

    // Create GStreamer pipeline based on mode
    setup_pipeline();

    // Create timer to publish data at 1/fps interval
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / fps_)),
        std::bind(&SourceNodeBase::timer_callback, this));
  }

  ~SourceNodeBase()
  {
    // Base class will clean up GStreamer pipeline
  }

protected:
  // Method to fill specific message type - to be implemented by derived classes
  virtual void fill_message(typename std::unique_ptr<MessageT> & msg,
      const uint8_t * data,
      size_t size) = 0;

  // Setup the GStreamer pipeline
  const std::map<std::string, std::function<std::string()>> pipeline_template = {
    { "nv12",
        [this]() {
          std::ostringstream oss;
          oss << "filesrc location=" << url_ << " ! rawvideoparse width=" << width_
              << " height=" << height_ << " format=nv12"
              << " framerate=" << framerate_ << " ! appsink name=sink";
          return oss.str();
        } },
    { "mp4",
        [this]() {
          std::ostringstream oss;
          oss << "filesrc location=" << url_ << " ! qtdemux"
              << " ! " << pixel_format_ << "parse ! video/x-" << pixel_format_ <<  ",stream-format=byte-stream,alignment=au"
              << " ! appsink name=sink";
          return oss.str();
        } },
    { "h264",
        [this]() {
          std::ostringstream oss;
          oss << "filesrc location=" << url_
              << " ! h264parse ! video/x-h264,stream-format=byte-stream,alignment=au ! appsink "
                 "name=sink";
          return oss.str();
        } },
    { "h265",
        [this]() {
          std::ostringstream oss;
          oss << "filesrc location=" << url_
              << " ! h265parse ! video/x-h265,stream-format=byte-stream,alignment=au ! appsink "
                 "name=sink";
          return oss.str();
        } },
  };

  void setup_pipeline() override
  {
    std::string pipeline_str;
    try {
      discover_pipeline();
      pipeline_str = pipeline_template.at(format_)();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
          this->get_logger(), "Invalid format: %s. Use 'encoder' or 'decoder'", format_.c_str());
      return;
    }

    if (!create_pipeline(pipeline_str)) {
      return;
    }

    // Get the appsink element
    sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
    if (!sink_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get appsink element");
      return;
    }

    // Configure appsink
    app_sink_ = GST_APP_SINK(sink_);
    gst_app_sink_set_emit_signals(app_sink_, TRUE);
    gst_app_sink_set_drop(app_sink_, FALSE);
    gst_app_sink_set_max_buffers(app_sink_, 3);

    // Connect the new-sample signal
    g_signal_connect(app_sink_, "new-sample", G_CALLBACK(on_new_sample_static), this);

    // Start the pipeline
    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start pipeline");
    }
  }

  static GstFlowReturn on_new_sample_static(GstAppSink * sink, gpointer user_data)
  {
    return static_cast<SourceNodeBase *>(user_data)->on_new_sample(sink);
  }

  GstFlowReturn on_new_sample(GstAppSink * sink)
  {
    // just keep the count of available samples
    available_sample++;

    return GST_FLOW_OK;
  }

  void timer_callback()
  {
    static int count = 0;
    if (app_sink_ && available_sample > 0) {
      GstSample * sample = gst_app_sink_pull_sample(app_sink_);
      if (!sample) {
        RCLCPP_WARN(this->get_logger(), "No sample available");
        return;
      }
      count++;

      GstBuffer * buffer = gst_sample_get_buffer(sample);
      GstMapInfo map;

      auto msg = std::make_unique<MessageT>();
      // Fill message header
      auto nano = this->now().nanoseconds();
      msg->header.stamp.sec = nano / 1000000000;
      msg->header.stamp.nanosec = nano % 1000000000;
      msg->header.frame_id = std::to_string(count);

      if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        // Store the buffer data for later publication
        // Use specialized method to fill message-specific fields
        fill_message(msg, map.data, map.size);
        gst_buffer_unmap(buffer, &map);
      }

      // Publish the message
      publisher_->publish(std::move(msg));

      gst_sample_unref(sample);
      available_sample--;

      if (available_sample == 0 && gst_app_sink_is_eos(app_sink_)) {
        // Stop the timer if EOS is reached
        timer_->cancel();
      }
    }
  }

protected:
  GstElement * sink_ = nullptr;
  GstAppSink * app_sink_ = nullptr;

  typename rclcpp::Publisher<MessageT>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex buffer_mutex_;
  std::atomic<uint32_t> available_sample = 0;
  GstClockTime buffer_timestamp_ = GST_CLOCK_TIME_NONE;
  std::thread main_thread_;
};

/**
 * @brief Source node specialized for CompressedImage messages
 */
class CompressedReader : public SourceNodeBase<sensor_msgs::msg::CompressedImage>
{
public:
  explicit CompressedReader(const rclcpp::NodeOptions & options)
    : SourceNodeBase<sensor_msgs::msg::CompressedImage>("CompressedReader", options)
  {
  }

protected:
  void fill_message(sensor_msgs::msg::CompressedImage::UniquePtr & msg,
      const uint8_t * data,
      size_t size) override
  {
    // Set format based on mode
    msg->format = format_;

    // Copy the data
    msg->data.resize(size);
    memcpy(msg->data.data(), data, size);
  }
};

/**
 * @brief Source node specialized for qrb_ros::transport::type::Image messages
 */
class ImageReader : public SourceNodeBase<qrb_ros::transport::type::Image>
{
public:
  explicit ImageReader(const rclcpp::NodeOptions & options)
    : SourceNodeBase<qrb_ros::transport::type::Image>("ImageReader", options)
  {
  }

protected:
  void fill_message(std::unique_ptr<qrb_ros::transport::type::Image> & msg,
      const uint8_t * data,
      size_t size) override
  {
    // Configure message based on mode
    msg->encoding = format_;
    msg->width = std::stoi(width_);
    msg->height = std::stoi(height_);
    size_t aligned_size = qrb_ros::transport::image_utils::get_image_align_size(
        msg->width, msg->height, msg->encoding);
    msg->dmabuf = lib_mem_dmabuf::DmaBuffer::alloc(aligned_size, "/dev/dma_heap/system");
    if (!msg->dmabuf) {
      RCLCPP_ERROR(this->get_logger(), "Failed to allocate dmabuf");
      return;
    }
    qrb_ros::transport::image_utils::save_image_to_dmabuf(
        msg->dmabuf, data, msg->width, msg->height, msg->width, msg->encoding, true);
    msg->dmabuf->set_auto_release(false);
  }
};

}  // namespace video
}  // namespace qrb_ros

// Register the components with class loader
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::video::CompressedReader)
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::video::ImageReader)