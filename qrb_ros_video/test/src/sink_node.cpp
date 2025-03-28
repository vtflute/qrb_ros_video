/*
**************************************************************************************************
* Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#include <gst/allocators/gstdmabuf.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/video/gstvideometa.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "base_node.hpp"
#include "qrb_ros_transport_image_type/image.hpp"
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
class CompressedWriter;
class ImageWriter;

/**
 * @brief Base sink node template class for different message types
 * @tparam MessageT The message type to subscribe to
 */
template <typename MessageT>
class SinkNodeBase : public BaseNode
{
public:
  explicit SinkNodeBase(const std::string & node_name, const rclcpp::NodeOptions & options)
    : BaseNode(node_name, options), frame_counter_(0)
  {
    // Create subscription for input data - specific callback will be set by
    // derived class
    setup_subscription();

    // Create GStreamer pipeline based on mode
    setup_pipeline();
  }

  ~SinkNodeBase()
  {
    // Send EOS (End of Stream) and wait for pipeline to process it
    if (app_src_) {
      gst_app_src_end_of_stream(app_src_);

      // Give some time for the pipeline to complete
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Base class will clean up GStreamer pipeline
  }

protected:
  // Setup the subscription for the specific message type
  virtual void setup_subscription()
  {
    subscription_ = this->create_subscription<MessageT>(
        "input", 10, std::bind(&SinkNodeBase::handle_message, this, std::placeholders::_1));
  }

  virtual void handle_message(const std::shared_ptr<MessageT> msg) { this->process_message(msg); }

  // Method to extract data from specific message type - to be implemented by
  // derived classes
  virtual void extract_data(const std::shared_ptr<MessageT> & msg, GstBuffer * buffer) = 0;

  void process_message(const std::shared_ptr<MessageT> & msg)
  {
    if (!app_src_ || !pipeline_) {
      RCLCPP_ERROR(this->get_logger(), "Pipeline or appsrc is NULL");
      return;
    }

    // Extract data from the message
    GstBuffer * buffer = nullptr;
    extract_data(msg, buffer);
    if (!buffer) {
      RCLCPP_ERROR(this->get_logger(), "Failed to extract data from message");
      return;
    }

    RCLCPP_DEBUG(
        this->get_logger(), "Received frame of size: %zu bytes", gst_buffer_get_size(buffer));

    // Set buffer timestamp and duration (assuming 30fps)

    // Push the buffer into appsrc
    GstFlowReturn ret = gst_app_src_push_buffer(app_src_, buffer);
    if (ret != GST_FLOW_OK) {
      RCLCPP_ERROR(this->get_logger(), "Failed to push buffer to GStreamer pipeline: %d", ret);
    } else {
      frame_counter_++;
      RCLCPP_DEBUG(this->get_logger(), "Pushed frame %ld to pipeline", frame_counter_);
    }
  }

  const std::map<std::string, std::function<std::string()>> pipeline_template = {
    { "mp4",
        [this]() {
          // Efficient string construction using std::ostringstream
          std::ostringstream oss;
          oss << "appsrc name=src ! " << pixel_format_
              << "parse ! qtmux ! filesink location=" << url_;

          return oss.str();
        } },
    { "raw",
        [this]() {
          std::ostringstream oss;
          oss << "appsrc name=src format=time is-live=true ! filesink "
                 "location="
              << url_;
          return oss.str();
        } },
  };

  void setup_pipeline() override
  {
    std::string pipeline_str;
    try {
      pipeline_str = pipeline_template.at(format_)();

    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Invalid format: %s.", format_.c_str());
      return;
    }

    if (!create_pipeline(pipeline_str)) {
      return;
    }

    // Get the appsrc element
    src_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
    if (!src_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get appsrc element");
      return;
    }

    // Configure appsrc
    app_src_ = GST_APP_SRC(src_);
    gst_app_src_set_stream_type(app_src_, GST_APP_STREAM_TYPE_STREAM);
    gst_app_src_set_emit_signals(app_src_, TRUE);
    g_object_set(G_OBJECT(app_src_), "format", GST_FORMAT_TIME, NULL);
    g_object_set(G_OBJECT(app_src_), "is-live", TRUE, NULL);
    g_object_set(G_OBJECT(app_src_), "do-timestamp", TRUE, NULL);

    // Set caps based on mode
    GstCaps * caps;
    std::map<std::string, std::string> mime_mapping = { { "h264", "video/x-h264" },
      { "h265", "video/x-h265" } };
    if (format_ == "raw") {
      // For h264 encoding input (raw video)
      caps =
          gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "I420", "width", G_TYPE_INT,
              1920, "height", G_TYPE_INT, 1080, "framerate", GST_TYPE_FRACTION, 30, 1, NULL);
    } else {
      // For decoder input (h264)
      caps = gst_caps_new_simple(mime_mapping.at(format_).c_str(), "stream-format", G_TYPE_STRING,
          "byte-stream", "alignment", G_TYPE_STRING, "au", NULL);
    }

    gst_app_src_set_caps(app_src_, caps);
    gst_caps_unref(caps);

    // Connect need-data signal
    g_signal_connect(app_src_, "need-data", G_CALLBACK(on_need_data_static), this);

    // Start the pipeline
    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(this->get_logger(), "Failed to start pipeline");
    }
  }

  static void on_need_data_static(GstAppSrc * src, guint length, gpointer user_data)
  {
    static_cast<SinkNodeBase *>(user_data)->on_need_data(src, length);
  }

  void on_need_data(GstAppSrc * src, guint length)
  {
    // This function is called when the appsrc element needs more data
    // We don't need to do anything here, as we push data when we receive it via
    // ROS messages
    RCLCPP_INFO(this->get_logger(), "Src: %s, Need %d bytes", gst_element_get_name(src), length);
  }

  uint64_t frame_counter_;
  typename rclcpp::Subscription<MessageT>::SharedPtr subscription_;

  GstElement * src_ = nullptr;
  GstAppSrc * app_src_ = nullptr;
};

/**
 * @brief Sink node specialized for CompressedImage messages
 */
class CompressedWriter : public SinkNodeBase<sensor_msgs::msg::CompressedImage>
{
public:
  explicit CompressedWriter(const rclcpp::NodeOptions & options)
    : SinkNodeBase<sensor_msgs::msg::CompressedImage>("compressed_writer", options)
  {
  }

protected:
  struct ReleaseData
  {
    std::shared_ptr<sensor_msgs::msg::CompressedImage> data;
  };

  void extract_data(const std::shared_ptr<sensor_msgs::msg::CompressedImage> & msg,
      GstBuffer * buffer) override
  {
    auto data = msg->data;

    auto release_data = new ReleaseData{ msg };

    // Create a new buffer that wraps the vector's data
    buffer = gst_buffer_new_wrapped_full(GST_MEMORY_FLAG_READONLY, data.data(), data.size(), 0,
        data.size(), release_data, [](gpointer data) {
          auto release_data = static_cast<ReleaseData *>(data);
          delete release_data;
        });
    auto stamp = msg->header.stamp;
    GST_BUFFER_PTS(buffer) = stamp.sec * GST_SECOND + stamp.nanosec;

    return;
  }

private:
};

/**
 * @brief Sink node specialized for qrb_ros::transport::type::Image messages
 */
class ImageWriter : public SinkNodeBase<qrb_ros::transport::type::Image>
{
public:
  explicit ImageWriter(const rclcpp::NodeOptions & options)
    : SinkNodeBase<qrb_ros::transport::type::Image>("image_writer", options)
  {
  }

protected:
  struct ReleaseData
  {
    std::shared_ptr<qrb_ros::transport::type::Image> data;
  };

  void extract_data(const std::shared_ptr<qrb_ros::transport::type::Image> & msg,
      GstBuffer * buffer) override
  {
    std::map<std::string, GstVideoFormat> format_map = {
      { "nv12", GST_VIDEO_FORMAT_NV12 },
      { "i420", GST_VIDEO_FORMAT_I420 },
    };
    // Create a buffer from the dmabuf
    GstAllocator * allocator = gst_dmabuf_allocator_new();
    GstMemory * memory =
        gst_dmabuf_allocator_alloc(allocator, msg->dmabuf->fd(), msg->dmabuf->size());

    auto release_data = new ReleaseData{ msg };
    gst_mini_object_set_qdata(GST_MINI_OBJECT(memory),
        g_quark_from_string("qrb_ros::transport::type::Image"), release_data, [](gpointer data) {
          auto release_data = static_cast<ReleaseData *>(data);
          delete release_data;
        });

    buffer = gst_buffer_new();
    gst_buffer_append_memory(buffer, memory);

    auto stamp = msg->header.stamp;
    auto width = msg->width;
    auto height = msg->height;
    GstVideoFormat format = GST_VIDEO_FORMAT_UNKNOWN;
    try {
      format = format_map.at(msg->encoding);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Invalid encoding: %s", msg->encoding.c_str());
      return;
    }
    GST_BUFFER_PTS(buffer) = stamp.sec * GST_SECOND + stamp.nanosec;
    gst_buffer_add_video_meta(buffer, GST_VIDEO_FRAME_FLAG_NONE, format, width, height);

    return;
  }

private:
};

}  // namespace video
}  // namespace qrb_ros

// Register the components with class loader
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::video::CompressedWriter)
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::video::ImageWriter)
