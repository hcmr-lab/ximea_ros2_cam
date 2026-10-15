#ifndef XIMEA_ROS2_CAM__XIMEA_ROS2_CAM_HPP_
#define XIMEA_ROS2_CAM__XIMEA_ROS2_CAM_HPP_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <m3api/xiApi.h>
#include "ximea_ros2_cam/msg/xi_image_info.hpp"

namespace ximea_ros2_cam {

class XimeaRosCam : public rclcpp::Node {
public:
  explicit XimeaRosCam(const rclcpp::NodeOptions & options);
  virtual ~XimeaRosCam() override;

  XimeaRosCam(const XimeaRosCam &) = delete;
  XimeaRosCam & operator=(const XimeaRosCam &) = delete;

private:
  // One frame handed from the capture thread to the publish thread.
  struct PendingFrame {
    std::vector<uint8_t> data;
    XI_IMG meta;            // `meta.bp` is stale; only the scalar fields are used
    int64_t stamp_ns{0};
  };

  // Capture thread: opens/configures the camera, then loops on xiGetImage and
  // reconnects on failure. It is the only thread that touches xi_handle_.
  void captureLoop();
  bool openDevice();
  void closeDevice();
  void grabLoop();
  bool configureCamera();

  // Publish thread: takes the newest pending frame and publishes it.
  void publishLoop();

  void shutdown();
  // Interruptible sleep. Returns false if shutdown was requested.
  bool sleepFor(double seconds);

  // Returns how late (ns) this frame was delivered relative to the least-delayed
  // frame seen so far. Also maintains lat_offset_ns_, the running minimum of
  // (receipt time - camera hardware time).
  int64_t trackLatency(int64_t recv_ns, int64_t hw_ns);
  // Legacy anchoring (first frame, reset every hw_anchor_resync_period_s).
  int64_t legacyAnchoredStamp(int64_t recv_ns, int64_t hw_ns);
  void resetTimestampState();

  static const std::map<std::string, int> & imageFormatMap();
  static const std::map<std::string, std::string> & encodingMap();
  static const std::map<std::string, int> & bytesPerPixelMap();

  image_transport::CameraPublisher image_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr image_count_pub_;
  rclcpp::Publisher<ximea_ros2_cam::msg::XiImageInfo>::SharedPtr xi_image_info_pub_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_manager_;

  HANDLE xi_handle_{nullptr};

  // Threads and shutdown signalling
  std::thread capture_thread_;
  std::thread publish_thread_;
  std::atomic<bool> stop_{false};
  std::mutex stop_mutex_;
  std::condition_variable stop_cv_;

  // Latest-frame slot between the two threads (a new frame replaces an unpublished one)
  std::mutex slot_mutex_;
  std::condition_variable slot_cv_;
  PendingFrame slot_;
  bool slot_full_{false};

  std::atomic<uint64_t> frame_count_{0};

  // Pipeline health, reported (warnings only) from the capture thread
  std::atomic<uint32_t> stat_dropped_{0};
  std::atomic<int64_t> stat_publish_max_ns_{0};

  // Timestamp state (capture thread only)
  bool lat_valid_{false};
  int64_t lat_offset_ns_{0};
  int64_t lat_last_hw_ns_{0};
  bool hw_anchor_set_{false};
  int64_t hw_anchor_ros_ns_{0};
  int64_t hw_anchor_us_{0};
  int64_t last_stamp_ns_{0};

  std::string camera_name_, serial_no_, frame_id_, format_, encoding_;
  int format_int_{0};
  int bytes_per_pixel_{1};
  double poll_open_period_s_{2.0};
  double poll_frame_period_s_{0.001};   // deprecated: grabbing is no longer timer-driven
  bool use_hardware_timestamps_{false};
  bool hw_anchor_min_latency_{true};
  double hw_anchor_resync_period_s_{0.0};
  double hw_clock_drift_ppm_{100.0};
  double timestamp_offset_s_{0.0};
  double lag_warn_threshold_s_{0.25};

  bool publish_xi_image_info_{false};

  // --------- Camera configuration (loaded from parameters) ---------
  int    img_capture_timeout_ms_{1000};
  int    trigger_mode_{0};
  int    hw_trigger_edge_{0};
  bool   framerate_control_{false};
  int    framerate_set_{30};
  bool   auto_exposure_{true};
  int    exposure_time_us_{3000};
  double manual_gain_db_{0.0};
  int    auto_exposure_time_limit_us_{30000};
  double auto_exposure_priority_{0.8};
  double auto_gain_limit_db_{2.0};
  int    wb_mode_{0};
  double wb_kr_{1.0}, wb_kg_{1.0}, wb_kb_{1.0};
  int    roi_left_{0}, roi_top_{0}, roi_width_{0}, roi_height_{0};
  int    num_cams_in_bus_{1};
  double bw_safety_ratio_{0.9};
  int transport_buffer_commit_{32};
  int buffers_queue_size_{4};
  bool recent_frame_{true};
  int warmup_frames_{10};
  std::string camera_info_url_;
};

} // namespace ximea_ros2_cam

#endif // XIMEA_ROS2_CAM__XIMEA_ROS2_CAM_HPP_
