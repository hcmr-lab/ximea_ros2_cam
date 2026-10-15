#include "ximea_ros2_cam/ximea_ros2_cam.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>

namespace ximea_ros2_cam {

const std::map<std::string, int> & XimeaRosCam::imageFormatMap() {
  static const std::map<std::string, int> m = {
    {"XI_MONO8",      XI_MONO8},
    {"XI_MONO16",     XI_MONO16},
    {"XI_RGB24",      XI_RGB24},
    {"XI_RGB32",      XI_RGB32},
    {"XI_RGB_PLANAR", XI_RGB_PLANAR},
    {"XI_RAW8",       XI_RAW8},
    {"XI_RAW16",      XI_RAW16}
  };
  return m;
}

const std::map<std::string, std::string> & XimeaRosCam::encodingMap() {
  static const std::map<std::string, std::string> m = {
    {"XI_MONO8",      "mono8"},
    {"XI_MONO16",     "mono16"},
    {"XI_RGB24",      "bgr8"},
    {"XI_RGB32",      "bgra8"},
    {"XI_RGB_PLANAR", "not_applicable"},
    {"XI_RAW8",       "mono8"},
    {"XI_RAW16",      "mono16"}
  };
  return m;
}

const std::map<std::string, int> & XimeaRosCam::bytesPerPixelMap() {
  static const std::map<std::string, int> m = {
    {"XI_MONO8",      1},
    {"XI_MONO16",     2},
    {"XI_RGB24",      3},
    {"XI_RGB32",      4},
    {"XI_RGB_PLANAR", 3},
    {"XI_RAW8",       1},
    {"XI_RAW16",      2}
  };
  return m;
}

XimeaRosCam::XimeaRosCam(const rclcpp::NodeOptions & options)
: Node("ximea_cam_node", options) 
{
  // identity / topics
  serial_no_       = this->declare_parameter("serial_no", "");
  camera_name_     = this->declare_parameter("camera_name", "ximea_camera");
  frame_id_        = this->declare_parameter("frame_id", "camera_optical_frame");
  camera_info_url_ = this->declare_parameter("camera_info_url", "");

  // image format
  format_ = this->declare_parameter("image_format", "XI_MONO8");

  // timing
  poll_open_period_s_      = this->declare_parameter("poll_open_period_s", 2.0);
  poll_frame_period_s_     = this->declare_parameter("poll_frame_period_s", 0.001);
  img_capture_timeout_ms_  = this->declare_parameter("img_capture_timeout_ms", 1000);

  // triggering
  trigger_mode_     = this->declare_parameter("trigger_mode", 0);
  hw_trigger_edge_  = this->declare_parameter("hw_trigger_edge", 0);

  // framerate
  framerate_control_ = this->declare_parameter("framerate_control", false);
  framerate_set_     = this->declare_parameter("framerate_set", 30);

  // exposure / gain
  auto_exposure_                = this->declare_parameter("auto_exposure", true);
  exposure_time_us_             = this->declare_parameter("exposure_time_us", 3000);
  manual_gain_db_               = this->declare_parameter("manual_gain_db", 0.0);
  auto_exposure_priority_       = this->declare_parameter("auto_exposure_priority", 0.8);
  auto_exposure_time_limit_us_  = this->declare_parameter("auto_exposure_time_limit_us", 30000);
  auto_gain_limit_db_           = this->declare_parameter("auto_gain_limit_db", 2.0);

  // white balance
  wb_mode_ = this->declare_parameter("wb_mode", 0);
  wb_kr_   = this->declare_parameter("wb_kr", 1.0);
  wb_kg_   = this->declare_parameter("wb_kg", 1.0);
  wb_kb_   = this->declare_parameter("wb_kb", 1.0);

  // ROI
  roi_left_   = this->declare_parameter("roi_left", 0);
  roi_top_    = this->declare_parameter("roi_top", 0);
  roi_width_  = this->declare_parameter("roi_width", 0);
  roi_height_ = this->declare_parameter("roi_height", 0);

  // bandwidth / optimization
  num_cams_in_bus_         = this->declare_parameter("num_cams_in_bus", 1);
  bw_safety_ratio_         = this->declare_parameter("bw_safety_ratio", 0.9);
  transport_buffer_commit_ = this->declare_parameter("transport_buffer_commit", 32);
  buffers_queue_size_      = this->declare_parameter("buffers_queue_size", 4);
  recent_frame_            = this->declare_parameter("recent_frame", true);
  warmup_frames_           = this->declare_parameter("warmup_frames", 10);

  if (bw_safety_ratio_ <= 0.0 || bw_safety_ratio_ > 1.0) {
    RCLCPP_WARN(get_logger(), "bw_safety_ratio=%.3f out of bounds; clamping to 0.9", bw_safety_ratio_);
    bw_safety_ratio_ = 0.9;
  }

  // toggles
  use_hardware_timestamps_ = this->declare_parameter("use_hardware_timestamps", false);
  hw_anchor_resync_period_s_ = this->declare_parameter("hw_anchor_resync_period_s", 60.0);
  hw_anchor_min_latency_   = this->declare_parameter("hw_anchor_min_latency", true);
  hw_clock_drift_ppm_      = this->declare_parameter("hw_clock_drift_ppm", 100.0);
  timestamp_offset_s_      = this->declare_parameter("timestamp_offset_s", 0.0);
  lag_warn_threshold_s_    = this->declare_parameter("lag_warn_threshold_s", 0.25);
  publish_xi_image_info_   = this->declare_parameter("publish_xi_image_info", false);

  auto & fmt_map = imageFormatMap();
  auto & enc_map = encodingMap();
  auto & bpp_map = bytesPerPixelMap();

  if (fmt_map.find(format_) == fmt_map.end()) {
    RCLCPP_ERROR(this->get_logger(), "Invalid format '%s'. Defaulting to XI_MONO8", format_.c_str());
    format_ = "XI_MONO8";
  }

  format_int_      = fmt_map.at(format_);
  encoding_        = enc_map.at(format_);
  bytes_per_pixel_ = bpp_map.at(format_);

  // Disable image_transport plugins we don't publish through (compressedDepth,
  // theora). Effective on Jazzy and newer (image_common PR #264). On Humble
  // the parameter is declared but not yet honored by image_transport —
  // filter with `ros2 bag record --regex --exclude '.*/(compressedDepth|theora)$'`
  // as a workaround there.
  this->declare_parameter<std::vector<std::string>>(
    "image_raw.disable_pub_plugins",
    std::vector<std::string>{
      "image_transport/compressedDepth",
      "image_transport/theora"
    });

  auto qos = rmw_qos_profile_sensor_data;
  image_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);
  
  image_count_pub_ = this->create_publisher<std_msgs::msg::UInt32>("image_count", 10);
  xi_image_info_pub_ = this->create_publisher<ximea_ros2_cam::msg::XiImageInfo>("xi_image_info", 10);

  cam_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_name_);

  // Global prerequisite: Enable bandwidth calculations for accurate querying later
  xiSetParamInt(0, XI_PRM_AUTO_BANDWIDTH_CALCULATION, XI_ON);

  RCLCPP_INFO(this->get_logger(), "Driver initialized. Polling for XIMEA camera Serial=%s", serial_no_.c_str());
  if (use_hardware_timestamps_) {
    if (hw_anchor_min_latency_) {
      RCLCPP_INFO(this->get_logger(),
                  "HW timestamps: min-latency anchoring (drift allowance %.0f ppm, offset %.4f s)",
                  hw_clock_drift_ppm_, timestamp_offset_s_);
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "HW timestamps: legacy first-frame anchoring, re-anchor period=%.1f s",
                  hw_anchor_resync_period_s_);
    }
  }

  // Started last: both threads use the members initialised above.
  capture_thread_ = std::thread(&XimeaRosCam::captureLoop, this);
  publish_thread_ = std::thread(&XimeaRosCam::publishLoop, this);
}

XimeaRosCam::~XimeaRosCam() {
  shutdown();
}

void XimeaRosCam::shutdown() {
  {
    std::lock_guard<std::mutex> lock(stop_mutex_);
    stop_ = true;
  }
  stop_cv_.notify_all();
  {
    // Taking the lock orders the stop_ write before the publish thread's wait predicate
    std::lock_guard<std::mutex> lock(slot_mutex_);
  }
  slot_cv_.notify_all();

  if (capture_thread_.joinable()) capture_thread_.join();
  if (publish_thread_.joinable()) publish_thread_.join();
  closeDevice();
}

bool XimeaRosCam::sleepFor(double seconds) {
  std::unique_lock<std::mutex> lock(stop_mutex_);
  stop_cv_.wait_for(lock, std::chrono::duration<double>(seconds), [this]() { return stop_.load(); });
  return !stop_;
}

void XimeaRosCam::captureLoop() {
  // The first attempt happens one period after start, so cameras with different
  // poll_open_period_s values open one after the other.
  while (sleepFor(poll_open_period_s_)) {
    if (!openDevice()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Searching for camera %s...", serial_no_.c_str());
      continue;
    }
    grabLoop();
    closeDevice();
  }
}

bool XimeaRosCam::openDevice() {
  XI_RETURN stat;
  if (serial_no_.empty()) {
    stat = xiOpenDevice(0, &xi_handle_);
  } else {
    stat = xiOpenDeviceBy(XI_OPEN_BY_SN, serial_no_.c_str(), &xi_handle_);
  }
  if (stat != XI_OK || !xi_handle_) {
    xi_handle_ = nullptr;
    return false;
  }

  if (!configureCamera()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to configure camera parameters. Aborting.");
    closeDevice();
    return false;
  }

  XI_RETURN as = xiStartAcquisition(xi_handle_);
  if (as != XI_OK) {
    RCLCPP_ERROR(get_logger(), "xiStartAcquisition failed with status %d", as);
    closeDevice();
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Successfully connected and started acquisition on camera SN=%s", serial_no_.c_str());
  return true;
}

void XimeaRosCam::closeDevice() {
  if (xi_handle_) {
    xiStopAcquisition(xi_handle_);
    xiCloseDevice(xi_handle_);
    xi_handle_ = nullptr;
    RCLCPP_INFO(this->get_logger(), "Closed XIMEA hardware device connection.");
  }
}


namespace {
  bool checkXi(rclcpp::Logger logger, XI_RETURN s, const char * what) {
    if (s == XI_OK) return true;
    RCLCPP_ERROR(logger, "xiAPI: %s failed with status %d", what, s);
    return false;
  }
}  // namespace

bool XimeaRosCam::configureCamera() {
  auto log = this->get_logger();
  XI_RETURN s;

  // -------- Image format --------
  s = xiSetParamInt(xi_handle_, XI_PRM_IMAGE_DATA_FORMAT, format_int_);
  if (!checkXi(log, s, "set XI_PRM_IMAGE_DATA_FORMAT")) return false;

  // -------- ROI --------
  int sensor_max_w = 0, sensor_max_h = 0;
  xiGetParamInt(xi_handle_, XI_PRM_WIDTH  XI_PRM_INFO_MAX, &sensor_max_w);
  xiGetParamInt(xi_handle_, XI_PRM_HEIGHT XI_PRM_INFO_MAX, &sensor_max_h);
  if (roi_width_  <= 0) roi_width_  = sensor_max_w;
  if (roi_height_ <= 0) roi_height_ = sensor_max_h;
  if (roi_left_ < 0 || roi_top_ < 0 ||
      roi_left_ + roi_width_  > sensor_max_w ||
      roi_top_  + roi_height_ > sensor_max_h) {
    RCLCPP_ERROR(log, "ROI out of bounds: left=%d top=%d w=%d h=%d sensor=%dx%d",
                 roi_left_, roi_top_, roi_width_, roi_height_, sensor_max_w, sensor_max_h);
    return false;
  }

  int inc_w = 1, inc_h = 1, inc_x = 1, inc_y = 1;
  xiGetParamInt(xi_handle_, XI_PRM_WIDTH XI_PRM_INFO_INCREMENT, &inc_w);
  xiGetParamInt(xi_handle_, XI_PRM_HEIGHT XI_PRM_INFO_INCREMENT, &inc_h);
  xiGetParamInt(xi_handle_, XI_PRM_OFFSET_X XI_PRM_INFO_INCREMENT, &inc_x);
  xiGetParamInt(xi_handle_, XI_PRM_OFFSET_Y XI_PRM_INFO_INCREMENT, &inc_y);

  roi_width_  = (roi_width_ / inc_w) * inc_w;
  roi_height_ = (roi_height_ / inc_h) * inc_h;
  roi_left_   = (roi_left_ / inc_x) * inc_x;
  roi_top_    = (roi_top_ / inc_y) * inc_y;

  if (!checkXi(log, xiSetParamInt(xi_handle_, XI_PRM_WIDTH, roi_width_), "set ROI Width")) return false;
  if (!checkXi(log, xiSetParamInt(xi_handle_, XI_PRM_HEIGHT, roi_height_), "set ROI Height")) return false;
  if (!checkXi(log, xiSetParamInt(xi_handle_, XI_PRM_OFFSET_X, roi_left_), "set ROI Offset X")) return false;
  if (!checkXi(log, xiSetParamInt(xi_handle_, XI_PRM_OFFSET_Y, roi_top_), "set ROI Offset Y")) return false;

  // -------- Triggering --------
  switch (trigger_mode_) {
    case 2:  // hardware
      xiSetParamInt(xi_handle_, XI_PRM_TRG_SOURCE,
        (hw_trigger_edge_ == 1) ? XI_TRG_EDGE_FALLING : XI_TRG_EDGE_RISING);
      xiSetParamInt(xi_handle_, XI_PRM_GPI_SELECTOR, 1);
      xiSetParamInt(xi_handle_, XI_PRM_GPI_MODE,    XI_GPI_TRIGGER);
      break;
    case 1:  // software
      xiSetParamInt(xi_handle_, XI_PRM_TRG_SOURCE, XI_TRG_SOFTWARE);
      break;
    default:
      xiSetParamInt(xi_handle_, XI_PRM_TRG_SOURCE, XI_TRG_OFF);
      break;
  }

  // -------- Exposure / gain --------
  if (auto_exposure_) {
    xiSetParamInt(xi_handle_,   XI_PRM_AEAG, 1);
    auto_exposure_priority_ = std::clamp(auto_exposure_priority_, 0.5, 1.0);
    xiSetParamFloat(xi_handle_, XI_PRM_EXP_PRIORITY, static_cast<float>(auto_exposure_priority_));
    xiSetParamFloat(xi_handle_, XI_PRM_AE_MAX_LIMIT, static_cast<float>(auto_exposure_time_limit_us_));
    xiSetParamFloat(xi_handle_, XI_PRM_AG_MAX_LIMIT, static_cast<float>(auto_gain_limit_db_));
    RCLCPP_INFO(log, "Auto-exposure ON (max %d us, priority %.2f)", auto_exposure_time_limit_us_, auto_exposure_priority_);
  } else {
    xiSetParamInt(xi_handle_,   XI_PRM_AEAG, 0);
    xiSetParamInt(xi_handle_,   XI_PRM_EXPOSURE, exposure_time_us_);
    xiSetParamFloat(xi_handle_, XI_PRM_GAIN, static_cast<float>(manual_gain_db_));
    RCLCPP_INFO(log, "Manual exposure %d us, gain %.2f dB", exposure_time_us_, manual_gain_db_);
  }

  // -------- White balance --------
  switch (wb_mode_) {
    case 2:
      if (xiSetParamInt(xi_handle_, XI_PRM_AUTO_WB, 1) != XI_OK) {
        RCLCPP_WARN(log, "Auto White Balance rejected (likely mono camera)");
      }
      break;
    case 1:
      xiSetParamInt(xi_handle_,   XI_PRM_AUTO_WB, 0);
      xiSetParamFloat(xi_handle_, XI_PRM_WB_KR, static_cast<float>(wb_kr_));
      xiSetParamFloat(xi_handle_, XI_PRM_WB_KG, static_cast<float>(wb_kg_));
      xiSetParamFloat(xi_handle_, XI_PRM_WB_KB, static_cast<float>(wb_kb_));
      break;
    default:
      xiSetParamInt(xi_handle_, XI_PRM_AUTO_WB, 0);
      break;
  }

  // -------- Bandwidth limiting --------
  int avail_bw_mbps = 0;
  if (xiGetParamInt(xi_handle_, XI_PRM_AVAILABLE_BANDWIDTH, &avail_bw_mbps) == XI_OK) {
    if (num_cams_in_bus_ > 1) avail_bw_mbps /= num_cams_in_bus_;
    const int limit_mbps = static_cast<int>(avail_bw_mbps * bw_safety_ratio_);
    xiSetParamInt(xi_handle_, XI_PRM_LIMIT_BANDWIDTH,      limit_mbps);
    XI_RETURN bw_mode_s = xiSetParamInt(xi_handle_, XI_PRM_LIMIT_BANDWIDTH_MODE, XI_ON);
    if (bw_mode_s != XI_OK && bw_mode_s != 100) {
      RCLCPP_WARN(log, "XI_PRM_LIMIT_BANDWIDTH_MODE set failed: %d", bw_mode_s);
    }
    // Status 100 (XI_NOT_SUPPORTED) is expected on older models — limit is still applied via XI_PRM_LIMIT_BANDWIDTH alone.
  } else {
    RCLCPP_WARN(log, "Failed to read available bandwidth.");
  }

  // -------- Framerate --------
  if (trigger_mode_ == 0 && framerate_control_) {
    xiSetParamInt(xi_handle_, XI_PRM_ACQ_TIMING_MODE, XI_ACQ_TIMING_MODE_FRAME_RATE);
    XI_RETURN fr = xiSetParamInt(xi_handle_, XI_PRM_FRAMERATE, framerate_set_);
    if (fr != XI_OK) {
      float fr_max = 0.f;
      xiGetParamFloat(xi_handle_, XI_PRM_FRAMERATE XI_PRM_INFO_MAX, &fr_max);
      RCLCPP_WARN(log,
        "Framerate %d Hz rejected (status %d, max achievable %.1f Hz at current settings). "
        "Falling back to free-run.",
        framerate_set_, fr, fr_max);
      xiSetParamInt(xi_handle_, XI_PRM_ACQ_TIMING_MODE, XI_ACQ_TIMING_MODE_FREE_RUN);
    }
  } else if (trigger_mode_ == 0) {
    xiSetParamInt(xi_handle_, XI_PRM_ACQ_TIMING_MODE, XI_ACQ_TIMING_MODE_FREE_RUN);
  }

  // -------- Performance & Buffer Tuning --------
  // A deep queue turns any slowdown into seconds of latency, so it is kept small by
  // default. buffers_queue_size <= 0 selects the camera's maximum.
  int qsize_min = 1, qsize_max = 0;
  xiGetParamInt(xi_handle_, XI_PRM_BUFFERS_QUEUE_SIZE XI_PRM_INFO_MIN, &qsize_min);
  if (xiGetParamInt(xi_handle_, XI_PRM_BUFFERS_QUEUE_SIZE XI_PRM_INFO_MAX, &qsize_max) == XI_OK && qsize_max > 0) {
    const int lo = std::min(std::max(qsize_min, 1), qsize_max);
    const int qsize = (buffers_queue_size_ <= 0) ? qsize_max : std::clamp(buffers_queue_size_, lo, qsize_max);
    s = xiSetParamInt(xi_handle_, XI_PRM_BUFFERS_QUEUE_SIZE, qsize);
    if (s != XI_OK) {
      RCLCPP_WARN(log, "Could not set buffers queue size to %d (status %d)", qsize, s);
    } else {
      RCLCPP_INFO(log, "xiAPI buffers queue size: %d (camera max %d)", qsize, qsize_max);
    }
  }

  // Optimize transport buffer commit (for high framerates)
  s = xiSetParamInt(xi_handle_, XI_PRM_ACQ_TRANSPORT_BUFFER_COMMIT, transport_buffer_commit_);
  if (s != XI_OK) {
    RCLCPP_WARN(log, "Could not set transport buffer commit to %d (status %d)", transport_buffer_commit_, s);
  }

  if (recent_frame_) {
    s = xiSetParamInt(xi_handle_, XI_PRM_RECENT_FRAME, 1);
    if (s != XI_OK) {
      RCLCPP_WARN(log, "Could not enable XI_PRM_RECENT_FRAME (status %d)", s);
    } else {
      RCLCPP_INFO(log, "XI_PRM_RECENT_FRAME enabled for lower latency.");
    }
  }

  // Optimize transport buffer size for small payloads (latency reduction)
  int payload = 0;
  int tb_default = 0, tb_inc = 0, tb_min = 0;
  if (xiGetParamInt(xi_handle_, XI_PRM_IMAGE_PAYLOAD_SIZE, &payload) == XI_OK &&
      xiGetParamInt(xi_handle_, XI_PRM_ACQ_TRANSPORT_BUFFER_SIZE, &tb_default) == XI_OK &&
      xiGetParamInt(xi_handle_, XI_PRM_ACQ_TRANSPORT_BUFFER_SIZE XI_PRM_INFO_INCREMENT, &tb_inc) == XI_OK &&
      xiGetParamInt(xi_handle_, XI_PRM_ACQ_TRANSPORT_BUFFER_SIZE XI_PRM_INFO_MIN, &tb_min) == XI_OK) 
  {
    if (payload < tb_default + tb_inc) {
      int tb_new = payload;
      if (tb_inc > 0) {
        int remainder = tb_new % tb_inc;
        if (remainder) tb_new += tb_inc - remainder;
      }
      if (tb_new < tb_min) tb_new = tb_min;
      
      if (xiSetParamInt(xi_handle_, XI_PRM_ACQ_TRANSPORT_BUFFER_SIZE, tb_new) == XI_OK) {
        RCLCPP_INFO(log, "Tuned transport buffer size to match payload: %d bytes", tb_new);
      }
    }
  }

  return true;
}

void XimeaRosCam::resetTimestampState() {
  lat_valid_ = false;
  hw_anchor_set_ = false;
}

int64_t XimeaRosCam::trackLatency(int64_t recv_ns, int64_t hw_ns) {
  const int64_t d = recv_ns - hw_ns;
  if (!lat_valid_ || hw_ns < lat_last_hw_ns_) {
    // First frame, or the camera clock restarted
    lat_offset_ns_ = d;
    lat_valid_ = true;
  } else {
    // The minimum may creep upwards by the allowed clock drift so it can follow a
    // slowly diverging camera/host clock, but not a queue backlog (which grows
    // orders of magnitude faster). Anything lower is taken immediately.
    lat_offset_ns_ += static_cast<int64_t>(
      static_cast<double>(hw_ns - lat_last_hw_ns_) * hw_clock_drift_ppm_ * 1e-6);
    if (d < lat_offset_ns_) lat_offset_ns_ = d;
  }
  lat_last_hw_ns_ = hw_ns;
  return d - lat_offset_ns_;
}

int64_t XimeaRosCam::legacyAnchoredStamp(int64_t recv_ns, int64_t hw_ns) {
  const int64_t hw_us = hw_ns / 1000;
  if (hw_anchor_set_ && hw_anchor_resync_period_s_ > 0.0 &&
      recv_ns - hw_anchor_ros_ns_ >= static_cast<int64_t>(hw_anchor_resync_period_s_ * 1e9)) {
    hw_anchor_set_ = false;
  }
  if (!hw_anchor_set_) {
    hw_anchor_ros_ns_ = recv_ns;
    hw_anchor_us_ = hw_us;
    hw_anchor_set_ = true;
    return recv_ns;
  }
  return hw_anchor_ros_ns_ + (hw_us - hw_anchor_us_) * 1000LL;
}

void XimeaRosCam::grabLoop() {
  using steady = std::chrono::steady_clock;
  constexpr int64_t kReportPeriodNs = 5'000'000'000LL;
  const int64_t lag_warn_ns = static_cast<int64_t>(lag_warn_threshold_s_ * 1e9);
  const int64_t ts_offset_ns = static_cast<int64_t>(timestamp_offset_s_ * 1e9);
  const size_t frame_bytes_per_px = static_cast<size_t>(bytes_per_pixel_);

  int fail_count = 0;
  int warmup_count = 0;
  int64_t window_start_ns = 0;
  int64_t lag_max_ns = 0;
  int64_t copy_max_ns = 0;
  resetTimestampState();

  while (!stop_) {
    XI_IMG xi_img;
    std::memset(&xi_img, 0, sizeof(xi_img));
    xi_img.size = sizeof(XI_IMG);

    const XI_RETURN s = xiGetImage(xi_handle_, img_capture_timeout_ms_, &xi_img);
    const int64_t recv_ns = this->get_clock()->now().nanoseconds();

    if (s != XI_OK || !xi_img.bp) {
      if (++fail_count > 10) {
        RCLCPP_ERROR(this->get_logger(), "Lost connection to camera. Attempting reconnect...");
        return;
      }
      if (s != XI_TIMEOUT) sleepFor(0.01);  // avoid spinning on immediate errors
      continue;
    }
    fail_count = 0;

    // Discard warmup frames after startup/reconnect to let USB and auto-exposure settle.
    if (warmup_count < warmup_frames_) {
      if (++warmup_count == warmup_frames_) {
        RCLCPP_INFO(this->get_logger(), "Warmup complete (%d frames discarded). Streaming images.", warmup_frames_);
      }
      continue;
    }

    // ---- Timestamp ----
    const int64_t hw_ns = (static_cast<int64_t>(xi_img.tsSec) * 1000000LL + xi_img.tsUSec) * 1000LL;
    // Always tracked: how late this frame is compared with the best one seen. A queue
    // backlog shows up here whether or not hardware timestamps are used.
    const int64_t excess_ns = trackLatency(recv_ns, hw_ns);

    int64_t stamp_ns;
    if (!use_hardware_timestamps_) {
      stamp_ns = recv_ns;
    } else if (hw_anchor_min_latency_) {
      stamp_ns = recv_ns - excess_ns;
    } else {
      stamp_ns = legacyAnchoredStamp(recv_ns, hw_ns);
    }
    stamp_ns += ts_offset_ns;
    if (stamp_ns <= last_stamp_ns_) stamp_ns = last_stamp_ns_ + 1000;  // keep stamps increasing
    last_stamp_ns_ = stamp_ns;

    // ---- Hand the frame to the publish thread ----
    const auto copy_t0 = steady::now();
    PendingFrame frame;
    const size_t payload_bytes =
      static_cast<size_t>(xi_img.width) * frame_bytes_per_px * xi_img.height;
    frame.data.assign(static_cast<const uint8_t *>(xi_img.bp),
                      static_cast<const uint8_t *>(xi_img.bp) + payload_bytes);
    frame.meta = xi_img;
    frame.stamp_ns = stamp_ns;
    copy_max_ns = std::max<int64_t>(copy_max_ns,
      std::chrono::duration_cast<std::chrono::nanoseconds>(steady::now() - copy_t0).count());

    {
      std::lock_guard<std::mutex> lock(slot_mutex_);
      if (slot_full_) ++stat_dropped_;   // publisher has not taken the previous frame yet
      slot_ = std::move(frame);
      slot_full_ = true;
    }
    slot_cv_.notify_one();

    // ---- Health report: logged only when something is wrong ----
    lag_max_ns = std::max(lag_max_ns, excess_ns);
    if (window_start_ns == 0) window_start_ns = recv_ns;
    if (recv_ns - window_start_ns >= kReportPeriodNs) {
      const uint32_t dropped = stat_dropped_.exchange(0);
      const double publish_max_ms = static_cast<double>(stat_publish_max_ns_.exchange(0)) * 1e-6;
      const bool lagging = lag_warn_ns > 0 && lag_max_ns > lag_warn_ns;
      if (lagging || dropped > 0) {
        RCLCPP_WARN(this->get_logger(),
          "Pipeline falling behind (last %.0f s): frame lag up to %.3f s (threshold %.3f s), "
          "%u frame(s) dropped before publishing, publish() up to %.1f ms, frame copy up to %.1f ms.",
          static_cast<double>(recv_ns - window_start_ns) * 1e-9,
          static_cast<double>(lag_max_ns) * 1e-9, lag_warn_threshold_s_,
          dropped, publish_max_ms, static_cast<double>(copy_max_ns) * 1e-6);
      }
      lag_max_ns = 0;
      copy_max_ns = 0;
      window_start_ns = recv_ns;
    }
  }
}

void XimeaRosCam::publishLoop() {
  using steady = std::chrono::steady_clock;
  while (true) {
    PendingFrame f;
    {
      std::unique_lock<std::mutex> lock(slot_mutex_);
      slot_cv_.wait(lock, [this]() { return slot_full_ || stop_.load(); });
      if (stop_) return;
      f = std::move(slot_);
      slot_full_ = false;
    }

    const auto t0 = steady::now();
    try {
      const rclcpp::Time stamp(f.stamp_ns, RCL_ROS_TIME);
      const XI_IMG & xi_img = f.meta;

      auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
      img_msg->header.stamp    = stamp;
      img_msg->header.frame_id = frame_id_;
      img_msg->height          = xi_img.height;
      img_msg->width           = xi_img.width;
      img_msg->encoding        = encoding_;
      img_msg->step            = xi_img.width * bytes_per_pixel_;
      img_msg->data            = std::move(f.data);

      auto info_msg = std::make_unique<sensor_msgs::msg::CameraInfo>(cam_info_manager_->getCameraInfo());
      info_msg->header.stamp    = stamp;
      info_msg->header.frame_id = frame_id_;
      if (info_msg->width == 0) {
        info_msg->width  = xi_img.width;
        info_msg->height = xi_img.height;
      }

      image_pub_.publish(std::move(*img_msg), std::move(*info_msg));

      std_msgs::msg::UInt32 count_msg;
      count_msg.data = ++frame_count_;
      image_count_pub_->publish(count_msg);

      if (publish_xi_image_info_) {
        auto info_msg_custom = std::make_unique<ximea_ros2_cam::msg::XiImageInfo>();
        info_msg_custom->header.stamp           = stamp;
        info_msg_custom->header.frame_id        = frame_id_;
        info_msg_custom->size                   = xi_img.size;
        info_msg_custom->bp_size                = xi_img.bp_size;
        info_msg_custom->frm                    = xi_img.frm;
        info_msg_custom->width                  = xi_img.width;
        info_msg_custom->height                 = xi_img.height;
        info_msg_custom->nframe                 = xi_img.nframe;
        info_msg_custom->acq_nframe             = xi_img.acq_nframe;
        info_msg_custom->ts_sec                 = xi_img.tsSec;
        info_msg_custom->ts_usec                = xi_img.tsUSec;
        info_msg_custom->gpi_level              = xi_img.GPI_level;
        info_msg_custom->black_level            = xi_img.black_level;
        info_msg_custom->padding_x              = xi_img.padding_x;
        info_msg_custom->absolute_offset_x      = xi_img.AbsoluteOffsetX;
        info_msg_custom->absolute_offset_y      = xi_img.AbsoluteOffsetY;
        info_msg_custom->exposure_time_us       = xi_img.exposure_time_us;
        info_msg_custom->gain_db                = xi_img.gain_db;
        info_msg_custom->image_user_data        = xi_img.image_user_data;

        xi_image_info_pub_->publish(std::move(info_msg_custom));
      }
    } catch (const std::exception & e) {
      if (stop_ || !rclcpp::ok()) return;   // context went away during shutdown
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Publish failed: %s", e.what());
    }

    const int64_t dur_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(steady::now() - t0).count();
    int64_t prev = stat_publish_max_ns_.load();
    while (dur_ns > prev && !stat_publish_max_ns_.compare_exchange_weak(prev, dur_ns)) {}
  }
}

} // namespace ximea_ros2_cam

RCLCPP_COMPONENTS_REGISTER_NODE(ximea_ros2_cam::XimeaRosCam)
