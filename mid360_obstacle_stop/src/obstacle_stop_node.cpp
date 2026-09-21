#include "mid360_obstacle_stop/obstacle_stop_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace mid360_obstacle_stop
{
namespace
{
// Fixed loop periods - not tuning parameters.
constexpr std::chrono::milliseconds kControlPeriod{50};      // 20 Hz state machine
constexpr std::chrono::milliseconds kWatchdogPeriod{200};    // 5 Hz LiDAR watchdog
constexpr int kThrottleMs{2000};                             // log throttle for repeated warnings
constexpr int kWatchdogThrottleMs{5000};

// uint8_t promotes to int in varargs - keep the log format specifiers unambiguous.
constexpr unsigned int asUnsigned(uint8_t value) {return static_cast<unsigned int>(value);}
}  // namespace

ObstacleStopNode::ObstacleStopNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mid360_obstacle_stop", options)
{
  // Single mutually exclusive callback group: point cloud, vehicle status, timers and service
  // replies can never run concurrently, so the state machine below needs no mutex.
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  loadParameters();

  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&ObstacleStopNode::onSetParameters, this, std::placeholders::_1));

  last_cloud_time_ = now();
  last_command_time_ = now();
  pause_ack_time_ = now();
  last_diagnostic_time_ = now();

  detector_.setConfig(detector_config_);
  px4_ = std::make_unique<Px4CommandInterface>(
    *this, vehicle_command_service_, hold_method_, callback_group_);

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group_;

  // Livox MID360 point cloud. SensorDataQoS (best effort) is compatible with both a best
  // effort and a reliable publisher, so it is the safe choice here.
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&ObstacleStopNode::onPointCloud, this, std::placeholders::_1),
    subscription_options);

  // PX4 publishes /fmu/out/* as BEST_EFFORT + KEEP_LAST (see uxrce_dds_client/utilities.hpp in
  // PX4 v1.16). This is the QoS profile recommended by the PX4 ROS 2 user guide.
  const rmw_qos_profile_t status_qos_profile = rmw_qos_profile_sensor_data;
  const rclcpp::QoS status_qos(
    rclcpp::QoSInitialization(status_qos_profile.history, 5), status_qos_profile);

  vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    vehicle_status_topic_, status_qos,
    std::bind(&ObstacleStopNode::onVehicleStatus, this, std::placeholders::_1),
    subscription_options);

  control_timer_ = create_wall_timer(
    kControlPeriod, std::bind(&ObstacleStopNode::updateStateMachine, this), callback_group_);
  watchdog_timer_ = create_wall_timer(
    kWatchdogPeriod, std::bind(&ObstacleStopNode::checkLidarTimeout, this), callback_group_);

  RCLCPP_INFO(
    get_logger(),
    "Obstacle stop node started | point cloud: %s | vehicle_status: %s | service: %s",
    pointcloud_topic_.c_str(), vehicle_status_topic_.c_str(),
    vehicle_command_service_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "ROI: %.2f m < horizontal < %.2f m, %.2f m < z < %.2f m, >= %d danger points, "
    "%zu confirm frames, %zu clear frames",
    detector_config_.min_range, detector_config_.stop_distance,
    detector_config_.z_min, detector_config_.z_max,
    detector_config_.min_obstacle_points, confirm_frames_, clear_frames_);
  RCLCPP_INFO(
    get_logger(),
    "hold_method: %s | auto_resume: %s | lidar_timeout: %.2f s (stop on timeout: %s)",
    holdMethodName(hold_method_), auto_resume_ ? "true" : "false",
    lidar_timeout_, lidar_timeout_stop_enabled_ ? "true" : "false");

  if (!px4_->serviceReady()) {
    RCLCPP_WARN(
      get_logger(),
      "VehicleCommand service '%s' is not available yet - is PX4 (uXRCE-DDS client) running? "
      "The node keeps running and will request the pause once the service appears.",
      vehicle_command_service_.c_str());
  }
}

void ObstacleStopNode::loadParameters()
{
  pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "/livox/lidar");
  vehicle_status_topic_ =
    declare_parameter<std::string>("vehicle_status_topic", "/fmu/out/vehicle_status");
  vehicle_command_service_ =
    declare_parameter<std::string>("vehicle_command_service", "/fmu/vehicle_command");

  DetectorConfig config;
  config.stop_distance = declare_parameter<double>("stop_distance", 3.0);
  config.min_range = declare_parameter<double>("min_range", 0.5);
  config.z_min = declare_parameter<double>("z_min", -0.6);
  config.z_max = declare_parameter<double>("z_max", 0.8);
  config.min_obstacle_points = declare_parameter<int>("min_obstacle_points", 10);

  resume_distance_ = declare_parameter<double>("resume_distance", 4.0);
  auto_resume_ = declare_parameter<bool>("auto_resume", false);
  lidar_timeout_ = declare_parameter<double>("lidar_timeout", 0.5);
  lidar_timeout_stop_enabled_ = declare_parameter<bool>("lidar_timeout_stop_enabled", false);

  command_timeout_ = declare_parameter<double>("command_timeout", 2.0);
  command_retry_cooldown_ = declare_parameter<double>("command_retry_cooldown", 2.0);
  hold_confirm_timeout_ = declare_parameter<double>("hold_confirm_timeout", 2.0);
  diagnostic_period_ = declare_parameter<double>("diagnostic_period", 0.0);

  const int confirm_frames = declare_parameter<int>("confirm_frames", 3);
  const int clear_frames = declare_parameter<int>("clear_frames", 10);
  confirm_frames_ = static_cast<std::size_t>(std::max(1, confirm_frames));
  clear_frames_ = static_cast<std::size_t>(std::max(1, clear_frames));

  const std::string hold_method = declare_parameter<std::string>("hold_method", "do_reposition_hold");
  if (hold_method == "do_reposition_hold") {
    hold_method_ = HoldMethod::kDoRepositionHold;
  } else if (hold_method == "do_pause_continue") {
    hold_method_ = HoldMethod::kDoPauseContinue;
    RCLCPP_WARN(
      get_logger(),
      "hold_method='do_pause_continue' selected: PX4 v1.16 does not implement "
      "MAV_CMD_DO_PAUSE_CONTINUE (193), the commander will answer UNSUPPORTED. "
      "Only use this if your flight stack really supports it.");
  } else {
    hold_method_ = HoldMethod::kDoRepositionHold;
    RCLCPP_ERROR(
      get_logger(), "Unknown hold_method '%s' - falling back to 'do_reposition_hold'.",
      hold_method.c_str());
  }

  // --- sanity checks: log and repair instead of crashing ---
  if (config.min_range < 0.0) {
    RCLCPP_WARN(get_logger(), "min_range %.2f < 0 - using 0.0", config.min_range);
    config.min_range = 0.0;
  }
  if (config.stop_distance <= config.min_range) {
    RCLCPP_WARN(
      get_logger(), "stop_distance %.2f <= min_range %.2f - using %.2f",
      config.stop_distance, config.min_range, config.min_range + 1.0);
    config.stop_distance = config.min_range + 1.0;
  }
  if (config.z_max <= config.z_min) {
    RCLCPP_WARN(
      get_logger(), "z_max %.2f <= z_min %.2f - swapping the two values",
      config.z_max, config.z_min);
    std::swap(config.z_min, config.z_max);
  }
  if (config.min_obstacle_points < 1) {
    RCLCPP_WARN(
      get_logger(), "min_obstacle_points %d < 1 - using 1", config.min_obstacle_points);
    config.min_obstacle_points = 1;
  }
  if (resume_distance_ < config.stop_distance) {
    RCLCPP_WARN(
      get_logger(), "resume_distance %.2f < stop_distance %.2f - using %.2f",
      resume_distance_, config.stop_distance, config.stop_distance);
    resume_distance_ = config.stop_distance;
  }
  if (lidar_timeout_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "lidar_timeout %.2f <= 0 - using 0.5 s", lidar_timeout_);
    lidar_timeout_ = 0.5;
  }
  if (command_timeout_ <= 0.0) {
    command_timeout_ = 2.0;
  }
  if (command_retry_cooldown_ < 0.0) {
    command_retry_cooldown_ = 0.0;
  }
  if (diagnostic_period_ < 0.0) {
    diagnostic_period_ = 0.0;
  }

  detector_config_ = config;
}

// ----------------------------------------------------------------- point cloud

void ObstacleStopNode::onPointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  last_cloud_time_ = now();
  cloud_received_ = true;
  processPointCloud(*msg);
}

void ObstacleStopNode::processPointCloud(const sensor_msgs::msg::PointCloud2 & cloud)
{
  if (cloud.data.empty() || cloud.point_step == 0U) {
    return;  // empty frame: keep the previous counters
  }

  if (!ObstacleDetector::hasXyzFields(cloud)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), kWatchdogThrottleMs,
      "Point cloud on '%s' has no x/y/z fields (fields: [%s]). Check the Livox driver output "
      "format (xfer_format=0 -> sensor_msgs/PointCloud2).",
      pointcloud_topic_.c_str(), ObstacleDetector::fieldNames(cloud).c_str());
    return;
  }

  try {
    last_detection_ = detector_.detect(cloud);
  } catch (const std::exception & e) {
    // e.g. x/y/z fields that are not FLOAT32
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), kWatchdogThrottleMs,
      "Point cloud on '%s' could not be parsed (%s) - expected FLOAT32 x/y/z fields",
      pointcloud_topic_.c_str(), e.what());
    return;
  }

  if (last_detection_.obstacle) {
    obstacle_frames_ = std::min(obstacle_frames_ + 1U, confirm_frames_);
  } else {
    obstacle_frames_ = 0U;  // a single clear frame resets the confirmation counter
  }

  const bool clear_for_resume =
    (last_detection_.nearest_distance < 0.0) ||
    (last_detection_.nearest_distance > resume_distance_);

  if (clear_for_resume) {
    clear_frame_count_ = std::min(clear_frame_count_ + 1U, clear_frames_);
  } else {
    clear_frame_count_ = 0U;
  }
}

// -------------------------------------------------------------- vehicle status

void ObstacleStopNode::onVehicleStatus(px4_msgs::msg::VehicleStatus::ConstSharedPtr msg)
{
  status_received_ = true;
  armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);

  if (msg->nav_state == nav_state_) {
    return;
  }

  const uint8_t previous = nav_state_;
  nav_state_ = msg->nav_state;

  if (nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION) {
    RCLCPP_INFO(
      get_logger(), "PX4 entered AUTO_MISSION (nav_state=%u, previous=%u)",
      asUnsigned(nav_state_), asUnsigned(previous));
  } else if (previous == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION) {
    RCLCPP_INFO(
      get_logger(), "PX4 left AUTO_MISSION: nav_state %u -> %u (%s)",
      asUnsigned(previous), asUnsigned(nav_state_), navStateName(nav_state_));
  } else {
    RCLCPP_DEBUG(
      get_logger(), "PX4 nav_state %u -> %u (%s)", asUnsigned(previous), asUnsigned(nav_state_),
      navStateName(nav_state_));
  }
}

// -------------------------------------------------------------- state machine

void ObstacleStopNode::updateStateMachine()
{
  const rclcpp::Time now_ts = now();

  // Bench diagnostics (off by default): one summary line per period. Use it on the ground to
  // confirm that an unobstructed vehicle really produces danger=0 before the first flight.
  if (diagnostic_period_ > 0.0 &&
    rclcpp::Duration(now_ts - last_diagnostic_time_).seconds() >= diagnostic_period_)
  {
    last_diagnostic_time_ = now_ts;
    RCLCPP_INFO(
      get_logger(),
      "detect: roi=%zu, danger=%zu, nearest=%.2f m | frames obstacle=%zu/%zu clear=%zu/%zu | "
      "nav_state=%u (%s), state=%s",
      last_detection_.roi_points, last_detection_.danger_points, last_detection_.nearest_distance,
      obstacle_frames_, confirm_frames_, clear_frame_count_, clear_frames_,
      asUnsigned(nav_state_), navStateName(nav_state_), stateName(state_));
  }

  // Re-arm the automatic pause once the danger area has been clear again for a while. This is
  // what stops the node from fighting a pilot who resumed the mission manually.
  if (suppress_until_clear_ && clear_frame_count_ >= clear_frames_) {
    suppress_until_clear_ = false;
    RCLCPP_INFO(get_logger(), "Danger area clear again - automatic Mission PAUSE re-armed");
  }

  switch (state_) {
    case StopState::kClear: {
      if (suppress_until_clear_) {
        break;
      }
      if (!commandCooldownElapsed(now_ts)) {
        break;
      }

      const bool obstacle_confirmed = obstacle_frames_ >= confirm_frames_;
      const bool lidar_timeout_trigger = lidar_timeout_stop_enabled_ && lidar_timeout_active_;
      if (!obstacle_confirmed && !lidar_timeout_trigger) {
        break;
      }

      if (!status_received_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), kThrottleMs,
          "Obstacle/timeout condition present but no PX4 VehicleStatus received on '%s' yet - "
          "automatic Mission PAUSE inhibited (is PX4 running?)",
          vehicle_status_topic_.c_str());
        break;
      }

      if (!isPx4Mission()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), kThrottleMs,
          "Obstacle/timeout condition present (%zu danger points, nearest %.2f m) but the "
          "vehicle is not in AUTO_MISSION (nav_state=%u, %s) - automatic Mission PAUSE inhibited",
          last_detection_.danger_points, last_detection_.nearest_distance,
          asUnsigned(nav_state_), navStateName(nav_state_));
        break;
      }

      requestMissionPause(obstacle_confirmed ? "obstacle" : "LiDAR timeout");
      break;
    }

    case StopState::kPausePending: {
      if (rclcpp::Duration(now_ts - last_command_time_).seconds() <= command_timeout_) {
        break;
      }
      RCLCPP_ERROR(
        get_logger(), "No VehicleCommandAck for the Mission PAUSE within %.1f s - stay in CLEAR "
        "(the pause is NOT considered successful, it will be retried)", command_timeout_);
      state_ = StopState::kClear;
      last_command_time_ = now_ts;
      break;
    }

    case StopState::kPaused: {
      if (!paused_by_obstacle_) {
        state_ = StopState::kClear;  // defensive: never hold without owning the pause
        break;
      }

      if (!hold_confirmed_) {
        if (nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER) {
          hold_confirmed_ = true;
          hold_state_unknown_ = false;
          RCLCPP_INFO(
            get_logger(),
            "PX4 is holding: nav_state=AUTO_LOITER - Mission PAUSE complete (vehicle stopped by PX4)");
        } else if (rclcpp::Duration(now_ts - pause_ack_time_).seconds() > hold_confirm_timeout_) {
          hold_confirmed_ = true;
          hold_state_unknown_ = true;
          RCLCPP_ERROR(
            get_logger(),
            "Pause was ACKed as ACCEPTED but PX4 still reports nav_state=%u (%s) after %.1f s - "
            "the vehicle may not have entered Hold. Check PX4 / QGroundControl.",
            asUnsigned(nav_state_), navStateName(nav_state_), hold_confirm_timeout_);
        }
        break;  // do not interpret the mode change before the hold was observed
      }

      if (nav_state_ != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER) {
        handleExternalHoldRelease();
        break;
      }

      if (auto_resume_ && !lidar_timeout_active_ && clear_frame_count_ >= clear_frames_ &&
        commandCooldownElapsed(now_ts))
      {
        requestMissionContinue();
      }
      break;
    }

    case StopState::kResumePending: {
      if (nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION) {
        RCLCPP_INFO(get_logger(), "Mission is running again (nav_state=AUTO_MISSION)");
        paused_by_obstacle_ = false;
        hold_confirmed_ = false;
        state_ = StopState::kClear;
        break;
      }
      if (rclcpp::Duration(now_ts - last_command_time_).seconds() <= command_timeout_) {
        break;
      }
      RCLCPP_ERROR(
        get_logger(),
        "No VehicleCommandAck for the Mission CONTINUE within %.1f s - staying PAUSED (Hold)",
        command_timeout_);
      state_ = StopState::kPaused;
      last_command_time_ = now_ts;
      break;
    }
  }
}

// ------------------------------------------------------------ PX4 interaction

void ObstacleStopNode::requestMissionPause(const std::string & reason)
{
  last_command_time_ = now();
  const bool obstacle_reason = (reason != "LiDAR timeout");

  if (!px4_->serviceReady()) {
    RCLCPP_ERROR(
      get_logger(),
      "Mission PAUSE requested (%s) but the VehicleCommand service '%s' is NOT available - "
      "no command sent (is PX4 connected?)", reason.c_str(), vehicle_command_service_.c_str());
    return;
  }

  state_ = StopState::kPausePending;

  RCLCPP_WARN(
    get_logger(),
    "Requesting Mission PAUSE (%s): %zu danger point(s), nearest %.2f m, nav_state=AUTO_MISSION, "
    "armed=%s, method=%s",
    reason.c_str(), obstacle_reason ? last_detection_.danger_points : 0U,
    last_detection_.nearest_distance, armed_ ? "true" : "false", holdMethodName(hold_method_));

  px4_->requestHold(
    [this](bool accepted, uint8_t result, uint32_t command) {
      onCommandAck(accepted, result, command);
    });
}

void ObstacleStopNode::requestMissionContinue()
{
  if (!paused_by_obstacle_) {
    RCLCPP_ERROR(
      get_logger(),
      "Refusing to continue the mission: this pause was not created by this node");
    return;
  }

  last_command_time_ = now();

  if (!px4_->serviceReady()) {
    RCLCPP_ERROR(
      get_logger(), "Mission CONTINUE requested but the VehicleCommand service '%s' is not available",
      vehicle_command_service_.c_str());
    return;
  }

  state_ = StopState::kResumePending;

  RCLCPP_INFO(
    get_logger(),
    "Danger area clear for %zu frames (nearest %.2f m) - requesting Mission CONTINUE (auto_resume)",
    clear_frame_count_, last_detection_.nearest_distance);

  px4_->requestContinue(
    [this](bool accepted, uint8_t result, uint32_t command) {
      onCommandAck(accepted, result, command);
    });
}

void ObstacleStopNode::onCommandAck(bool accepted, uint8_t result, uint32_t command)
{
  if (state_ == StopState::kPausePending) {
    if (accepted) {
      paused_by_obstacle_ = true;
      hold_confirmed_ = false;
      hold_state_unknown_ = false;
      pause_ack_time_ = now();
      state_ = StopState::kPaused;
      RCLCPP_WARN(
        get_logger(),
        "PX4 accepted the Mission PAUSE (vehicle_command=%u, ack=ACCEPTED) - PX4 now owns the "
        "deceleration, the stop and the hold", command);
      return;
    }

    if (result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_UNSUPPORTED ||
      result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_DENIED)
    {
      // Never pretend to be paused: keep CLEAR and stop hammering an unsupported command.
      suppress_until_clear_ = true;
      state_ = StopState::kClear;
      RCLCPP_ERROR(
        get_logger(),
        "PX4 rejected the Mission PAUSE: ack=%s (hold_method='%s'). PX4 v1.16 does not implement "
        "MAV_CMD_DO_PAUSE_CONTINUE(193) - use hold_method='do_reposition_hold'. Automatic pause is "
        "disabled until the danger area is clear again.",
        ackResultName(result), holdMethodName(hold_method_));
      return;
    }

    state_ = StopState::kClear;
    RCLCPP_ERROR(
      get_logger(),
      "Mission PAUSE not accepted: ack=%s - staying CLEAR and retrying in %.1f s",
      ackResultName(result), command_retry_cooldown_);
    return;
  }

  if (state_ == StopState::kResumePending) {
    if (accepted) {
      paused_by_obstacle_ = false;
      hold_confirmed_ = false;
      hold_state_unknown_ = false;
      state_ = StopState::kClear;
      RCLCPP_INFO(
        get_logger(),
        "PX4 accepted the Mission CONTINUE (vehicle_command=%u, ack=ACCEPTED) - obstacle pause released",
        command);
      return;
    }

    state_ = StopState::kPaused;
    RCLCPP_ERROR(
      get_logger(),
      "Mission CONTINUE not accepted: ack=%s - staying PAUSED (Hold) and retrying in %.1f s",
      ackResultName(result), command_retry_cooldown_);
    return;
  }

  // A late ACK (state changed meanwhile) must never change the state.
  RCLCPP_WARN(
    get_logger(), "Ignoring late VehicleCommandAck for command %u (ack=%s, state=%s)",
    command, ackResultName(result), stateName(state_));
}

void ObstacleStopNode::handleExternalHoldRelease()
{
  if (hold_state_unknown_) {
    RCLCPP_WARN(
      get_logger(),
      "Hold state could not be confirmed anymore: nav_state=%u (%s) - releasing the obstacle "
      "pause (this node no longer owns the mission state)",
      asUnsigned(nav_state_), navStateName(nav_state_));
  } else if (nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION) {
    RCLCPP_WARN(
      get_logger(),
      "Mission was resumed externally (QGroundControl/RC) while the obstacle pause was active - "
      "this node gives up ownership and will not continue this mission");
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Hold was left externally: nav_state=%u (%s) - this node gives up ownership of the mission",
      asUnsigned(nav_state_), navStateName(nav_state_));
  }

  paused_by_obstacle_ = false;
  hold_confirmed_ = false;
  hold_state_unknown_ = false;
  suppress_until_clear_ = true;  // only allow a new automatic pause after a clear period
  obstacle_frames_ = 0U;
  state_ = StopState::kClear;
}

// ------------------------------------------------------------------ watchdog

void ObstacleStopNode::checkLidarTimeout()
{
  const double age = rclcpp::Duration(now() - last_cloud_time_).seconds();
  lidar_timeout_active_ = (age > lidar_timeout_);

  if (lidar_timeout_active_ && !lidar_timeout_was_active_) {
    RCLCPP_ERROR(
      get_logger(), "MID360 point cloud timeout: %.2f s on '%s' (cloud received before: %s)",
      age, pointcloud_topic_.c_str(), cloud_received_ ? "yes" : "no");
  } else if (!lidar_timeout_active_ && lidar_timeout_was_active_) {
    RCLCPP_INFO(get_logger(), "MID360 point cloud received again (age %.2f s)", age);
  } else if (lidar_timeout_active_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kWatchdogThrottleMs,
      "MID360 point cloud timeout: %.2f s on '%s'%s", age, pointcloud_topic_.c_str(),
      lidar_timeout_stop_enabled_ ? " (automatic pause on timeout enabled)" : "");
  }

  if (!lidar_timeout_active_ && !lidar_timeout_stop_enabled_) {
    RCLCPP_DEBUG(
      get_logger(), "point cloud age %.2f s (lidar_timeout %.2f s)", age, lidar_timeout_);
  }

  lidar_timeout_was_active_ = lidar_timeout_active_;
}

// ------------------------------------------------------- runtime parameters

rcl_interfaces::msg::SetParametersResult ObstacleStopNode::onSetParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  // Copy first, validate everything, then apply - so a rejected update changes nothing.
  DetectorConfig config = detector_config_;
  double resume_distance = resume_distance_;
  double lidar_timeout = lidar_timeout_;
  double command_timeout = command_timeout_;
  double command_retry_cooldown = command_retry_cooldown_;
  double hold_confirm_timeout = hold_confirm_timeout_;
  double diagnostic_period = diagnostic_period_;
  int confirm_frames = static_cast<int>(confirm_frames_);
  int clear_frames = static_cast<int>(clear_frames_);
  bool auto_resume = auto_resume_;
  bool lidar_timeout_stop_enabled = lidar_timeout_stop_enabled_;

  for (const auto & parameter : parameters) {
    const std::string & name = parameter.get_name();
    try {
      if (name == "stop_distance") {
        config.stop_distance = parameter.as_double();
      } else if (name == "min_range") {
        config.min_range = parameter.as_double();
      } else if (name == "z_min") {
        config.z_min = parameter.as_double();
      } else if (name == "z_max") {
        config.z_max = parameter.as_double();
      } else if (name == "min_obstacle_points") {
        config.min_obstacle_points = static_cast<int>(parameter.as_int());
      } else if (name == "resume_distance") {
        resume_distance = parameter.as_double();
      } else if (name == "confirm_frames") {
        confirm_frames = static_cast<int>(parameter.as_int());
      } else if (name == "clear_frames") {
        clear_frames = static_cast<int>(parameter.as_int());
      } else if (name == "auto_resume") {
        auto_resume = parameter.as_bool();
      } else if (name == "lidar_timeout") {
        lidar_timeout = parameter.as_double();
      } else if (name == "lidar_timeout_stop_enabled") {
        lidar_timeout_stop_enabled = parameter.as_bool();
      } else if (name == "command_timeout") {
        command_timeout = parameter.as_double();
      } else if (name == "command_retry_cooldown") {
        command_retry_cooldown = parameter.as_double();
      } else if (name == "hold_confirm_timeout") {
        hold_confirm_timeout = parameter.as_double();
      } else if (name == "diagnostic_period") {
        diagnostic_period = parameter.as_double();
      }
      // pointcloud_topic / vehicle_status_topic / vehicle_command_service / hold_method are
      // startup only (they would require recreating publishers/subscribers/services).
    } catch (const std::exception & e) {
      result.successful = false;
      result.reason = std::string("parameter type error: ") + e.what();
      return result;
    }
  }

  if (config.min_range < 0.0 || config.stop_distance <= config.min_range ||
    config.z_max <= config.z_min)
  {
    result.successful = false;
    result.reason = "invalid ROI: require min_range >= 0 and min_range < stop_distance and z_min < z_max";
    return result;
  }
  if (config.min_obstacle_points < 1) {
    result.successful = false;
    result.reason = "min_obstacle_points must be >= 1";
    return result;
  }
  if (resume_distance < config.stop_distance) {
    result.successful = false;
    result.reason = "resume_distance must be >= stop_distance";
    return result;
  }
  if (confirm_frames < 1 || clear_frames < 1) {
    result.successful = false;
    result.reason = "confirm_frames and clear_frames must be >= 1";
    return result;
  }
  if (lidar_timeout <= 0.0 || command_timeout <= 0.0 || command_retry_cooldown < 0.0 ||
    hold_confirm_timeout <= 0.0 || diagnostic_period < 0.0)
  {
    result.successful = false;
    result.reason = "timings must be > 0 (command_retry_cooldown >= 0, diagnostic_period >= 0)";
    return result;
  }

  detector_config_ = config;
  detector_.setConfig(config);
  resume_distance_ = resume_distance;
  lidar_timeout_ = lidar_timeout;
  command_timeout_ = command_timeout;
  command_retry_cooldown_ = command_retry_cooldown;
  hold_confirm_timeout_ = hold_confirm_timeout;
  diagnostic_period_ = diagnostic_period;
  confirm_frames_ = static_cast<std::size_t>(confirm_frames);
  clear_frames_ = static_cast<std::size_t>(clear_frames);
  auto_resume_ = auto_resume;
  lidar_timeout_stop_enabled_ = lidar_timeout_stop_enabled;

  RCLCPP_INFO(
    get_logger(),
    "Parameters updated: stop_distance=%.2f m, resume_distance=%.2f m, min_obstacle_points=%d, "
    "confirm_frames=%zu, clear_frames=%zu, auto_resume=%s, lidar_timeout=%.2f s, "
    "lidar_timeout_stop_enabled=%s",
    detector_config_.stop_distance, resume_distance_, detector_config_.min_obstacle_points,
    confirm_frames_, clear_frames_, auto_resume_ ? "true" : "false", lidar_timeout_,
    lidar_timeout_stop_enabled_ ? "true" : "false");

  return result;
}

// ------------------------------------------------------------------- helpers

bool ObstacleStopNode::isPx4Mission() const
{
  return status_received_ &&
         nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION;
}

bool ObstacleStopNode::commandCooldownElapsed(const rclcpp::Time & now_ts) const
{
  return rclcpp::Duration(now_ts - last_command_time_).seconds() >= command_retry_cooldown_;
}

const char * ObstacleStopNode::stateName(StopState state)
{
  switch (state) {
    case StopState::kClear:
      return "CLEAR";
    case StopState::kPausePending:
      return "PAUSE_PENDING";
    case StopState::kPaused:
      return "PAUSED";
    case StopState::kResumePending:
      return "RESUME_PENDING";
  }
  return "UNKNOWN";
}

const char * ObstacleStopNode::navStateName(uint8_t nav_state)
{
  using VehicleStatus = px4_msgs::msg::VehicleStatus;
  switch (nav_state) {
    case VehicleStatus::NAVIGATION_STATE_MANUAL:
      return "MANUAL";
    case VehicleStatus::NAVIGATION_STATE_ALTCTL:
      return "ALTCTL";
    case VehicleStatus::NAVIGATION_STATE_POSCTL:
      return "POSCTL";
    case VehicleStatus::NAVIGATION_STATE_AUTO_MISSION:
      return "AUTO_MISSION";
    case VehicleStatus::NAVIGATION_STATE_AUTO_LOITER:
      return "AUTO_LOITER(Hold)";
    case VehicleStatus::NAVIGATION_STATE_AUTO_RTL:
      return "AUTO_RTL";
    case VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF:
      return "AUTO_TAKEOFF";
    case VehicleStatus::NAVIGATION_STATE_AUTO_LAND:
      return "AUTO_LAND";
    case VehicleStatus::NAVIGATION_STATE_AUTO_PRECLAND:
      return "AUTO_PRECLAND";
    case VehicleStatus::NAVIGATION_STATE_ORBIT:
      return "ORBIT";
    case VehicleStatus::NAVIGATION_STATE_OFFBOARD:
      return "OFFBOARD";
    case VehicleStatus::NAVIGATION_STATE_DESCEND:
      return "DESCEND";
    case VehicleStatus::NAVIGATION_STATE_TERMINATION:
      return "TERMINATION";
    case VehicleStatus::NAVIGATION_STATE_STAB:
      return "STAB";
    case VehicleStatus::NAVIGATION_STATE_ACRO:
      return "ACRO";
    default:
      return "OTHER";
  }
}

const char * ObstacleStopNode::ackResultName(uint8_t result)
{
  using Ack = px4_msgs::msg::VehicleCommandAck;
  switch (result) {
    case Ack::VEHICLE_CMD_RESULT_ACCEPTED:
      return "ACCEPTED";
    case Ack::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
      return "TEMPORARILY_REJECTED";
    case Ack::VEHICLE_CMD_RESULT_DENIED:
      return "DENIED";
    case Ack::VEHICLE_CMD_RESULT_UNSUPPORTED:
      return "UNSUPPORTED";
    case Ack::VEHICLE_CMD_RESULT_FAILED:
      return "FAILED";
    case Ack::VEHICLE_CMD_RESULT_IN_PROGRESS:
      return "IN_PROGRESS";
    case Ack::VEHICLE_CMD_RESULT_CANCELLED:
      return "CANCELLED";
    default:
      return "UNKNOWN";
  }
}

}  // namespace mid360_obstacle_stop

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mid360_obstacle_stop::ObstacleStopNode>();
  // SingleThreadedExecutor: combined with the mutually exclusive callback group every callback is
  // serialized, so no locking is required inside the node.
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
