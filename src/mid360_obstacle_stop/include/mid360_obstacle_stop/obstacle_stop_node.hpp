// Mission-mode obstacle stop node for a Livox MID360 + PX4 (uXRCE-DDS / ROS 2).
//
// Behaviour (stage 1):
//   PX4 is flying an AUTO_MISSION -> MID360 sees an obstacle inside the safety cylinder ->
//   this node asks PX4 (through the /fmu/vehicle_command service) to stop the mission ->
//   PX4 decelerates, holds and reports AUTO_LOITER. ROS 2 never sends setpoints, never
//   switches to Offboard and never touches attitude/position control.
//
// Structure (kept separated on purpose, see also the design notes in README.md):
//   * ObstacleDetector      - point cloud -> ROI/danger decision (no ROS comms)
//   * Px4CommandInterface   - VehicleCommand service calls + ACK parsing
//   * ObstacleStopNode      - parameters, subscriptions, state machine, logging, watchdog
//
// Threading: every callback (point cloud, vehicle status, timers, service replies) runs in a
// single mutually exclusive callback group, therefore the state below needs no locking even if
// the executor is later changed to a MultiThreadedExecutor. main() uses rclcpp::spin().
//
// Safety rules implemented here:
//   * only NAVIGATION_STATE_AUTO_MISSION may be paused,
//   * one pause request per obstacle event (state machine, never per point cloud frame),
//   * PAUSED is only entered after PX4 answers VEHICLE_CMD_RESULT_ACCEPTED,
//   * automatic resume is off by default and only ever resumes a pause that this node created,
//   * after an external mode change / manual resume the node re-arms only once the danger area
//     was clear again (it never fights the pilot).

#ifndef MID360_OBSTACLE_STOP__OBSTACLE_STOP_NODE_HPP_
#define MID360_OBSTACLE_STOP__OBSTACLE_STOP_NODE_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_status.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "mid360_obstacle_stop/obstacle_detector.hpp"
#include "mid360_obstacle_stop/px4_command_interface.hpp"

namespace mid360_obstacle_stop
{

enum class StopState
{
  kClear,          // no obstacle inside stop_distance, mission may run freely
  kPausePending,   // pause request sent, waiting for the PX4 ACK
  kPaused,         // PX4 accepted the pause, vehicle is holding
  kResumePending,  // continue request sent, waiting for the PX4 ACK
};

class ObstacleStopNode : public rclcpp::Node
{
public:
  explicit ObstacleStopNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ---------------------------------------------------------------- callbacks
  void onPointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void onVehicleStatus(px4_msgs::msg::VehicleStatus::ConstSharedPtr msg);
  void onCommandAck(bool accepted, uint8_t result, uint32_t command);
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  // ------------------------------------------------------- detection / state machine
  void processPointCloud(const sensor_msgs::msg::PointCloud2 & cloud);
  void updateStateMachine();
  void requestMissionPause(const std::string & reason);
  void requestMissionContinue();
  void handleExternalHoldRelease();
  void armReArmGrace();
  bool isPx4Mission() const;
  bool commandCooldownElapsed(const rclcpp::Time & now) const;
  void checkLidarTimeout();

  // ------------------------------------------------------------------- helpers
  void loadParameters();
  static const char * stateName(StopState state);
  static const char * navStateName(uint8_t nav_state);
  static const char * ackResultName(uint8_t result);

  // ---------------------------------------------------------------- parameters
  std::string pointcloud_topic_;
  std::string vehicle_status_topic_;
  std::string vehicle_command_service_;

  DetectorConfig detector_config_;
  double resume_distance_{4.0};
  std::size_t confirm_frames_{3U};   // threshold: consecutive danger frames before a pause
  std::size_t clear_frames_{10U};    // threshold: consecutive clear frames before the pause re-arms

  bool auto_resume_{false};
  double lidar_timeout_{0.5};
  bool lidar_timeout_stop_enabled_{false};

  HoldMethod hold_method_{HoldMethod::kDoRepositionHold};
  double command_timeout_{2.0};
  double command_retry_cooldown_{2.0};
  double hold_confirm_timeout_{2.0};
  double diagnostic_period_{0.0};   // > 0: print one detection summary line per period [s]
  double rearm_grace_{5.0};         // [s] mute window after this node loses mission ownership

  // ------------------------------------------------------------- ROS interfaces
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  std::unique_ptr<Px4CommandInterface> px4_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  ObstacleDetector detector_;

  // -------------------------------------------------------------------- state
  StopState state_{StopState::kClear};
  bool paused_by_obstacle_{false};   // true only while a pause created by this node is active
  bool hold_confirmed_{false};       // PX4 reported AUTO_LOITER after our pause ACK
  bool hold_state_unknown_{false};   // ACK accepted but AUTO_LOITER never observed

  uint8_t nav_state_{0};
  bool armed_{false};
  bool status_received_{false};

  std::size_t obstacle_frames_{0U};     // counter, saturates at confirm_frames_
  std::size_t clear_frame_count_{0U};   // counter, saturates at clear_frames_
  DetectionResult last_detection_{};

  rclcpp::Time last_cloud_time_;
  bool cloud_received_{false};
  bool lidar_timeout_active_{false};
  bool lidar_timeout_was_active_{false};

  rclcpp::Time last_command_time_;   // last pause/continue request (cooldown + ACK timeout)
  rclcpp::Time pause_ack_time_;      // when the pause was accepted
  rclcpp::Time last_diagnostic_time_;  // last periodic detection summary

  bool rearm_pending_{false};   // automatic pause muted until rearm_time_
  rclcpp::Time rearm_time_;     // when the mute expires
};

}  // namespace mid360_obstacle_stop

#endif  // MID360_OBSTACLE_STOP__OBSTACLE_STOP_NODE_HPP_
