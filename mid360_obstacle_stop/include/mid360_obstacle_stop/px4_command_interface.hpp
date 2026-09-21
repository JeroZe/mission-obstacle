// Thin wrapper around the PX4 "VehicleCommand" ROS 2 service (/fmu/vehicle_command).
//
// Verified against PX4 v1.16.0 and px4_msgs release/1.16:
//   * Service name  : /fmu/vehicle_command            (uxrce_dds_client, PX4 >= v1.15)
//   * Service type  : px4_msgs/srv/VehicleCommand     (fields: "request", "reply")
//   * Reply         : px4_msgs/msg/VehicleCommandAck  (field "result")
//
// "Hold / pause the mission" on PX4 v1.16 - important, verified in the PX4 source:
//   * MAV_CMD_DO_PAUSE_CONTINUE (193) is NOT implemented by the PX4 v1.16 flight stack.
//     Commander::handle_command() has no case for it and its "default:" branch answers
//     VEHICLE_CMD_RESULT_UNSUPPORTED (3), so the vehicle would keep flying.
//   * What PX4 actually uses (and what the QGroundControl "Pause" button sends, see
//     PX4FirmwarePlugin::pauseVehicle) is VEHICLE_CMD_DO_REPOSITION (192) with
//     param2 = MAV_DO_REPOSITION_FLAGS_CHANGE_MODE and NaN lat/lon/alt. Commander then
//     switches to NAVIGATION_STATE_AUTO_LOITER (Hold) and the navigator builds a braking
//     aware stop point (Navigator::preproject_stop_point). Deceleration, stopping and
//     holding stay entirely inside PX4 - ROS 2 never touches attitude/position control.
//
// Both variants are selectable through the "hold_method" parameter so that the node keeps
// working if a later PX4 release implements MAV_CMD_DO_PAUSE_CONTINUE.

#ifndef MID360_OBSTACLE_STOP__PX4_COMMAND_INTERFACE_HPP_
#define MID360_OBSTACLE_STOP__PX4_COMMAND_INTERFACE_HPP_

#include <cstdint>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>

namespace mid360_obstacle_stop
{

enum class HoldMethod
{
  kDoRepositionHold,   // PX4 v1.16 native "pause mission" (same as QGC pause button)
  kDoPauseContinue,    // MAV_CMD_DO_PAUSE_CONTINUE, not implemented by PX4 v1.16
};

inline const char * holdMethodName(HoldMethod method)
{
  switch (method) {
    case HoldMethod::kDoRepositionHold:
      return "do_reposition_hold";
    case HoldMethod::kDoPauseContinue:
      return "do_pause_continue";
  }
  return "unknown";
}

class Px4CommandInterface
{
public:
  using Client = rclcpp::Client<px4_msgs::srv::VehicleCommand>;
  using AckCallback = std::function<void(bool accepted, uint8_t result, uint32_t command)>;

  Px4CommandInterface(
    rclcpp::Node & node, const std::string & service_name, HoldMethod method,
    rclcpp::CallbackGroup::SharedPtr callback_group)
  : node_(node),
    logger_(node.get_logger()),
    method_(method),
    service_name_(service_name)
  {
    // Services on the PX4 side are RELIABLE + KEEP_LAST(1); the default ROS 2 service QoS
    // profile (reliable, volatile, depth 10) is compatible with that.
    client_ = node_.create_client<px4_msgs::srv::VehicleCommand>(
      service_name_, rmw_qos_profile_services_default, std::move(callback_group));
  }

  bool serviceReady() const {return client_->service_is_ready();}

  const std::string & serviceName() const {return service_name_;}

  HoldMethod method() const {return method_;}

  // Ask PX4 to stop and hold the current mission (Mission -> Hold/Loiter).
  void requestHold(AckCallback ack)
  {
    px4_msgs::msg::VehicleCommand command{};
    const float nan_f = std::numeric_limits<float>::quiet_NaN();
    const double nan_d = std::numeric_limits<double>::quiet_NaN();

    if (method_ == HoldMethod::kDoRepositionHold) {
      command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_REPOSITION;
      command.param1 = -1.0f;   // ground speed: <= 0 -> keep the configured speed
      command.param2 = 1.0f;    // MAV_DO_REPOSITION_FLAGS_CHANGE_MODE (bit 0 only!)
      command.param3 = 0.0f;    // loiter radius (fixed wing only)
      command.param4 = nan_f;   // yaw: keep current heading
      command.param5 = nan_d;   // latitude: NaN -> stay at the current position
      command.param6 = nan_d;   // longitude: NaN -> stay at the current position
      command.param7 = nan_f;   // altitude: NaN -> stay at the current altitude

    } else {
      command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_PAUSE_CONTINUE;
      command.param1 = 0.0f;    // 0 = pause
    }

    send(command, std::move(ack));
  }

  // Continue the mission that was previously paused by this node.
  void requestContinue(AckCallback ack)
  {
    px4_msgs::msg::VehicleCommand command{};

    if (method_ == HoldMethod::kDoRepositionHold) {
      // PX4 specific command (100001): switch the nav_state directly. PX4 resumes the stored
      // mission item, i.e. the mission continues where it was interrupted.
      command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE;
      command.param1 = static_cast<float>(
        px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION);

    } else {
      command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_PAUSE_CONTINUE;
      command.param1 = 1.0f;    // 1 = continue
    }

    send(command, std::move(ack));
  }

private:
  void send(px4_msgs::msg::VehicleCommand & command, AckCallback ack)
  {
    command.target_system = 1;      // MAVLink system id of the autopilot
    command.target_component = 1;   // MAV_COMP_ID_AUTOPILOT1
    command.source_system = 1;
    // Must stay below vehicle_command::COMPONENT_MODE_EXECUTOR_START (1000), otherwise PX4
    // treats the command as coming from a mode executor.
    command.source_component = 1;
    command.from_external = true;
    // PX4 uses microseconds; uXRCE-DDS converts the ROS timestamp to autopilot time.
    command.timestamp = static_cast<uint64_t>(node_.get_clock()->now().nanoseconds() / 1000);

    auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();
    request->request = command;

    const uint32_t command_id = command.command;
    auto logger = logger_;

    client_->async_send_request(
      request,
      [ack = std::move(ack), command_id, logger](Client::SharedFuture future) {
        try {
          const auto reply = future.get()->reply;
          ack(
            reply.result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED,
            reply.result, command_id);

        } catch (const std::exception & e) {
          // Service disappeared / request could not be delivered: never report success.
          RCLCPP_ERROR(
            logger, "vehicle_command %u: no valid reply from PX4 (%s)",
            command_id, e.what());
          ack(false, px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_FAILED, command_id);
        }
      });
  }

  rclcpp::Node & node_;
  rclcpp::Logger logger_;
  HoldMethod method_;
  std::string service_name_;
  Client::SharedPtr client_;
};

}  // namespace mid360_obstacle_stop

#endif  // MID360_OBSTACLE_STOP__PX4_COMMAND_INTERFACE_HPP_
