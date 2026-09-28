// Obstacle detection for a MID360 point cloud.
//
// Stage 1 implementation: 360 deg horizontal safety cylinder around the vehicle.
// The whole cloud is scanned in the sensor/body frame, so it does not matter whether the
// vehicle flies forwards, sideways or backwards.
//
// This file deliberately has no dependency on PX4 or on the node itself, so the detection
// logic stays unit-testable and can later be replaced by a velocity aligned safety corridor
// (feed a different ROI into the same structs).

#ifndef MID360_OBSTACLE_STOP__OBSTACLE_DETECTOR_HPP_
#define MID360_OBSTACLE_STOP__OBSTACLE_DETECTOR_HPP_

#include <cstddef>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace mid360_obstacle_stop
{

struct DetectorConfig
{
  double min_range{0.5};         // [m] ignore points closer than this (self hits, props, landing gear)
  double stop_distance{3.0};     // [m] danger radius (horizontal distance)
  double z_min{-0.6};            // [m] ROI lower bound in body frame
  double z_max{0.8};             // [m] ROI upper bound in body frame
  int min_obstacle_points{10};   // points inside the danger cylinder required for a single frame
};

struct DetectionResult
{
  bool obstacle{false};              // current frame counts as "obstacle present"
  std::size_t roi_points{0};         // points inside the ROI (min_range..inf, z_min..z_max)
  std::size_t danger_points{0};      // points inside the ROI and closer than stop_distance
  double nearest_distance{-1.0};     // nearest horizontal distance inside ROI, < 0 when empty
};

class ObstacleDetector
{
public:
  // False when the cloud does not carry x/y/z fields (e.g. Livox custom format).
  static bool hasXyzFields(const sensor_msgs::msg::PointCloud2 & cloud);

  // Comma separated field list, for diagnostics/logging.
  static std::string fieldNames(const sensor_msgs::msg::PointCloud2 & cloud);

  void setConfig(const DetectorConfig & config) {config_ = config;}
  const DetectorConfig & config() const {return config_;}

  // Scans the full cloud: 360 deg horizontally, z_min < z < z_max.
  DetectionResult detect(const sensor_msgs::msg::PointCloud2 & cloud) const;

private:
  DetectorConfig config_{};
};

}  // namespace mid360_obstacle_stop

#endif  // MID360_OBSTACLE_STOP__OBSTACLE_DETECTOR_HPP_
