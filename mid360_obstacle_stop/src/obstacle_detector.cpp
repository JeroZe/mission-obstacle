#include "mid360_obstacle_stop/obstacle_detector.hpp"

#include <cmath>

#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace mid360_obstacle_stop
{

bool ObstacleDetector::hasXyzFields(const sensor_msgs::msg::PointCloud2 & cloud)
{
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;

  for (const auto & field : cloud.fields) {
    if (field.name == "x") {
      has_x = true;
    } else if (field.name == "y") {
      has_y = true;
    } else if (field.name == "z") {
      has_z = true;
    }
  }

  return has_x && has_y && has_z;
}

std::string ObstacleDetector::fieldNames(const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::string names;
  for (const auto & field : cloud.fields) {
    if (!names.empty()) {
      names += ", ";
    }
    names += field.name;
  }
  return names;
}

DetectionResult ObstacleDetector::detect(const sensor_msgs::msg::PointCloud2 & cloud) const
{
  DetectionResult result{};

  if (cloud.data.empty() || cloud.point_step == 0U || !hasXyzFields(cloud)) {
    return result;
  }

  const double min_range_sq = config_.min_range * config_.min_range;
  const double stop_distance_sq = config_.stop_distance * config_.stop_distance;
  const double z_min = config_.z_min;
  const double z_max = config_.z_max;
  const std::size_t min_points =
    config_.min_obstacle_points > 0 ? static_cast<std::size_t>(config_.min_obstacle_points) : 1U;

  sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");

  // Drive the loop by the point count instead of iterator::end(): the iterators already handle
  // row padding/point_step themselves, and this avoids depending on API that varies between
  // sensor_msgs releases.
  const std::size_t point_count = static_cast<std::size_t>(cloud.width) * cloud.height;

  for (std::size_t i = 0U; i < point_count; ++i, ++it_x, ++it_y, ++it_z) {
    const float x = *it_x;
    const float y = *it_y;
    const float z = *it_z;

    // Drop NaN/inf points (MID360 occasionally reports them).
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    // Height ROI: z_min < z < z_max.
    if (!(z > z_min && z < z_max)) {
      continue;
    }

    const double distance_sq = static_cast<double>(x) * x + static_cast<double>(y) * y;

    // Ignore the vehicle itself (props, landing gear, ...).
    if (distance_sq < min_range_sq) {
      continue;
    }

    ++result.roi_points;

    const double distance = std::sqrt(distance_sq);
    if (result.nearest_distance < 0.0 || distance < result.nearest_distance) {
      result.nearest_distance = distance;
    }

    if (distance_sq < stop_distance_sq) {
      ++result.danger_points;
    }
  }

  // Never stop on a single noisy point: require a minimum number of points in the danger cylinder.
  result.obstacle = result.danger_points >= min_points;

  return result;
}

}  // namespace mid360_obstacle_stop
