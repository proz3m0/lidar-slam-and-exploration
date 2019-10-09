#include "maze_planner/marker.h"

namespace maze_planner
{
visualization_msgs::Marker createSphereMarker(double position_x, double position_y, MarkerColour colour, double scale)
{
  // Publish start marker
  visualization_msgs::Marker marker{};

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "map";

  marker.id = 0;
  marker.type = marker.SPHERE;

  marker.pose.position.x = position_x;
  marker.pose.position.y = position_y;

  if (colour == MarkerColour::RED || colour == MarkerColour::FUCHSIA)
  {
    marker.color.r = 1.0;
  }

  if (colour == MarkerColour::GREEN || colour == MarkerColour::YELLOW)
  {
    marker.color.g = 1.0;
  }

  if (colour == MarkerColour::BLUE)
  {
    marker.color.b = 1.0;
  }

  marker.color.a = 1.f;

  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;

  return marker;
}

visualization_msgs::Marker createSphereListMarker(const std::vector<maze_planner::WorldPosition>& world_positions,
                                                  MarkerColour colour, double scale)
{
  // Publish start marker
  visualization_msgs::Marker marker{};

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "map";

  marker.id = 0;
  marker.type = marker.SPHERE_LIST;

  marker.points.reserve(world_positions.size());

  for (const auto& world_position : world_positions)
  {
    geometry_msgs::Point point{};

    point.x = world_position.x;
    point.y = world_position.y;

    marker.points.push_back(point);
  }

  if (colour == MarkerColour::RED || colour == MarkerColour::YELLOW || colour == MarkerColour::FUCHSIA)
  {
    marker.color.r = 1.0;
  }

  if (colour == MarkerColour::GREEN || colour == MarkerColour::YELLOW)
  {
    marker.color.g = 1.0;
  }

  if (colour == MarkerColour::BLUE || colour == MarkerColour::FUCHSIA)
  {
    marker.color.b = 1.0;
  }

  marker.color.a = 1.f;

  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;

  return marker;
}

visualization_msgs::Marker createDeleteMarker()
{
  visualization_msgs::Marker marker{};
  marker.action = marker.DELETE;

  return marker;
}

}  // namespace astar_path_planner
