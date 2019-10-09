#ifndef ASTAR_PATH_PLANNER_MARKER_H
#define ASTAR_PATH_PLANNER_MARKER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "maze_planner/occupancy_grid.h"

namespace maze_planner
{
enum class MarkerColour
{
  RED,
  GREEN,
  BLUE,
  YELLOW,
  FUCHSIA
};

visualization_msgs::Marker createSphereMarker(double position_x, double position_y, MarkerColour colour, double scale);
visualization_msgs::Marker createSphereListMarker(const std::vector<maze_planner::WorldPosition>& world_positions,
                                                  MarkerColour colour, double scale);
visualization_msgs::Marker createDeleteMarker();

}  // namespace maze_planner

#endif  // ASTAR_PATH_PLANNER_MARKER_H
