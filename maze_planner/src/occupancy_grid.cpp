#include "maze_planner/occupancy_grid.h"

namespace maze_planner
{
OccupancyGrid::OccupancyGrid(const nav_msgs::OccupancyGrid& map, const double inflation_radius)
{
  // Copy the occupancy grid message
  map_ = map;

  // Access occupancy grid message data with an image
  map_image_ = cv::Mat(map.info.height, map.info.width, CV_8U, &map_.data.front());

  // Dilate the image
  int element_diameter =
      2 * static_cast<int>(std::round(inflation_radius / map.info.resolution)) + 1;  // element_diameter is always odd

  int offset = (element_diameter - 1) / 2;  // Centre of the element

  cv::Mat element =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(element_diameter, element_diameter), cv::Size(offset, offset));

  cv::dilate(map_image_, map_image_, element);

  // Map geometry for particle filter
  map_x_min_ = map_.info.origin.position.x;
  map_x_max_ = map_.info.width * map_.info.resolution + map_.info.origin.position.x;

  map_y_min_ = map_.info.origin.position.y;
  map_y_max_ = map_.info.height * map_.info.resolution + map_.info.origin.position.y;
}

bool OccupancyGrid::isOutOfBounds(GridPosition grid_position)
{
  return grid_position.x < 0 || grid_position.x > map_image_.cols ||  //
         grid_position.y < 0 || grid_position.y > map_image_.rows;
}

bool OccupancyGrid::isOutOfBounds(WorldPosition world_position)
{
  return world_position.x < map_x_min_ || world_position.x > map_x_max_ ||  //
         world_position.y < map_y_min_ || world_position.y > map_y_max_;
}

bool OccupancyGrid::isOccupied(int id)
{
  return map_.data[id] != 0;
}

bool OccupancyGrid::isOccupied(GridPosition grid_position)
{
  return isOccupied(getCellId(grid_position));
}

bool OccupancyGrid::isOccupied(WorldPosition world_position)
{
  return isOccupied(getGridPosition(world_position));
}

GridPosition OccupancyGrid::getGridPosition(int id)
{
  return { id % static_cast<int>(map_.info.width), id / static_cast<int>(map_.info.width) };
}

GridPosition OccupancyGrid::getGridPosition(WorldPosition world_position)
{
  GridPosition grid_position{};

  grid_position.x =
      static_cast<int>(std::floor((world_position.x - map_.info.origin.position.x) / map_.info.resolution));

  grid_position.y =
      static_cast<int>(std::floor((world_position.y - map_.info.origin.position.y) / map_.info.resolution));

  return grid_position;
}

WorldPosition OccupancyGrid::getWorldPosition(GridPosition grid_position)
{
  WorldPosition world_position{};

  world_position.x = static_cast<double>(grid_position.x) * map_.info.resolution + map_.info.origin.position.x +
                     (map_.info.resolution / 2.);
  world_position.y = static_cast<double>(grid_position.y) * map_.info.resolution + map_.info.origin.position.y +
                     (map_.info.resolution / 2.);

  return world_position;
}

WorldPosition OccupancyGrid::getWorldPosition(int id)
{
  return getWorldPosition(getGridPosition(id));
}

int OccupancyGrid::getCellId(GridPosition grid_position)
{
  return grid_position.y * static_cast<int>(map_.info.width) + grid_position.x;
}

Cell OccupancyGrid::getCell(int id)
{
  Cell cell{};

  cell.id = id;
  cell.occupied = isOccupied(id);
  cell.grid_position = getGridPosition(id);
  cell.world_position = getWorldPosition(cell.grid_position);

  return cell;
}

Cell OccupancyGrid::getCell(GridPosition grid_position)
{
  return getCell(getCellId(grid_position));
}

Cell OccupancyGrid::getCell(WorldPosition world_position)
{
  return getCell(getGridPosition(world_position));
}

nav_msgs::OccupancyGrid OccupancyGrid::getOccupancyGridMsg()
{
  return map_;
}

    std::vector<int> OccupancyGrid::getAdjacentCellIDs(int id)
{
  // Return the unoccupied cells adjacent to "id"

  // Grid position of the given cell, use this to get adjacent cell grid positions
  GridPosition grid_position = getGridPosition(id);

  // Fill this with adjacent cells
  std::vector<int> ids;

  // Use "isOutOfBounds" and "isOccupied" to check if the adjacent cells are out of bounds or occupied
  // The "AdjacentCell" structure has three fields: "id", "cost", and "world_position"
  // Use "getCellId" and "getWorldPosition" to get a cell ID and world position from a grid position
  // "cost" is the cost of moving from the parent to the adjacent cell in metres
  // "map_.info.resolution" is the distance between cells
  // Only return diagonal cells if "diagonal_movement" is true
  // Keep in mind that the distance between diagonal cells is larger than horizontal/vertical cells

  // YOUR CODE HERE
  // sets the cell additions depending on whether diagonal movement
  std::vector<int> x_adds = {-1, 0, 1,-1, 1,-1, 0, 1};
  std::vector<int> y_adds= {-1,-1,-1, 0, 0, 1, 1, 1};

  // initialises the world pos of the input for cost calculations
  WorldPosition world_pos = getWorldPosition(grid_position);
  // iterates through the cell additions for all the cells
  for(unsigned i = 0; i < x_adds.size(); i++) {
      // initialises test grid positions
      GridPosition t_grid_pos = {grid_position.x + x_adds[i], grid_position.y + y_adds[i]};
      // checks if grid position is valid
      if(!isOutOfBounds(t_grid_pos) && !isOccupied(t_grid_pos)) {
          WorldPosition t_world_pos = getWorldPosition(t_grid_pos);
          ids.push_back(getCellId(t_grid_pos));
      }
  }

  return ids;
}

    double OccupancyGrid::cellDistance(int id_a, int id_b) {
        WorldPosition w_a = getWorldPosition(id_a);
        WorldPosition w_b = getWorldPosition(id_b);
        return sqrt(pow(w_a.x-w_b.x,2)+pow(w_a.y-w_b.y,2));
    }

    nav_msgs::MapMetaData OccupancyGrid::getMapData() {
        return map_.info;
    }

    std::vector<geometry_msgs::PoseStamped> OccupancyGrid::pathFromNodes(std::vector<std::vector<int>> ids) {
        std::vector<geometry_msgs::PoseStamped> path;
        std::vector<geometry_msgs::Pose2D> path_2D;

        for(auto id_vec:ids) {
            if(id_vec.size() < 2) {
                geometry_msgs::Pose2D p;
                WorldPosition w = getWorldPosition(id_vec[0]);
                p.x = w.x;
                p.y = w.y;
                path_2D.push_back(p);
                continue;
            } else {
                geometry_msgs::Pose2D p;
                WorldPosition w = getWorldPosition(id_vec[0]);
                WorldPosition w2 = getWorldPosition(id_vec[1]);
                p.x = w.x;
                p.y = w.y;
                p.theta = atan2(w2.y-w.y, w2.x-w.x);
                path_2D.push_back(p);
            }
            for(auto i = 1; i < id_vec.size(); i++) {
                geometry_msgs::Pose2D p;
                WorldPosition w = getWorldPosition(id_vec[i-1]);
                WorldPosition w2 = getWorldPosition(id_vec[i]);
                p.x = w.x;
                p.y = w.y;
                p.theta = atan2(w2.y-w.y, w2.x-w.x);
                path_2D.push_back(p);
            }
        }

        for(auto p:path_2D){
            geometry_msgs::PoseStamped p_s;
            p_s.pose.position.x = p.x;
            p_s.pose.position.y = p.y;
            p_s.pose.position.z = 0;

            tf2::Quaternion q;
            q.setRPY(0.0,0.0, p.theta);
            p_s.pose.orientation = tf2::toMsg(q);
            path.push_back(p_s);
        }

        return path;

    }

}  // namespace astar_path_planner