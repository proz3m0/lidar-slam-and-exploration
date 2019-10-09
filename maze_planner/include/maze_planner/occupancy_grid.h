// An undirected graph

#ifndef ASTAR_PATH_PLANNER_OCCUPANCY_GRID_H
#define ASTAR_PATH_PLANNER_OCCUPANCY_GRID_H

#include <vector>
#include <utility>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <nav_msgs/OccupancyGrid.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace maze_planner {
    struct WorldPosition {
        double x = 0.;
        double y = 0.;
    };

    struct GridPosition {
        int x = 0;
        int y = 0;
    };

    struct Cell {
        int id = -1;
        bool occupied = true;
        GridPosition grid_position{};
        WorldPosition world_position{};  // The world position of the centre of the cell
    };

    struct AdjacentCell {
        int id = -1;
        double cost = 0.;                // Cost of moving from the parent in metres
        WorldPosition world_position{};  // The world position of the centre of the cell
    };

    class OccupancyGrid {
    public:
        // Constructors
        OccupancyGrid() = default;

        OccupancyGrid(const nav_msgs::OccupancyGrid &map, double inflation_radius);

        // Public methods
        bool isOutOfBounds(WorldPosition world_position);

        bool isOccupied(WorldPosition world_position);

        nav_msgs::MapMetaData getMapData();

        WorldPosition getWorldPosition(int id);  // World position of the centre of the cell
        Cell getCell(WorldPosition world_position);

        Cell getCell(int id);

        nav_msgs::OccupancyGrid getOccupancyGridMsg();  // This is for publishing the inflated map
        bool isOutOfBounds(GridPosition grid_position);

        bool isOccupied(int id);

        bool isOccupied(GridPosition grid_position);

        GridPosition getGridPosition(WorldPosition world_position);

        GridPosition getGridPosition(int id);

        WorldPosition getWorldPosition(GridPosition grid_position);  // World position of the centre of the cell
        int getCellId(GridPosition grid_position);

        Cell getCell(GridPosition grid_position);

        std::vector<int> getAdjacentCellIDs(int id);

        double cellDistance(int id_a, int id_b);

        std::vector<geometry_msgs::PoseStamped> pathFromNodes(std::vector<std::vector<int>> ids);



    private:
        nav_msgs::OccupancyGrid map_{};
        cv::Mat map_image_{};

        // Limits of the map in metres
        double map_x_min_ = 0., map_x_max_ = 0., map_y_min_ = 0., map_y_max_ = 0.;

        // Private methods

    };

}  // namespace astar_path_planner

#endif  // ASTAR_PATH_PLANNER_OCCUPANCY_GRID_H
