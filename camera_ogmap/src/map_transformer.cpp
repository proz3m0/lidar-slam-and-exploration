#include "camera_ogmap/map_transformer.h"

MapTransformer::MapTransformer() {}

MapTransformer::MapTransformer(nav_msgs::MapMetaData map_data):map_data_(map_data) {}

bool MapTransformer::gridCellToMapPoint(GridCell &grid_cell, geometry_msgs::Pose2D &map_point) {
    // checks if grid cell is within map
    if(grid_cell.x >= 0 && grid_cell.x < map_data_.width &&
       grid_cell.y >= 0 && grid_cell.y < map_data_.height) {
        map_point.x = map_data_.origin.position.x + grid_cell.x * map_data_.resolution;
        map_point.y = map_data_.origin.position.y + grid_cell.y * map_data_.resolution;
        return true;
    }
    return false;
}

bool MapTransformer::mapPointToGridCell(geometry_msgs::Pose2D &map_point, GridCell &grid_cell) {
    // checks if map point is within map
    if(static_cast<int>(map_point.x-map_data_.origin.position.x) >= 0 &&
       static_cast<int>((map_point.x-map_data_.origin.position.x)/map_data_.resolution) <= map_data_.width &&
       static_cast<int>(map_point.y-map_data_.origin.position.y) >= 0 &&
       static_cast<int>((map_point.y-map_data_.origin.position.y)/map_data_.resolution) <= map_data_.height) {
        grid_cell.x = static_cast<int>((map_point.x-map_data_.origin.position.x)/map_data_.resolution);
        grid_cell.y = static_cast<int>((map_point.y-map_data_.origin.position.y)/map_data_.resolution);
        return true;
    }
    return false;
}

std::vector<std::vector<GridCell>>
MapTransformer::mapPointToGridCells(std::vector<std::vector<geometry_msgs::Pose2D>> map_points) {
    std::vector<std::vector<GridCell>> grid_cells;
    for(const auto& map_point_vec:map_points) {
        std::vector<GridCell> grid_cells_vec;
        for(auto map_point:map_point_vec) {
            GridCell grid_cell{};
            if(mapPointToGridCell(map_point, grid_cell)) grid_cells_vec.push_back(grid_cell);
        }
        grid_cells.push_back(grid_cells_vec);
    }
    return grid_cells;
}

bool MapTransformer::gridCellToIndex(GridCell &grid_cell, int &index) {
    // checks if grid cell is within map
    if(grid_cell.x >= 0 && grid_cell.x < map_data_.width &&
       grid_cell.y >= 0 && grid_cell.y < map_data_.height) {
        index = grid_cell.y*map_data_.width + grid_cell.x;
        return true;
    }
    return false;
}
