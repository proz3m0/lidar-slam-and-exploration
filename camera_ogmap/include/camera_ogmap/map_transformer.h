#ifndef MAPTRANSFORMER_H
#define MAPTRANSFORMER_H

#include <utility>
#include <vector>
#include <random>
#include <chrono>
#include <iostream>
#include <ros/package.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>

struct GridCell {
    int x,y;
};

/** \brief A class that transforms from map to grid and grid to map */
class MapTransformer
{
private:
    /** \brief map data */
    nav_msgs::MapMetaData map_data_;
public:
    /*********************************************************************
        * CONSTRUCTORS
    *********************************************************************/
    /** \brief Default Constructor */
    MapTransformer();
    /** \brief Constructor that initializes all variables */
    MapTransformer(nav_msgs::MapMetaData map_data);
    /*********************************************************************
        * METHODS
    *********************************************************************/
    /** \brief returns whether conversion was successful
      * \param grid_cell grid cell*/
    bool gridCellToIndex(GridCell &grid_cell, int &index);
    /** \brief returns whether conversion was successful
      * \param grid_cell grid cell*/
    bool gridCellToMapPoint(GridCell &grid_cell, geometry_msgs::Pose2D &map_point);
    /** \brief returns whether conversion was successful
      * \param map_points point in map coordinates*/
    bool mapPointToGridCell(geometry_msgs::Pose2D &map_point, GridCell &grid_cell);
    /** \brief returns a vector of vector of grid cells
      * \param map_points points in map coordintates */
    std::vector<std::vector<GridCell>> mapPointToGridCells(std::vector<std::vector<geometry_msgs::Pose2D>> map_points);
};


#endif //MAPTRANSFORMER_H
