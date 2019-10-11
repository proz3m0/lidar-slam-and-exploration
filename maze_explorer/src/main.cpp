#include "ros/ros.h"
#include "maze_explorer_node.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_processing");

    ros::NodeHandle nh;
    
    MazeExplorer mp(nh);
    
    // steps for executing the algorithm
    mp.dilateOccupiedSpace(10);
    mp.findIntersection();
    mp.showMap();
    mp.arrangeQueue();
    mp.showQueuedPoint();
    
    ros::spin();

    return 0;
}