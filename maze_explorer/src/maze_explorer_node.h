#include <iostream>

#include <tuple>
#include <vector>
#include <queue>
#include <deque>
#include <algorithm>
#include <mutex>

#include "ros/ros.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include "maze_explorer/RequestMazePath.h"

const  cv::Vec3b white(255,255,255),
                 red(0, 0, 255),
                 blue(255, 0 , 0),
                 black(0, 0, 0),
                 grey(127, 127, 127),
                 green(0, 255, 0);

const int OCCUPIED_THRES = 65;

// row, col
struct matPoint
{
    int row;
    int col; 

    bool operator==(const matPoint &lhs)
    {
        return (lhs.row == row) && (lhs.col == col);
    };
};
// a,b,c (ab = top left corner), c = width and height (square)
struct IntersectionData
{
    int topleftrow;
    int topleftcol;
    int size;
};

class MazeExplorer
{
public:

    MazeExplorer(ros::NodeHandle nh);
    
    void dilateOccupiedSpace(int amount);

    void showMap();
    
    void findIntersection();

    void findDeadend();

    void arrangeQueue();

    void showQueuedPoint();

    geometry_msgs::Pose localToGlobal(matPoint &local);

    matPoint globalToLocal(geometry_msgs::Pose &pose);

    void convertOccupancyGrid();

    void inflatePoint(double size);

    std::vector<geometry_msgs::PoseStamped> getIntersectionPose(); 

protected:

    //======================== ROS ========================//

    ros::NodeHandle nh_;
    ros::Subscriber occupMapSub_;
    ros::ServiceServer exploreServer_;

    void occupMapCB(const nav_msgs::OccupancyGrid::ConstPtr &data);
    bool exploreMazeAlgorithm(maze_explorer::RequestMazePath::Request  &req,
                              maze_explorer::RequestMazePath::Response &res);

    bool checkedMapMetadata_;

    struct MapMetadataStruct
    {
        nav_msgs::MapMetaData data;
        std::mutex mutex;
    };
    MapMetadataStruct mapMetadata_;

    struct OccupMapStruct
    {
        nav_msgs::OccupancyGrid data;
        std::mutex mutex;
    };
    OccupMapStruct occupMap_;


    //======================== Functions ========================//
    //vertical and horizontal trace
    std::vector<bool> vhTrace(int traceAmount, int row, int col);

    bool isIntersection(std::vector<bool> input);
    
    bool isDeadend(std::vector<bool> input);

    void addIntersectionSquare(int size, matPoint intersectionPoint);

    void addDeadendSquare(int size, matPoint deadendPoint);

    bool checkIntersectionSqaure(matPoint point);

    bool checkDeadendSqaure(matPoint point);

    std::vector<matPoint> getNeighbour(matPoint point);

    //======================== Variables ========================//
    cv::Mat map_;
    matPoint robotStartingPose_;
    
    std::vector<matPoint> intersectionpoint_;

    std::vector<matPoint> deadendpoint_;

    std::vector<IntersectionData> blockedintersection_;

    std::vector<IntersectionData> blockeddeadend_;
        
    std::vector<matPoint> queuedIntersectionpoint_;

};


