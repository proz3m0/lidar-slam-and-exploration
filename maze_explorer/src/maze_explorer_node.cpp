#include "maze_explorer_node.h"

MazeExplorer::MazeExplorer(ros::NodeHandle nh):
nh_(nh)
{
    // normalize pgm 0 - 4096 range to 0 - 255
    cv::Mat img = cv::imread("map_real.pgm");
    img.convertTo(img, CV_8U, 255.0 / 4096.0);

    cv::normalize(img, img, 0, 255, cv::NORM_MINMAX);

    map_ = img;
    robotStartingPose_.row = 212;  //row
    robotStartingPose_.col = 154;  //col

    checkedMapMetadata_ = false;

    // advertisers and publishers
    occupMapSub_ = nh_.subscribe("/map",500, &MazeExplorer::occupMapCB, this);

    // create a service server
    exploreServer_ = nh_.advertiseService("/get_explore_path", &MazeExplorer::exploreMazeAlgorithm, this);
};

//======================== ROS ========================//

void MazeExplorer::occupMapCB(const nav_msgs::OccupancyGrid::ConstPtr &data)
{
    occupMap_.mutex.lock();

    if(checkedMapMetadata_ == false)
    {
        mapMetadata_.mutex.lock();
        mapMetadata_.data = data->info;
        mapMetadata_.mutex.unlock();
        checkedMapMetadata_ = true;
    }

    occupMap_.data = *data;
    occupMap_.mutex.unlock();
};

bool MazeExplorer::exploreMazeAlgorithm(maze_explorer::RequestMazePath::Request  &req,
                                        maze_explorer::RequestMazePath::Response &res)
{
    convertOccupancyGrid();

    robotStartingPose_ = globalToLocal(req.robot_pose.pose);

    dilateOccupiedSpace(8);
    findIntersection();
    //showMap();
    arrangeQueue();
    //showQueuedPoint();

    res.path = getIntersectionPose();

    return true;
};

//=====================================================//

void MazeExplorer::dilateOccupiedSpace(int amount)
{
    cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(amount,amount));

    cv::erode(map_, map_, erodeElement);
};

void MazeExplorer::showMap()
{
    cv::imshow("map", map_);
    cv:cvWaitKey(0);
};

void MazeExplorer::findIntersection()
{
    for(int row = 0; row < map_.rows; row++)
    {
        for(int col = 0; col < map_.cols; col++)
        {
            matPoint current_point;
            current_point.row = row;
            current_point.col = col;
            if(map_.at<cv::Vec3b>(row,col) == white && checkIntersectionSqaure(current_point) == false)
            {
                //sequence is up down left right
                std::vector<bool> result;

                result = vhTrace(18, row, col);
                if(isIntersection(result) == true){
                    map_.at<cv::Vec3b>(row,col) = green;
                    addIntersectionSquare(18,current_point);
                    intersectionpoint_.push_back(current_point);
                }

            }
        }
    }
};

std::vector<bool> MazeExplorer::vhTrace(int traceAmount, int row, int col)
{
    std::vector<bool> traceResult;
    int currentRow, currentCol;
    //sequence is up down left right
    for(int sides = 0; sides < 4; sides++)
    {
        currentCol = col;
        currentRow = row;
        bool clear = true;
        for(int trace = 1; trace <= traceAmount; trace++)
        {
            if(sides == 0) currentCol-=1;
            else if(sides == 1) currentCol+=1;
            else if(sides == 2) currentRow-=1;
            else if(sides == 3) currentRow+=1;

            //map_.at<cv::Vec3b>(currentPoint) = green;
            if(map_.at<cv::Vec3b>(currentRow, currentCol) != white){
                clear = false;
                break;
            }
            
        }
        if(clear == true) traceResult.push_back(true);
        else traceResult.push_back(false);
    }

    return traceResult;
};

bool MazeExplorer::isIntersection(std::vector<bool> input)
{
    // 4 side intersection
    if(input[0] == true && input[1] == true && input[2] == true && input[3] == true) return true;
    // up and left
    else if(input[0] == true && input[2] == true) return true;
    // up and right
    else if(input[0] == true && input[3] == true) return true;
    //down and left
    else if(input[1] == true && input[2] == true) return true;
    //down and right
    else if(input[1] == true && input[3] == true) return true;
    //no intersection
    else return false;
};

void MazeExplorer::addIntersectionSquare(int size, matPoint intersectionPoint)
{
    // a,b,c (ab = top left corner), c = width and height (square)
    int topleft_row = intersectionPoint.row-size;
    int topleft_col = intersectionPoint.col-size;

    IntersectionData intersection;
    intersection.topleftrow = topleft_row;
    intersection.topleftcol = topleft_col;
    intersection.size = size*2;

    blockedintersection_.push_back(intersection);
};

bool MazeExplorer::checkIntersectionSqaure(matPoint point)
{
    bool check = false;
    if( blockedintersection_.size() != 0)
    {
        for(auto id : blockedintersection_)
        {
            // check row
            if(point.row >= id.topleftrow && point.row <= id.topleftrow+id.size){
                // check col
                if(point.col >= id.topleftcol && point.col <= id.topleftcol+id.size){
                    check = true;
                    break;
                }
            }
        }
    }

    return check;
};  

void MazeExplorer::arrangeQueue()
{

    std::vector<matPoint> explored;
    std::deque<matPoint> queue;
    queue.push_back(robotStartingPose_);
    matPoint current_point;
    std::vector<matPoint> neighbour, actual_neighbour;

    //graph search // to check if something exist std::find(v.begin(), v.end(), elem)
    while(!queue.empty())
    {
        current_point = queue.front();
        queue.pop_front();

        //check if current point exist in intersectionpoint_    
        std::vector<matPoint>::iterator flag = std::find(intersectionpoint_.begin(), intersectionpoint_.end(), current_point);
        if (flag != intersectionpoint_.end ()){
            queuedIntersectionpoint_.push_back(current_point);
        }
        /*
        if( std::find(intersectionpoint_.begin(), intersectionpoint_.end(), current_point) != intersectionpoint_.end() ){
            queuedIntersectionpoint_.push_back(current_point);
        }
        */

        //put into explored
        explored.push_back(current_point);

        //get neighbour
        neighbour = getNeighbour(current_point);

        //get unexplored neighbour
        for(auto cell : neighbour){
            if( std::find(explored.begin(), explored.end(), cell) == explored.end()){
                actual_neighbour.push_back(cell);
            }
        }

        //put into queue
        for(auto cell : actual_neighbour) {
            queue.push_back(cell);
            explored.push_back(cell);
        }

        actual_neighbour.clear();
        neighbour.clear();
    }

    std::cout << queuedIntersectionpoint_.size() << std::endl;
};

std::vector<matPoint> MazeExplorer::getNeighbour(matPoint point)
{
    //4 connected
    std::vector<matPoint> neighbour;    
    int row, col;
    cv::Vec3b val;

    for(int sides = 0; sides < 4; sides++)
    {
        row = point.row;
        col = point.col;

        if(sides == 0) col-=1;
        else if(sides == 1) col+=1;
        else if(sides == 2) row-=1;
        else if(sides == 3) row+=1;

        if(map_.at<cv::Vec3b>(row, col) == white || map_.at<cv::Vec3b>(row, col) == green){
            matPoint pt;
            pt.row = row;
            pt.col = col;

            neighbour.push_back(pt);
        }
    }

    return neighbour;
};

void MazeExplorer::showQueuedPoint()
{
    for(auto cell : queuedIntersectionpoint_){
        cv::Point pt(cell.col,cell.row);
        cv::circle(map_,pt, 5, CV_RGB(0,255,0), 1);
        imshow("map",map_);
        cv::waitKey(0);
    }
};  

geometry_msgs::Pose MazeExplorer::localToGlobal(matPoint &local)
{
    mapMetadata_.mutex.lock();
    
    int px = mapMetadata_.data.width - local.row;
    int py = mapMetadata_.data.height - local.col;

    geometry_msgs::Pose pose;
    pose.position.x = mapMetadata_.data.origin.position.x + (px * mapMetadata_.data.resolution);
    pose.position.y = mapMetadata_.data.origin.position.y + (py * mapMetadata_.data.resolution);
    pose.orientation.w = 1;

    mapMetadata_.mutex.unlock();

    return pose;
};

matPoint MazeExplorer::globalToLocal(geometry_msgs::Pose &pose)
{
    mapMetadata_.mutex.lock();

    double deltax = (pose.position.x - mapMetadata_.data.origin.position.x) / mapMetadata_.data.resolution;
    double deltay = (pose.position.y - mapMetadata_.data.origin.position.y) / mapMetadata_.data.resolution;

    matPoint local;
    local.row = mapMetadata_.data.width - (int)deltax;
    local.col = mapMetadata_.data.height - (int)deltay;

    mapMetadata_.mutex.unlock();

    return local;
};

void MazeExplorer::convertOccupancyGrid()
{
    int width, height, px, py;
    double resolution;
    
    nav_msgs::OccupancyGrid ogmap;

    occupMap_.mutex.lock();
    ogmap = occupMap_.data;
    occupMap_.mutex.unlock();

    cv::Mat cvmap(ogmap.info.width, ogmap.info.height, CV_8UC3, cv::Scalar(127, 127, 127));

    for(int x = 0; x < ogmap.info.width; x++){
        for(int y = 0; x < ogmap.info.height; y++){

            // opencv local coordinates
            px = ogmap.info.width - x;
            py = ogmap.info.height - y;

            int val = ogmap.data[x+ ogmap.info.width * y];

            if(val < OCCUPIED_THRES) cvmap.at<cv::Vec3b>(px,py) = white;
            else if(val >= OCCUPIED_THRES) cvmap.at<cv::Vec3b>(px,py) = black;
            else cvmap.at<cv::Vec3b>(px,py) = grey;
        }
    }
    
    map_ = cvmap;
};

std::vector<geometry_msgs::PoseStamped> MazeExplorer::getIntersectionPose()
{
    std::vector<geometry_msgs::PoseStamped> queuedIntersectionPose;

    for(auto point : queuedIntersectionpoint_){
        
        geometry_msgs::PoseStamped posestamp;
        posestamp.pose = localToGlobal(point);
        queuedIntersectionPose.push_back(posestamp);
    }

    return queuedIntersectionPose;
};