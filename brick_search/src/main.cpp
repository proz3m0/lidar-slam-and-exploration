#include "ros/ros.h"
#include "brick_search.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "brick_search");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private;
    
    BrickSearch bs(nh, nh_private);
    
    ros::spin();

    return 0;
}