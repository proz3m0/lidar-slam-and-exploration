/*! @file
 *  @brief Assignment 3 Mainstream
 *  @author {Gia Huy Nguyen + 12542344}
 *  @date {19 September 2019}
*/

#include <ros/ros.h>
#include "BrickSearch.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "brick_search");

  ros::NodeHandle nh{};

  BrickSearch bs(nh);

  // Asynchronous spinner doesn't block
  ros::AsyncSpinner spinner(1);
  spinner.start();

  bs.mainLoop();

  return 0; 
}

