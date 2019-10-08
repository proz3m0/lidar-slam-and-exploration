#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>
#include <thread>
#include <signal.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>

#include "explore_move/RequestMazePath.h"

/** \brief class for exploration*/
class ExploreMoveTest {
private:
    /*********************************************************************
        * ROS VARIABLES
    *********************************************************************/
    /** \brief ros nodehandle */
    ros::NodeHandle nh_;
    /** \brief ros service client to get exploration path */
    ros::ServiceServer req_path_serv_;

public:
    /*********************************************************************
        * CONSTRUCTOR
    *********************************************************************/
    /** \brief CameraOGMap constructor
      * \param nh nodehandle */
    ExploreMoveTest(ros::NodeHandle nh)
            : nh_(nh) {
        // Nodehande for input
        ros::NodeHandle pn("~");

        // Sets up services
        req_path_serv_ = nh_.advertiseService("/request_path", &ExploreMoveTest::reqPathServCallback, this);

    }
    /*********************************************************************
        * CALLBACKS
    *********************************************************************/
    /** \brief callback function for brick found */
    bool reqPathServCallback(explore_move::RequestMazePath::Request& req, explore_move::RequestMazePath::Response& res) {
        geometry_msgs::PoseStamped p;
        p.header.frame_id = "map";
        p.pose.position.x= -1.67850255966;
        p.pose.position.y= -0.908212840557;
        p.pose.position.z= 0.0;
        p.pose.orientation.x= 0.0;
        p.pose.orientation.y= 0.0;
        p.pose.orientation.z= -0.730893937109;
        p.pose.orientation.w= 0.682491064188;
        res.path.push_back(p);
        p.pose.position.x= -0.254936695099;
        p.pose.position.y= -1.35199248791;
        p.pose.position.z= 0.0;
        p.pose.orientation.x= 0.0;
        p.pose.orientation.y= 0.0;
        p.pose.orientation.z= 0.671246292726;
        p.pose.orientation.w= 0.741234385672;
        res.path.push_back(p);
        p.pose.position.x= -1.513256073;
        p.pose.position.y= 1.46023607254;
        p.pose.position.z= 0.0;
        p.pose.orientation.x= 0.0;
        p.pose.orientation.y= 0.0;
        p.pose.orientation.z= 0.670839664255;
        p.pose.orientation.w= 0.741602416974;
        res.path.push_back(p);
        p.pose.position.x= -1.65417218208;
        p.pose.position.y= 1.26307356358;
        p.pose.position.z= 0.0;
        p.pose.orientation.x= 0.0;
        p.pose.orientation.y= 0.0;
        p.pose.orientation.z= -0.750804089056;
        p.pose.orientation.w= 0.660524957785;
        res.path.push_back(p);
        p.pose.position.x= 2.05267047882;
        p.pose.position.y= 0.706128358841;
        p.pose.position.z= 0.0;
        p.pose.orientation.x= 0.0;
        p.pose.orientation.y= 0.0;
        p.pose.orientation.z= 0.622250442247;
        p.pose.orientation.w= 0.782818233771;
        res.path.push_back(p);
        return true;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "explore_move_test");
    ros::NodeHandle nh;
    ExploreMoveTest explore_move_test(nh);
    ros::spin();
    return 0;
}
