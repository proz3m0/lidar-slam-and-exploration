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
#include <std_srvs/Trigger.h>
#include <angles/angles.h>
#include <actionlib_msgs/GoalID.h>

#include "maze_planner/PlanPath.h"
#include "camera_ogmap/FreeSpace.h"

/** \brief class for exploration*/
class ExploreMove {
private:
    /*********************************************************************
        * ROS VARIABLES
    *********************************************************************/
    /** \brief ros nodehandle */
    ros::NodeHandle nh_;
    /** \brief ros publisher to publish move_base poses */
    ros::Publisher pose_pub_;
    /** \brief ros publisher to publish cancel pose msg */
    ros::Publisher cancel_pose_pub_;
    /** \brief ros service client to get exploration path */
    ros::ServiceClient req_path_client_;
    /** \brief ros service client to get exploration path */
    ros::ServiceClient free_space_client_;
    /** \brief ros service for exploration start */
    ros::Subscriber start_explore_sub_;
    /** \brief ros subscriber to check whether brick has been found*/
    ros::Subscriber brick_found_sub_;
    /** \brief tf2 buffer for transforms */
    tf2_ros::Buffer tf2_buffer_;
    /** \brief tf2 listener for transforms */
    tf2_ros::TransformListener tf2_listener_;
    /** \brief robots camera frame */
    std::string robot_frame_;
    /** \brief map frame */
    std::string map_frame_;
    /*********************************************************************
        * EXPLORE PATH VARIABLES
    *********************************************************************/
    /** \brief path containing exploration way points */
    std::vector<geometry_msgs::PoseStamped> path_;
    /** \brief path containing exploration way points in transform form */
    std::vector<tf2::Transform> path_tf_;
    /** \brief count of how many steps have been taken on path */
    int path_count_;
    /** \brief rate for path checking */
    double path_rate_;
    /** \brief x value for comparison, if x difference is less it can move to next step */
    double comp_x_;
    /** \brief y value for comparison, if y difference is less it can move to next step */
    double comp_y_;
    /** \brief yaw value for comparison, if yaw difference is less it can move to next step */
    double comp_yaw_;
    /*********************************************************************
        * BOOLEANS
    *********************************************************************/
    /** \brief sets true when brick has been found */
    bool brick_found_;
    /** \brief bool to start exploration*/
    bool start_;
    /*********************************************************************
        * PRIVATE FUNCTIONS
    *********************************************************************/
    /** \brief takes in pose and transforms, returns whether successful
      * \param transform pose to transform
      * \param target_frame target frame for transformation
      * \param source_frame source frame for transformation */
    bool fetchTransform(tf2::Transform &transform, std::string target_frame, std::string source_frame) {
        geometry_msgs::TransformStamped local_transformStamped;
        tf2::Stamped<tf2::Transform> tf_stamped;
        try {
            local_transformStamped = tf2_buffer_.lookupTransform(target_frame, source_frame,ros::Time(0));
            tf2::fromMsg(local_transformStamped, tf_stamped);
            transform.setRotation(tf_stamped.getRotation());
            transform.setOrigin(tf_stamped.getOrigin());
            return true;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return false;
        }
    }
    /** \brief takes in 2 poses and compares whether they are close together
      * \param path_pose
      * \param robot_pose*/
    void setPathTF() {
        for(auto&& pose:path_) {
            pose.header.frame_id = map_frame_;
            tf2::Transform tf;
            tf2::fromMsg(pose.pose, tf);
            path_tf_.push_back(tf);
        }
    }
    /** \brief takes in 2 poses and compares whether they are close together
      * \param path_pose
      * \param robot_pose*/
    bool poseCompare(tf2::Transform path_pose, tf2::Transform robot_pose) {
        // Gets yaw from both poses
        double path_y, robot_y, p,r;
        tf2::Matrix3x3 path_rot(path_pose.getRotation());
        path_rot.getEulerYPR(path_y,p,r);
        tf2::Matrix3x3 robot_rot(robot_pose.getRotation());
        robot_rot.getEulerYPR(robot_y,p,r);

        // Compares poses and returns true if x y and yaw are within limits
        return  fabs(path_pose.getOrigin().getX() - robot_pose.getOrigin().getX()) < comp_x_ &&
                fabs(path_pose.getOrigin().getY() - robot_pose.getOrigin().getY()) < comp_y_ &&
                fabs(angles::normalize_angle(path_y - robot_y)) < comp_yaw_;
    }
public:
    /*********************************************************************
        * CONSTRUCTOR
    *********************************************************************/
    /** \brief CameraOGMap constructor
      * \param nh nodehandle */
    ExploreMove(ros::NodeHandle nh)
            : nh_(nh), tf2_listener_(tf2_buffer_), brick_found_(false), start_(false), path_count_(0) {
        // Nodehande for input
        ros::NodeHandle pn("~");

        // Sets up frames
        pn.param<std::string>("robot_frame", robot_frame_, "base_footprint");
        pn.param<std::string>("map_frame", map_frame_, "map");

        // Sets up subscribers, publishers and services
        std::string pose_topic, req_path_topic, brick_found_topic, start_explore_topic, free_space_topic, cancel_pose_topic;
        pn.param<std::string>("move_base_pose_topic", pose_topic, "/move_base_simple/goal");
        pn.param<std::string>("req_path_topic", req_path_topic, "/request_path");
        pn.param<std::string>("brick_found_topic", brick_found_topic, "/brick_found");
        pn.param<std::string>("start_explore_topic", start_explore_topic, "/start_explore");
        pn.param<std::string>("free_space_topic", free_space_topic, "/free_space");
        pn.param<std::string>("cancel_pose_topic", cancel_pose_topic, "/move_base/cancel");
        // Advertises publishers
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic, 10);
        cancel_pose_pub_ = nh_.advertise<actionlib_msgs::GoalID>(cancel_pose_topic, 10);
        // Sets up services
        req_path_client_ = nh_.serviceClient<maze_planner::PlanPath>(req_path_topic);
        free_space_client_ = nh_.serviceClient<camera_ogmap::FreeSpace>(free_space_topic);
        // Sets up subscribers
        brick_found_sub_ = nh_.subscribe<std_msgs::Bool>(brick_found_topic, 10, &ExploreMove::brickFoundCallback, this);
        start_explore_sub_ = nh_.subscribe<std_msgs::Bool>(start_explore_topic, 10, &ExploreMove::startExploreCallback, this);

        // Sets up path rate
        pn.param<double>("path_rate", path_rate_, 2.0);
        pn.param<double>("comp_x", comp_x_, 0.1);
        pn.param<double>("comp_y", comp_y_, 0.1);
        pn.param<double>("comp_yaw", comp_yaw_, 30*M_PI/180);

    }
    /*********************************************************************
        * CALLBACKS
    *********************************************************************/
    /** \brief callback function for brick found */
    void brickFoundCallback(const std_msgs::BoolConstPtr &brick_found_msg) {
        brick_found_ = brick_found_msg->data;
    }
    /** \brief callback function for brick found */
    void startExploreCallback(const std_msgs::BoolConstPtr &start_msg) {
        if(start_msg->data) start_ = true;
    }
    /*********************************************************************
        * THREADS
    *********************************************************************/
    /** \brief thread that saves the map to base_frame tf */
    void pathFollowThread(){

        // busy wait before starting
        while(!start_)ros::Duration(0.5).sleep();

        // Delay to allow for initialisation
        ros::Duration(0.5).sleep();

        // Fetchs map to robot transform to find start pose
        tf2::Transform tf;
        bool transform_success = false;
        do {
            if(!fetchTransform(tf, map_frame_, robot_frame_)) {
                transform_success = false;
                ROS_WARN("Explore Move: pathFollowThread: TF failed");
            }
            else transform_success = true;
        } while(!transform_success);

        // Sets up path service
        maze_planner::PlanPath req_path_serv;
        req_path_serv.request.robot_pose.header.frame_id = map_frame_;
        tf2::toMsg(tf, req_path_serv.request.robot_pose.pose);
        ROS_INFO("Explore Move: pathFollowThread: Start Pose [x:%f\ty:%f", tf.getOrigin().getX(), tf.getOrigin().getY());

        // Calls service
        if (req_path_client_.call(req_path_serv)) {
            ROS_INFO("Explore Move: pathFollowThread: Service Called and Path Recieved, size: %d", req_path_serv.response.path.size());
            path_ = req_path_serv.response.path;
            setPathTF();
        }
        else {
            ROS_ERROR("Explore Move: pathFollowThread: Service Called and Failed");
            return;
        }

        // Sets first move_base pose
        pose_pub_.publish(path_[path_count_]);

        ros::Rate rate_limiter(path_rate_);
        while(ros::ok() && !brick_found_) {
            // Gets current robot transform and if close to target sends new goal
            if (!fetchTransform(tf, map_frame_, robot_frame_)) {
                ROS_WARN("Explore Move: saveTFThread: fetchTransform failed");
            } else {
                if(poseCompare(path_tf_[path_count_], tf)) {
                    path_count_ ++;
                    if(path_count_ < path_.size()){
                        ROS_INFO("Explore Move: new goal pose, count: %d", path_count_);
                        pose_pub_.publish(path_[path_count_]);
                    }
                    else break;
                }
                else {
                    camera_ogmap::FreeSpace free_space_serv;
                    free_space_serv.request.pose = path_[path_count_];
                    if (free_space_client_.call(free_space_serv)) {
                        if (free_space_serv.response.free_space) {
                            path_count_++;
                            if (path_count_ < path_.size())pose_pub_.publish(path_[path_count_]);
                            else break;
                        }
                    }
                }
            }

            rate_limiter.sleep();
        }
        // cancels goal
        cancel_pose_pub_.publish(actionlib_msgs::GoalID{});
        ROS_INFO("Explore Move: pathFollowThread: Brick Found or Exploration Complete");
        // if brick is not found use frontier exploration to explore the rest of the maze
        if(!brick_found_) {

        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "explore_move");
    ros::NodeHandle nh;
    ExploreMove explore_move(nh);
    std::thread tfThread(&ExploreMove::pathFollowThread, std::ref(explore_move));
    ros::spin();
    return 0;
}
