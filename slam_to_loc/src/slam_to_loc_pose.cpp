#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>
#include <thread>
#include <signal.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <ros/package.h>

/** \brief class for storing the pose from slam and setting the intial localisation pose*/
class SLAMtoLocPose {
private:
    /*********************************************************************
        * ROS VARIABLES
    *********************************************************************/
    /** \brief ros nodehandle */
    ros::NodeHandle nh_;
    /** \brief ros publisher to publish initial pose for localization */
    ros::Publisher initial_pose_pub_;
    /** \brief ros subscriber to subsribe to localization node */
    ros::Subscriber loc_pose_sub_;
    /** \brief tf2 buffer for transforms */
    tf2_ros::Buffer tf2_buffer_;
    /** \brief tf2 listener for transforms */
    tf2_ros::TransformListener tf2_listener_;
    /** \brief variable for storing SLAM pose */
    geometry_msgs::PoseWithCovarianceStamped pose_;
    /** \brief robots base frame */
    std::string base_frame_;
    /** \brief map frame */
    std::string map_frame_;
    /** \brief save tf rate */
    double save_tf_rate_;
    /** \brief pose comparison max delta */
    double pose_comparison_delta_;
    /*********************************************************************
        * BOOLEANS
    *********************************************************************/
    /** \brief bool for  */
    bool save_tf_;
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
public:
    /*********************************************************************
        * CONSTRUCTOR
    *********************************************************************/
    /** \brief SLAMtoLocPose constructor
      * \param nh nodehandle */
    SLAMtoLocPose(ros::NodeHandle nh)
            : nh_(nh), tf2_listener_(tf2_buffer_), save_tf_(true){
        // Nodehande for input
        ros::NodeHandle pn("~");

        // Sets up frames
        pn.param<std::string>("base_frame", base_frame_, "base_link");
        pn.param<std::string>("map_frame", map_frame_, "map");

        // Sets up rate and pose delta
        pn.param<double>("save_tf_rate", save_tf_rate_, 2);
        pn.param<double>("pose_comparison_delta", pose_comparison_delta_, 0.2);

        // Sets up covariance
        double cov_xx,cov_yy, cov_aa;
        pn.param<double>("cov_xx", cov_xx, 0.2);
        pn.param<double>("cov_yy", cov_yy, 0.2);
        pn.param<double>("cov_aa", cov_aa, M_PI/12);
        pose_.pose.covariance[0] = cov_xx*cov_xx;
        pose_.pose.covariance[7] = cov_yy*cov_yy;
        pose_.pose.covariance[35] = cov_aa*cov_aa;

        // Sets up subscribers, publishers and services
        std::string loc_pose_topic, loc_initial_pose_topic, save_tf_trigger_topic;
        pn.param<std::string>("loc_pose_topic", loc_pose_topic, "/amcl_pose");
        pn.param<std::string>("loc_initial_pose_topic", loc_initial_pose_topic, "/initialpose");
        pn.param<std::string>("save_tf_trigger_topic", save_tf_trigger_topic, "/slam_to_loc_saveoff");
        // Subscribes to topics
        loc_pose_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(loc_pose_topic, 1, &SLAMtoLocPose::poseCallback, this);
        // Advertises publishers
        initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(loc_initial_pose_topic, 100);

    }
    /*********************************************************************
        * CALLBACKS
    *********************************************************************/
    /** \brief callback function for localisation pose */
    void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg) {
        ROS_INFO("Slam to Loc Pose: poseCallback: pose recieved");
        if(!save_tf_){
            // compares pose_msg with saved pose_ and if the same will
            if(fabs(pose_msg->pose.pose.position.x-pose_.pose.pose.position.x) < pose_comparison_delta_ &&
               fabs(pose_msg->pose.pose.position.y-pose_.pose.pose.position.y) < pose_comparison_delta_ &&
               fabs(pose_msg->pose.pose.position.z-pose_.pose.pose.position.z) < pose_comparison_delta_ &&
               fabs(pose_msg->pose.pose.orientation.w-pose_.pose.pose.orientation.w) < pose_comparison_delta_ &&
               fabs(pose_msg->pose.pose.orientation.x-pose_.pose.pose.orientation.x) < pose_comparison_delta_ &&
               fabs(pose_msg->pose.pose.orientation.y-pose_.pose.pose.orientation.y) < pose_comparison_delta_ &&
               fabs(pose_msg->pose.pose.orientation.z-pose_.pose.pose.orientation.z) < pose_comparison_delta_ ){
                ROS_INFO("Slam to Loc Pose: Pose Callback: Match Initial Pose set, shutting down node");
                ros::shutdown();
            }
        }

    }
    /*********************************************************************
        * THREADS
    *********************************************************************/
    /** \brief thread that saves the map to base_frame tf */
    void saveTFThread(){
        ros::Rate rate_limiter(save_tf_rate_);
        ros::Duration(1.0).sleep();
        while(ros::ok()) {
            if(save_tf_) {
                tf2::Transform tf;
                if (!fetchTransform(tf, map_frame_, base_frame_)) {
                    ROS_WARN("Slam to Loc Pose: saveTFThread: fetchTransform failed");
                } else {
                    tf2::Matrix3x3 rot(tf.getRotation());
                    double y,p,r;
                    rot.getEulerYPR(y,p,r);
                    ROS_INFO("Slam to Loc Pose: saveTFThread: saved pose [x:%f\ty:%f\tyaw:%f]",
                            tf.getOrigin().getX(), tf.getOrigin().getY(), y*180/M_PI);
                    tf2::toMsg(tf, pose_.pose.pose);
                    save_tf_ = false;
                }
            }
            initial_pose_pub_.publish(pose_);
            rate_limiter.sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "structure_pose_est_node");
    ros::NodeHandle nh;
    SLAMtoLocPose slam_to_loc_pose(nh);
    std::thread tfThread(&SLAMtoLocPose::saveTFThread, std::ref(slam_to_loc_pose));
    ros::spin();
    return 0;
}
