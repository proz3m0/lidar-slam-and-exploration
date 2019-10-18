#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <random>
#include <mutex>
#include <atomic>
#include <math.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

//MESSAGES
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

//OPENCV
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>

//SYNCRHONISER FOR DEPTH AND IMAGE
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//TRANSFOM 2
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>

//POINTCLOUD
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

//MOVEBASE
#include <nav_msgs/GetMap.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

static const uint QUEUE_SIZE = 10;
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;


/*  Intrinsic camera matrix
#     [fx  0 cx]
# K = [ 0 fy cy]
#     [ 0  0  1]
[1206.8897719532354,        0.0       , 960.5]
[         0.0      ,1206.8897719532354, 540.5]
[         0.0      ,        0.0      ,   1.0 ] 
*/

static const double FOCAL_LENGTH_X = 1206.8897719532354;
static const double FOCAL_LENGTH_Y = 1206.8897719532354;
static const double PRINCIPAL_POINT_X = 960.5;
static const double PRINCIPAL_POINT_Y = 540.5;

class BrickSearch 
{
public:

    BrickSearch(ros::NodeHandle nh,ros::NodeHandle nh_private);
    ~BrickSearch();

    void syncCallBack(const sensor_msgs::ImageConstPtr& colorMsg, const sensor_msgs::ImageConstPtr& depthMsg);

    void amclPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped& pose_msg);

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg_ptr);

    void pcCallback(const sensor_msgs::PointCloud2ConstPtr &pc_msg);

    void findRedBlob(const cv_bridge::CvImagePtr& cv_ptr_rgb);
    
    bool fetchTransform(geometry_msgs::TransformStamped &transform, std::string target_frame, std::string source_frame);

    void findXYZ(cv::KeyPoint keypoint);

protected:
    nav_msgs::OccupancyGrid map_{};
    cv::Mat map_image_{};

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    image_transport::Subscriber depth_sub_;
    image_transport::Publisher depth_pub_;
    ros::Publisher pose_pub_;

    //Message filters for synchronisation (https://gist.github.com/tdenewiler/e2172f628e49ab633ef2786207793336)
    message_filters::Subscriber<sensor_msgs::Image> colorMsg_;
    message_filters::Subscriber<sensor_msgs::Image> depthMsg_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;

    // Action client
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_client_{ "move_base", true };

    //TF2 stuff
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
    std::string camera_frame_;
    std::string map_frame_;
    sensor_msgs::PointCloud2ConstPtr pc_msg_;

    //amcl pose buffer
    struct PoseDataBuffer
    {
        geometry_msgs::Pose pose;
        std::mutex mutex_;
    };
    PoseDataBuffer poseBuffer_;

    //HSV image
    cv::Mat imageHsv_;

    std::vector<cv::KeyPoint> keypoints_;

    // for brick detection
    std::atomic<bool> brick_found_{ false };
    cv::Mat test_image_;

    // ros publisher
    ros::Publisher brick_found_pub_;
    image_transport::Publisher test_image_pub_{};

    // ros subscriber
    ros::Subscriber amcl_pose_sub_{};
    ros::Subscriber camera_info_sub_{};
    ros::Subscriber pc_sub_{};

};
