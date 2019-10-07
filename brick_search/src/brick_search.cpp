#include <atomic>
#include <cmath>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

namespace
{
double wrapAngle(double angle)
{
  // Function to wrap an angle between 0 and 2*Pi
  while (angle < 0.)
  {
    angle += 2 * M_PI;
  }

  while (angle > (2 * M_PI))
  {
    angle -= 2 * M_PI;
  }

  return angle;
}

geometry_msgs::Pose pose2dToPose(const geometry_msgs::Pose2D& pose_2d)
{
  geometry_msgs::Pose pose{};

  pose.position.x = pose_2d.x;
  pose.position.y = pose_2d.y;

  pose.orientation.w = std::cos(pose_2d.theta);
  pose.orientation.z = std::sin(pose_2d.theta / 2.);

  return pose;
}
}  // namespace

namespace brick_search
{
class BrickSearch
{
public:
  // Constructor
  explicit BrickSearch(ros::NodeHandle& nh);

  // Publich methods
  void mainLoop();

private:
  // Variables
  nav_msgs::OccupancyGrid map_{};
  cv::Mat map_image_{};
  std::atomic<bool> localised_{ false };
  std::atomic<bool> brick_found_{ false };
  std::vector<cv::KeyPoint> keypoints_;
  cv::Point brick_location_;

  // Transform listener
  tf2_ros::Buffer transform_buffer_{};
  tf2_ros::TransformListener transform_listener_{ transform_buffer_ };

  // Subscribe to the AMCL pose to get covariance
  ros::Subscriber amcl_pose_sub_{};

  // Velocity command publisher
  ros::Publisher cmd_vel_pub_{};

  // Image transport and subscriber
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_{};
  image_transport::Subscriber depth_image_sub_{};
  image_transport::Publisher test_image_pub_{};

  // Image buffer
  struct ImageDataBuffer
  {
    int image_msg_count_ = 0;
    std::deque<cv::Mat> imageDeq_;
    std::mutex buffer_mutex_;
  };
  cv::Mat image_;
  cv::Mat depth_image_;
  cv::Mat test_image_;
  ImageDataBuffer imageBuffer;
  ImageDataBuffer depthImageBuffer;

  // Action client
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_client_{ "move_base", true };

  // Private methods
  geometry_msgs::Pose2D getPose2d();
  void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_msg);
  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg_ptr);
  void depthImageCallback(const sensor_msgs::ImageConstPtr& image_msg_ptr);
  void brickIGotYouInMySight(void);
  void brickWhereAreYou(void);
};

// Constructor
BrickSearch::BrickSearch(ros::NodeHandle& nh) : it_{ nh }
{
  // Wait for "static_map" service to be available
  ROS_INFO("Waiting for \"static_map\" service...");
  ros::service::waitForService("static_map");

  // Get the map
  nav_msgs::GetMap get_map{};

  if (!ros::service::call("static_map", get_map))
  {
    ROS_ERROR("Unable to get map");
    ros::shutdown();
  }
  else
  {
    map_ = get_map.response.map;
    ROS_INFO("Map received");
  }

  // This allows you to access the map data as an OpenCV image
  map_image_ = cv::Mat(map_.info.height, map_.info.width, CV_8U, &map_.data.front());

  // Wait for the transform to be become available
  ROS_INFO("Waiting for transform from \"map\" to \"base_link\"");
  while (ros::ok() && !transform_buffer_.canTransform("map", "base_link", ros::Time(0.)))
  {
    ros::Duration(0.1).sleep();
  }
  ROS_INFO("Transform available");

  // Subscribe to "amcl_pose" to get pose covariance
  amcl_pose_sub_ = nh.subscribe("amcl_pose", 1, &BrickSearch::amclPoseCallback, this);

  // Subscribe to the camera
  image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &BrickSearch::imageCallback, this);

  // Subscribe to the camera
  depth_image_sub_ = it_.subscribe("/camera/depth/image_raw", 1, &BrickSearch::depthImageCallback, this);

  // Advertise "cmd_vel" publisher to control TurtleBot manually
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, false);

  // Publishing an image to "/map_image/fbe" topic
  test_image_pub_ = it_.advertise("/map_image/test", 1);

  // Action client for "move_base"
  ROS_INFO("Waiting for \"move_base\" action...");
  move_base_action_client_.waitForServer();
  ROS_INFO("\"move_base\" action available");

  // Reinitialise AMCL
  ros::ServiceClient global_localization_service_client = nh.serviceClient<std_srvs::Empty>("global_localization");
  std_srvs::Empty srv{};
  global_localization_service_client.call(srv);
}

geometry_msgs::Pose2D BrickSearch::getPose2d()
{
  // Lookup latest transform
  geometry_msgs::TransformStamped transform_stamped =
  transform_buffer_.lookupTransform("map", "base_link", ros::Time(0.), ros::Duration(0.2));

  // Return a Pose2D message
  geometry_msgs::Pose2D pose{};
  pose.x = transform_stamped.transform.translation.x;
  pose.y = transform_stamped.transform.translation.y;

  double qw = transform_stamped.transform.rotation.w;
  double qz = transform_stamped.transform.rotation.z;

  pose.theta = qz >= 0. ? wrapAngle(2. * std::acos(qw)) : wrapAngle(-2. * std::acos(qw));

  return pose;
}

void BrickSearch::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_msg)
{
  // Check the covariance
  double frobenius_norm = 0.;

  for (const auto e : pose_msg.pose.covariance)
  {
    frobenius_norm += std::pow(e, 2.);
  }

  frobenius_norm = std::sqrt(frobenius_norm);

  if (frobenius_norm < 0.05)
  {
    localised_ = true;

    // Unsubscribe from "amcl_pose" because we should only need to localise once at start up
    amcl_pose_sub_.shutdown();
  }
}

void BrickSearch::brickIGotYouInMySight(void)
{
  // Variables
  cv::Mat hsv,mask1,mask2;

  // Convert that frame to seen color
  cv::cvtColor(image_,image_,cv::COLOR_BGR2RGB);

  // Convert that frame from BGR to HSV
  cv::cvtColor(image_, hsv, cv::COLOR_BGR2HSV);
    
  // Creating masks to detect the upper and lower red color.
  cv::inRange(hsv,cv:: Scalar(0, 120, 70),cv::Scalar(10, 255, 255), mask1);
  cv::inRange(hsv,cv::Scalar(170, 120, 70),cv::Scalar(180, 255, 255), mask2);
    
  // Generate the final mask
  mask1 = mask1 + mask2;
  cv::bitwise_not(mask1,mask1);
  cv::copyMakeBorder(mask1,mask1,1,1,1,1,cv::BORDER_CONSTANT,255);

  // Setup SimpleBlobDetector parameters.
  cv::SimpleBlobDetector::Params params;

  // Change thresholds
  params.minThreshold = 10;
  params.thresholdStep = 10;
  params.maxThreshold = 220;

  // Filter by Color
  params.filterByColor = true;
  params.blobColor = 0;

  // Filter by Area.
  params.filterByArea = true;
  params.minArea = 1000;
  params.maxArea = 2076601;

  // Filter by Circularity
  params.filterByCircularity = false;

  // Filter by Convexity
  params.filterByConvexity = false;

  // Filter by Inertia
  params.filterByInertia = false;

  // Set up the detector with parameters
  cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    
  // Detect Blobs
  detector->detect(mask1,keypoints_);
  ROS_INFO_STREAM("Size of keypoints: " << keypoints_.size());
    
  // If keypoints is filled then brick is found
  if (keypoints_.empty() == true) brick_found_ = false;
  else brick_found_ = true;

  // Published the blob image on rqt_image_view
  cv::drawKeypoints(mask1,keypoints_,test_image_, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", test_image_).toImageMsg();
  test_image_pub_.publish(msg);
}

void BrickSearch::brickWhereAreYou(void)
{
  // Find biggest Blob
  int id = 0, id_max;
  float max = 0;
  for (auto& i:keypoints_)
  {
    if (max < i.size)
    {
      max = i.size;
      id_max = id;
    }
    id++;
  }

  // Define Blob centre in pixel
  cv::Point2f blob_centre = keypoints_.at(id_max).pt;

  // *FIND_THE_POSITION_OF_BRICK_IN_3D*

  depthImageBuffer.buffer_mutex_.lock();
  if(imageBuffer.imageDeq_.size()>2)
  {
    depth_image_ = depthImageBuffer.imageDeq_.front();
    depthImageBuffer.imageDeq_.pop_front();
  }
  depthImageBuffer.buffer_mutex_.unlock();
}

void BrickSearch::depthImageCallback(const sensor_msgs::ImageConstPtr& image_msg_ptr)
{
  // Use this method to identify when the brick is visible
  // The camera publishes at 30 fps, it's probably a good idea to analyse images at a lower rate than that
  if (depthImageBuffer.image_msg_count_ < 25)
  {
    depthImageBuffer.image_msg_count_++;
    return;
  }
  else
  {
    depthImageBuffer.image_msg_count_= 0;
  }

  // Copy the image message to a cv_bridge image pointer
  cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(image_msg_ptr);
  
  // Save the depth image
  depthImageBuffer.buffer_mutex_.lock();
  depthImageBuffer.imageDeq_.push_back(image_ptr->image);
  if(imageBuffer.imageDeq_.size()>2)
  {
    depth_image_ = depthImageBuffer.imageDeq_.front();
    depthImageBuffer.imageDeq_.pop_front();
  }
  depthImageBuffer.buffer_mutex_.unlock();
}

void BrickSearch::imageCallback(const sensor_msgs::ImageConstPtr& image_msg_ptr)
{
  // Use this method to identify when the brick is visible
  // The camera publishes at 30 fps, it's probably a good idea to analyse images at a lower rate than that
  if (imageBuffer.image_msg_count_ < 25)
  {
    imageBuffer.image_msg_count_++;
    return;
  }
  else
  {
    imageBuffer.image_msg_count_ = 0;
  }

  // Copy the image message to a cv_bridge image pointer
  cv_bridge::CvImagePtr image_ptr = cv_bridge::toCvCopy(image_msg_ptr);

  // Analyse image to find the brick
  imageBuffer.buffer_mutex_.lock();
  imageBuffer.imageDeq_.push_back(image_ptr->image);
  if(imageBuffer.imageDeq_.size()>2)
  {
    image_ = imageBuffer.imageDeq_.front();
    imageBuffer.imageDeq_.pop_front();

    // Check if brick is founded
    BrickSearch::brickIGotYouInMySight();

    // Clear keypoints for next search
    keypoints_.clear();
  }
  imageBuffer.buffer_mutex_.unlock();

  // Inform current state
  ROS_INFO("imageCallback");
  ROS_INFO_STREAM("brick_found_: " << brick_found_);
}

void BrickSearch::mainLoop()
{
  while (ros::ok())
  {
    ROS_INFO("mainLoop");
    // Delay so the loop doesn't run too fast
    ros::Duration(0.2).sleep();
  }
}

}  // namespace brick_search

int main(int argc, char** argv)
{
  ros::init(argc, argv, "brick_search");

  ros::NodeHandle nh{};

  brick_search::BrickSearch bs(nh);

  // Asynchronous spinner doesn't block
  ros::AsyncSpinner spinner(1);
  spinner.start();

  bs.mainLoop();

  return 0;
}
