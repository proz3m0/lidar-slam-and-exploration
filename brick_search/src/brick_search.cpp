#include <atomic>
#include <cmath>
#include <mutex>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>


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
  cv::Point3f brick_location_;

  // Transform listener
  tf2_ros::Buffer transform_buffer_{};
  tf2_ros::TransformListener transform_listener_{ transform_buffer_ };

  // Subscribe to the AMCL pose to get covariance
  ros::Subscriber amcl_pose_sub_{};

  // Velocity command publisher
  ros::Publisher cmd_vel_pub_{};

  // Bool publisher for brick found
  ros::Publisher brick_found_pub_;

  // Image transport and subscriber
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_{};
  image_transport::Subscriber depth_image_sub_{};
  image_transport::Publisher test_image_pub_{};

  // Image buffer
  struct ImageDataBuffer
  {
    int image_msg_count_ = 0;
    std::atomic<bool> ready_{ false };
    std::deque<cv::Mat> imageDeq_;
    std::mutex buffer_mutex_;
  };
  ImageDataBuffer imageBuffer;
  ImageDataBuffer depthImageBuffer;
  cv::Mat image_;
  cv::Mat depth_image_;
  cv::Mat test_image_;

  // Camera info subscriber
  ros::Subscriber camera_info_sub_{};

  // Camera info buffer
  struct CameraInfoBuffer
  {
    std::deque<sensor_msgs::CameraInfoConstPtr> cameraInfoDeq_;
    std::mutex buffer_mutex_;
  };
  CameraInfoBuffer cameraBuffer;
  sensor_msgs::CameraInfoConstPtr camera_info_;

  // Action client
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_client_{ "move_base", true };

  // Private methods
  geometry_msgs::Pose2D getPose2d();
  void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_msg);
  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg_ptr);
  void depthImageCallback(const sensor_msgs::ImageConstPtr& image_msg_ptr);
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg_ptr);
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

  // Subscribe to the camera for RGB image
  image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &BrickSearch::imageCallback, this);

  // Subscribe to the RGB camera for RGB info
  camera_info_sub_ = nh.subscribe("/camera/rgb/camera_info", 1, &BrickSearch::cameraInfoCallback, this);

  // Subscribe to the camera for D image
  depth_image_sub_ = it_.subscribe("/camera/depth/image_raw", 1, &BrickSearch::depthImageCallback, this);

  // Advertise "cmd_vel" publisher to control TurtleBot manually
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, false);

  // Publishing an image to "/map_image/fbe" topic
  test_image_pub_ = it_.advertise("/map_image/test", 1);
  brick_found_pub_ = nh.advertise<std_msgs::Bool>("/brick_found", 10);

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

void BrickSearch::depthImageCallback(const sensor_msgs::ImageConstPtr& image_msg_ptr)
{
  // Use this method to identify when the brick is visible
  // The camera publishes at 30 fps, it's probably a good idea to analyse images at a lower rate than that
  if (depthImageBuffer.image_msg_count_ < 15)
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
  if(depthImageBuffer.imageDeq_.size()>2)
  {
    depth_image_ = depthImageBuffer.imageDeq_.front();
    depthImageBuffer.imageDeq_.pop_front();
    depthImageBuffer.ready_ = true;
  }
  depthImageBuffer.buffer_mutex_.unlock();

  // Inform current state
  //ROS_INFO("depthImageCallback");
}

void BrickSearch::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg_ptr)
{
    // Save the camera info
    cameraBuffer.buffer_mutex_.lock();
    cameraBuffer.cameraInfoDeq_.push_back(info_msg_ptr);
    if(cameraBuffer.cameraInfoDeq_.size()>2)
    {
      camera_info_ = cameraBuffer.cameraInfoDeq_.front();
      cameraBuffer.cameraInfoDeq_.pop_front();
    }
    cameraBuffer.buffer_mutex_.unlock();

    // Inform current state
    //ROS_INFO("cameraInfoCallback");
}

void BrickSearch::imageCallback(const sensor_msgs::ImageConstPtr& image_msg_ptr)
{
  // The camera publishes at 30 fps, it's probably a good idea to analyse images at a lower rate than that
  if (imageBuffer.image_msg_count_ < 15)
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

    // If brick is found then find its location
    //depthImageBuffer.buffer_mutex_.lock();
    //if (brick_found_ == true && depthImageBuffer.ready_ == true) BrickSearch::brickWhereAreYou();
    //depthImageBuffer.buffer_mutex_.unlock();

    // Clear keypoints for next search
    keypoints_.clear();
  }
  imageBuffer.buffer_mutex_.unlock();

  // Inform current state
  ROS_INFO_STREAM("brick_found_: " << brick_found_);
}

void BrickSearch::brickIGotYouInMySight(void)
{
  // Variables
  cv::Mat hsv,mask1,mask2,mask3;

  // Convert that frame to seen color
  cv::cvtColor(image_,image_,cv::COLOR_BGR2RGB);

  // Convert that frame from BGR to HSV
  cv::cvtColor(image_, hsv, cv::COLOR_BGR2HSV);
    
  // Creating masks to detect the upper and lower red color.
  cv::inRange(hsv,cv:: Scalar(0, 120, 70),cv::Scalar(10, 255, 255), mask1);
  cv::inRange(hsv,cv::Scalar(170, 120, 70),cv::Scalar(180, 255, 255), mask2);
    
  // Generate the final mask
  mask1 = mask1 + mask2;
  mask3 = mask1;
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
  //ROS_INFO_STREAM("Size of keypoints: " << keypoints_.size());
    
  // If keypoints is filled then brick is found
  if (keypoints_.empty()) brick_found_ = false;
  else {
      brick_found_ = true;
      std_msgs::Bool brick_found;
      brick_found.data = true;
      brick_found_pub_.publish(brick_found);
  }

  // Published the blob image on rqt_image_view
  cv::drawKeypoints(image_,keypoints_,test_image_, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", test_image_).toImageMsg();
  test_image_pub_.publish(msg);

  // Inform current state
  //ROS_INFO("brickIGotYouInMySight");
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
  float x = blob_centre.x;
  float y = blob_centre.y;
  ROS_INFO_STREAM("Blob centre X: "<< x);
  ROS_INFO_STREAM("Blob centre Y: "<< y);
  //ROS_INFO("HERE");

  // *FIND_THE_POSITION_OF_BRICK_IN_3D
  // The ratio from RGB to D
  float rgb_d_rows_ratio = (float)image_.rows / (float)depth_image_.rows;
  float rgb_d_cols_ratio = (float)image_.cols / (float)depth_image_.cols;
  //ROS_INFO("HERE");
  // get camera intrinsics
  cameraBuffer.buffer_mutex_.lock();
  float fx = camera_info_->K[0];
  float fy = camera_info_->K[4];
  float px = camera_info_->K[2];
  float py = camera_info_->K[5];
  //ROS_INFO_STREAM("fx: "<<fx);
  //ROS_INFO_STREAM("fy: "<<fy);
  //ROS_INFO_STREAM("px: "<<px);
  //ROS_INFO_STREAM("py: "<<px);
  cameraBuffer.buffer_mutex_.unlock();
  //ROS_INFO("HERE");

  // Find depth of the blob centre RGB pixel
  float depth = depth_image_.at<uchar>(cv::Point((int)(x / rgb_d_rows_ratio),(int)(y / rgb_d_cols_ratio))) * (3000 / 255) + 500;
  //ROS_INFO_STREAM("Rows ratio: "<<(int)(x / rgb_d_rows_ratio));
  //ROS_INFO_STREAM("Cols ratio: "<<(int)(y / rgb_d_cols_ratio));
  //ROS_INFO_STREAM("Depth: "<<depth);

  //ROS_INFO("HERE");

  if (depth>0)
  {
    brick_location_.x = (x - px) * depth / fx;
    brick_location_.y = (y - py) * depth / fy;
    brick_location_.z = depth;
  }

  // Inform current state
  ROS_INFO_STREAM(" | X: "<<brick_location_.x<<
                  " | Y: "<<brick_location_.y<<
                  " | Z: "<<brick_location_.z);
  //ROS_INFO("brickWhereAreYou");
}

void BrickSearch::mainLoop()
{
  while (ros::ok())
  {
    //ROS_INFO("mainLoop");
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
