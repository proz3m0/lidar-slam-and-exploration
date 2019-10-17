#include "brick_search.h"

BrickSearch::BrickSearch(ros::NodeHandle nh, ros::NodeHandle nh_private):it_(nh),nh_(nh),nh_private_(nh_private),tf2_listener_(tf2_buffer_)
{
    ROS_INFO("Brick search started");

    colorMsg_.subscribe(nh_,"/camera/rgb/image_raw",1);
    depthMsg_.subscribe(nh_,"/camera/depth/image_raw",1);

    std::string pc_sub_topic;
    nh_private_.param<std::string>("pc_topic", pc_sub_topic, "/camera/depth_registered/points");
    nh_private_.param<std::string>("base_frame", camera_frame_, "camera_rgb_optical_frame");
    nh_private_.param<std::string>("map_frame", map_frame_, "map");

    //Note, change MySyncPolicy(queueSize) to change how many messages it compares
    sync_.reset(new Sync(MySyncPolicy(QUEUE_SIZE), colorMsg_, depthMsg_));
    //boost::bind uses generic numbers _1, _2, ..., _9 to represent arguments
    //After specifying function, first argument must be an instance of the member function's class
    //That's why 'this' is used because it references an instance of the class
    sync_->registerCallback(boost::bind(&BrickSearch::syncCallBack, this, _1, _2));

    amcl_pose_sub_ = nh_.subscribe("amcl_pose", 1, &BrickSearch::amclPoseCallBack, this);

    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);

    pc_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(pc_sub_topic, 1, &BrickSearch::pcCallback, this);

    // Publishing an image to "/image/test" topic
    test_image_pub_ = it_.advertise("/image/test", 1);
    brick_found_pub_ = nh.advertise<std_msgs::Bool>("/brick_found", 10);
};

BrickSearch::~BrickSearch(){};

void BrickSearch::amclPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped& pose_msg)
{ 
    poseBuffer_.mutex_.lock();
    poseBuffer_.pose = pose_msg.pose.pose;
    poseBuffer_.mutex_.unlock();
};

void BrickSearch::syncCallBack(const sensor_msgs::ImageConstPtr& colorMsg, const sensor_msgs::ImageConstPtr& depthMsg)
{   
    //Grab our images from messages
    cv_bridge::CvImagePtr cv_color_ptr;

    try
    {
        cv_color_ptr = cv_bridge::toCvCopy(colorMsg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    ROS_INFO_STREAM("brick_found_: " << brick_found_);
    findRedBlob(cv_color_ptr);
};

void BrickSearch::findRedBlob(const cv_bridge::CvImagePtr& cv_ptr_rgb)
{
    ROS_INFO("find blob");
    // Variables
    cv::Mat image,mask1,mask2,mask3;

    // Convert that frame to seen color
    cv::cvtColor(cv_ptr_rgb->image,image,cv::COLOR_BGR2RGB);

    // Convert to hsv
    cvtColor(cv_ptr_rgb->image, imageHsv_, cv::COLOR_BGR2HSV);

    // Creating masks to detect the upper and lower red color.
    cv::inRange(imageHsv_,cv:: Scalar(0, 120, 70),cv::Scalar(10, 255, 255), mask1);
    cv::inRange(imageHsv_,cv::Scalar(170, 120, 70),cv::Scalar(180, 255, 255), mask2);

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
    else
    {
      //calculate waypoint here
      if(brick_found_ == false) BrickSearch::findXYZ(keypoints_[0]);

      brick_found_ = true;
      std_msgs::Bool brick_found;
      brick_found.data = true;
      brick_found_pub_.publish(brick_found);
    }

    // Published the blob image on rqt_image_view
    cv::drawKeypoints(image,keypoints_,test_image_, cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", test_image_).toImageMsg();
    test_image_pub_.publish(msg);
};

geometry_msgs::PoseStamped BrickSearch::findXYZ(cv::KeyPoint keypoint)
{
    int row = keypoint.pt.x;
    int col = keypoint.pt.y;

    // ros msg to pcl
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*pc_msg_, *cloud);

    //checks if cloud is organised
    if(!cloud->isOrganized()) ROS_ERROR("POINT CLOUD NOT ORGANISED");

    // gets pose from pixel
    geometry_msgs::PoseStamped brick_pose;

    // ensures valid orientation
    brick_pose.pose.orientation.w = 1.0;

    // gets index from point cloud
    int index = col * cloud->width + row;
    brick_pose.pose.position.x = cloud->points[index].x;
    brick_pose.pose.position.y = cloud->points[index].y;
    brick_pose.pose.position.z = cloud->points[index].z;

    // transforms pose to map frame
    geometry_msgs::TransformStamped tf;
    if(!BrickSearch::fetchTransform(tf, map_frame_, pc_msg_->header.frame_id))ROS_ERROR("TF Failed");
    tf2::doTransform(brick_pose, brick_pose, tf);
    brick_pose.header.frame_id = map_frame_;
    return brick_pose;
};

bool BrickSearch::fetchTransform(geometry_msgs::TransformStamped &transform, std::string target_frame, std::string source_frame)
{
    try
    {
        transform = tf2_buffer_.lookupTransform(target_frame, source_frame,ros::Time(0), ros::Duration(1.0));
        return true;
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return false;
    }
};


void BrickSearch::pcCallback(const sensor_msgs::PointCloud2ConstPtr &pc_msg)
{
    // stores pc pointer so that when the pose is needed it can easily be extracted
    pc_msg_ = pc_msg;
};
