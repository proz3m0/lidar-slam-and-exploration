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
#include <nav_msgs/OccupancyGrid.h>

#include "camera_ogmap/map_transformer.h"
#include "camera_ogmap/camera_ray_tracer.h"

const int FREE_CELL = 0;
const int OCCUPIED_CELL = 100;
const int UNKNOWN_CELL = -1;

/** \brief class for exploration*/
class CameraOGMapNode {
private:
    /*********************************************************************
        * ROS VARIABLES
    *********************************************************************/
    /** \brief ros nodehandle */
    ros::NodeHandle nh_;
    /** \brief ros publisher to publish move_base poses */
    ros::Publisher cam_ogmap_pub_;
    /** \brief ros subscriber to check whether brick has been found*/
    ros::Subscriber map_sub_;
    /** \brief tf2 buffer for transforms */
    tf2_ros::Buffer tf2_buffer_;
    /** \brief tf2 listener for transforms */
    tf2_ros::TransformListener tf2_listener_;
    /** \brief robots camera frame */
    std::string camera_link_frame_;
    /** \brief map frame */
    std::string map_frame_;
    /*********************************************************************
        * CAMERA OGMAP VARIABLES
    *********************************************************************/
    /** \brief camera ray tracer */
    CameraRayTracer cam_ray_tracer_;
    /** \brief map transformer */
    MapTransformer map_transformer_;
    /** \brief rate of getting new camera tf*/
    double cam_rate_;
    /** \brief occupancy grid map*/
    nav_msgs::OccupancyGrid og_map_;
    /*********************************************************************
        * BOOLEANS
    *********************************************************************/
    /** \brief initialized */
    bool init_;
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
    /** \brief initialises the ogmap
      * \param map_msg */
    bool initOGMap(const nav_msgs::OccupancyGridConstPtr &map_msg) {
        // copies ogmap
        og_map_.info = map_msg->info;
        og_map_.data = map_msg->data;
        map_transformer_ = MapTransformer(map_msg->info);

        // marks free cells as unknown and unknown and occupied as occupied
        for(auto&& cell:og_map_.data) {
            switch(cell) {
                case FREE_CELL:
                    cell = UNKNOWN_CELL;
                    break;
                case UNKNOWN_CELL:
                    cell = OCCUPIED_CELL;
                    break;
                case OCCUPIED_CELL:
                    break;
                default:
                    ROS_WARN("Camera OGMap: initOGMap: invalid cell value");
            }
        }
        init_ = true;
        ROS_INFO("Camera OGMap: initOGMap: Map Initialised");
    }
    /** \brief proccesses camera pose for ogmap
      * \param map_msg */
    void processCameraTF(tf2::Transform cam_tf) {
        // turns cam_tf to Pose2D
        double y,p,r;
        tf2::Matrix3x3 cam_rot(cam_tf.getRotation());
        cam_rot.getEulerYPR(y,p,r);
        geometry_msgs::Pose2D cam_pose;
        cam_pose.x = cam_tf.getOrigin().getX();
        cam_pose.y = cam_tf.getOrigin().getY();
        cam_pose.theta = y;
        // gets camera ray tracers points
        std::vector<std::vector<geometry_msgs::Pose2D>> cam_map_points = cam_ray_tracer_.getRayTracePoints(cam_pose);
        // gets grid values
        std::vector<std::vector<GridCell>> cam_grid_cells = map_transformer_.mapPointToGridCells(cam_map_points);
        // traces and sets ogmap
        for(auto cam_ray:cam_grid_cells) {
            for(auto grid_cell:cam_ray) {
                // gets index
                int index;
                map_transformer_.gridCellToIndex(grid_cell, index);
                //ROS_INFO("INDEX: %d\t GRID CELL: x:%d\ty:%d", index, grid_cell.x, grid_cell.y);
                if(og_map_.data[index] == OCCUPIED_CELL) break;
                //ROS_INFO("WRITING CELL: x:%d\ty:%d", grid_cell.x, grid_cell.y);
                og_map_.data[index] = FREE_CELL;
            }
        }
    }

public:
    /*********************************************************************
        * CONSTRUCTOR
    *********************************************************************/
    /** \brief CameraOGMap constructor
      * \param nh nodehandle */
    CameraOGMapNode(ros::NodeHandle nh)
            : nh_(nh), tf2_listener_(tf2_buffer_), init_(false) {
        // Nodehande for input
        ros::NodeHandle pn("~");

        // Sets up frames
        pn.param<std::string>("camera_link_frame", camera_link_frame_, "camera_link");
        pn.param<std::string>("map_frame", map_frame_, "map");

        // Sets up subscribers, publishers and services
        std::string map_topic, cam_ogmap_topic;
        pn.param<std::string>("map_topic", map_topic, "/map");
        pn.param<std::string>("cam_ogmap_topic", cam_ogmap_topic, "/camera_og_map");
        // Advertises publishers
        cam_ogmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(cam_ogmap_topic, 10);
        // Sets up subscribers
        map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(map_topic, 10, &CameraOGMapNode::mapCallback, this);

        // Sets up CameraRayTracer
        double cam_h_fov, ang_res, cam_range, ray_int;
        pn.param<double>("cam_h_fov", cam_h_fov, 90*M_PI/180);
        pn.param<double>("ang_res", ang_res, 0.1*M_PI/180);
        pn.param<double>("cam_range", cam_range, 1.0);
        pn.param<double>("ray_int", ray_int, 0.01);
        cam_ray_tracer_ = CameraRayTracer(cam_h_fov, cam_range, ang_res, ray_int);

        // Sets up rate
        pn.param<double>("cam_rate", cam_rate_, 2.0);

    }
    /*********************************************************************
        * CALLBACKS
    *********************************************************************/
    /** \brief callback function for brick found */
    void mapCallback(const nav_msgs::OccupancyGridConstPtr &map_msg) {
        if(!init_) initOGMap(map_msg);
    }
    /*********************************************************************
        * THREADS
    *********************************************************************/
    /** \brief thread that saves the map to base_frame tf */
    void camTFThread(){

        // Delay to allow for initialisation
        ros::Duration(0.5).sleep();

        // Busy waiting for initialisation
        while(!init_);

        ros::Rate rate_limiter(cam_rate_);
        while(ros::ok()) {
            // Gets current robot transform and if close to target sends new goal
            tf2::Transform tf;
            if (!fetchTransform(tf, map_frame_, camera_link_frame_)) {
                ROS_WARN("Camera OGMap: camTFThread: fetchTransform failed");
            } else {
                // Gets camera pose and updates CameraOGMAP
                processCameraTF(tf);
                ROS_INFO("Camera OGMap: camTFThread: publishing");
                cam_ogmap_pub_.publish(og_map_);
            }
            rate_limiter.sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_ogmap");
    ros::NodeHandle nh;
    CameraOGMapNode camera_ogmap(nh);
    std::thread tfThread(&CameraOGMapNode::camTFThread, std::ref(camera_ogmap));
    ros::spin();
    return 0;
}
