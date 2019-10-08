#ifndef CAMERARAYTRACER_H
#define CAMERARAYTRACER_H

#include <utility>
#include <vector>
#include <random>
#include <chrono>
#include <iostream>
#include <ros/package.h>
#include <geometry_msgs/Pose2D.h>

constexpr double DEFAULT_FOV = M_PI/2;
constexpr double DEFAULT_CAM_RANGE = 2.0;
constexpr double DEFAULT_ANG_RES = 0.1*M_PI/180;
constexpr double DEFAULT_RAY_INT = 0.1;

/** \brief A class that returns the ray points of a camera */
class CameraRayTracer
{
private:
    /** \brief horizontal feild of view */
    double fov_;
    /** \brief camera range */
    double cam_range_;
    /** \brief angular resolution for ray tracing */
    double ang_res_;
    /** \brief distance interval for ray tracing */
    double ray_int_;
public:
    /*********************************************************************
        * CONSTRUCTORS
    *********************************************************************/
    /** \brief Default Constructor */
    CameraRayTracer();
    /** \brief Constructor that initializes all variables */
    CameraRayTracer(double fov, double cam_range, double ang_res, double ray_int);
    /*********************************************************************
        * METHODS
    *********************************************************************/
    /** \brief returns a vector of vector of 2d points
      * \param cam_pose the pose of the camera */
    std::vector<std::vector<geometry_msgs::Pose2D>> getRayTracePoints(geometry_msgs::Pose2D cam_pose);
};


#endif //CAMERARAYTRACER_H
