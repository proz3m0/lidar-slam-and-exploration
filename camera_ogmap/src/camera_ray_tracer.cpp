#include "camera_ogmap/camera_ray_tracer.h"

CameraRayTracer::CameraRayTracer()
:fov_(DEFAULT_FOV), cam_range_(DEFAULT_CAM_RANGE), ang_res_(DEFAULT_ANG_RES), ray_int_(DEFAULT_RAY_INT){}

CameraRayTracer::CameraRayTracer(double fov, double cam_range, double ang_res, double ray_int)
:fov_(fov), cam_range_(cam_range), ang_res_(ang_res), ray_int_(ray_int){}

std::vector<std::vector<geometry_msgs::Pose2D>> CameraRayTracer::getRayTracePoints(geometry_msgs::Pose2D cam_pose) {
    std::vector<std::vector<geometry_msgs::Pose2D>> trace_points;
    // Iterates for ang_res angles to generate points
    for(auto d_theta = -fov_/2; d_theta < fov_/2; d_theta = d_theta + ang_res_) {
        // Finds the max distance
        double max_d = cam_range_ / cos(fabs(d_theta));
        double theta = cam_pose.theta + d_theta;
        std::vector<geometry_msgs::Pose2D> ray_trace_points;
        double d = 0.0;
        // Iterates until d is equal to or greater then max_d
        while(d <= max_d) {
            geometry_msgs::Pose2D point;
            point.x = cam_pose.x + d*cos(theta);
            point.y = cam_pose.y + d*sin(theta);
            ray_trace_points.push_back(point);
            d += ray_int_;
        }
        trace_points.push_back(ray_trace_points);
    }
    return trace_points;
}
