#include <vector>
#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

#include "maze_planner/occupancy_grid.h"
#include "maze_planner/node.h"
#include "maze_planner/open_set.h"
#include "maze_planner/closed_set.h"
#include "maze_planner/marker.h"
#include "maze_planner/PlanPath.h"
#include "maze_planner/path_set.h"

namespace {
    double heuristicCost(maze_planner::WorldPosition a, maze_planner::WorldPosition b) {
        // Return a heuristic cost between two world positions
        // Returns the distance between the positions
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));  // YOUR CODE HERE
    }

    void waitForKey() {
        ROS_INFO("Paused, press enter to continue...");
        std::cin.get();
    }
}  // namespace
namespace maze_planner {
    class PathPlanner {
    public:
        PathPlanner(ros::NodeHandle nh);

    private:
        double inflation_radius_;
        double plan_path_main_loop_rate_;  // The target rate of the main loop in planPath()
        double robot_inflation_;
        ros::Duration plan_path_main_loop_duration_;

        OccupancyGrid occupancy_grid_;
        ros::NodeHandle nh_;
        ros::Publisher inflated_map_pub_{}, open_set_pub_{}, closed_set_pub_{}, path_pub_{};
        ros::Subscriber map_sub_;

        ros::ServiceServer plan_path_srv_{};

        bool map_set_;

        visualization_msgs::Marker createSetMarker(const std::vector<Node> &nodes, MarkerColour colour);

        bool planPath(maze_planner::PlanPath::Request &req, maze_planner::PlanPath::Response &res);
        void mapCallback(const nav_msgs::OccupancyGridConstPtr &map_msg);
    };

    PathPlanner::PathPlanner(ros::NodeHandle nh):nh_(nh), map_set_(false) {
        // Get parameters (a variable will not be changed if the ROS parameter has not been set)
        ros::NodeHandle pn("~");
        pn.param<double>("inflation_radius", inflation_radius_, 0.2);
        pn.param<double>("plan_path_main_loop_rate", plan_path_main_loop_rate_, 3);
        pn.param<double>("robot_inflation", robot_inflation_, 0.5);
        plan_path_main_loop_duration_ = ros::Duration(1. / plan_path_main_loop_rate_);

        // Map Subscriber
        map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 10, &PathPlanner::mapCallback, this);

        // Advertise PlanPath service server
        plan_path_srv_ = nh_.advertiseService("plan_path", &PathPlanner::planPath, this);
        inflated_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map_inflated", 1, true);

        // Advertise marker publishers
        open_set_pub_ = nh_.advertise<visualization_msgs::Marker>("path_planner/open_set", 1);
        closed_set_pub_ = nh_.advertise<visualization_msgs::Marker>("path_planner/closed_set", 1);
        path_pub_ = nh_.advertise<visualization_msgs::Marker>("path_planner/path", 1);
    }

    visualization_msgs::Marker PathPlanner::createSetMarker(const std::vector<Node> &nodes, MarkerColour colour) {
        std::vector<WorldPosition> world_positions{};

        world_positions.reserve(nodes.size());

        for (const auto &n : nodes) {
            world_positions.emplace_back(occupancy_grid_.getWorldPosition(n.id));
        }

        return createSphereListMarker(world_positions, colour, 0.05);
    }
    void PathPlanner::mapCallback(const nav_msgs::OccupancyGridConstPtr &map_msg) {
        if(!map_set_) {
            occupancy_grid_ = OccupancyGrid(*map_msg, inflation_radius_);
            ROS_INFO("Map received");
            // Publish the inflated map
            inflated_map_pub_.publish(occupancy_grid_.getOccupancyGridMsg());
            map_set_ = true;
        }
    }

    bool PathPlanner::planPath(maze_planner::PlanPath::Request &req, maze_planner::PlanPath::Response &res) {

        // Delete existing path markers
        path_pub_.publish(createDeleteMarker());

        WorldPosition start_position{};
        start_position.x = req.robot_pose.pose.position.x;
        start_position.y = req.robot_pose.pose.position.y;
        ROS_INFO("Maze Planner: PlanPath: Start Position: x:%f\ty:%f", start_position.x, start_position.y);

        OpenSet open_set{};
        ClosedSet closed_set{};
        PathSet path_set{};

        Node start_node{};

        start_node.id = occupancy_grid_.getCellId(occupancy_grid_.getGridPosition(start_position));
        // start_node has no parent

        open_set.push(start_node);

        std::vector<Node> open_set_hold{};

        int inflation = floor(robot_inflation_ / occupancy_grid_.getMapData().resolution);

        while (ros::ok() && !open_set.empty()) {
            ros::Time loop_start_time = ros::Time::now();

            // gets the top node
            Node n = open_set.pop();
            // pushes n onto the closed_set and gets the vector of adjacent cells
            bool recent = false;
            for(auto h:open_set_hold){
                if(n.id == h.id){
                    recent = true;
                    break;
                }
            }
            if(recent)path_set.push(n);
            else path_set.newPush(n);

            closed_set.push(n);

            // get inflated cells
            std::vector<int> inf_a, inf_cells;
            inf_a.push_back(n.id);
            for (auto i = 0; i < inflation; i++) {
                std::vector<int> inf_temp;
                while (!inf_a.empty()) {
                    int inf = inf_a.back();
                    inf_cells.push_back(inf);
                    inf_a.pop_back();

                    std::vector<int> inf_ids = occupancy_grid_.getAdjacentCellIDs(inf);
                    for (auto inf_id:inf_ids) {
                        if (std::find(inf_a.begin(), inf_a.end(), inf_id) == inf_a.end() &&
                            std::find(inf_cells.begin(), inf_cells.end(), inf_id) == inf_cells.end() &&
                            std::find(inf_temp.begin(), inf_temp.end(), inf_id) == inf_temp.end())
                            inf_temp.push_back(inf_id);
                    }
                }
                inf_a = inf_temp;
            }
            inf_cells.insert(inf_cells.end(), inf_a.begin(), inf_a.end());

            // turn vector of ids to vector of cnodes
            std::vector<Node> inf_nodes;
            for (auto id:inf_cells) {
                Node n;
                n.id = id;
                n.adjacent_cells = occupancy_grid_.getAdjacentCellIDs(id);
                inf_nodes.push_back(n);
            }

            // filter closed cells
            closed_set.filter(inf_nodes);
            // sort cells
            bool swap = true;
            while(swap) {
                swap = false;
                for(auto i = 0; i < inf_nodes.size()-1;i++) {
                    if(occupancy_grid_.cellDistance(inf_nodes[i].id, n.id) < occupancy_grid_.cellDistance(inf_nodes[i+1].id, n.id)) {
                        Node hold = inf_nodes[i+1];
                        inf_nodes[i+1] = inf_nodes[i];
                        inf_nodes[i] = hold;
                        swap = true;
                    }
                }
            }

            // add to open set
            open_set_hold = inf_nodes;
            for(auto && n:inf_nodes) {
                open_set.push(n);
            }
            // filters open for closed cells and adds to close
            closed_set.push(open_set.removeClosed(closed_set.getNodes()));


            // Publish the sets
            open_set_pub_.publish(createSetMarker(open_set.getNodes(), MarkerColour::YELLOW));
            closed_set_pub_.publish(createSetMarker(closed_set.getNodes(), MarkerColour::FUCHSIA));
            path_pub_.publish(createSetMarker(path_set.getNodes(), MarkerColour::GREEN));

            // Sleep if the duration of the loop is less than the target duration
//            ros::Duration loop_duration = ros::Time::now() - loop_start_time;
//
//            if (loop_duration < plan_path_main_loop_duration_) {
//                (plan_path_main_loop_duration_ - loop_duration).sleep();
//            }
        }

        // Gets path
        res.path = occupancy_grid_.pathFromNodes(path_set.getIds());
        ROS_INFO("Maze Planner: PlanPath: Path size: %d", res.path.size());

        // Delete open and closed set markers
        ros::Duration(0.5).sleep();
        open_set_pub_.publish(createDeleteMarker());
        closed_set_pub_.publish(createDeleteMarker());

        return true;
    }

}  // namespace astar_path_planner

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_planner");
    ros::NodeHandle nh;
    maze_planner::PathPlanner path_planner(nh);
    ros::spin();

    return 0;
}
