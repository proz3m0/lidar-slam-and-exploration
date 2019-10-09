#include "maze_planner/path_set.h"

#include <ros/ros.h>

namespace maze_planner {

    bool PathSet::contains(int id) {
        // Returns true if the node is in nodes_
        for (const auto &n_vec : nodes_) {
            for( const auto &n:n_vec){
                if (n.id == id) {
                    return true;
                }
            }
        }

        return false;
    }

    bool PathSet::empty() {
        return nodes_.empty();
    }

    std::vector<Node> PathSet::getNodes() {
        std::vector<Node> nodes;
        for(auto node_vec:nodes_) nodes.insert(nodes.begin(), node_vec.begin(), node_vec.end());
        return nodes;
    }

    void PathSet::push(Node n) {
        if(nodes_.empty()){
            std::vector<Node> nodes_vec = {n};
            nodes_.push_back(nodes_vec);
        }
        else{
            nodes_[nodes_.size() - 1].push_back(n);
        }
    }

    void PathSet::newPush(Node n) {
        std::vector<Node> nodes_vec = {n};
        nodes_.push_back(nodes_vec);
    }

    std::vector<std::vector<int>> PathSet::getIds() {
        std::vector<std::vector<int>> ids;
        for(auto node_vec:nodes_) {
            std::vector<int> id_vec;
            for(auto n:node_vec) {
                id_vec.push_back(n.id);
            }
            ids.push_back(id_vec);
        }
        return ids;
    }


}  // namespace astar_path_planner