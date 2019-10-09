#include "maze_planner/open_set.h"

#include <ros/ros.h>

namespace maze_planner {

    Node OpenSet::pop() {
        Node n = nodes_.back();
        nodes_.pop_back();
        return n;
    }

    bool OpenSet::contains(int id) {
        // Returns true if the node is in nodes_
        for (const auto &n : nodes_) {
            if (n.id == id) {
                return true;
            }
        }

        return false;
    }

    bool OpenSet::empty() {
        return nodes_.empty();
    }

    std::vector<Node> OpenSet::getNodes() {
        return nodes_;
    }


    std::ostream &operator<<(std::ostream &os, const OpenSet &open_set) {
        os << "\n\nOpen set:" << std::endl;

        for (const auto &n : open_set.nodes_) {
            os << n;
        }

        return os;
    }

    void OpenSet::push(Node n) {
        nodes_.push_back(n);
    }

    std::vector<Node> OpenSet::removeClosed(std::vector<Node> closed_nodes) {

        std::vector<int> closed_ids;
        for(auto c:closed_nodes) closed_ids.push_back(c.id);
        std::vector<Node> closed;
        std::vector<Node> open;
        for(auto && n:nodes_) {
            bool node_open = false;
            for(auto adj_id:n.adjacent_cells){
                if(std::find(closed_ids.begin(), closed_ids.end(), adj_id) == closed_ids.end() && !contains(adj_id)){
                    node_open = true;
                }
            }
            if(node_open) open.push_back(n);
            else closed.push_back(n);
        }
        nodes_ = open;
        return closed;
    }

}  // namespace astar_path_planner