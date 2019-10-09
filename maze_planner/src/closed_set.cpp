#include "maze_planner/closed_set.h"

#include <algorithm>

namespace maze_planner {
    size_t ClosedSet::size() {
        return nodes_.size();
    }

    void ClosedSet::push(Node n) {
        nodes_.push_back(n);
    }
    void ClosedSet::push(std::vector<Node> n) {
        for(auto && node:n)push(node);
    }

    bool ClosedSet::contains(int id) {
        // Return true if the node is in nodes_
        for (const auto &n : nodes_) {
            if (n.id == id) {
                return true;
            }
        }

        return false;
    }

    std::vector<Node> ClosedSet::getNodes() {
        return nodes_;
    }

    std::ostream &operator<<(std::ostream &os, const ClosedSet &closed_set) {
        os << "\n\nClosed set:" << std::endl;

        for (const auto &n : closed_set.nodes_) {
            os << n;
        }

        return os;
    }


    void ClosedSet::filter(std::vector<Node> &nodes) {
        for (auto it = nodes.begin(); it != nodes.end(); ) {
            if (contains(it->id)) it = nodes.erase(it);
            else ++it;
        }
    }

}  // namespace astar_path_planner
