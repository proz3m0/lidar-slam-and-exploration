#ifndef ASTAR_PATH_PLANNER_NODE_H
#define ASTAR_PATH_PLANNER_NODE_H

#include <limits>
#include <ostream>
#include <vector>

namespace maze_planner {
    struct Node {
        int id = -1;
        std::vector<int> adjacent_cells;

        friend std::ostream &operator<<(std::ostream &os, const Node &node);
    };
}  // namespace astar_path_planner
#endif  // ASTAR_PATH_PLANNER_NODE_H
