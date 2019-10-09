#ifndef ASTAR_PATH_PLANNER_PATH_SET_H
#define ASTAR_PATH_PLANNER_PATH_SET_H


#include <vector>
#include <ostream>
#include <algorithm>

#include "maze_planner/node.h"
#include "maze_planner/occupancy_grid.h"

namespace maze_planner
{
class PathSet
{
private:
  std::vector<std::vector<Node>> nodes_{};

public:
  void push(Node n);     // Adds a vector of new nodes
  void newPush(Node n);
  bool contains(int id);                   // Returns true if the node is in the open set
  bool empty();                            // Returns true if the open set is empty

  std::vector<Node> getNodes();

  std::vector<std::vector<int>> getIds();

};

}  // namespace astar_path_planner

#endif  // ASTAR_PATH_PLANNER_OPEN_SET_H
