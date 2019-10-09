#ifndef ASTAR_PATH_PLANNER_OPEN_SET_H
#define ASTAR_PATH_PLANNER_OPEN_SET_H


#include <vector>
#include <ostream>
#include <algorithm>

#include "maze_planner/node.h"
#include "maze_planner/occupancy_grid.h"

namespace maze_planner
{
class OpenSet
{
private:
  std::vector<Node> nodes_{};

public:
  void push(Node n);     // Adds a vector of new nodes
  Node pop();                               // Returns and reomves  the top node
  bool contains(int id);                   // Returns true if the node is in the open set
  bool empty();                            // Returns true if the open set is empty
  std::vector<Node> removeClosed(std::vector<Node> closed_nodes);

  std::vector<Node> getNodes();

  friend std::ostream& operator<<(std::ostream& os, const OpenSet& open_set);
};

}  // namespace astar_path_planner

#endif  // ASTAR_PATH_PLANNER_OPEN_SET_H
