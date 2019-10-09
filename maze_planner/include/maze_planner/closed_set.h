#ifndef ASTAR_PATH_PLANNER_CLOSED_SET_H
#define ASTAR_PATH_PLANNER_CLOSED_SET_H

#include <vector>

#include "maze_planner/node.h"

namespace maze_planner
{
class ClosedSet
{
private:
  std::vector<Node> nodes_{};

public:
  size_t size();
  void push(Node n);                             // Add a new node
  void push(std::vector<Node> n);                             // Add a new node
  bool contains(int id);                                // Returns true if the node is in the closed set
  void filter(std::vector<Node>& nodes);                // Removes nodes if in nodes_

  std::vector<Node> getNodes();

  friend std::ostream& operator<<(std::ostream& os, const ClosedSet& closed_set);
};
}  // namespace astar_path_planner
#endif  // ASTAR_PATH_PLANNER_CLOSED_SET_H
