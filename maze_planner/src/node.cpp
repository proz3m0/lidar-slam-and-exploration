#include "maze_planner/node.h"

namespace maze_planner
{
std::ostream& operator<<(std::ostream& os, const Node& node)
{
  os << "    "
     << "ID: " << node.id << ", ";

  return os;
}
}