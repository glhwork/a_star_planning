#include "a_star_planning/AStar.h"

int main() {
  astar::AStar plan_test;

  Eigen::Vector3d position;
  Eigen::Vector3d goal;

  position << 9000.0, 1000.0, 0.0;
  goal << 9000.0, 9000.0, 0.0;

  astar::LatVec x = plan_test.PathFind(position, goal);
  return 0;
}
