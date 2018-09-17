#include "a_star_planning/AStar.h"
#include <fstream>

int main() {
  astar::AStar plan_test;

  Eigen::Vector3d position;
  Eigen::Vector3d goal;

  position << 9000.0, 1000.0, 0.0;
  goal << 5000.0, 9000.0, 0.0;

  astar::LatVec x = plan_test.PathFind(position, goal);
  
  std::ofstream fout;
  fout.open("/home/george/Desktop/test.txt");
  for (size_t i = 0; i < x.size(); i++) {
  	fout << x[i]._x << "  "
  	     << x[i]._y << "  "
  	     << x[i]._z << std::endl;
  }
  fout.close();
  
  return 0;
}
