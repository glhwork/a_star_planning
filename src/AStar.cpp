#include <cmath>

#include "a_star_planning/AStar.h"

astar::AStar::AStar() {

  MapInit();

}

astar::LatVec astar::AStar::PathFind(const Eigen::Vector3d &position, 
                                     const Eigen::Vector3d &goal) {

  astar::Lattice cur_point, goal_point, path_point;
  cur_point = ClosestPoint(position);
  goal_point = ClosestPoint(goal);

  path_point = cur_point;
  _points[cur_point._row][cur_point._col]._check = true;

  for (size_t i = path_point._row - 1; i < path_point._row + 2; i++) {
    for (size_t j = path_point._col - 1; j < path_point._col + 2; j++) {
      if (!_points[i][j]._check) {

      }
    }
  }




}

astar::Lattice astar::AStar::ClosestPoint(const Eigen::Vector3d &position) {
  
  double distance;
  double min_dist = 100000.0;
  astar::Lattice closest_point;

  for (size_t i = 0; i < lattice_row; i++) {
    for (size_t j = 0; j < lattice_col; i++) {
      distance = sqrt(pow((position(1) - _points[i][j]._x), 2) +
                      pow((position(2) - _points[i][j]._y), 2) +
                      pow((position(3) - _points[i][j]._z), 2));
      if (distance < min_dist) {
        min_dist = distance;
        closest_point = _points[i][j];
      }                  
    }
  }
  return closest_point;
  
}

void astar::AStar::MapInit() {

  lattice_row = sizeof(_points) / sizeof(_points[0]);
  lattice_col = (sizeof(_points) / sizeof(astar::Lattice)) / lattice_row;

  for (size_t i = 0; i < lattice_row; i++) {
    for (size_t j = 0; j < lattice_col; j++) {
      _points[i][j]._x = i * (11172 / 20);
      _points[i][j]._y = j * (10000 / 10);
      _points[i][j]._check = false;
      _points[i][j]._row = i;
      _points[i][j]._col = j;
    }    
  }

}