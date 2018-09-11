#include <cmath>

#include "AStar/AStar.h"

AStar::AStar() {

  MapInit();

}

LatVec AStar::PathFind(const Eigen::Vector3d position) {

  Lattice cur_posi;
  cur_point = ClosestPoint(position);

}

Lattice AStar::ClosestPoint(const Eigen::Vector3d position) {
  
  double distance;
  double min_dist = 100000.0;
  Lattice closest_point;

  for (size_t i = 0; i < _points.rows(); i++) {
    for (size_t j = 0; j < _points.cols(); i++) {
      distance = sqrt(pow((position(1) - _points(i,j).x), 2) +
                      pow((position(2) - _points(i,j).y), 2) +
                      pow((position(3) - _points(i,j).z), 2));
      if (distance < min_dist) {
        min_dist = distance;
        closest_point = _points(i,j);
      }                  
    }
  }
  return closest_point;
  
}

void AStar::MapInit() {

  for (size_t i = 0; i < _points.rows(); i++) {
    for (size_t j = 0; j < _points.cols(); j++) {
      _points(i, j).x = i * (11172 / 200);
      _points(i, j).y = j * (10000 / 100);
      _points(i, j)._check = false;
    }    
  }

}