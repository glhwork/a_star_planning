#include <cmath>

#include "a_star_planning/AStar.h"

astar::AStar::AStar() {

  MapInit();
  _path.clear();

}

astar::LatVec astar::AStar::PathFind(const Eigen::Vector3d &position, 
                                     const Eigen::Vector3d &goal) {

  astar::Lattice goal_point, path_point;
  path_point = ClosestPoint(position);
  goal_point = ClosestPoint(goal);
  _points[path_point._row][path_point._col]._check = true;
  _path.push_back(path_point);

  
  astar::Lattice tmp_point;
  double min_f;
  double passed_dis = 0.0;

  while (!goal_point._check) {

    min_f = 1000000.0;
    for (size_t i = path_point._row - 1; i < path_point._row + 2; i++) {
      for (size_t j = path_point._col - 1; j < path_point._col + 2; j++) {
        
        if (!_points[i][j]._check) {
          double f = Weight(_points[i][j], goal_point, path_point) + passed_dis;
          if (f < min_f) {
            min_f = f;
            tmp_point = _points[i][j];
          }
        }

      }
    }

    passed_dis = passed_dis + DisFind(tmp_point, path_point);
    path_point = tmp_point;
    path_point._check = true;
    _points[path_point._row][path_point._col]._check = true;
    // std::cout << path_point._x << "  "
    //           << path_point._y << "  "
    //           << path_point._z << std::endl;

    _path.push_back(path_point);
    if (path_point._x == goal_point._x && 
        path_point._y == goal_point._y) {
      goal_point._check = true;
    }
    std::cout << goal_point._check << std::endl;

  }  // while loop end
  return _path;
}

double astar::AStar::Weight(const Lattice &test_point, 
                            const Lattice &goal_point,
                            Lattice &point) {
  
  point._g = DisFind(test_point, point);
  point._h = DisFind(test_point, goal_point);

  return point._g + point._h;
}

astar::Lattice astar::AStar::ClosestPoint(const Eigen::Vector3d &position) {
  
  double distance;
  double min_dist = 100000.0;
  astar::Lattice closest_point;

  for (size_t i = 0; i < lattice_row; i++) {
    for (size_t j = 0; j < lattice_col; j++) {
      
      distance = sqrt(pow((position(0) - _points[i][j]._x), 2) +
                      pow((position(1) - _points[i][j]._y), 2) +
                      pow((position(2) - _points[i][j]._z), 2));

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
      _points[i][j]._x = i * (10000.0 / lattice_row);
      _points[i][j]._y = j * (10000.0 / lattice_col);
      _points[i][j]._z = 0.0;
      _points[i][j]._row = i;
      _points[i][j]._col = j;
    }    
  }

  for (size_t i = 20; i < 201; i++) {
    _points[i][100]._x = 100000.0;
    _points[i][100]._y = 100000.0;    
  }


}

double astar::AStar::DisFind(const Lattice &x, const Lattice &y) {

  double distance;
  distance = sqrt(pow((x._x - y._x), 2) +
                  pow((x._y - y._y), 2) +
                  pow((x._z - y._z), 2));

  return distance;
}

