#include <cmath>

#include "a_star_planning/AStar.h"

astar::AStar::AStar() {

  MapInit();
  _path.clear();

}

astar::LatVec astar::AStar::PathFind(const Eigen::Vector3d& position, 
                                     const Eigen::Vector3d& goal) {

  astar::Lattice goal_point, path_point, start_point;
  start_point = ClosestPoint(position);
  goal_point = ClosestPoint(goal);
  path_point = start_point;
  _points[path_point._row][path_point._col]._check = true;
  
  astar::Lattice tmp_point;
  double min_f;
  double passed_dis = 0.0;

  while (!goal_point._check) {

    min_f = 1000000.0;
    for (size_t i = path_point._row - 1; i < path_point._row + 2; i++) {
      for (size_t j = path_point._col - 1; j < path_point._col + 2; j++) {
        
        if (!_points[i][j]._check) {
          double f = Weight(_points[i][j], goal_point, &path_point) + passed_dis;
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

    if (path_point._x == goal_point._x && 
        path_point._y == goal_point._y) {
      goal_point._check = true;
    }

  }  // searching loop end

  bool finish_find = false;
  path_point = goal_point;
  _points[path_point._row][path_point._col]._check = false;
  while (!finish_find) {

    double min_g = 1000000.0;
    for (size_t i = path_point._row - 1; i < path_point._row + 2; i++) {
      for (size_t j = path_point._col - 1; j < path_point._col + 2; j++) {

        if (_points[i][j]._check and
            (i != path_point._row or 
             j != path_point._col)) {

          double g = DisFind(start_point, _points[i][j]);
          if (g < min_g) {
            min_g = g;
            tmp_point = _points[i][j];
          } 

        }
      }
    }

    _path.push_back(tmp_point);
    path_point = tmp_point;
    _points[path_point._row][path_point._col]._check = false;

    if (path_point._x == start_point._x &&
        path_point._y == start_point._y &&
        path_point._z == start_point._z) {
      finish_find = true;
    }

  }  // finding shortest path end
  return _path;
}

double astar::AStar::Weight(const Lattice& test_point, 
                            const Lattice& goal_point,
                            Lattice* point) {

  point->_g = DisFind(test_point, *point);
  point->_h = DisFind(test_point, goal_point);

  return point->_g + point->_h;
}

astar::Lattice astar::AStar::ClosestPoint(const Eigen::Vector3d& position) {
  
  double distance;
  double min_dist = 100000.0;
  astar::Lattice closest_point;

  for (size_t i = 0; i < _lattice_row; i++) {
    for (size_t j = 0; j < _lattice_col; j++) {
      
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
  
}  // find the nearest point on the map

void astar::AStar::MapInit() {

  _lattice_row = sizeof(_points) / sizeof(_points[0]);
  _lattice_col = (sizeof(_points) / sizeof(astar::Lattice)) / _lattice_row;

  for (size_t i = 0; i < _lattice_row; i++) {
    for (size_t j = 0; j < _lattice_col; j++) {
      _points[i][j]._x = i * (10000.0 / _lattice_row);
      _points[i][j]._y = j * (10000.0 / _lattice_col);
      _points[i][j]._z = 0.0;
      _points[i][j]._row = i;
      _points[i][j]._col = j;
    }    
  }

  for (size_t i = _lattice_row / 10; i < _lattice_row + 1; i++) {
    _points[i][_lattice_row / 2]._x = 100000.0;
    _points[i][_lattice_row / 2]._y = 100000.0;    
  }


}  // create a map in matrix form

double astar::AStar::DisFind(const Lattice& x, const Lattice& y) {

  double distance;
  distance = sqrt(pow((x._x - y._x), 2) +
                  pow((x._y - y._y), 2) +
                  pow((x._z - y._z), 2));

  return distance;
}

