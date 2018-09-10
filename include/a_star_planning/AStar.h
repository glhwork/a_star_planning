#ifndef ASTAR_H
#define ASTAR_H
#include <Eigen/Dense>

struct Lattice {
  double _x;
  double _y;
  double _z;
  bool _check;
};

class AStar {
 public:
    AStar();
    ~AStar();

private:
  Eigen::Matrix<Lattice, 200, 100> _points;     
};

