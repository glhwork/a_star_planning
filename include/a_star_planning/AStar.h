#ifndef ASTAR_H
#define ASTAR_H

#include <Eigen/Dense>
#include <vector>
#include <iostream>

namespace astar {

struct Lattice {
  Lattice() 
    : _check(false), _g(0.0), _h(0.0) { }
  double _x;
  double _y;
  double _z;
  bool _check;

  int _row;
  int _col;

  double _g;
  double _h;
};  // struct Lattice

typedef std::vector<Lattice> LatVec;

class AStar {
 public:
  AStar();
  ~AStar() { }
  LatVec PathFind(const Eigen::Vector3d &position,
                  const Eigen::Vector3d &goal);
  Lattice ClosestPoint(const Eigen::Vector3d &position);
  void MapInit();
  double DisFind(const Lattice &x, const Lattice &y);
  double Weight(const Lattice &test_point,
                const Lattice &goal_point,
                Lattice &point);

 private:
  Lattice _points[200][200];
  int lattice_row;
  int lattice_col;
  LatVec _path;     
};  // class AStar

}  // namespace astar

#endif