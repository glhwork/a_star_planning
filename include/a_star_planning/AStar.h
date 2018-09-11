#ifndef ASTAR_H
#define ASTAR_H

#include <Eigen/Dense>
#include <vector>

namespace astar{

struct Lattice {
  double _x;
  double _y;
  double _z;
  bool _check;
};  // struct Lattice

typedef std::vector<Lattice> LatVec;

class AStar {
 public:
    AStar();
    ~AStar() { }
    LatVec PathFind(const Eigen::Vector3d position);
    Lattice ClosestPoint(const Eigen::Vector3d position);
    void MapInit();

private:
  Eigen::Matrix<Lattice, 200, 100> _points;
  std::vector<Lattice> path;     
};  // class AStar

}  // namespace astar

#endif