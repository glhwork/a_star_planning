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

  int _row;
  int _col;

  double _f;
  double _g;
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

 private:
  Lattice _points[200][100];
  int lattice_row;
  int lattice_col;
  std::vector<Lattice> _path;     
};  // class AStar

}  // namespace astar

#endif