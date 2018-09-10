#include "AStar/AStar.h"

AStar::AStar() {

  for (size_t i = 0; i < _points.rows(); i++) {
    for (size_t j = 0; j < _points.cols(); j++) {
      _points(i, j).x = i * (11172 / 200);
      _points(i, j).y = j * (10000 / 100);
      _points(i, j)._check = false;
    }    
  } 

}
