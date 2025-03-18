#pragma once

#include <Eigen/Core>

namespace copyleft {

struct IsotropicRemeshParameters {
  double targetEdgeLength = 0.01;
  int numberIter = 10;
};



} // namespace copyleft