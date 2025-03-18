#pragma once

#include <Eigen/Core>

namespace copyleft {

struct IsotropicRemeshParameters {
  double targetEdgeLength = 0.01;
  int numberIter = 10;
};

// V: vertex positions
// F: face indices
// C: patch indices
// SC: selected patch indices
void isotropicRemesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F,
                     const Eigen::VectorXi &C, const std::vector<int> &SC,
                     IsotropicRemeshParameters params, Eigen::MatrixXd &V_out,
                     Eigen::MatrixXi &F_out, Eigen::VectorXi &C_out);

} // namespace copyleft