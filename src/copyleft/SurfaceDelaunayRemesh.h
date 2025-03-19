#pragma once

#include <Eigen/Core>

namespace copyleft {

struct SurfaceDelaunayRemeshParameters {

};

void surfaceDelaunayRemesh(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F,
                           const Eigen::VectorXi &C, 
                           const SurfaceDelaunayRemeshParameters& params,
                           Eigen::MatrixXd &V_out,
                           Eigen::MatrixXi &F_out, Eigen::VectorXi &C_out);

} // namespace copyleft