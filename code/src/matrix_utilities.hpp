#pragma once

#include <Eigen/Dense>

namespace reprojection_calibration::pnp {

// Adopted from
// https://stackoverflow.com/questions/46110917/eigen-replicate-items-along-one-dimension-without-useless-allocations?noredirect=1&lq=1
//
// Given matrix:
//      A = 0, 1,
//          2, 3,
//          4, 5
//
// Return matrix:
//      B = 0, 1,
//          0, 1,
//          2, 3,
//          2, 3,
//          4, 5
//          4, 5
Eigen::MatrixXd InterleaveRowWise(Eigen::MatrixXd const& matrix);

Eigen::MatrixXd NormalizeColumnWise(Eigen::MatrixXd const& matrix);

}  // namespace reprojection_calibration::pnp