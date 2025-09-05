#pragma once

#include <Eigen/Dense>
#include <tuple>

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

// This function was designed with the normalization required for the DLT in mind! This means that after normalization
// we want the "average distance of a point x from the origin is equal to sqrt(n)" (where n is the dimension of the
// non-homogeneous point). See MVG 4.4.4 heading "Isotropic Scaling"
// NOTE(Jack): We need the transform to put the SVD result back into scaled coordinates, therefore we return both the
// normalized points and the transform that did the normalizing. It feels like we are missing some symetry because this
// transform gets applied and then deapplied, maybe it makes sense to make the code reflect that, but for now it does
// not.
std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> NormalizeColumnWise(Eigen::MatrixXd const& matrix);

}  // namespace reprojection_calibration::pnp