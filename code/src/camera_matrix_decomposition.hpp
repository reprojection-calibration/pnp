#pragma once

#include <Eigen/Dense>
#include <tuple>

namespace reprojection_calibration::pnp {

// Inspired by https://www.physicsforums.com/threads/rq-decomposition-from-qr-decomposition.261739/
// We implement RQ decomposition in terms of Eigen's built in QR decomposition
std::tuple<Eigen::Matrix3d, Eigen::Matrix3d> RqDecomposition(Eigen::Matrix3d const& matrix);

// NOTE(Jack): MVG section "6.2.4 Decomposition of the camera matrix" refers to the first three columns of the camera
// matrix P as M.
// Adopted from https://ksimek.github.io/2012/08/14/decompose/
std::tuple<Eigen::Matrix3d, Eigen::Matrix3d> DecomposeMIntoKr(Eigen::Matrix3d const& M);

// NOTE(Jack): MVG section "6.2.4 Finding the camera center"
Eigen::Vector4d CalculateCameraCenter(Eigen::Matrix<double, 3, 4> const& P);

}  // namespace reprojection_calibration::pnp