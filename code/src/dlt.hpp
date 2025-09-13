#pragma once

#include <Eigen/Dense>

namespace reprojection_calibration::pnp {

std::tuple<Eigen::Isometry3d, Eigen::Matrix3d> Dlt(Eigen::MatrixX2d const& pixels, Eigen::MatrixX3d const& points);

Eigen::Matrix<double, Eigen::Dynamic, 12> ConstructA(Eigen::MatrixX2d const& pixels, Eigen::MatrixX3d const& points);

Eigen::Matrix<double, 3, 4> SolveForP(Eigen::Matrix<double, Eigen::Dynamic, 12> const& A);

}  // namespace reprojection_calibration::pnp