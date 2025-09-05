#pragma once

#include <Eigen/Dense>

namespace reprojection_calibration::pnp {

Eigen::Matrix<double, Eigen::Dynamic, 12> ConstructA(Eigen::MatrixX2d const& pixels, Eigen::MatrixX3d const& points);

}  // namespace reprojection_calibration::pnp