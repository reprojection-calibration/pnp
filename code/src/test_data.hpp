#pragma once

#include <Eigen/Dense>

namespace reprojection_calibration::pnp {

Eigen::MatrixX2d const test_pixels{{360.00, 240.00}, {480.00, 360.00}, {240.00, 120.00},
                                   {480.00, 180.00}, {240.00, 300.00}, {402.86, 197.14}};

Eigen::MatrixX3d const test_points{{0.00, 0.00, 5.00},   {1.00, 1.00, 5.00},   {-1.00, -1.00, 5.00},
                                   {2.00, -1.00, 10.00}, {-2.00, 1.00, 10.00}, {0.50, -0.50, 7.00}};

}  // namespace reprojection_calibration::pnp