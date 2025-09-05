#pragma once

#include <Eigen/Dense>

namespace reprojection_calibration::pnp {

// NOTE(Jack): We see instantly how creating scalable test data should be a top priority! The code that created these
// values does not scale as an idea! Is it possible to have cmake run a python script that creates this data directly
// each time or something like that?

Eigen::MatrixX2d const test_pixels{{360.00, 240.00}, {480.00, 360.00}, {240.00, 120.00},
                                   {480.00, 180.00}, {240.00, 300.00}, {402.86, 197.14}};

// Same test_points but taken from a camera rotated 90 degrees around the z-axis
Eigen::MatrixX2d const test_pixels_90deg_z{{360.00, 240.00}, {240.00, 360.00}, {480.00, 120.00},
                                           {420.00, 360.00}, {300.00, 120.00}, {402.86, 282.86}};

// Same test_points as but taken from a camera shifted along the x-axis 2 meters
Eigen::MatrixX2d const test_pixels_2m_x{{600.00, 240.00}, {720.00, 360.00}, {480.00, 120.00},
                                        {600.00, 180.00}, {360.00, 300.00}, {574.29, 197.14}};

Eigen::MatrixX3d const test_points{{0.00, 0.00, 5.00},   {1.00, 1.00, 5.00},   {-1.00, -1.00, 5.00},
                                   {2.00, -1.00, 10.00}, {-2.00, 1.00, 10.00}, {0.50, -0.50, 7.00}};

}  // namespace reprojection_calibration::pnp