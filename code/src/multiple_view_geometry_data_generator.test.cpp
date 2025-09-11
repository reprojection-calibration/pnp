#include <gtest/gtest.h>

#include <Eigen/Dense>

namespace reprojection_calibration::pnp {

// MVG = "multiple view geometry"
struct MvgFrame {
    Eigen::Vector<double, 6> se3_pose;

    Eigen::MatrixX2d pixels;
    Eigen::MatrixX3d points;
};

}  // namespace reprojection_calibration::pnp

using namespace reprojection_calibration::pnp;

TEST(HHH, XXX) { EXPECT_EQ(1, 2); }