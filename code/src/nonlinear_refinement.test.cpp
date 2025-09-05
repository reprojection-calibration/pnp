#include <ceres/ceres.h>
#include <gtest/gtest.h>

#include "dlt.hpp"
#include "test_data.hpp"

namespace reprojection_calibration::pnp {

Eigen::Isometry3d NonlinearRefinement(Eigen::MatrixX2d const& pixels, Eigen::MatrixX3d const& points,
                                      Eigen::Isometry3d const& tf_initial) {
    (void)pixels;
    (void)points;
    (void)tf_initial;

    return Eigen::Isometry3d::Identity();
}

}  // namespace reprojection_calibration::pnp

using namespace reprojection_calibration::pnp;

TEST(ZZZ, XXX) { EXPECT_EQ(1, 2); }