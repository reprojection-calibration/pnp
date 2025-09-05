#include "dlt.hpp"

#include <gtest/gtest.h>

#include "test_data.hpp"

using namespace reprojection_calibration::pnp;

TEST(TestDlt, XXX) {
    Eigen::Isometry3d const tf{Dlt(test_pixels, test_points)};

    EXPECT_FLOAT_EQ(tf.linear().determinant(), 1);
    EXPECT_NEAR(tf.linear().diagonal().norm(), std::sqrt(3), 1e-3);  // Heuristic given the provided test data
    EXPECT_TRUE(tf.translation().isApprox(Eigen::Vector3d{5.22379e-05, -2.73534e-05, 0.215797},
                                          1e-3));  // Heuristic given the provided test data
}
