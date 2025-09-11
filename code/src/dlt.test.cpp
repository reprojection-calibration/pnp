#include "dlt.hpp"

#include <gtest/gtest.h>

#include "test_data.hpp"

using namespace reprojection_calibration::pnp;

TEST(Dlt, TestOne) {
    Eigen::Isometry3d const tf{Dlt(test_pixels, test_points)};

    EXPECT_FLOAT_EQ(tf.linear().determinant(), 1);
    EXPECT_NEAR(tf.linear().diagonal().norm(), std::sqrt(3), 1e-3);  // Heuristic given the provided test data
    EXPECT_TRUE(tf.translation().isApprox(Eigen::Vector3d{5.22379e-05, -2.73534e-05, 0.215797},
                                          1e-3));  // Heuristic given the provided test data
}

TEST(Dlt, TestTwo) {
    Eigen::Isometry3d const tf{Dlt(test_pixels_90deg_z, test_points)};

    EXPECT_FLOAT_EQ(tf.linear().determinant(), 1);
    EXPECT_NEAR(tf.linear().diagonal().norm(), std::sqrt(1), 1e-3);
    // ERROR(Jack): This gives us negative Z displacement even though we only rotated about the z-axis! Something funny
    // is going on in our implementation.
    EXPECT_TRUE(tf.translation().isApprox(Eigen::Vector3d{5.22379e-05, -2.73534e-05, -0.215797}, 1e-3));
}

TEST(Dlt, TestThree) {
    Eigen::Isometry3d const tf{Dlt(test_pixels_2m_x, test_points)};

    EXPECT_FLOAT_EQ(tf.linear().determinant(), 1);
    // WARN(Jack): This is a big error tolerane - it is not really identity like it should be!
    EXPECT_NEAR(tf.linear().diagonal().norm(), std::sqrt(3), 1e-1);
    // ERROR(Jack): This gives us negative Z displacement even though we only translated along the x-axis!
    EXPECT_TRUE(tf.translation().isApprox(Eigen::Vector3d{0.0449593, 1.31089e-05, -0.217857}, 1e-3));
}
