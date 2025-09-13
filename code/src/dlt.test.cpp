#include "dlt.hpp"

#include <gtest/gtest.h>

#include "multiple_view_geometry_data_generator.hpp"
#include "pose_utilities.hpp"
#include "test_data.hpp"

using namespace reprojection_calibration::pnp;

TEST(Dlt, TestOne) {
    auto const [tf, K]{Dlt(test_pixels, test_points)};

    EXPECT_FLOAT_EQ(tf.linear().determinant(), 1);
    EXPECT_NEAR(tf.linear().diagonal().norm(), std::sqrt(3), 1e-3);
    EXPECT_NEAR(tf.translation().norm(), 0.0, 1e-2);
}

TEST(Dlt, TestTwo) {
    auto const [tf, K]{Dlt(test_pixels_90deg_z, test_points)};

    EXPECT_FLOAT_EQ(tf.linear().determinant(), 1);
    EXPECT_NEAR(tf.linear().diagonal().norm(), std::sqrt(1), 1e-3);
    EXPECT_NEAR(tf.translation().norm(), 0.0, 1e-2);
}

TEST(Dlt, TestThree) {
    auto const [tf, K]{Dlt(test_pixels_2m_x, test_points)};

    EXPECT_FLOAT_EQ(tf.linear().determinant(), 1);
    EXPECT_NEAR(tf.linear().diagonal().norm(), std::sqrt(3), 1e-3);
    EXPECT_TRUE(tf.translation().isApprox(Eigen::Vector3d{-1.99894, 0.0, 0.0}, 1e-2));
}
