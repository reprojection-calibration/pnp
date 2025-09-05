#include "matrix_utilities.hpp"

#include <gtest/gtest.h>

#include "test_data.hpp"

using namespace reprojection_calibration::pnp;

TEST(TestInterleaveRowWise, XXX) {
    Eigen::MatrixX2d const interleaved_pixels{InterleaveRowWise(test_pixels)};

    EXPECT_EQ(interleaved_pixels.rows(), 12);
    // First pixel is duplicated
    EXPECT_TRUE(interleaved_pixels.row(0).isApprox(test_pixels.row(0)));
    EXPECT_TRUE(interleaved_pixels.row(1).isApprox(test_pixels.row(0)));
    // And for good measure lets check that the second pixel is duplicated too :)
    EXPECT_TRUE(interleaved_pixels.row(2).isApprox(test_pixels.row(1)));
    EXPECT_TRUE(interleaved_pixels.row(3).isApprox(test_pixels.row(1)));
}

TEST(TestNormalizeColumnWise, XXX) {
    auto const normalized_test_pixels{NormalizeColumnWise(test_pixels)};
    EXPECT_FLOAT_EQ(normalized_test_pixels.rowwise().norm().mean(), std::sqrt(test_pixels.cols()));

    auto const normalized_test_points{NormalizeColumnWise(test_points)};
    EXPECT_FLOAT_EQ(normalized_test_points.rowwise().norm().mean(), std::sqrt(test_points.cols()));
}