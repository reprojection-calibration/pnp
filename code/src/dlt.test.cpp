#include "dlt.hpp"

#include <gtest/gtest.h>

#include <iostream>

#include "../cmake-build-debug-docker-pnp-development/_deps/googletest-src/googletest/include/gtest/gtest.h"

using namespace reprojection_calibration::pnp;

Eigen::MatrixX2d const test_pixels{{360.00, 240.00}, {480.00, 360.00}, {240.00, 120.00},
                                   {480.00, 180.00}, {240.00, 300.00}, {402.86, 197.14}};

Eigen::MatrixX3d const test_points{{0.00, 0.00, 5.00},   {1.00, 1.00, 5.00},   {-1.00, -1.00, 5.00},
                                   {2.00, -1.00, 10.00}, {-2.00, 1.00, 10.00}, {0.50, -0.50, 7.00}};

namespace reprojection_calibration::pnp {

// Adopted from
// https://stackoverflow.com/questions/46110917/eigen-replicate-items-along-one-dimension-without-useless-allocations?noredirect=1&lq=1
//
// Given matrix:
//      A = 0, 1,
//          2, 3,
//          4, 5
//
// Return matrix:
//      B = 0, 1,
//          0, 1,
//          2, 3,
//          2, 3,
//          4, 5
//          4, 5
Eigen::MatrixXd InterleaveRowWise(Eigen::MatrixXd const& matrix) {
    Eigen::Index const n_rows{matrix.rows()};
    Eigen::MatrixXd const interleaved_matrix{matrix(Eigen::ArrayXi::LinSpaced(2 * n_rows, 0, n_rows), Eigen::all)};

    return interleaved_matrix;
}

// NOTE(Jack): The number of pixels and points has to match! However, because Dlt is part of the internal API, and the
// number of correspondences is already check in the public facing interface, we do not check it again here.
Eigen::Isometry3d Dlt(Eigen::MatrixX2d const& pixels, Eigen::MatrixX3d const& points) {
    static_cast<void>(points);

    // The 2n x 12 matrix assembled by stacking up the constraints from (Eq. 7.2)
    Eigen::Matrix<double, Eigen::Dynamic, 12> A(2 * pixels.rows(), 12);

    for (int const i : {0, 1, 2}) {
        A.middleCols(i * 4, 4).topRows(6) = points.rowwise().homogeneous();
    }

    std::cout << A << std::endl;

    return Eigen::Isometry3d::Identity();
}

}  // namespace reprojection_calibration::pnp

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
