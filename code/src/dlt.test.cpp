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

// The 2n x 12 matrix assembled by stacking up the constraints from (MVG Eq. 7.2)
// NOTE(Jack): I am not gonna test this because I hope this function changes soon, see the note in the function.
Eigen::Matrix<double, Eigen::Dynamic, 12> ConstructA(Eigen::MatrixX2d const& pixels, Eigen::MatrixX3d const& points) {
    Eigen::Matrix<double, Eigen::Dynamic, 12> A(2 * pixels.rows(), 12);
    for (int const i : {0, 1, 2}) {
        A.middleCols(i * 4, 4) = InterleaveRowWise(points).rowwise().homogeneous();
    }

    // TODO(Jack): There has to be a better way to do this than iterating over each pixel and doing this manually! This
    // entire function upsets me because it feels so manual, and that we are missing the proper linear algebra
    // abstraction to combine the matrices into A
    for (Eigen::Index i{0}; i < pixels.rows(); ++i) {
        auto const pixel_i{pixels.row(i)};
        auto A_i{A.middleRows(i * 2, 2)};

        // Construct O^T, -w_i*X_i^T, y_i * X_i^T
        A_i.block<1, 4>(0, 0) = Eigen::ArrayXXd::Zero(1, 4);
        A_i.block<1, 4>(0, 4) *= -1.0;
        A_i.block<1, 4>(0, 8) *= pixel_i(1);

        // Construct w_i*X_i^T, O^T, -x_i * X_i^T
        A_i.block<1, 4>(1, 0) *= 1.0;
        A_i.block<1, 4>(1, 4) = Eigen::ArrayXXd::Zero(1, 4);
        A_i.block<1, 4>(1, 8) *= -pixel_i(0);
    }

    return A;
}

Eigen::MatrixXd NormalizeColumnWise(Eigen::MatrixXd const& matrix) {
    Eigen::VectorXd const center{matrix.colwise().mean()};
    Eigen::MatrixXd const centered_matrix{matrix.rowwise() - center.transpose()};

    double const mean_magnitude{centered_matrix.rowwise().norm().mean()};
    Eigen::MatrixXd const normalized_matrix{std::sqrt(matrix.cols()) * centered_matrix.array() / mean_magnitude};

    return normalized_matrix;
}

// NOTE(Jack): The number of pixels and points has to match! However, because Dlt is part of the internal API, and the
// number of correspondences is already check in the public facing interface, we do not check it again here.
Eigen::Isometry3d Dlt(Eigen::MatrixX2d const& pixels, Eigen::MatrixX3d const& points) {
    auto const normalized_pixels{NormalizeColumnWise(pixels)};
    auto const normalized_points{NormalizeColumnWise(points)};

    Eigen::Matrix<double, Eigen::Dynamic, 12> const A{ConstructA(normalized_pixels, normalized_points)};

    Eigen::JacobiSVD<Eigen::MatrixXd> svd;
    svd.compute(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    std::cout << "Its singular values are:" << std::endl << svd.singularValues() << std::endl;
    std::cout << "Its left singular vectors are the columns of the thin U matrix:" << std::endl
              << svd.matrixU() << std::endl;
    std::cout << "Its right singular vectors are the columns of the thin V matrix:" << std::endl
              << svd.matrixV() << std::endl;

    return Eigen::Isometry3d::Identity();
}

}  // namespace reprojection_calibration::pnp

TEST(TestDlt, XXX) {
    Dlt(test_pixels, test_points);

    EXPECT_EQ(1, 2);
}

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