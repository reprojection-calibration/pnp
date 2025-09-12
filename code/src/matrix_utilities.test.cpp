#include "matrix_utilities.hpp"

#include <gtest/gtest.h>

#include "test_data.hpp"

using namespace reprojection_calibration::pnp;

TEST(MatrixUtilities, TestInterleaveRowWise) {
    Eigen::MatrixX2d const interleaved_pixels{InterleaveRowWise(test_pixels)};

    EXPECT_EQ(interleaved_pixels.rows(), 12);
    // First pixel is duplicated
    EXPECT_TRUE(interleaved_pixels.row(0).isApprox(test_pixels.row(0)));
    EXPECT_TRUE(interleaved_pixels.row(1).isApprox(test_pixels.row(0)));
    // And for good measure lets check that the second pixel is duplicated too :)
    EXPECT_TRUE(interleaved_pixels.row(2).isApprox(test_pixels.row(1)));
    EXPECT_TRUE(interleaved_pixels.row(3).isApprox(test_pixels.row(1)));
}

TEST(MatrixUtilities, TestNormalizeColumnWise) {
    auto const [normalized_test_pixels, tf_pixels]{NormalizeColumnWise(test_pixels)};
    EXPECT_FLOAT_EQ(normalized_test_pixels.rowwise().norm().mean(), std::sqrt(test_pixels.cols()));
    EXPECT_FLOAT_EQ(tf_pixels.determinant(), 0.00016085755);  // Heuristic test with no theoretical backing

    auto const [normalized_test_points, tf_points]{NormalizeColumnWise(test_points)};
    EXPECT_FLOAT_EQ(normalized_test_points.rowwise().norm().mean(), std::sqrt(test_points.cols()));
    EXPECT_FLOAT_EQ(tf_points.determinant(), 0.3336733);  // Heuristic test with no theoretical backing
}

// Inspired by https://www.physicsforums.com/threads/rq-decomposition-from-qr-decomposition.261739/
// We implement RQ decomposition in terms of Eigen's built in QR decomposition
std::tuple<Eigen::Matrix3d, Eigen::Matrix3d> RqDecomposition(Eigen::Matrix3d const& matrix) {
    Eigen::Matrix3d const reverse_rows{{0, 0, 1}, {0, 1, 0}, {1, 0, 0}};
    Eigen::Matrix3d const reversed{reverse_rows * matrix};

    Eigen::HouseholderQR<Eigen::Matrix3d> qr(reversed.transpose());
    Eigen::Matrix3d const Q{qr.householderQ()};
    Eigen::Matrix3d const R{qr.matrixQR().triangularView<Eigen::Upper>()};

    Eigen::Matrix3d const R_star{reverse_rows * R.transpose() * reverse_rows};
    Eigen::Matrix3d const Q_star{reverse_rows * Q.transpose()};

    return {R_star, Q_star};
}

// NOTE(Jack): MVG section "6.2.4 Decomposition of the camera matrix" refers to the first three columns of the camera
// matrix P as M.
// Adopted from https://ksimek.github.io/2012/08/14/decompose/
std::tuple<Eigen::Matrix3d, Eigen::Matrix3d> DecomposeIntoKr(Eigen::Matrix3d const& M) {
    const auto [K, R]{RqDecomposition(M)};

    // TODO(Jack): Fix hacky sign names here
    Eigen::Vector3d const signage{K.diagonal().array().sign()};
    Eigen::Matrix3d sign_mat{Eigen::Matrix3d::Identity()};
    sign_mat.diagonal() = signage;

    Eigen::Matrix3d const K_star{K * sign_mat};
    Eigen::Matrix3d const R_star{sign_mat * R};

    return {K_star, R_star};
}

TEST(MatrixUtilities, TestRqDecomposition) {
    Eigen::Matrix3d const M{{600, 0, 360}, {0, 600, 240}, {0, 0, 1}};

    auto const [R, Q]{RqDecomposition(M)};

    EXPECT_TRUE(R.isUpperTriangular());
    EXPECT_FLOAT_EQ((Q * Q.transpose()).diagonal().sum(), 3.0);  // Matrix is orthogonal - Q*Q^T returns identity
}

TEST(MatrixUtilities, TestDecomposeIntoRk) {
    Eigen::Matrix3d const M{{600, 0, 360}, {0, 600, 240}, {0, 0, 1}};

    auto const [K, R]{DecomposeIntoKr(M)};

    EXPECT_TRUE(K.isUpperTriangular());
    EXPECT_EQ(K.diagonal().array().sign().sum(), 3);  // All diagonal entries of K must be positive
    EXPECT_FLOAT_EQ((R * R.transpose()).diagonal().sum(), 3.0);
    EXPECT_FLOAT_EQ(R.determinant(), 1.0);
}
