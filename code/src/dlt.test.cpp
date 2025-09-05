#include "dlt.hpp"

#include <gtest/gtest.h>

#include <iostream>

#include "matrix_utilities.hpp"
#include "test_data.hpp"

using namespace reprojection_calibration::pnp;

namespace reprojection_calibration::pnp {

// NOTE(Jack): The number of pixels and points has to match! However, because Dlt is part of the internal API, and the
// number of correspondences is already check in the public facing interface, we do not check it again here.
Eigen::Isometry3d Dlt(Eigen::MatrixX2d const& pixels, Eigen::MatrixX3d const& points) {
    auto const [normalized_pixels, tf_pixels]{NormalizeColumnWise(pixels)};
    auto const [normalized_points, tf_points]{NormalizeColumnWise(points)};

    Eigen::Matrix<double, Eigen::Dynamic, 12> const A{ConstructA(normalized_pixels, normalized_points)};
    Eigen::JacobiSVD<Eigen::MatrixXd> svd;
    svd.compute(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // TODO (Jack): There has to be a more expressive way to pack .col(11) into P
    Eigen::Matrix<double, 3, 4> P;
    P.row(0) = svd.matrixV().col(11).topRows(4);
    P.row(1) = svd.matrixV().col(11).middleRows(4, 4);
    P.row(2) = svd.matrixV().col(11).bottomRows(4);

    // NOTE(Jack): In MVG Algorithm 7.1 part (iii) they do a denormalization like:
    //      P = tf_pixels.inverse() * P * tf_points;
    // If you then project the original points with this P and then .hnormalized() them, you will get the original
    // test pixel values. However we follow the opencv "findExtrinsicCameraParams2()" which does not denormalize.

    // WARN(Jack): Should we check for negative determinant of R like they do in opencv?
    svd.compute(P.leftCols(3), Eigen::ComputeThinU | Eigen::ComputeThinV);
    auto const R = svd.matrixU() * svd.matrixV();
    double const scale{P.leftCols(3).norm() / R.norm()};
    auto const T{scale * P.rightCols(1)};

    Eigen::Isometry3d tf;
    tf.linear() = R;
    tf.translation() = T;

    return tf;
}

}  // namespace reprojection_calibration::pnp

TEST(TestDlt, XXX) {
    Eigen::Isometry3d const tf{Dlt(test_pixels, test_points)};

    EXPECT_FLOAT_EQ(tf.linear().determinant(), 1);
    EXPECT_NEAR(tf.linear().diagonal().norm(), std::sqrt(3), 1e-3);  // Heuristic given the provided test data
    EXPECT_TRUE(tf.translation().isApprox(
        Eigen::Vector3d{5.22379e-05, -2.73534e-05, 0.215797}, 1e-3));  // Heuristic given the provided test data
}
