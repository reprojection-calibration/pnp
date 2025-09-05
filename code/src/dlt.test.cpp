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
    auto const normalized_pixels{NormalizeColumnWise(pixels)};
    auto const normalized_points{NormalizeColumnWise(points)};

    Eigen::Matrix<double, Eigen::Dynamic, 12> const A{ConstructA(normalized_pixels, normalized_points)};

    Eigen::JacobiSVD<Eigen::MatrixXd> svd;
    svd.compute(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // TODO (Jack): There has to be a more expressive way to pack .col(11) into P
    Eigen::Matrix<double, 3, 4> P;
    P.row(0) = svd.matrixV().col(11).topRows(4);
    P.row(1) = svd.matrixV().col(11).middleRows(4, 4);
    P.row(2) = svd.matrixV().col(11).bottomRows(4);
    P /= P(2, 3);

    std::cout << "P: " << P << std::endl;

    return Eigen::Isometry3d::Identity();
}

}  // namespace reprojection_calibration::pnp

TEST(TestDlt, XXX) {
    Dlt(test_pixels, test_points);

    EXPECT_EQ(1, 2);
}
