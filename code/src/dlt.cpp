#include "dlt.hpp"

#include "matrix_utilities.hpp"

namespace reprojection_calibration::pnp {

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

}  // namespace reprojection_calibration::pnp