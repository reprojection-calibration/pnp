#include "dlt.hpp"

#include "matrix_utilities.hpp"

namespace reprojection_calibration::pnp {

// NOTE(Jack): The number of pixels and points has to match! However, because Dlt is part of the internal API, and the
// number of correspondences is already check in the public facing interface, we do not check it again here.
// NOTE(Jack): In MVG Algorithm 7.1 part (iii) they do a denormalization like:
//      P = tf_pixels.inverse() * P * tf_points;
// If you then project the original points with this P and then .hnormalized() them, you will get the original
// test pixel values. However we follow the opencv "findExtrinsicCameraParams2()" which does not denormalize.
Eigen::Isometry3d Dlt(Eigen::MatrixX2d const& pixels, Eigen::MatrixX3d const& points) {
    auto const [normalized_pixels, tf_pixels]{NormalizeColumnWise(pixels)};
    auto const [normalized_points, tf_points]{NormalizeColumnWise(points)};
    Eigen::Matrix<double, Eigen::Dynamic, 12> const A{ConstructA(normalized_pixels, normalized_points)};

    Eigen::JacobiSVD<Eigen::MatrixXd> svd;
    svd.compute(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix<double, 3, 4> P;
    // TODO (Jack): There has to be a more expressive way to pack .col(11) into P
    P.row(0) = svd.matrixV().col(11).topRows(4);
    P.row(1) = svd.matrixV().col(11).middleRows(4, 4);
    P.row(2) = svd.matrixV().col(11).bottomRows(4);

    Eigen::Matrix3d P_rotation_component{P.leftCols(3)};
    if (P_rotation_component.determinant() < 0) {
        P_rotation_component *= -1.0;
    }

    svd.compute(P_rotation_component, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d const R{svd.matrixU() * svd.matrixV()};
    double const scale{P_rotation_component.norm() / R.norm()};
    Eigen::Vector3d const T{scale * P.rightCols(1)};

    return ToIsometry3d(R, T);
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

}  // namespace reprojection_calibration::pnp