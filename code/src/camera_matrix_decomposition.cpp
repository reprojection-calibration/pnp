#include "camera_matrix_decomposition.hpp"

namespace reprojection_calibration::pnp {

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
std::tuple<Eigen::Matrix3d, Eigen::Matrix3d> DecomposeMIntoKr(Eigen::Matrix3d const& M) {
    const auto [K, R]{RqDecomposition(M)};

    // TODO(Jack): Fix hacky sign names here
    Eigen::Vector3d const signage{K.diagonal().array().sign()};
    Eigen::Matrix3d sign_mat{Eigen::Matrix3d::Identity()};
    sign_mat.diagonal() = signage;

    Eigen::Matrix3d const K_star{K * sign_mat};
    Eigen::Matrix3d const R_star{sign_mat * R};

    return {K_star, R_star};
}

}  // namespace reprojection_calibration::pnp