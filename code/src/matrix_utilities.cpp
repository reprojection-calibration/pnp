#include "matrix_utilities.hpp"

namespace reprojection_calibration::pnp {

Eigen::MatrixXd InterleaveRowWise(Eigen::MatrixXd const& matrix) {
    Eigen::Index const n_rows{matrix.rows()};
    Eigen::MatrixXd const interleaved_matrix{matrix(Eigen::ArrayXi::LinSpaced(2 * n_rows, 0, n_rows), Eigen::all)};

    return interleaved_matrix;
}

Eigen::MatrixXd NormalizeColumnWise(Eigen::MatrixXd const& matrix) {
    Eigen::VectorXd const center{matrix.colwise().mean()};
    Eigen::MatrixXd const centered_matrix{matrix.rowwise() - center.transpose()};

    double const mean_magnitude{centered_matrix.rowwise().norm().mean()};
    Eigen::MatrixXd const normalized_matrix{std::sqrt(matrix.cols()) * centered_matrix.array() / mean_magnitude};

    return normalized_matrix;
}

}  // namespace reprojection_calibration::pnp