#include "pnp/pnp.hpp"

namespace reprojection_calibration::pnp {

PnpResult Pnp(Eigen::MatrixX2d const& pixels, Eigen::MatrixX3d const& points) {
    if (not(pixels.rows() == points.rows())) {
        return PnpStatusCode::MismatchedCorrespondence;
    } else if (pixels.rows() < 6) {
        return PnpStatusCode::NotEnoughPoints;
    }

    return Eigen::Isometry3d::Identity();
}

}  // namespace reprojection_calibration::pnp