#pragma once

#include <Eigen/Dense>
#include <variant>
#include <string>

namespace reprojection_calibration::pnp {

enum class PnpStatusCode {
    MismatchedCorrespondence,
    NotEnoughPoints,
};

// TODO(Jack): Is it bad to use a using declaration here in the public API section?
using PnpResult = std::variant<Eigen::Isometry3d, PnpStatusCode>;

PnpResult Pnp(Eigen::MatrixX2d const& pixels, Eigen::MatrixX3d const& points);

}  // namespace reprojection_calibration::pnp