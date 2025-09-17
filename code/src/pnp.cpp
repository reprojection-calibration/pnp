#include "pnp/pnp.hpp"

#include "dlt.hpp"
#include "nonlinear_refinement.hpp"

namespace reprojection_calibration::pnp {

// TODO(Jack): What is the canonical way to deal with the camera matrix? I want the pnp algo to know nothing about K if
// possible, because the more we let information about the camera and camera models percolate through the code base, the
// more combined everything will get. That being said, should we specifically pass the pixels here in image coordinates?
// That seems to be the only reasonable coordinate system given how people interact with pixels. Then the next question
// I have is should we normalize the points at the top level pnp function and then feed those both to the DLT and the
// nonlinear refinement? Right now I do not understand what coordinate context to do the nonlinear refinement in.
PnpResult Pnp(Eigen::MatrixX2d const& pixels, Eigen::MatrixX3d const& points) {
    if (not(pixels.rows() == points.rows())) {
        return PnpStatusCode::MismatchedCorrespondence;
    } else if (pixels.rows() < 6) {
        return PnpStatusCode::NotEnoughPoints;
    }

    auto const [tf, K]{Dlt(pixels, points)};
    auto const [tf_star, _]{NonlinearRefinement(pixels, points, tf, K)};

    return tf_star;
}

}  // namespace reprojection_calibration::pnp