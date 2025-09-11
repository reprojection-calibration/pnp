#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "pose_utilities.hpp"

namespace reprojection_calibration::pnp {

// MVG = "multiple view geometry"
struct MvgFrame {
    Se3 pose;
    Eigen::MatrixX2d pixels;
    Eigen::MatrixX3d points;
};

class MvgFrameGenerator {
   public:
    MvgFrameGenerator(Eigen::MatrixX3d const& points, Eigen::Matrix3d const& K);

    MvgFrame Generate();

   private:
    Eigen::MatrixX3d points_;
    Eigen::Matrix3d K_;
};

}  // namespace reprojection_calibration::pnp

using namespace reprojection_calibration::pnp;

TEST(HHH, HHH) { EXPECT_EQ(1, 2); }
