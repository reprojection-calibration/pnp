#include <gtest/gtest.h>

#include <Eigen/Dense>

namespace reprojection_calibration::pnp {

using Se3 = Eigen::Vector<double, 6>;

// MVG = "multiple view geometry"
struct MvgFrame {
    Se3 pose;
    Eigen::MatrixX2d pixels;
    Eigen::MatrixX3d points;
};

Se3 ToSe3(Eigen::Isometry3d const& matrix) {
    Eigen::AngleAxisd const rotation(matrix.linear());
    Eigen::Vector3d const so3{rotation.angle() * rotation.axis()};

    Se3 pose;
    pose << so3, matrix.translation();

    return pose;
}

// Eigen::Isometry3d FromSe3(Se3 const& se3) { return Eigen::Isometry3d::Identity(); }

}  // namespace reprojection_calibration::pnp

using namespace reprojection_calibration::pnp;

TEST(HHH, XXX) {
    Eigen::Isometry3d const identity{Eigen::Isometry3d::Identity()};
    EXPECT_FLOAT_EQ(ToSe3(identity).sum(), 0.0);

    // Test with two different rotations
    Eigen::Isometry3d const yaw_rotation{Eigen::AngleAxisd(0.5 * EIGEN_PI, Eigen::Vector3d::UnitZ())};
    EXPECT_TRUE(ToSe3(yaw_rotation).isApprox(Se3{0, 0, 0.5 * EIGEN_PI, 0, 0, 0}));

    Eigen::Isometry3d const pitch_yaw_rotation{Eigen::AngleAxisd(0.5 * EIGEN_PI, Eigen::Vector3d::UnitY()) *
                                               Eigen::AngleAxisd(0.5 * EIGEN_PI, Eigen::Vector3d::UnitZ())};
    EXPECT_TRUE(ToSe3(pitch_yaw_rotation).isApprox(Se3{1.2092, 1.2092, 1.2092, 0, 0, 0}, 1e-3));  // Heuristic result

    // Tests with translation only
    Eigen::Isometry3d translation{Eigen::Isometry3d::Identity()};
    translation.translation() = Eigen::Vector3d(-1, 0, 1);
    EXPECT_TRUE(ToSe3(translation).isApprox(Se3{0, 0, 0, -1, 0, 1}));

    // Test with a translation and rotation
    Eigen::Isometry3d translation_and_rotation{Eigen::Isometry3d::Identity()};
    translation_and_rotation.translation() = Eigen::Vector3d(-1, 0, 1);
    translation_and_rotation.rotate(Eigen::AngleAxisd(0.5 * EIGEN_PI, Eigen::Vector3d::UnitZ()));
    EXPECT_TRUE(ToSe3(translation_and_rotation).isApprox(Se3{0, 0, 0.5 * EIGEN_PI, -1, 0, 1}));
}
