#include <ceres/ceres.h>  // TODO(Jack): Add header ordering to the clang format, this is madness!
#include <ceres/rotation.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>

namespace reprojection_calibration::pnp {

// TODO(Jack): What is the final and best type for camera going to be? Raw pointer smells to me, or is at least not 100%
// necessary considering how far along with ceres we are (not far).
template <typename T>
Eigen::Vector<T, 2> PinholeProjection(T const* const camera, Eigen::Vector<T, 3> const& point) {
    T const& fx{camera[0]};
    T const& fy{camera[1]};
    T const& cx{camera[2]};
    T const& cy{camera[3]};

    T const& x{point[0]};
    T const& y{point[1]};
    T const& z{point[2]};

    // TODO(Jack): Can/should we replace this with eigen matrix operations?
    T const u{(fx * x / z) + cx};
    T const v{(fy * y / z) + cy};

    return {u, v};
}

// TODO(Jack): Can we use the se3 type here?
template <typename T>
Eigen::Vector<T, 3> TransformPoint(Eigen::Vector<T, 6> const& tf, Eigen::Vector<T, 3> const& point) {
    Eigen::Vector<T, 3> const axis_angle{tf.topRows(3)};
    Eigen::Vector<T, 3> transformed_point;
    ceres::AngleAxisRotatePoint(axis_angle.data(), point.data(), transformed_point.data());

    Eigen::Vector<T, 3> const translation{tf.bottomRows(3)};
    transformed_point += translation;

    return transformed_point;
}

}  // namespace reprojection_calibration::pnp

using namespace reprojection_calibration::pnp;

TEST(NonlinearRefinement, TestTransformPointsTranslation) {
    Eigen::Vector<double, 6> const tf{0, 0, 0, 1, 2, 3};  // Translation only
    Eigen::Vector3d const point{5, 10, 15};

    Eigen::Vector<double, 3> const transformed_point{TransformPoint(tf, point)};

    EXPECT_FLOAT_EQ(transformed_point[0], 6.0);
    EXPECT_FLOAT_EQ(transformed_point[1], 12.0);
    EXPECT_FLOAT_EQ(transformed_point[2], 18.0);
}

TEST(NonlinearRefinement, TestTransformPointsRotation) {
    Eigen::Vector<double, 6> const tf{0, 0, M_PI_2, 0, 0, 0};  // Rotation only
    Eigen::Vector<double, 3> const point{5, 10, 15};

    Eigen::Vector<double, 3> const transformed_point{TransformPoint(tf, point)};

    EXPECT_FLOAT_EQ(transformed_point[0], -10.0);
    EXPECT_FLOAT_EQ(transformed_point[1], 5.0);
    EXPECT_FLOAT_EQ(transformed_point[2], 15.0);
}

TEST(NonlinearRefinement, TestPinholeProjection) {
    Eigen::Array<double, 4, 1> const pinhole_intrinsics{600, 600, 360, 240};

    Eigen::Vector3d const center_point{0, 0, 10};
    Eigen::Vector2d const center_pixel{PinholeProjection(pinhole_intrinsics.data(), center_point)};
    EXPECT_TRUE(center_pixel.isApprox(Eigen::Vector2d{pinhole_intrinsics[2], pinhole_intrinsics[3]}));

    Eigen::Vector3d const left_point{-360, 0, 600};
    Eigen::Vector2d const left_pixel{PinholeProjection(pinhole_intrinsics.data(), left_point)};
    EXPECT_TRUE(left_pixel.isApprox(Eigen::Vector2d{0, pinhole_intrinsics[3]}));

    // NOTE(Jack): I am not 100% sure if the pixels at x=720 should actually be part of the valid pixel group! These are
    // technically one out of bounds based on the discretization of the pixels in the image. Our camera model currently
    // does no bounds checking so this is not detected. If we add "valid pixel checking" these tests at the right and
    // bottom might change as they are invalid pixels.
    Eigen::Vector3d const right_point{360, 0, 600};
    Eigen::Vector2d const right_pixel{PinholeProjection(pinhole_intrinsics.data(), right_point)};
    EXPECT_TRUE(right_pixel.isApprox(Eigen::Vector2d{720, pinhole_intrinsics[3]}));

    Eigen::Vector3d const top_point{0, -240, 600};
    Eigen::Vector2d const top_pixel{PinholeProjection(pinhole_intrinsics.data(), top_point)};
    EXPECT_TRUE(top_pixel.isApprox(Eigen::Vector2d{pinhole_intrinsics[2], 0}));

    Eigen::Vector3d const bottom_point{0, 240, 600};
    Eigen::Vector2d const bottom_pixel{PinholeProjection(pinhole_intrinsics.data(), bottom_point)};
    EXPECT_TRUE(bottom_pixel.isApprox(Eigen::Vector2d{pinhole_intrinsics[2], 480}));
}