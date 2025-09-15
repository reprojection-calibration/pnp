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
// NOTE(Jack): We use Eigen::Ref here so we can pass both maps (in the PinholeCostFunction.operator()) and the direct
// types (in the testing for example).
template <typename T>
Eigen::Vector<T, 3> TransformPoint(Eigen::Ref<Eigen::Vector<T, 6> const> const& tf,
                                   Eigen::Ref<Eigen::Vector<T, 3> const> const& point) {
    Eigen::Vector<T, 3> const axis_angle{tf.topRows(3)};
    Eigen::Vector<T, 3> transformed_point;
    ceres::AngleAxisRotatePoint(axis_angle.data(), point.data(), transformed_point.data());

    Eigen::Vector<T, 3> const translation{tf.bottomRows(3)};
    transformed_point += translation;

    return transformed_point;
}

// Relation between eigen and ceres: https://groups.google.com/g/ceres-solver/c/7ZH21XX6HWU
struct PinholeCostFunction {
    explicit PinholeCostFunction(double const u, double const v) : u_{u}, v_{v} {}

    // This is the contact line were ceres requirements for using raw pointers hits our desire to use more expressive
    // eigen types. That is why here in the operator() we find the usage of the Eigen::Map class.
    template <typename T>
    bool operator()(T const* const pinhole_intrinsics, T const* const input_pose, T const* const input_point,
                    T* const residual) const {
        // WARN(Jack): Is there a way to canonically force the program to make sure the right memory is allocated and
        // referenced to by these raw pointers? Ceres forces this pointer interface so I don't think we can easily do
        // that but consider how we  can design the program to handle that automatically.
        Eigen::Map<Eigen::Vector<T, 6> const> pose(input_pose);
        Eigen::Map<Eigen::Vector<T, 3> const> point(input_point);
        Eigen::Vector<T, 3> const point_co{TransformPoint<T>(pose, point)};

        Eigen::Vector<T, 2> const pixel{PinholeProjection(pinhole_intrinsics, point_co)};

        residual[0] = T(u_) - pixel[0];
        residual[1] = T(v_) - pixel[1];

        return true;
    }

    static ceres::CostFunction* Create(double const u, double const v) {
        return new ceres::AutoDiffCostFunction<PinholeCostFunction, 2, 4, 6, 3>(new PinholeCostFunction(u, v));
    }

    double u_;
    double v_;
};

}  // namespace reprojection_calibration::pnp

using namespace reprojection_calibration::pnp;

TEST(NonlinearRefinement, TestTransformPointsTranslation) {
    Eigen::Vector<double, 6> const tf{0, 0, 0, 1, 2, 3};  // Translation only
    Eigen::Vector3d const point{5, 10, 15};

    Eigen::Vector<double, 3> const transformed_point{TransformPoint<double>(tf, point)};

    EXPECT_FLOAT_EQ(transformed_point[0], 6.0);
    EXPECT_FLOAT_EQ(transformed_point[1], 12.0);
    EXPECT_FLOAT_EQ(transformed_point[2], 18.0);
}

TEST(NonlinearRefinement, TestTransformPointsRotation) {
    Eigen::Vector<double, 6> const tf{0, 0, M_PI_2, 0, 0, 0};  // Rotation only
    Eigen::Vector<double, 3> const point{5, 10, 15};

    Eigen::Vector<double, 3> const transformed_point{TransformPoint<double>(tf, point)};

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