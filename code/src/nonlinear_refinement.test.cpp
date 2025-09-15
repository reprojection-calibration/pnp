#include <ceres/ceres.h>  // TODO(Jack): Add header ordering to the clang format, this is madness!
#include <ceres/rotation.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "multiple_view_geometry_data_generator.hpp"

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
    explicit PinholeCostFunction(Eigen::Vector2d const& pixel, Eigen::Vector3d const& point)
        : pixel_{pixel}, point_{point} {}

    // This is the contact line were ceres requirements for using raw pointers hits our desire to use more expressive
    // eigen types. That is why here in the operator() we find the usage of the Eigen::Map class.
    template <typename T>
    bool operator()(T const* const pinhole_intrinsics, T const* const input_pose, T* const residual) const {
        // WARN(Jack): Is there a way to canonically force the program to make sure the right memory is allocated and
        // referenced to by these raw pointers? Ceres forces this pointer interface so I don't think we can easily do
        // that but consider how we  can design the program to handle that automatically.
        Eigen::Map<Eigen::Vector<T, 6> const> pose(input_pose);
        Eigen::Vector<T, 3> const point_co{TransformPoint<T>(pose, point_.cast<T>())};

        Eigen::Vector<T, 2> const pixel{PinholeProjection(pinhole_intrinsics, point_co)};

        residual[0] = T(pixel_[0]) - pixel[0];
        residual[1] = T(pixel_[1]) - pixel[1];

        return true;
    }

    static ceres::CostFunction* Create(Eigen::Vector2d const& pixel, Eigen::Vector3d const& point) {
        return new ceres::AutoDiffCostFunction<PinholeCostFunction, 2, 4, 6>(new PinholeCostFunction(pixel, point));
    }

    Eigen::Vector2d pixel_;
    Eigen::Vector3d point_;
};

}  // namespace reprojection_calibration::pnp

using namespace reprojection_calibration::pnp;

TEST(NonlinearRefinement, xxx) {
    MvgFrameGenerator const generator{MvgFrameGenerator()};
    MvgFrame frame{generator.Generate()};  // This should be const!

    std::array<double, 4> pinhole_intrinsics{600, 600, 360, 240};
    Se3 pose{frame.pose};

    ceres::Problem problem;
    for (Eigen::Index i{0}; i < frame.pixels.rows(); ++i) {
        ceres::CostFunction* const cost_function{PinholeCostFunction::Create(frame.pixels.row(i), frame.points.row(i))};
        problem.AddResidualBlock(cost_function, nullptr /* squared loss */, pinhole_intrinsics.data(), pose.data());
    }
    EXPECT_FLOAT_EQ(2.0, 0.0);
}

// We test that a point on the optical axis (0,0,z) projects to the center of the image (cx, cy) and has residual zero.
TEST(NonlinearRefinement, TestPinholeCostFunctionResidual) {
    // NOTE(Jack): The reason that we have these ugly unfamiliar std::arrays and calls to .data(), but nowhere else, is
    // because in this test we are essentially manually simulating all the magic that Ceres will do behind the scenes
    // for us, managing the memory and passing arguments etc. during the optimization process. It is my hope and vision
    // that these raw pointers etc. can be limited to testing, and not actually filter into the rest of the code if
    // handled smartly (ex. using Eigen::Map and Eigen::Ref).
    std::array<double, 4> const pinhole_intrinsics{600, 600, 360, 240};
    Eigen::Vector2d const pixel{pinhole_intrinsics[2], pinhole_intrinsics[3]};
    Eigen::Vector3d const point{0, 0, 10};  // Point that will project to the center of the image
    PinholeCostFunction const cost_function{pixel, point};

    std::array<double, 6> const pose{0, 0, 0, 0, 0, 0};
    std::array<double, 2> residual{};
    cost_function.operator()<double>(pinhole_intrinsics.data(), pose.data(), residual.data());

    EXPECT_FLOAT_EQ(residual[0], 0.0);
    EXPECT_FLOAT_EQ(residual[1], 0.0);
}

// NOTE: We do not test cost_function->Evaluate() in the following test because
// allocating the memory of the input pointers takes some thought, but cost_function->Evaluate()
// should be tested when there is interest and time :)
TEST(NonlinearRefinement, TestPinholeCostFunctionCreate) {
    Eigen::Vector2d const pixel{360, 240};
    Eigen::Vector3d const point{0, 0, 10};
    ceres::CostFunction const* const cost_function{PinholeCostFunction::Create(pixel, point)};

    EXPECT_EQ(std::size(cost_function->parameter_block_sizes()), 2);
    EXPECT_EQ(cost_function->parameter_block_sizes()[0], 4);  // pinhole intrinsics
    EXPECT_EQ(cost_function->parameter_block_sizes()[1], 6);  // camera pose
    EXPECT_EQ(cost_function->num_residuals(), 2);

    // WARN: The plain use of the ParameterCostFunction::Create() method does not
    // ensure the destruction of the resource allocated by the Create() method.
    // Therefore we need to call delete here, extra. Ceres normally handles this
    // when it uses create. But we do it here explicitly and without a class that
    // manages the memory allocation just cause this is a small part of a small
    // test. If we end up really ever using it outside of Ceres then we need to do
    // RAII.
    delete cost_function;
}

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
    // bottom might change as they are invalid pixels by one pixel.
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