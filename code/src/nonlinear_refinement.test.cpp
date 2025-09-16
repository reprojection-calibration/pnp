#include "nonlinear_refinement.hpp"

#include <ceres/ceres.h>  // TODO(Jack): Add header ordering to the clang format, this is madness!
#include <ceres/rotation.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "multiple_view_geometry_data_generator.hpp"

namespace reprojection_calibration::pnp {

Eigen::Matrix3d ToK(Eigen::Array<double, 4, 1> const& array) {
    Eigen::Matrix3d K{Eigen::Matrix3d::Identity()};
    K(0, 0) = array[0];
    K(1, 1) = array[1];
    K(0, 2) = array[2];
    K(1, 2) = array[3];

    return K;
};

// TODO(Jack): Increase consistency of the use of SE3 or se3 - we really only introduced the se3 in the general source
// code so that we could test pose values easily. Unless we are in the core optimization logic or testing we should be
// using SE3. Or at least that is my idea right now :)
// TODO(Jack): A function that converts from the matrix and array representation of K easily
std::tuple<Eigen::Isometry3d, Eigen::Matrix3d> NonlinearRefinement(Eigen::MatrixX2d const& pixels,
                                                                   Eigen::MatrixX3d const& points,
                                                                   Eigen::Isometry3d const& initial_pose,
                                                                   Eigen::Matrix3d const& initial_K) {
    Se3 pose_to_optimize{ToSe3(initial_pose)};
    Eigen::Array<double, 4, 1> pinhole_intrinsics_to_optimize{initial_K(0, 0), initial_K(1, 1), initial_K(0, 2),
                                                              initial_K(1, 2)};

    ceres::Problem problem;
    for (Eigen::Index i{0}; i < pixels.rows(); ++i) {
        ceres::CostFunction* const cost_function{PinholeCostFunction::Create(pixels.row(i), points.row(i))};
        problem.AddResidualBlock(cost_function, nullptr, pinhole_intrinsics_to_optimize.data(),
                                 pose_to_optimize.data());
    }

    // TODO(Jack): Law of useful return states that we should probably be returning this diagnostic information so that
    // people can diagnose failures.
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    return {FromSe3(pose_to_optimize), ToK(pinhole_intrinsics_to_optimize)};
}

}  // namespace reprojection_calibration::pnp

using namespace reprojection_calibration::pnp;

TEST(NonlinearRefinement, xxx) {
    MvgFrameGenerator const generator{MvgFrameGenerator()};
    MvgFrame const frame{generator.Generate()};
    Eigen::Array<double, 4, 1> const pinhole_intrinsics{600, 600, 360, 240};  // This should be retrieved from MVG!

    // Note the inverse on the tf!!!
    auto const [tf, K]{
        NonlinearRefinement(frame.pixels, frame.points, FromSe3(frame.pose).inverse(), ToK(pinhole_intrinsics))};

    EXPECT_TRUE(tf.isApprox(FromSe3(frame.pose).inverse())) << "Optimization result:\n"
                                                            << ToSe3(tf) << "\noptimization input:\n"
                                                            << ToSe3(FromSe3(frame.pose).inverse());
    EXPECT_TRUE(K.isApprox(ToK(pinhole_intrinsics))) << "Optimization result:\n"
                                                     << K << "\noptimization input:\n"
                                                     << ToK(pinhole_intrinsics);
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