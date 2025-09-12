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

    MvgFrame Generate() const;

    static Eigen::Vector3d TrackPoint(Eigen::Vector3d const& point, Eigen::Vector3d const& position);

    static Eigen::MatrixX2d Project(Eigen::MatrixX4d const& points, Eigen::Matrix3d const& K);

   private:
    Eigen::MatrixX3d points_;  // TODO(Jack): Should we add the w postfix to specify the coordinate frame?
    Eigen::Matrix3d K_;
};

MvgFrameGenerator::MvgFrameGenerator(Eigen::MatrixX3d const& points, Eigen::Matrix3d const& K)
    : points_{points}, K_{K} {}

MvgFrame MvgFrameGenerator::Generate() const {
    // Generate pose
    Eigen::Vector3d camera_position{Eigen::Vector3d::Random()};
    camera_position.normalize();
    double const sphere_radius{10 * points_.rowwise().norm().mean()};  // Arbitrary choice to multiply by the scalar
    camera_position *= sphere_radius;
    camera_position += points_.colwise().mean();  // Camera position should be centered on pointcloud center

    // Because the points can be arbitrarily set via the constructor we need to set the position being viewed
    // dynamically based on the pointclouds centroid.
    Eigen::Vector3d const origin{points_.colwise().mean()};
    Eigen::Vector3d const camera_direction{TrackPoint(origin, camera_position)};

    // TODO(Jack): Can we construct this directly or do we need to use << - this is done in more than one place.
    Se3 viewing_pose;
    viewing_pose << camera_direction, camera_position;

    // Project points
    Eigen::Isometry3d const tf_co_w{FromSe3(viewing_pose)};
    Eigen::MatrixX4d const points_homog_co{(tf_co_w * points_.rowwise().homogeneous().transpose()).transpose()};
    // TODO: use static method!!!!
    Eigen::MatrixX2d const pixels{(K_ * points_homog_co.leftCols(3).transpose()).transpose().rowwise().hnormalized()};
    std::cout << pixels << std::endl;

    // Construct return value

    return MvgFrame{};
}

Eigen::Vector3d MvgFrameGenerator::TrackPoint(Eigen::Vector3d const& origin, Eigen::Vector3d const& camera_position) {
    // WARN(Jack): In testing this function I found that when the x and y coordinates of the origin and camera_position
    // are the same, i.e. the points  are aligned along the z-plane, the algorithm returns nans. There is probably a
    // simple test we can use to check this condition to avoid the nans, but for now we will just take this risk and
    // hope that we never have the same x and y coordinates for both camera and origin. This current implementation also
    // does not explicitly handle the case where the two points are the same, I assume that takes some error handling to
    // prevent nans as well.
    Eigen::Vector3d const origin_direction{(origin - camera_position).normalized()};
    Eigen::Vector3d const camera_forward_direction{0, 0, 1};

    double const angle{std::acos(origin_direction.transpose() * camera_forward_direction)};

    Eigen::Vector3d const cross_product{camera_forward_direction.cross(origin_direction)};
    Eigen::Vector3d const axis{cross_product / cross_product.norm()};

    Eigen::Vector3d const tracking_direction{angle * axis};

    return tracking_direction;
}

Eigen::MatrixX2d MvgFrameGenerator::Project(Eigen::MatrixX4d const& points, Eigen::Matrix3d const& K) {
    Eigen::MatrixX2d const pixels{(K * points.leftCols(3).transpose()).transpose().rowwise().hnormalized()};

    return pixels;
}

}  // namespace reprojection_calibration::pnp

using namespace reprojection_calibration::pnp;

TEST(HHH, TestMvgFrameGenerator) {
    Eigen::MatrixX3d const test_points{{0.00, 0.00, 5.00},   {1.00, 1.00, 5.00},   {-1.00, -1.00, 5.00},
                                       {2.00, -1.00, 10.00}, {-2.00, 1.00, 10.00}, {0.50, -0.50, 7.00}};

    Eigen::Matrix3d const K{{600, 0, 360}, {0, 600, 240}, {0, 0, 1}};

    auto const gen{MvgFrameGenerator(test_points, K)};
    auto const frame_i{gen.Generate()};
    auto const frame_j{gen.Generate()};
    auto const frame_k{gen.Generate()};

    EXPECT_EQ(1, 2);
}

TEST(HHH, TestTrackPoint) {
    Eigen::Vector3d tracking_direction{MvgFrameGenerator::TrackPoint({0, 0, 0}, {2, 0, 0})};
    EXPECT_TRUE(tracking_direction.isApprox(Eigen::Vector3d{0, -EIGEN_PI / 2.0, 0}));

    tracking_direction = MvgFrameGenerator::TrackPoint({0, 0, 0}, {0, 2, 0});
    EXPECT_TRUE(tracking_direction.isApprox(Eigen::Vector3d{EIGEN_PI / 2.0, 0, 0}));

    tracking_direction = MvgFrameGenerator::TrackPoint({0, 0, 0}, {2, 2, 2});
    EXPECT_TRUE(tracking_direction.isApprox(Eigen::Vector3d{1.54593, -1.54593, 0}, 1e-4));  // Heuristic
}

TEST(HHH, TestProject) {
    Eigen::MatrixX3d const points_w{{0.00, 0.00, 5.00},   {1.00, 1.00, 5.00},   {-1.00, -1.00, 5.00},
                                    {2.00, -1.00, 10.00}, {-2.00, 1.00, 10.00}, {0.50, -0.50, 7.00}};
    Eigen::Matrix3d const K{{600, 0, 360}, {0, 600, 240}, {0, 0, 1}};

    Eigen::Isometry3d const tf_co_w{Eigen::Isometry3d::Identity()};
    Eigen::MatrixX4d const points_homog_co{(tf_co_w * points_w.rowwise().homogeneous().transpose()).transpose()};
    Eigen::MatrixX2d const pixels{MvgFrameGenerator::Project(points_homog_co, K)};
    std::cout << pixels << std::endl;
    Eigen::MatrixX2d const test_pixels{{360.00, 240.00}, {480.00, 360.00}, {240.00, 120.00},
                                       {480.00, 180.00}, {240.00, 300.00}, {402.857, 197.144}};
    ASSERT_TRUE(pixels.isApprox(test_pixels, 1e-3));
}
