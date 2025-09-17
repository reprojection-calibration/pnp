#include "pnp/pnp.hpp"

#include <gtest/gtest.h>

#include "multiple_view_geometry_data_generator.hpp"

using namespace reprojection_calibration::pnp;

TEST(Pnp, TestPnp) {
    MvgFrameGenerator const generator{MvgFrameGenerator()};
    for (size_t i{0}; i < 20; ++i) {
        MvgFrame const frame_i{generator.Generate()};

        PnpResult const pnp_result{Pnp(frame_i.pixels, frame_i.points)};
        EXPECT_TRUE(std::holds_alternative<Eigen::Isometry3d>(pnp_result));

        Eigen::Isometry3d const pose_i{std::get<Eigen::Isometry3d>(pnp_result)};
        EXPECT_TRUE(pose_i.isApprox(FromSe3(frame_i.pose)));
    }
}

TEST(Pnp, MismatchedCorrespondence) {
    Eigen::MatrixX2d const four_pixels(4, 2);
    Eigen::MatrixX3d const five_points(5, 3);
    PnpResult const pnp_result{Pnp(four_pixels, five_points)};

    EXPECT_TRUE(std::holds_alternative<PnpStatusCode>(pnp_result));
    PnpStatusCode const pnp_status_code{std::get<PnpStatusCode>(pnp_result)};
    EXPECT_EQ(pnp_status_code, PnpStatusCode::MismatchedCorrespondence);
}

TEST(Pnp, NotEnoughPoints) {
    Eigen::MatrixX2d const five_pixels(5, 2);
    Eigen::MatrixX3d const five_points(5, 3);
    PnpResult const pnp_result{Pnp(five_pixels, five_points)};

    EXPECT_TRUE(std::holds_alternative<PnpStatusCode>(pnp_result));
    PnpStatusCode const pnp_status_code{std::get<PnpStatusCode>(pnp_result)};
    EXPECT_EQ(pnp_status_code, PnpStatusCode::NotEnoughPoints);
}
