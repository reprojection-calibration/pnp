#include <gtest/gtest.h>

#include "pnp/pnp.hpp"

using namespace reprojection_calibration::pnp;

TEST(TestPnpErrorHandling, MismatchedCorrespondence) {
    Eigen::MatrixX2d const four_pixels(4, 2);
    Eigen::MatrixX3d const five_points(5, 3);
    PnpResult const pnp_result{Pnp(four_pixels, five_points)};

    EXPECT_TRUE(std::holds_alternative<PnpStatusCode>(pnp_result));
    PnpStatusCode const pnp_status_code{std::get<PnpStatusCode>(pnp_result)};
    EXPECT_EQ(PnpStatusCode::MismatchedCorrespondence, pnp_status_code);
}

TEST(TestPnpErrorHandling, NotEnoughPoints) {
    Eigen::MatrixX2d const five_pixels(5, 2);
    Eigen::MatrixX3d const five_points(5, 3);
    PnpResult const pnp_result{Pnp(five_pixels, five_points)};

    EXPECT_TRUE(std::holds_alternative<PnpStatusCode>(pnp_result));
    PnpStatusCode const pnp_status_code{std::get<PnpStatusCode>(pnp_result)};
    EXPECT_EQ(PnpStatusCode::NotEnoughPoints, pnp_status_code);
}
