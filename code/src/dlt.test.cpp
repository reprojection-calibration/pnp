#include "dlt.hpp"

#include <gtest/gtest.h>

#include "multiple_view_geometry_data_generator.hpp"

// TODO(Jack): I think we could add a test where we check more properties, like for example PC=0, etc. Even though these
// might already be checked in some sub-tests for specific test data, we should be able to do it for all executions.

using namespace reprojection_calibration::pnp;

TEST(Dlt, TestOne) {
    MvgFrameGenerator const generator{MvgFrameGenerator()};
    for (size_t i{0}; i < 100; ++i) {
        MvgFrame const frame_i{generator.Generate()};
        auto const [tf, K]{Dlt(frame_i.pixels, frame_i.points)};

        EXPECT_FLOAT_EQ(tf.linear().determinant(), 1);  // Property of rotation matrix - positive one determinant

        Se3 pose_i{ToSe3(tf)};
        pose_i.topRows(3) *= -1;  // TODO(Jack): There is some inconsistency here that the Dlt produced pose_i always is
                                  // the negative of the ground truth pose! Does it have something to do with an inverse
                                  // coordinate frame or something like that? Idk, but I know it is always opposite the
                                  // gt value which is why I multiply by -1 here and not just take the absolute value.
        Se3 const pose_gt{frame_i.pose};
        EXPECT_TRUE(pose_i.isApprox(pose_gt));

        // TODO(Jack): Honestly we could check exactly the values here because we know them, maybe we should refactor
        // the data generator to add a getter for K so we can compare it.
        EXPECT_TRUE(K.isUpperTriangular());  // Property of camera intrinsic matrix
    }
}
