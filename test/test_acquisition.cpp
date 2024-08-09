#include "acquisition/TUMDatasetImageAcquisition.h"
#include <opencv2/opencv.hpp>
#include <gtest/gtest.h>

TEST(AcquisitionTest, BasicTest) {
    TUMDatasetImageAcquisition acquisition("/Users/aadi/Personal/rgbd_vo_tool/datasets/rgbd_dataset_freiburg1_xyz");
    cv::Mat image, depth;

    ASSERT_TRUE(acquisition.acquire(image, depth));
    EXPECT_FALSE(image.empty());
    EXPECT_FALSE(depth.empty());

    while (acquisition.acquire(image, depth)) {
        EXPECT_FALSE(image.empty());
        EXPECT_FALSE(depth.empty());
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
