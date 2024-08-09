#include "outlier_rejection/OutlierRejection.h"
#include <stdexcept>

RANSACOutlierRejection::RANSACOutlierRejection(const RANSACConfig& config) : config(config) {}

void RANSACOutlierRejection::rejectOutliers(const std::vector<cv::KeyPoint>& keypoints1,
                                            const std::vector<cv::KeyPoint>& keypoints2,
                                            const std::vector<cv::DMatch>& matches,
                                            std::vector<cv::DMatch>& inliers) {
    if (keypoints1.empty() || keypoints2.empty() || matches.empty()) {
        throw std::runtime_error("Input keypoints or matches are empty.");
    }

    std::vector<cv::Point2f> points1, points2;
    for (const auto& match : matches) {
        points1.push_back(keypoints1[match.queryIdx].pt);
        points2.push_back(keypoints2[match.trainIdx].pt);
    }

    std::vector<uchar> status;
    cv::findFundamentalMat(points1, points2, status, cv::FM_RANSAC, config.reprojectionThreshold, config.confidence);

    inliers.clear();
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            inliers.push_back(matches[i]);
        }
    }
}
