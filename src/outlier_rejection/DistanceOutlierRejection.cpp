#include "outlier_rejection/OutlierRejection.h"
#include <algorithm>

DistanceOutlierRejection::DistanceOutlierRejection(const DistanceConfig& config) : config(config) {}

void DistanceOutlierRejection::rejectOutliers(const std::vector<cv::KeyPoint>& keypoints1,
                                              const std::vector<cv::KeyPoint>& keypoints2,
                                              const std::vector<cv::DMatch>& matches,
                                              std::vector<cv::DMatch>& inliers) {
    if (keypoints1.empty() || keypoints2.empty() || matches.empty()) {
        throw std::runtime_error("Input keypoints or matches are empty.");
    }

    auto min_max = std::minmax_element(matches.begin(), matches.end(),
                                       [](const cv::DMatch &m1, const cv::DMatch &m2) {
                                           return m1.distance < m2.distance;
                                       });
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    inliers.clear();
    for (const auto& match : matches) {
        if (match.distance <= std::max(2 * min_dist, config.distThreshold)) {
            inliers.push_back(match);
        }
    }
}
