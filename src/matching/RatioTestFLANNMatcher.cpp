#include "matching/RatioTestFLANNMatcher.h"
#include <stdexcept>

RatioTestFLANNMatcher::RatioTestFLANNMatcher(const RatioTestFLANNMatcherConfig& config)
    : ratio_thresh(config.ratio_thresh) {
    try {
        flannMatcher = cv::makePtr<cv::FlannBasedMatcher>(config.indexParams, config.searchParams);
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to create RatioTestFLANNMatcher: " + std::string(e.what()));
    }
}

void RatioTestFLANNMatcher::match(const cv::Mat& descriptors1, const cv::Mat& descriptors2, std::vector<cv::DMatch>& matches) {
    if (descriptors1.empty() || descriptors2.empty()) {
        throw std::runtime_error("One or both of the descriptor matrices are empty.");
    }
    try {
        cv::Mat descriptors1_32f, descriptors2_32f;
        descriptors1.convertTo(descriptors1_32f, CV_32F);
        descriptors2.convertTo(descriptors2_32f, CV_32F);

        std::vector<std::vector<cv::DMatch>> knnMatches;
        flannMatcher->knnMatch(descriptors1_32f, descriptors2_32f, knnMatches, 2);

        for (const auto& knnMatch : knnMatches) {
            if (knnMatch.size() == 2 && knnMatch[0].distance < ratio_thresh * knnMatch[1].distance) {
                matches.push_back(knnMatch[0]);
            }
        }
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to match descriptors: " + std::string(e.what()));
    }
}
