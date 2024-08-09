#include "matching/BruteForceMatcher.h"
#include <stdexcept>

BruteForceMatcher::BruteForceMatcher(const BFMatcherConfig& config) {
    try {
        bfMatcher = cv::BFMatcher::create(config.normType, config.crossCheck);
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to create BFMatcher: " + std::string(e.what()));
    }
}

void BruteForceMatcher::match(const cv::Mat& descriptors1, const cv::Mat& descriptors2, std::vector<cv::DMatch>& matches) {
    if (descriptors1.empty() || descriptors2.empty()) {
        throw std::runtime_error("One or both of the descriptor matrices are empty.");
    }
    try {
        bfMatcher->match(descriptors1, descriptors2, matches);
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to match descriptors: " + std::string(e.what()));
    }
}
