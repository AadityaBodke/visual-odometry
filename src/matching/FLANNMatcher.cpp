#include "matching/FLANNMatcher.h"
#include <stdexcept>

FLANNMatcher::FLANNMatcher(const FLANNMatcherConfig& config) {
    try {
        flannMatcher = cv::makePtr<cv::FlannBasedMatcher>(config.indexParams, config.searchParams);
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to create FLANNMatcher: " + std::string(e.what()));
    }
}

void FLANNMatcher::match(const cv::Mat& descriptors1, const cv::Mat& descriptors2, std::vector<cv::DMatch>& matches) {
    if (descriptors1.empty() || descriptors2.empty()) {
        throw std::runtime_error("One or both of the descriptor matrices are empty.");
    }
    try {
        cv::Mat descriptors1_32f, descriptors2_32f;
        descriptors1.convertTo(descriptors1_32f, CV_32F);
        descriptors2.convertTo(descriptors2_32f, CV_32F);
        flannMatcher->match(descriptors1_32f, descriptors2_32f, matches);
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to match descriptors: " + std::string(e.what()));
    }
}
