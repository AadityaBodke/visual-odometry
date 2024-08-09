#include "homogenization/ANMSHomogenizer.h"
#include <stdexcept>

ANMSHomogenizer::ANMSHomogenizer(const ANMSConfig& config) : config(config) {}

std::vector<cv::KeyPoint> ANMSHomogenizer::homogenizeKeypoints(const std::vector<cv::KeyPoint>& keypoints, const cv::Size& imageSize) {
    if (keypoints.empty()) {
        throw std::runtime_error("Input keypoints are empty.");
    }

    if (config.method == "topn") {
        return topN(keypoints, config.numRetPoints);
    } else if (config.method == "brown") {
        return brownANMS(keypoints, config.numRetPoints);
    } else if (config.method == "sdc") {
        return sdc(keypoints, config.numRetPoints, config.tolerance, imageSize.width, imageSize.height);
    } else if (config.method == "kdtree") {
        return kdTree(keypoints, config.numRetPoints, config.tolerance, imageSize.width, imageSize.height);
    } else if (config.method == "rangetree") {
        return rangeTree(keypoints, config.numRetPoints, config.tolerance, imageSize.width, imageSize.height);
    } else if (config.method == "ssc") {
        return ssc(keypoints, config.numRetPoints, config.tolerance, imageSize.width, imageSize.height);
    } else {
        throw std::runtime_error("Unknown ANMS method: " + config.method);
    }
}

cv::Mat ANMSHomogenizer::homogenizeDescriptors(const cv::Mat& descriptors) {
    // For now, just return the descriptors as is. Implement specific homogenization if needed.
    return descriptors.clone();
}
