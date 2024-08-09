#include "detection/FASTKeypointDetector.h"

FASTKeypointDetector::FASTKeypointDetector(const FASTConfig& config) : config(config) {}

void FASTKeypointDetector::detect(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints) {
    cv::FAST(image, keypoints, config.threshold, config.nonmaxSuppression);
}
