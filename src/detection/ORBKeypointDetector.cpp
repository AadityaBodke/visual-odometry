#include "detection/ORBKeypointDetector.h"

ORBKeypointDetector::ORBKeypointDetector(const ORBConfig& config) {
    orb = cv::ORB::create(config.nfeatures, config.scaleFactor, config.nlevels, config.edgeThreshold,
                          config.firstLevel, config.WTA_K, config.scoreType, config.patchSize, config.fastThreshold);
}

void ORBKeypointDetector::detect(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints) {
    orb->detect(image, keypoints);
}
