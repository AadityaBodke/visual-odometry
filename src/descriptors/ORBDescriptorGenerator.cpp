#include "descriptors/ORBDescriptorGenerator.h"
#include <stdexcept>

ORBDescriptorGenerator::ORBDescriptorGenerator(const ORBDescriptorConfig& config) {
    try {
        orb = cv::ORB::create(config.nfeatures, config.scaleFactor, config.nlevels, config.edgeThreshold,
                              config.firstLevel, config.WTA_K, config.scoreType, config.patchSize, config.fastThreshold);
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to create ORB descriptor generator: " + std::string(e.what()));
    }
}

void ORBDescriptorGenerator::compute(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) {
    if (image.empty()) {
        throw std::runtime_error("Input image is empty.");
    }
    try {
        orb->compute(image, keypoints, descriptors);
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to compute ORB descriptors: " + std::string(e.what()));
    }
}
