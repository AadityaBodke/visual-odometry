#include "acquisition/TUMDatasetImageAcquisition.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>

TUMDatasetImageAcquisition::TUMDatasetImageAcquisition(const std::string& datasetPath, const std::string& rgbFile, const std::string& depthFile) 
    : datasetPath(datasetPath), rgbFile(rgbFile), depthFile(depthFile), currentIndex(0) {
    try {
        loadFilenames();
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to initialize TUMDatasetImageAcquisition: " + std::string(e.what()));
    }
    if (imageFiles.empty() || depthFiles.empty()) {
        throw std::runtime_error("No image or depth files found in the dataset directory.");
    }
}

std::vector<std::string> TUMDatasetImageAcquisition::readFileList(const std::string& filename, std::vector<double>& timestamps) {
    std::vector<std::string> fileList;
    std::ifstream file(datasetPath + "/" + filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string timestampStr, filePath;
        ss >> timestampStr >> filePath;
        fileList.push_back(datasetPath + "/" + filePath);
        timestamps.push_back(std::stod(timestampStr));
    }

    return fileList;
}

void TUMDatasetImageAcquisition::loadFilenames() {
    imageFiles = readFileList(rgbFile, timestamps);
    depthFiles = readFileList(depthFile, timestamps);

    if (imageFiles.size() != depthFiles.size()) {
        throw std::runtime_error("Mismatch between the number of image and depth files.");
    }
}

bool TUMDatasetImageAcquisition::acquire(cv::Mat& image, cv::Mat& depth) {
    if (currentIndex >= imageFiles.size() || currentIndex >= depthFiles.size()) {
        std::cerr << "No more images to acquire." << std::endl;
        return false;
    }

    image = cv::imread(imageFiles[currentIndex], cv::IMREAD_COLOR);
    depth = cv::imread(depthFiles[currentIndex], cv::IMREAD_UNCHANGED);

    if (image.empty() || depth.empty()) {
        throw std::runtime_error("Failed to read image or depth file at index " + std::to_string(currentIndex));
    }

    currentIndex++;
    return true;
}

double TUMDatasetImageAcquisition::getCurrentTimestamp() const {
    return timestamps[currentIndex - 1];
}
