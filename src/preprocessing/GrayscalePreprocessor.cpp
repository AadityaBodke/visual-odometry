#include "preprocessing/GrayscalePreprocessor.h"

void GrayscalePreprocessor::preprocess(const cv::Mat& input, cv::Mat& output) {
    cv::cvtColor(input, output, cv::COLOR_BGR2GRAY);
}
