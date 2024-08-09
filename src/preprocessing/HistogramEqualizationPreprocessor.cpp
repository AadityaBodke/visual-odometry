#include "preprocessing/HistogramEqualizationPreprocessor.h"

void HistogramEqualizationPreprocessor::preprocess(const cv::Mat& input, cv::Mat& output) {
    if (input.channels() == 3) {
        cv::Mat gray;
        cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
        cv::equalizeHist(gray, output);
    } else {
        cv::equalizeHist(input, output);
    }
}
