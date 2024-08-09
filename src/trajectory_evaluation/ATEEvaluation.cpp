#include "trajectory_evaluation/ATEEvaluation.h"
#include <stdexcept>

ATEEvaluation::ATEEvaluation(double scale) : scale(scale) {}

double ATEEvaluation::evaluate(const std::vector<cv::Mat>& estimatedTrajectory,
                               const std::vector<cv::Mat>& groundTruthTrajectory) const {
    if (estimatedTrajectory.size() != groundTruthTrajectory.size()) {
        throw std::invalid_argument("Trajectory sizes do not match.");
    }

    double totalError = 0.0;
    for (size_t i = 0; i < estimatedTrajectory.size(); ++i) {
        cv::Mat errorMat = groundTruthTrajectory[i].inv() * estimatedTrajectory[i];
        double error = cv::norm(errorMat.col(3).rowRange(0, 3)) * scale;
        totalError += error;
    }

    return totalError / estimatedTrajectory.size();
}

std::string ATEEvaluation::getName() const {
    return "Absolute Trajectory Error (ATE)";
}
