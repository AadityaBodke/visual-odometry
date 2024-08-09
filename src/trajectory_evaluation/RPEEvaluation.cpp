#include "trajectory_evaluation/RPEEvaluation.h"
#include <stdexcept>

RPEEvaluation::RPEEvaluation(double delta) : delta(delta) {}

double RPEEvaluation::evaluate(const std::vector<cv::Mat>& estimatedTrajectory,
                               const std::vector<cv::Mat>& groundTruthTrajectory) const {
    if (estimatedTrajectory.size() != groundTruthTrajectory.size()) {
        throw std::invalid_argument("Trajectory sizes do not match.");
    }

    double totalError = 0.0;
    size_t count = 0;

    for (size_t i = 0; i < estimatedTrajectory.size() - static_cast<size_t>(delta); ++i) {
        cv::Mat errorMat = (groundTruthTrajectory[i].inv() * groundTruthTrajectory[i + static_cast<size_t>(delta)]).inv() *
                           (estimatedTrajectory[i].inv() * estimatedTrajectory[i + static_cast<size_t>(delta)]);
        double error = cv::norm(errorMat.col(3).rowRange(0, 3));
        totalError += error;
        count++;
    }

    return totalError / count;
}

std::string RPEEvaluation::getName() const {
    return "Relative Pose Error (RPE)";
}
