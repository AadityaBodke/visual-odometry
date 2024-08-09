#include "trajectory_evaluation/TrajectoryEvaluator.h"

TrajectoryEvaluator::TrajectoryEvaluator(const std::vector<cv::Mat>& groundTruthTrajectory)
    : groundTruthTrajectory(groundTruthTrajectory) {}

double TrajectoryEvaluator::computeATE(const std::vector<cv::Mat>& estimatedTrajectory) {
    double error = 0.0;
    for (size_t i = 0; i < estimatedTrajectory.size(); ++i) {
        cv::Mat diff = groundTruthTrajectory[i].col(3) - estimatedTrajectory[i].col(3);
        error += cv::norm(diff);
    }
    return error / estimatedTrajectory.size();
}

double TrajectoryEvaluator::computeRPE(const std::vector<cv::Mat>& estimatedTrajectory) {
    double error = 0.0;
    for (size_t i = 1; i < estimatedTrajectory.size(); ++i) {
        cv::Mat gtDelta = groundTruthTrajectory[i-1].inv() * groundTruthTrajectory[i];
        cv::Mat estDelta = estimatedTrajectory[i-1].inv() * estimatedTrajectory[i];
        cv::Mat delta = gtDelta.inv() * estDelta;
        cv::Vec3d deltaTrans = delta.col(3).rowRange(0, 3);
        error += cv::norm(deltaTrans);
    }
    return error / (estimatedTrajectory.size() - 1);
}
