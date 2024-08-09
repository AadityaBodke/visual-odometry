#include "pose_integration/PoseIntegration.h"

void BasicPoseIntegration::integratePose(const cv::Mat& R, const cv::Mat& t) {
    cv::Mat Rt = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(Rt(cv::Rect(0, 0, 3, 3)));
    t.copyTo(Rt(cv::Rect(3, 0, 1, 3)));
    
    currentPose = currentPose * Rt;
    trajectory.push_back(currentPose.clone());
}

std::vector<cv::Mat> BasicPoseIntegration::getTrajectory() const {
    return trajectory;
}
