#include "pose_estimation/PoseEstimation.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp> // Include this header for cv2eigen

void PNPoseEstimation::estimatePose(const std::vector<cv::KeyPoint>& keypoints1,
                                    const std::vector<cv::KeyPoint>& keypoints2,
                                    const std::vector<cv::DMatch>& matches,
                                    const cv::Mat& K,
                                    Eigen::Matrix4d& pose) {
    std::vector<cv::Point2f> points1, points2;
    for (const auto& match : matches) {
        points1.push_back(keypoints1[match.queryIdx].pt);
        points2.push_back(keypoints2[match.trainIdx].pt);
    }

    cv::Mat R, t;
    cv::Mat E = cv::findEssentialMat(points1, points2, K, cv::RANSAC);
    cv::recoverPose(E, points1, points2, K, R, t);

    Eigen::Matrix3d R_eigen;
    Eigen::Vector3d t_eigen;
    cv::cv2eigen(R, R_eigen); // This should now be correctly recognized
    cv::cv2eigen(t, t_eigen); // This should now be correctly recognized

    pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = R_eigen;
    pose.block<3, 1>(0, 3) = t_eigen;
}
