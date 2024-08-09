#include "trajectory/TUMTrajectoryEstimator.h"
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <eigen3/Eigen/Geometry>

TrajectoryEstimator::TrajectoryEstimator() {}

void TrajectoryEstimator::addPose(const cv::Mat& R, const cv::Mat& t, double timestamp) {
    timestamps.push_back(timestamp);
    rotations.push_back(R.clone());
    translations.push_back(t.clone());
}

void TrajectoryEstimator::saveTrajectory(const std::string& filename) {
    std::ofstream file(filename);

    for (size_t i = 0; i < timestamps.size(); ++i) {
        Eigen::Matrix3f rotation_matrix;
        cv::cv2eigen(rotations[i], rotation_matrix);
        Eigen::Quaternionf q(rotation_matrix);
        cv::Mat t = translations[i];

        file << timestamps[i] << " "
             << t.at<double>(0) << " "
             << t.at<double>(1) << " "
             << t.at<double>(2) << " "
             << q.x() << " "
             << q.y() << " "
             << q.z() << " "
             << q.w() << "\n";
    }

    file.close();
}
