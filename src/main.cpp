#include "acquisition/TUMDatasetImageAcquisition.h"
#include "detection/ORBKeypointDetector.h"
#include "detection/FASTKeypointDetector.h"
#include "descriptors/ORBDescriptorGenerator.h"
#include "preprocessing/GrayscalePreprocessor.h"
#include "preprocessing/HistogramEqualizationPreprocessor.h"
#include "matching/BruteForceMatcher.h"
#include "matching/FLANNMatcher.h"
#include "homogenization/ANMSHomogenizer.h"
#include "outlier_rejection/OutlierRejection.h"
#include "pose_estimation/PoseEstimation.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

int main() {
    try {
        // Initialize the image acquisition module with the dataset path
        TUMDatasetImageAcquisition imageAcquisition("/Users/aadi/Personal/rgbd_vo_tool/datasets/rgbd_dataset_freiburg1_xyz");

        // Set up the ORB keypoint detector and descriptor generator
        ORBConfig orbConfig;
        orbConfig.nfeatures = 1000;
        orbConfig.scaleFactor = 1.5f;
        orbConfig.nlevels = 10;
        ORBKeypointDetector orbDetector(orbConfig);

        // Set up the FAST keypoint detector as an alternative
        FASTConfig fastConfig;
        fastConfig.threshold = 50;
        FASTKeypointDetector fastDetector(fastConfig);

        // Set up the ORB descriptor generator
        ORBDescriptorConfig descriptorConfig;
        descriptorConfig.nfeatures = 1000;
        descriptorConfig.scaleFactor = 1.5f;
        descriptorConfig.nlevels = 10;
        ORBDescriptorGenerator descriptorGenerator(descriptorConfig);

        // Set up the feature matchers
        BFMatcherConfig bfMatcherConfig;
        BruteForceMatcher bfMatcher(bfMatcherConfig);

        FLANNMatcherConfig flannMatcherConfig;
        FLANNMatcher flannMatcher(flannMatcherConfig);

        // Set up the ANMS keypoint homogenizer
        ANMSConfig anmsConfig;
        anmsConfig.numRetPoints = 500;
        anmsConfig.method = "ssc"; 
        ANMSHomogenizer homogenizer(anmsConfig);

        // Set up the RANSAC outlier rejection
        RANSACConfig ransacConfig;
        RANSACOutlierRejection ransacRejection(ransacConfig);

        // Set up the pose estimation module
        PNPoseEstimation poseEstimation;

        // Initialize the trajectory with the ground truth initial pose
        Eigen::Matrix3d initialRotation = Eigen::Quaterniond(-0.3986, 0.6132, 0.5962, -0.3311).toRotationMatrix();
        Eigen::Vector3d initialTranslation(1.3563, 0.6305, 1.6380);

        Eigen::Matrix4d trajectory = Eigen::Matrix4d::Identity();
        trajectory.block<3, 3>(0, 0) = initialRotation;
        trajectory.block<3, 1>(0, 3) = initialTranslation;

        std::vector<Eigen::Matrix4d> poses;
        std::vector<double> timestamps;
        poses.push_back(trajectory);

        // Main loop for processing each frame
        cv::Mat image, depth, cameraMatrix = cv::Mat::eye(3, 3, CV_64F); // Dummy camera matrix
        while (imageAcquisition.acquire(image, depth)) {
            double timestamp = imageAcquisition.getCurrentTimestamp(); 
            timestamps.push_back(timestamp);

            // Detect keypoints using ORB
            std::vector<cv::KeyPoint> orbKeypoints;
            orbDetector.detect(image, orbKeypoints);

            // Optionally use FAST keypoints
            std::vector<cv::KeyPoint> fastKeypoints;
            fastDetector.detect(image, fastKeypoints);

            // Homogenize the keypoints
            orbKeypoints = homogenizer.homogenizeKeypoints(orbKeypoints, image.size());

            // Compute descriptors
            cv::Mat orbDescriptors;
            descriptorGenerator.compute(image, orbKeypoints, orbDescriptors);

            // Homogenize descriptors if needed
            orbDescriptors = homogenizer.homogenizeDescriptors(orbDescriptors);

            // Match features
            std::vector<cv::DMatch> bfMatches;
            bfMatcher.match(orbDescriptors, orbDescriptors, bfMatches);

            // Apply RANSAC outlier rejection
            std::vector<cv::DMatch> ransacInliers;
            ransacRejection.rejectOutliers(orbKeypoints, orbKeypoints, bfMatches, ransacInliers);

            // If we have valid inliers, estimate the pose
            if (!ransacInliers.empty()) {
                Eigen::Matrix4d currentPose;
                poseEstimation.estimatePose(orbKeypoints, orbKeypoints, ransacInliers, cameraMatrix, currentPose);

                // Update the trajectory by integrating the current pose
                trajectory = trajectory * currentPose;
                poses.push_back(trajectory);
            }

            // Visualization (optional)
            cv::Mat bfOutput;
            cv::drawMatches(image, orbKeypoints, image, orbKeypoints, ransacInliers, bfOutput);
            cv::imshow("Brute Force Matches", bfOutput);

            if (cv::waitKey(30) >= 0) break; // Exit loop on key press
        }

        // Save the estimated trajectory to a file
        std::ofstream trajectoryFile("estimated_trajectory.txt");
        for (size_t i = 0; i < poses.size(); ++i) {
            double timestamp = timestamps[i]; 
            Eigen::Quaterniond q(poses[i].block<3, 3>(0, 0));
            Eigen::Vector3d t = poses[i].block<3, 1>(0, 3);
            trajectoryFile << timestamp << " " << t.x() << " " << t.y() << " " << t.z() << " "
                           << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }
        trajectoryFile.close();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
