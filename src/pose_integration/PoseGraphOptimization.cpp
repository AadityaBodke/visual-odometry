#include "pose_integration/PoseIntegration.h"
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <opencv2/core/eigen.hpp>

PoseGraphOptimization::PoseGraphOptimization() {
    trajectory.push_back(currentPose.clone());
}

void PoseGraphOptimization::integratePose(const cv::Mat& R, const cv::Mat& t) {
    cv::Mat Rt = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(Rt(cv::Rect(0, 0, 3, 3)));
    t.copyTo(Rt(cv::Rect(3, 0, 1, 3)));
    
    currentPose = currentPose * Rt;
    trajectory.push_back(currentPose.clone());
    edges.emplace_back(R.clone(), t.clone());
}

std::vector<cv::Mat> PoseGraphOptimization::getTrajectory() const {
    return trajectory;
}

void PoseGraphOptimization::optimizeGraph() {
    using namespace g2o;
    
    SparseOptimizer optimizer;
    auto linearSolver = std::make_unique<LinearSolverDense<BlockSolverX::PoseMatrixType>>();
    auto blockSolver = std::make_unique<BlockSolverX>(std::move(linearSolver));
    OptimizationAlgorithmLevenberg* algorithm = new OptimizationAlgorithmLevenberg(std::move(blockSolver));
    optimizer.setAlgorithm(algorithm);

    // Add vertices
    for (size_t i = 0; i < trajectory.size(); ++i) {
        VertexSE3* vertex = new VertexSE3();
        vertex->setId(static_cast<int>(i));
        if (i == 0) {
            vertex->setFixed(true);
        }
        Eigen::Isometry3d pose;
        cv::cv2eigen(trajectory[i], pose.matrix());
        vertex->setEstimate(pose);
        optimizer.addVertex(vertex);
    }

    // Add edges
    for (size_t i = 1; i < trajectory.size(); ++i) {
        EdgeSE3* edge = new EdgeSE3();
        edge->vertices()[0] = optimizer.vertex(static_cast<int>(i - 1));
        edge->vertices()[1] = optimizer.vertex(static_cast<int>(i));
        
        Eigen::Matrix3d R_eigen;
        cv::cv2eigen(edges[i - 1].first, R_eigen);
        Eigen::Vector3d t_eigen;
        cv::cv2eigen(edges[i - 1].second, t_eigen);
        
        Eigen::Isometry3d relativePose = Eigen::Isometry3d::Identity();
        relativePose.linear() = R_eigen;
        relativePose.translation() = t_eigen;
        
        edge->setMeasurement(relativePose);
        edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity());
        optimizer.addEdge(edge);
    }

    optimizer.initializeOptimization();
    optimizer.optimize(10);

    // Update the trajectory with optimized poses
    for (size_t i = 0; i < trajectory.size(); ++i) {
        VertexSE3* vertex = static_cast<VertexSE3*>(optimizer.vertex(static_cast<int>(i)));
        Eigen::Isometry3d optimizedPose = vertex->estimate();
        cv::eigen2cv(optimizedPose.matrix(), trajectory[i]);
    }
}
