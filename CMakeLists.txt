cmake_minimum_required(VERSION 3.10)
project(visual-odometry)

option(USE_UBUNTU_20 "Set to ON if you are using Ubuntu 20.04" OFF)

# Set build type
set(CMAKE_BUILD_TYPE "Release")

# Set C++ standard and optimization flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# Include project source and module directories
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Find required packages
find_package(OpenCV 4 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(fmt REQUIRED)
find_package(GTest REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(g2o REQUIRED)


# Include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/include/acquisition  # Include acquisition directory
    ${Pangolin_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
    ${CMAKE_CURRENT_LIST_DIR}/include/anms_lib_include
    /opt/homebrew/include  # Ensure Homebrew include path is added
)

# Link directories for G2O libraries
link_directories(
    /opt/homebrew/lib  # Ensure Homebrew lib path is added
)

# Define sources for the image acquisition module
set(ACQUISITION_SOURCES
    src/acquisition/TUMDatasetImageAcquisition.cpp
)

# Define sources for the keypoint detection module
set(DETECTION_SOURCES
    src/detection/ORBKeypointDetector.cpp
    src/detection/FASTKeypointDetector.cpp
)

# Define sources for the preprocessing module
set(PREPROCESSING_SOURCES
    src/preprocessing/GrayscalePreprocessor.cpp
    src/preprocessing/HistogramEqualizationPreprocessor.cpp
)

# Define sources for the descriptor generation module
set(DESCRIPTOR_SOURCES
    src/descriptors/ORBDescriptorGenerator.cpp
)

# Define sources for the feature matching module
set(MATCHING_SOURCES
    src/matching/BruteForceMatcher.cpp
    src/matching/FLANNMatcher.cpp
    src/matching/RatioTestFLANNMatcher.cpp
)

# Define sources for the homogenization module
set(HOMOGENIZATION_SOURCES
    src/homogenization/ANMSHomogenizer.cpp
    src/anms/anms.cpp
)

# Define sources for the outlier rejection module
set(OUTLIER_REJECTION_SOURCES
    src/outlier_rejection/RANSACOutlierRejection.cpp
    src/outlier_rejection/DistanceOutlierRejection.cpp
)

set(POSE_ESTIMATION_SOURCES
    src/pose_estimation/PNPoseEstimation.cpp
)

# Add image acquisition module to the build
add_library(acquisition ${ACQUISITION_SOURCES})
target_include_directories(acquisition PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(acquisition ${OpenCV_LIBS})

# Add keypoint detection module to the build
add_library(detection ${DETECTION_SOURCES})
target_include_directories(detection PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(detection ${OpenCV_LIBS})

# Add preprocessing module to the build
add_library(preprocessing ${PREPROCESSING_SOURCES})
target_include_directories(preprocessing PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(preprocessing ${OpenCV_LIBS})

# Add descriptor generation module to the build
add_library(descriptor ${DESCRIPTOR_SOURCES})
target_include_directories(descriptor PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(descriptor ${OpenCV_LIBS})

# Add feature matching module to the build
add_library(matching ${MATCHING_SOURCES})
target_include_directories(matching PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(matching ${OpenCV_LIBS})

# Add homogenization module to the build
add_library(homogenization ${HOMOGENIZATION_SOURCES})
target_include_directories(homogenization PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(homogenization ${OpenCV_LIBS})

# Add outlier rejection module to the build
add_library(outlier_rejection ${OUTLIER_REJECTION_SOURCES})
target_include_directories(outlier_rejection PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(outlier_rejection ${OpenCV_LIBS})

# Add pose estimation module to the build
add_library(pose_estimation ${POSE_ESTIMATION_SOURCES})
target_include_directories(pose_estimation PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(pose_estimation ${OpenCV_LIBS})


# Add main executable
add_executable(main_executable src/main.cpp)
target_include_directories(main_executable PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(main_executable
    ${OpenCV_LIBS}
    acquisition
    detection
    preprocessing
    descriptor
    matching
    homogenization
    outlier_rejection
    pose_estimation
    g2o_core
    g2o_stuff
    g2o_types_slam3d
    g2o_solver_eigen
    g2o_solver_dense
)

# # Enable testing
# enable_testing()
# add_subdirectory(test)

# Installation rules
install(TARGETS main_executable
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES ${CMAKE_CURRENT_LIST_DIR}/config/default_config.yaml DESTINATION config)
