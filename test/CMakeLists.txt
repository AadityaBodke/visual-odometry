# CMakeLists.txt for tests
enable_testing()

# Find Google Test
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# Define the test executable
add_executable(test_acquisition test_acquisition.cpp)
target_include_directories(test_acquisition PUBLIC ${CMAKE_CURRENT_LIST_DIR}/../include)
target_link_libraries(test_acquisition
    ${OpenCV_LIBS}
    acquisition
    GTest::GTest
    GTest::Main
)

add_test(NAME AcquisitionTest COMMAND test_acquisition)
