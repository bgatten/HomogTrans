cmake_minimum_required(VERSION 3.10)

project(HomogeneousTransformations)

# Set C++ standard (change as needed)
set(CMAKE_CXX_STANDARD 11)

# Add the library source files
add_library(Htf
    src/htf.cpp
)

# Include directories for library headers
target_include_directories(Htf
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add any required external dependencies (e.g., Eigen and OpenCV)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

target_link_libraries(Htf
    PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
)

# Optional: Enable testing
enable_testing()

add_executable(tests
    test/test.cpp
)

target_link_libraries(tests
    PRIVATE
        Htf
        gtest  # Assuming you're using Google Test
        pthread
)

add_test(NAME MyTests COMMAND tests)
