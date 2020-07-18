
cmake_minimum_required(VERSION 3.0)

project(autocalibrate)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(autocalibrate
	src/main.cpp
)
