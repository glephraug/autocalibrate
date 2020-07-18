
cmake_minimum_required(VERSION 3.0)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(autocalibrate
	src/main.cpp
)
