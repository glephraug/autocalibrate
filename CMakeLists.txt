
cmake_minimum_required(VERSION 3.0)

project(autocalibrate)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(autocalibrate
	src/main.cpp
	src/types.h
	src/match.h
	src/match.cpp
 "src/autocalibrate.h" "src/autocalibrate.cpp")

target_include_directories(autocalibrate PRIVATE
	${OpenCV_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(autocalibrate
	${OpenCV_LIBS}
)
