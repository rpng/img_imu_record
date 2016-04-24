cmake_minimum_required(VERSION 2.8)
project(img_imu_record)


# Eigen3 is needed
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	roslib
	std_msgs
	sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    cv_bridge
    roslib
    rosbag
	sensor_msgs
)


include_directories(
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${OPENCV_INCLUDE_DIRS}
)

# Build our run file
add_executable(${PROJECT_NAME}
		src/main.cpp
)

# Link our libraries
target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
)

