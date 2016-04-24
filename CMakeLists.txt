cmake_minimum_required(VERSION 2.8)
project(img_imu_record)

#SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	roslib
	std_msgs
	sensor_msgs
	cv_bridge
)

find_package(Boost REQUIRED COMPONENTS thread system)
find_package(OpenCV 3 REQUIRED)

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
	${OPENCV_INCLUDE_DIRS}
)

# Build our run file
add_executable(${PROJECT_NAME}
		src/main.cpp
)

# Link our libraries
target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
    ${OPENCV_LIBRARIES}
    ${Boost_LIBRARIES}
)

