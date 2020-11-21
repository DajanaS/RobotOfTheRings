cmake_minimum_required(VERSION 2.8.3)
project(competition3)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
	sensor_msgs
	nav_msgs
	std_msgs
	geometry_msgs
	tf
	image_transport
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(savePic src/savePic.cpp)
add_executable(faceRecognition src/faceRecognition.cpp)
#add_executable(faceApproach src/faceApproach.cpp)
add_executable(picGoals src/picGoals.cpp)
add_executable(blob src/blob.cpp)
add_executable(filter src/filter.cpp)
#add_executable(ringApproach src/faceApproach.cpp)

target_link_libraries(blob ${catkin_LIBRARIES})
target_link_libraries(picGoals ${catkin_LIBRARIES})
target_link_libraries(savePic ${catkin_LIBRARIES})
target_link_libraries(faceRecognition ${catkin_LIBRARIES})
target_link_libraries(filter ${catkin_LIBRARIES})
#target_link_libraries(faceApproach ${catkin_LIBRARIES})
#target_link_libraries(ringApproach ${catkin_LIBRARIES})
