cmake_minimum_required(VERSION 3.0.2)
project(gtsam_ros)
 
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS
	thread
	system
	regex
	timer
	serialization
	filesystem
)

add_service_files(
  FILES
  ISAMinput.srv 
  )

generate_messages(
  DEPENDENCIES
  geometry_msgs 
)
find_package(Eigen3 REQUIRED)
find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
#	${EIGEN3_INCLUDE_DIR}

)

add_executable(CameraResectioning src/examples/CameraResectioning)
add_executable(creating_fg src/examples/creating_fg)
add_executable(localization src/examples/localization)
add_executable(pose_slam src/examples/pose_slam)
add_executable(gtsamSolver src/gtsamSolver)

target_link_libraries(CameraResectioning ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}  gtsam)

target_link_libraries(creating_fg ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
	${Boost_LIBRARIES} gtsam)

target_link_libraries(localization ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}  gtsam)

target_link_libraries(pose_slam ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}  gtsam)

target_link_libraries(gtsamSolver ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}  gtsam)
