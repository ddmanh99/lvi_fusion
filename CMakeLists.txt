cmake_minimum_required(VERSION 2.8.3)
project(lvi_fusion)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  cv_bridge
  pcl_conversions
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
  rospy
)
find_package(Ceres)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(OpenMP REQUIRED)
find_package(GTSAM 4 REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development Required Numpy)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
 INCLUDE_DIRS include
 DEPENDS GTSAM PCL
#  LIBRARIES lvi_fusion
 CATKIN_DEPENDS 
 roscpp
 std_msgs
 nav_msgs
 geometry_msgs
 sensor_msgs
 message_runtime
  message_generation
 visualization_msgs
  tf
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${Python3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
	include
	# ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
  ${EIGEN3_LIBS}
  &{PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
add_dependencies(${PROJECT_NAME}_imuPreintegration ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration Boost::timer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${EIGEN3_LIBS} ${PCL_LIBRARIES} gtsam)

add_executable(mapping2d  src/mapping2d.cpp)
target_link_libraries(mapping2d ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBS})

add_executable(${PROJECT_NAME}_laser2cloud src/laser2cloud.cpp)
target_link_libraries(${PROJECT_NAME}_laser2cloud Boost::timer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${EIGEN3_LIBS} ${PCL_LIBRARIES})

add_executable(${PROJECT_NAME}_cloudPreprocess src/cloudPreprocess.cpp)
add_dependencies(${PROJECT_NAME}_cloudPreprocess ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_cloudPreprocess Boost::timer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${EIGEN3_LIBS} ${PCL_LIBRARIES} gtsam)

add_executable(${PROJECT_NAME}_mappingOpti src/mappingOpti.cpp)
add_dependencies(${PROJECT_NAME}_mappingOpti ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_mappingOpti Boost::timer ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS} gtsam)


add_executable(${PROJECT_NAME}_plot_test src/handleData/plot_test.cpp)
target_link_libraries(${PROJECT_NAME}_plot_test Boost::timer ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${EIGEN3_LIBS} ${Python3_LIBRARIES} gtsam)




install(DIRECTORY launch src config include
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS src/handleData/matplot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/handleData/save_data.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/handleData/plot_data.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/tm/pub_data.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)