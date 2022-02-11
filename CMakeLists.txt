cmake_minimum_required(VERSION 3.0.0)
project(ma_loam)

add_compile_options(-Wpedantic -Wall -Wextra -O3 -g)

set(CATKIN_PACKAGES
  cv_bridge
  gazebo_ros
  image_transport
  nav_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros)

find_package(catkin REQUIRED ${CATKIN_PACKAGES})
find_package(Ceres REQUIRED)
find_package(CGAL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_PACKAGES}
)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/cluster_icp.cpp
  src/cluster_icp_ros.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC 
  src
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${CGAL_LIBRARIES}
  ${PCL_LIBRARIES})

# Independent runner from LOAM
add_executable(${PROJECT_NAME}_runner src/main.cpp)
target_include_directories(${PROJECT_NAME}_runner PUBLIC
  src 
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_runner
  ${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${PCL_LIBRARIES}
  ${pcl_ros_LIBRARIES})

# Scan registration
add_executable(${PROJECT_NAME}_scan_registration
  src/aloam_mod/scanRegistration.cpp)
target_include_directories(${PROJECT_NAME}_scan_registration PUBLIC
  src
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_scan_registration PUBLIC
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})

# Odometry
add_executable(${PROJECT_NAME}_odometry
  src/aloam_mod/laserOdometry.cpp)
target_include_directories(${PROJECT_NAME}_odometry PUBLIC
  src
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_odometry PUBLIC
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${PCL_LIBRARIES})

# Mapping
add_executable(${PROJECT_NAME}_mapping
  src/aloam_mod/laserMapping.cpp)
target_include_directories(${PROJECT_NAME}_mapping PUBLIC
  src
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_mapping PUBLIC
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${PCL_LIBRARIES})

# KITTI helper (though the model does not exist, we can still use 
# the preprocessing phase to improve accuracy)
add_executable(${PROJECT_NAME}_kitti_helper
  src/aloam_mod/kittiHelper.cpp)
target_include_directories(${PROJECT_NAME}_kitti_helper PUBLIC
  src
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_kitti_helper PUBLIC
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES})

#############
## Install ##
#############

## Mark launch files for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark python scripts for installation
install(PROGRAMS
  scripts/*.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
install(TARGETS ${PROJECT_NAME}_runner
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
