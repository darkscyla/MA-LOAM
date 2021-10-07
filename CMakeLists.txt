cmake_minimum_required(VERSION 3.0.0)
project(ma_loam)

add_compile_options(-Wpedantic -Wall -Wextra -O3)

set(CATKIN_PACKAGES
  gazebo_ros
  pcl_conversions
  pcl_ros)

find_package(catkin REQUIRED ${CATKIN_PACKAGES})
find_package(Ceres REQUIRED)
find_package(CGAL REQUIRED)
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

add_executable(${PROJECT_NAME}_runner src/main.cpp)
target_include_directories(${PROJECT_NAME}_runner PUBLIC src 
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_runner ${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${PCL_LIBRARIES}
  ${pcl_ros_LIBRARIES})

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
