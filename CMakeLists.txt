cmake_minimum_required(VERSION 3.0.0)
project(ma_loam)

add_compile_options(-Wpedantic -Wall -Wextra)

set(CATKIN_PACKAGES
    gazebo_ros)

find_package(catkin REQUIRED ${CATKIN_PACKAGES})

###################################
## catkin specific configuration ##
###################################

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${CATKIN_PACKAGES}
)

###########
## Build ##
###########

#############
## Install ##
#############

## Mark launch files for installation
install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
