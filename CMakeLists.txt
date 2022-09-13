cmake_minimum_required(VERSION 3.0.2)
project(ap_planning)

add_compile_options(-std=c++17)
if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  affordance_primitive_msgs
  affordance_primitives
  bio_ik
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  pluginlib
  roscpp
  tf2_eigen
)

## System dependencies are found with CMake's conventions
find_package(ompl REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread serialization program_options)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS
  include
 LIBRARIES
  ${PROJECT_NAME}
 CATKIN_DEPENDS
  affordance_primitive_msgs
  affordance_primitives
  bio_ik
  moveit_core
  moveit_ros_planning_interface
  roscpp
 DEPENDS
  OMPL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
include_directories(SYSTEM
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${OMPL_INCLUDE_DIRS})

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/ap_planning.cpp
  src/ik_solver.cpp
  src/screw_motion_planner.cpp
  src/state_sampling.cpp
  src/state_utils.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(panda_demo src/panda_demo.cpp)
target_link_libraries(panda_demo ${PROJECT_NAME})

add_executable(puma_tests src/puma_tests.cpp)
target_link_libraries(puma_tests ${PROJECT_NAME})

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME} panda_demo puma_tests
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ap_planning.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
