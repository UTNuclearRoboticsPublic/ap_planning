cmake_minimum_required(VERSION 3.8)
project(sample_based_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-std=c++17)
if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(Boost REQUIRED system filesystem date_time thread serialization program_options)
find_package(catkin REQUIRED COMPONENTS
  affordance_primitives
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  tf2_eigen
)

find_package(ompl REQUIRED)

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
  affordance_primitives
  moveit_ros_planning_interface
  roscpp
 DEPENDS
  OMPL
)


include_directories(
  include
)
include_directories(SYSTEM
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${OMPL_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/screw_motion_planner.cpp
  src/state_utils.cpp
  src/state_sampling.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(panda_demo src/panda_demo.cpp)
target_link_libraries(panda_demo ${PROJECT_NAME})

add_executable(puma_tests src/puma_tests.cpp)
target_link_libraries(puma_tests ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} panda_demo puma_tests
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
