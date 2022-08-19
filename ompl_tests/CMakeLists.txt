cmake_minimum_required(VERSION 3.8)
project(ompl_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(Boost REQUIRED system filesystem date_time thread serialization program_options)
find_package(catkin REQUIRED COMPONENTS
  affordance_primitives
  roscpp
  moveit_ros_planning_interface
  moveit_visual_tools
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
#  DEPENDS system_lib
)


include_directories(
  include
)
include_directories(SYSTEM
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${OMPL_INCLUDE_DIRS})

# add_executable(sphere_constraint src/constrained_sphere.cpp)
# ament_target_dependencies(sphere_constraint OMPL Boost)

# add_executable(circle_constraint src/constrained_circle.cpp)
# ament_target_dependencies(circle_constraint OMPL Boost)

# add_executable(screw_constraint src/constrained_screw.cpp)
# ament_target_dependencies(screw_constraint affordance_primitives OMPL Boost)

add_executable(sample_circle src/sample_circle.cpp)
target_link_libraries(sample_circle ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# add_executable(state_sampling_demo src/state_sampling_demo.cpp)
# target_link_libraries(state_sampling_demo ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS sample_circle
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
