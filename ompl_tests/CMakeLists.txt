cmake_minimum_required(VERSION 3.8)
project(ompl_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread serialization program_options)

find_package(affordance_primitives REQUIRED)
find_package(ompl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(tf2_eigen REQUIRED)

include_directories(include)
include_directories(SYSTEM ${OMPL_INCLUDE_DIRS})

# add_executable(sphere_constraint src/constrained_sphere.cpp)
# ament_target_dependencies(sphere_constraint OMPL Boost)

# add_executable(circle_constraint src/constrained_circle.cpp)
# ament_target_dependencies(circle_constraint OMPL Boost)

# add_executable(screw_constraint src/constrained_screw.cpp)
# ament_target_dependencies(screw_constraint affordance_primitives OMPL Boost)

add_executable(sample_circle src/sample_circle.cpp)
ament_target_dependencies(sample_circle affordance_primitives moveit_ros_planning_interface moveit_visual_tools rclcpp OMPL Boost)

add_executable(state_sampling_demo src/state_sampling_demo.cpp)
ament_target_dependencies(state_sampling_demo affordance_primitives moveit_ros_planning_interface moveit_visual_tools rclcpp OMPL Boost)

install(TARGETS sample_circle state_sampling_demo
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
