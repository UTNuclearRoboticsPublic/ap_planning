#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <iostream>
#include <queue>
#include <thread>
#include <utility>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>

#include <affordance_primitive_msgs/ScrewStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <affordance_primitives/screw_model/affordance_utils.hpp>
#include <affordance_primitives/screw_model/screw_axis.hpp>
#include <affordance_primitives/screw_planning/screw_planning.hpp>
#include <ap_planning/ap_planning.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

void show_screw(const affordance_primitives::ScrewStamped &screw_msg,
                moveit_visual_tools::MoveItVisualTools &visual_tools) {
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
  Eigen::Vector3d axis, origin;
  tf2::fromMsg(screw_msg.axis, axis);
  tf2::fromMsg(screw_msg.origin, origin);

  Eigen::Vector3d end_point = origin + 0.2 * axis.normalized();
  geometry_msgs::Point end = tf2::toMsg(end_point);

  visual_tools.publishArrow(screw_msg.origin, end);
  visual_tools.trigger();
}

void show_trajectory(const trajectory_msgs::JointTrajectory &traj,
                     moveit_visual_tools::MoveItVisualTools &visual_tools) {
  moveit_msgs::DisplayTrajectory joint_traj;
  joint_traj.model_id = "panda";
  joint_traj.trajectory.push_back(moveit_msgs::RobotTrajectory());
  joint_traj.trajectory.at(0).joint_trajectory = traj;

  moveit_msgs::RobotState start_msg;
  start_msg.joint_state.name = traj.joint_names;
  auto first_waypoint = traj.points.at(0);
  start_msg.joint_state.position = first_waypoint.positions;
  joint_traj.trajectory_start = start_msg;

  joint_traj.trajectory.at(0).joint_trajectory.header.frame_id = "panda_link0";

  int time = 0;

  for (auto &wp : joint_traj.trajectory.at(0).joint_trajectory.points) {
    wp.time_from_start.sec = time;
    ++time;
  }

  visual_tools.publishTrajectoryPath(joint_traj);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ap_planning");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr &kinematic_model =
      robot_model_loader.getModel();

  moveit::core::RobotStatePtr kinematic_state(
      new moveit::core::RobotState(kinematic_model));
  std::vector<double> default_joint_state{0, -0.785, 0,    -2.356,
                                          0, 1.571,  0.785};
  kinematic_state->setJointGroupPositions("panda_arm", default_joint_state);

  ros::Duration(2.0).sleep();

  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();
  visual_tools.loadRemoteControl();
  visual_tools.setRobotStateTopic("/display_robot_state");
  visual_tools.trigger();

  std::queue<ap_planning::APPlanningRequest> planning_queue;
  ap_planning::APPlanningRequest single_request;
  single_request.screw_msg.header.frame_id = "panda_link0";
  single_request.ee_frame_name = "panda_link8";

  // For now, all requests start at same point
  single_request.start_pose.pose.position.x = 0.5;
  single_request.start_pose.pose.position.z = 0.3;
  single_request.start_pose.pose.orientation.x = 1.0;
  single_request.start_pose.pose.orientation.w = 0;

  // Add some test cases
  single_request.theta = 0.25 * M_PI;
  single_request.screw_msg.origin = single_request.start_pose.pose.position;
  single_request.screw_msg.axis.x = 1;
  planning_queue.push(single_request);

  // This time, send a joint configuration
  single_request.start_joint_state = default_joint_state;
  planning_queue.push(single_request);

  single_request.start_joint_state.clear();
  single_request.theta = 0.5 * M_PI;
  single_request.screw_msg.axis.x = -1;
  planning_queue.push(single_request);

  single_request.theta = 0.25 * M_PI;
  single_request.screw_msg.axis.x = 0;
  single_request.screw_msg.axis.z = 1;
  planning_queue.push(single_request);

  single_request.screw_msg.origin.x -= 0.1;
  single_request.screw_msg.pitch = 0.1;
  planning_queue.push(single_request);

  single_request.screw_msg.origin = geometry_msgs::Point();
  planning_queue.push(single_request);

  single_request.screw_msg.origin = single_request.start_pose.pose.position;
  single_request.screw_msg.axis.x = -1;
  single_request.screw_msg.axis.z = 1;
  single_request.screw_msg.is_pure_translation = true;
  planning_queue.push(single_request);

  ap_planning::ScrewPlanner ap_planner("panda_arm");
  ap_planning::NaivePlanner naive_planner(nh);
  if (!naive_planner.initialize()) {
    ROS_ERROR_STREAM("Init failed");
    return EXIT_FAILURE;
  }

  // Plan each screw request
  while (planning_queue.size() > 0 && ros::ok()) {
    auto req = planning_queue.front();
    planning_queue.pop();
    visual_tools.prompt(
        "Press 'next' in the RvizVisualToolsGui window to plan next screw");

    show_screw(req.screw_msg, visual_tools);

    ap_planning::APPlanningResponse result;
    bool success = ap_planner.plan(req, result);
    if (success) {
      std::cout << "\n\n\nScrew planning: Success!!\n\n";
      std::cout << "Trajectory is: " << result.percentage_complete * 100
                << "% complete, and has length: " << result.path_length << "\n";

      show_trajectory(result.joint_trajectory, visual_tools);
    } else {
      std::cout << "\n\n\nScrew planning: Fail!!\n\n";
      continue;
    }

    // Now move to naive planner
    visual_tools.prompt(
        "Press 'next' in the RvizVisualToolsGui window to plan again using "
        "naive planner");

    // Create the AP goal message
    affordance_primitive_msgs::AffordancePrimitiveGoal ap_goal;
    ap_goal.moving_frame_name = req.ee_frame_name;
    ap_goal.moving_frame_source = ap_goal.LOOKUP;
    ap_goal.theta_dot = 0.3;
    ap_goal.screw_distance = req.theta;
    ap_goal.screw = req.screw_msg;

    // Try planning
    trajectory_msgs::JointTrajectory naive_output;
    auto naive_res = naive_planner.plan(ap_goal, kinematic_state, naive_output);
    if (naive_res == ap_planning::SUCCESS) {
      std::cout << "\n\n\nNaive planning: Success!!\n\n";
      show_trajectory(naive_output, visual_tools);
    } else {
      std::cout << "\n\n\nNaive planning: Fail!!\n\n";
      continue;
    }
  }

  ros::shutdown();
  return 0;
}
